#!/usr/bin/env python3
"""agrobot_bridge — publica a telemetria do Husky no sistema Agrobot (RabbitMQ).

Mesmo contrato que o drone `x650-jetson` já usa (envelope `Agrobot_v2`), mas falando AMQP
direto com o broker em vez de passar por tópicos ROS globais: o Husky roda atrás de um
FastDDS Discovery Server, então os tópicos dele não aparecem no grafo multicast onde o
conversor `rabbitros` do drone escuta.

    ROS (discovery server do Husky)                     RabbitMQ 10.0.0.96
    ───────────────────────────────                     ──────────────────
    sensors/ins_0/gps_0/fix, gps_1/fix  ─┐
    sensors/ins_0/fixposition/fpa/gnsscorr│
    platform/bms/state                   ├─► agrobot_bridge ─► agent_telemetry   (1 s)
    platform/odom/filtered               │                  ─► agent_keep_alive  (10 s)
    gps_waypoint_follower/status        ─┘                  ─► agent_details     (20 s)

Posição e rumo vêm das duas antenas do Fixposition (mesma conta do `gps_waypoint_follower`
em `pose_source: dual_gnss`), então a telemetria funciona com ou sem missão em andamento.
O estado da missão, quando o seguidor está no ar, entra no campo `status`.

Uso:
    ros2 run agrobot_husky_nav agrobot_bridge --ros-args \
        --params-file $(ros2 pkg prefix agrobot_husky_nav)/share/agrobot_husky_nav/config/agrobot_bridge.yaml \
        -r __ns:=/a300_00096
"""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String

try:
    import pika
except ImportError:  # pika fica em /home/robot/pylibs (PYTHONPATH), não nos pacotes do sistema
    pika = None

try:
    from fixposition_driver_msgs.msg import FpaGnsscorr
except ImportError:
    FpaGnsscorr = None

EARTH_R = 6371008.8

# estado do gps_waypoint_follower -> enums do contrato Agrobot_v2
TELEMETRY_STATUS = {'RUNNING': 'IN_MISSION', 'BLOCKED': 'IN_MISSION', 'PAUSED': 'IN_MISSION',
                    'IDLE': 'IDLE', 'DONE': 'IDLE', 'ABORTED': 'IDLE'}
KEEPALIVE_STATUS = {'RUNNING': 'IN_MISSION', 'BLOCKED': 'IN_MISSION', 'PAUSED': 'IN_MISSION',
                    'IDLE': 'ONLINE', 'DONE': 'ONLINE', 'ABORTED': 'ONLINE'}


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def bearing_enu(lat0, lon0, lat1, lon1):
    lat0r = math.radians(lat0)
    e = math.radians(lon1 - lon0) * math.cos(lat0r) * EARTH_R
    n = math.radians(lat1 - lat0) * EARTH_R
    return math.atan2(n, e)


class AgrobotBridge(Node):
    def __init__(self):
        super().__init__('agrobot_bridge')
        p = self.declare_parameter
        p('broker_host', '10.0.0.96'); p('broker_port', 5672)
        p('broker_user', 'agrobot'); p('broker_password', 'agrobot'); p('broker_vhost', '/')
        p('agent_source', 'husky'); p('agent_destination', 'service'); p('envelope_version', 'Agrobot_v2')
        # `agent_id` é o id numérico do agente na camada de serviços (tabela agents);
        # o orquestrador usa `agentId` para casar a telemetria com a linha do banco.
        p('agent_id', ''); p('protocol', 'ROS2')
        p('agent_name', 'Husky'); p('agent_type', 'GROUND'); p('agent_model', 'A300')
        p('serial_number', 'a300-00096'); p('controller', 'Clearpath A300 MCU')
        p('firmware', 'agrobot_husky_nav'); p('supported_activities', ['MAPPING', 'MONITORING'])
        p('weight_kg', 70.0); p('max_load_kg', 100.0); p('max_speed_ms', 1.3)
        p('autonomy_minutes', 180); p('sensors', ['gnss_rtk', 'lidar_3d', 'imu']); p('actuators', [''])
        p('telemetry_period_s', 1.0); p('keep_alive_period_s', 10.0); p('details_period_s', 20.0)
        p('dual_gnss_yaw_offset_deg', 90.0); p('publish_when_stale', True)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cfg = {n: g(n) for n in (
            'broker_host', 'broker_port', 'broker_user', 'broker_password', 'broker_vhost',
            'agent_source', 'agent_destination', 'envelope_version', 'agent_name', 'agent_type',
            'agent_model', 'serial_number', 'controller', 'firmware', 'supported_activities', 'agent_id', 'protocol',
            'weight_kg', 'max_load_kg', 'max_speed_ms', 'autonomy_minutes', 'sensors', 'actuators',
            'publish_when_stale')}
        self.yaw_offset = math.radians(float(g('dual_gnss_yaw_offset_deg')))

        # ------------------------------------------------------------------ estado
        self.ant = [None, None]
        self.lat = self.lon = self.alt = self.yaw = None
        self.pose_time = 0.0
        self.gnss_fix = [None, None]
        self.battery_pct = None
        self.speed = 0.0
        self.follower = {}
        self.follower_time = 0.0
        self.seq = 0
        self.sent = {'agent_telemetry': 0, 'agent_keep_alive': 0, 'agent_details': 0}
        self.failures = 0
        self.conn = self.ch = None

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(NavSatFix, 'sensors/ins_0/gps_0/fix',
                                 lambda m: self.on_ant(0, m), sensor_qos)
        self.create_subscription(NavSatFix, 'sensors/ins_0/gps_1/fix',
                                 lambda m: self.on_ant(1, m), sensor_qos)
        if FpaGnsscorr is not None:
            self.create_subscription(FpaGnsscorr, 'sensors/ins_0/fixposition/fpa/gnsscorr',
                                     self.on_gnsscorr, sensor_qos)
        self.create_subscription(BatteryState, 'platform/bms/state', self.on_battery, 10)
        self.create_subscription(Odometry, 'platform/odom/filtered', self.on_odom, sensor_qos)
        self.create_subscription(String, 'gps_waypoint_follower/status', self.on_follower, 10)

        self.create_timer(float(g('telemetry_period_s')), self.send_telemetry)
        self.create_timer(float(g('keep_alive_period_s')), self.send_keep_alive)
        self.create_timer(float(g('details_period_s')), self.send_details)
        self.create_timer(0.5, self.service_connection)
        self.create_timer(30.0, self.log_summary)

        self.connect()
        self.get_logger().info('ponte pronta: %s -> amqp://%s:%d (agente "%s")' % (
            self.get_namespace(), self.cfg['broker_host'], self.cfg['broker_port'], self.cfg['agent_source']))
        self.send_details()

    # ---------------------------------------------------------------- AMQP
    def connect(self):
        if pika is None:
            self.get_logger().error('pika não encontrado (use PYTHONPATH=/home/robot/pylibs)')
            return False
        try:
            params = pika.ConnectionParameters(
                host=self.cfg['broker_host'], port=int(self.cfg['broker_port']),
                virtual_host=self.cfg['broker_vhost'],
                credentials=pika.PlainCredentials(self.cfg['broker_user'], self.cfg['broker_password']),
                heartbeat=30, blocked_connection_timeout=15, socket_timeout=5, connection_attempts=1)
            self.conn = pika.BlockingConnection(params)
            self.ch = self.conn.channel()
            self.failures = 0
            self.get_logger().info('conectado ao broker %s:%d' % (self.cfg['broker_host'], int(self.cfg['broker_port'])))
            return True
        except Exception as e:  # noqa: BLE001
            self.conn = self.ch = None
            self.failures += 1
            if self.failures <= 3 or self.failures % 20 == 0:
                self.get_logger().warn('falha ao conectar no broker (%d): %s' % (self.failures, e))
            return False

    def service_connection(self):
        """Mantém o heartbeat vivo e reconecta quando o broker cai."""
        if self.conn is not None and self.conn.is_open:
            try:
                self.conn.process_data_events(0)
                return
            except Exception:  # noqa: BLE001
                self.conn = self.ch = None
        if self.failures == 0 or self.failures % 10 == 0:
            self.connect()
        else:
            self.failures += 1

    def publish(self, queue, payload):
        """Publica o envelope Agrobot_v2 na fila (default exchange, routing_key = fila)."""
        self.seq += 1
        now = self.get_clock().now()
        envelope = {
            'source': self.cfg['agent_source'],
            'destination': self.cfg['agent_destination'],
            # `agentId`/`protocol` são exigidos pelo envelope do orquestrador; `source`/`destination`
            # são os do contrato publicado. Mandamos os quatro para funcionar nas duas leituras.
            'agentId': self.cfg['agent_id'] or self.cfg['agent_source'],
            'protocol': self.cfg['protocol'],
            'version': self.cfg['envelope_version'],
            'correlationId': str(now.nanoseconds),
            'timestamp': int(now.nanoseconds // 1_000_000_000),
            'seq': self.seq,
            'payload': payload,
        }
        body = json.dumps(envelope)
        if self.ch is None and not self.connect():
            return False
        try:
            self.ch.basic_publish(
                exchange='', routing_key=queue, body=body,
                properties=pika.BasicProperties(content_type='application/json', delivery_mode=2,
                                                timestamp=envelope['timestamp']))
            self.sent[queue] = self.sent.get(queue, 0) + 1
            return True
        except Exception as e:  # noqa: BLE001
            self.conn = self.ch = None
            self.failures += 1
            if self.failures <= 3:
                self.get_logger().warn('falha ao publicar em %s: %s' % (queue, e))
            return False

    # ---------------------------------------------------------------- ROS
    def now_s(self):
        return time.monotonic()

    def on_ant(self, idx, msg: NavSatFix):
        if msg.status.status < 0 or msg.latitude == 0.0:
            return
        self.ant[idx] = (msg.latitude, msg.longitude, msg.altitude, self.now_s())
        a1, a2 = self.ant
        if a1 is None or a2 is None or abs(a1[3] - a2[3]) > 0.5:
            return
        self.lat = (a1[0] + a2[0]) / 2.0
        self.lon = (a1[1] + a2[1]) / 2.0
        self.alt = (a1[2] + a2[2]) / 2.0
        self.yaw = wrap(bearing_enu(a1[0], a1[1], a2[0], a2[1]) + self.yaw_offset)
        self.pose_time = self.now_s()

    def on_gnsscorr(self, msg):
        self.gnss_fix = [int(msg.gnss1_fix), int(msg.gnss2_fix)]

    def on_battery(self, msg: BatteryState):
        if not math.isnan(msg.percentage):
            self.battery_pct = round(msg.percentage * 100.0, 1)

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.speed = round(math.hypot(v.x, v.y), 3)

    def on_follower(self, msg: String):
        try:
            self.follower = json.loads(msg.data)
            self.follower_time = self.now_s()
        except Exception:  # noqa: BLE001
            pass

    # ---------------------------------------------------------------- estado
    def mission_state(self):
        if self.now_s() - self.follower_time > 5.0:
            return None
        return self.follower.get('state')

    def compass_heading(self):
        """ENU (0 = leste, anti-horário) -> bússola (0 = norte, horário)."""
        if self.yaw is None:
            return None
        return round((90.0 - math.degrees(self.yaw)) % 360.0, 1)

    def pose_ok(self):
        return self.lat is not None and self.now_s() - self.pose_time < 3.0

    # ---------------------------------------------------------------- mensagens
    def send_telemetry(self):
        if not self.pose_ok() and not self.cfg['publish_when_stale']:
            return
        state = self.mission_state()
        status = TELEMETRY_STATUS.get(state, 'IDLE') if state else 'IDLE'
        if not self.pose_ok():
            status = 'ERROR'
        payload = {
            'position': {'lat': self.lat, 'lng': self.lon, 'altitude': self.alt},
            'battery': self.battery_pct if self.battery_pct is not None else 0,
            'speed': self.speed,
            'heading': self.compass_heading(),
            'status': status,
        }
        if self.lat is None:  # ainda sem fix: manda posição nula em vez de omitir o campo
            payload['position'] = {'lat': 0.0, 'lng': 0.0, 'altitude': 0.0}
            payload['heading'] = 0.0
        self.publish('agent_telemetry', payload)

    def send_keep_alive(self):
        state = self.mission_state()
        status = KEEPALIVE_STATUS.get(state, 'ONLINE') if state else 'ONLINE'
        if not self.pose_ok():
            status = 'ERROR'
        self.publish('agent_keep_alive', {
            'name': self.cfg['agent_name'],
            'serialNumber': self.cfg['serial_number'],
            'battery': self.battery_pct if self.battery_pct is not None else 0,
            'status': status,
        })

    def send_details(self):
        self.publish('agent_details', {
            'name': self.cfg['agent_name'],
            'type': self.cfg['agent_type'],
            'model': self.cfg['agent_model'],
            'serialNumber': self.cfg['serial_number'],
            'controller': self.cfg['controller'],
            'firmware': self.cfg['firmware'],
            'supportedActivities': list(self.cfg['supported_activities']),
            'physical': {
                'weight': float(self.cfg['weight_kg']),
                'maxLoad': float(self.cfg['max_load_kg']),
                'maxSpeed': float(self.cfg['max_speed_ms']),
                'autonomyMinutes': int(self.cfg['autonomy_minutes']),
                'sensors': list(self.cfg['sensors']),
                'actuators': [a for a in self.cfg['actuators'] if a],
            },
        })

    def log_summary(self):
        self.get_logger().info(
            'broker=%s | enviados tel=%d ka=%d det=%d | pose=%s fix=%s bat=%s vel=%.2f missao=%s' % (
                'ok' if self.ch is not None else 'CAIDO',
                self.sent['agent_telemetry'], self.sent['agent_keep_alive'], self.sent['agent_details'],
                'ok' if self.pose_ok() else 'sem', self.gnss_fix, self.battery_pct, self.speed,
                self.mission_state() or 'sem seguidor'))


def main(args=None):
    rclpy.init(args=args)
    node = AgrobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.conn is not None and node.conn.is_open:
                node.conn.close()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
