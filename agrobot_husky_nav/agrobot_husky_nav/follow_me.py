#!/usr/bin/env python3
"""follow_me — o Husky segue uma pessoa que carrega um marcador ArUco.

O alvo é identificado por um **marcador ArUco** impresso (nada de rede neural, nada de SDK da
ZED). Com a calibração de fábrica da câmera e o tamanho real do marcador, uma única imagem já
dá a posição 3D do alvo; o lidar não precisa entrar no laço de controle e fica só como
segurança. O controle mantém o alvo **centrado** e a uma **distância fixa**.

    zed/left/image_raw + camera_info ──► detecta ArUco ──► pose do alvo em base_link
                                                             │
    scan (MID360) ──► guarda de obstáculos ─────────────────►├──► cmd_vel (TwistStamped)
    joy_teleop/joy ──► bypass do operador ──────────────────►┘

Estados: `IDLE`, `FOLLOWING`, `LOST` (alvo sumiu), `BLOCKED` (obstáculo que não é o alvo),
`PAUSED` (joystick ou saúde), `ABORTED`. Igual ao seguidor de waypoints, o operador manda:
segurar **L1** pausa e assume, **Círculo** aborta, **Options** inicia ou retoma.

A guarda de obstáculos ignora o setor onde o alvo está — senão o robô pararia por causa da
própria pessoa que ele deve seguir.

Gere o marcador para impressão com `tools/make_aruco.py` (mesmo dicionário e id daqui).
"""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Joy, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

IDLE, FOLLOWING, LOST, BLOCKED, PAUSED, ABORTED = 'IDLE', 'FOLLOWING', 'LOST', 'BLOCKED', 'PAUSED', 'ABORTED'


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class FollowMe(Node):
    def __init__(self):
        super().__init__('follow_me')
        p = self.declare_parameter
        p('image_topic', 'zed/left/image_raw')
        p('camera_info_topic', 'zed/left/camera_info')
        p('scan_topic', 'scan')
        p('marker_dict', 'DICT_4X4_50')
        p('marker_id', -1)                 # -1 = segue qualquer marcador do dicionário
        p('marker_size_m', 0.20)           # lado do quadrado preto impresso
        # pose da câmera em base_link (frente do robô). MEDIR e ajustar.
        p('camera_xyz', [0.25, 0.0, 0.60])
        p('camera_yaw_deg', 0.0)
        # controle
        p('target_distance', 1.5)
        p('distance_deadband', 0.20)
        p('bearing_deadband_deg', 4.0)
        p('k_linear', 0.6)
        p('k_angular', 1.2)
        p('max_linear', 0.5)
        p('max_angular', 0.8)
        p('min_linear', 0.08)
        p('allow_reverse', True)           # recua se a pessoa se aproximar demais
        p('control_rate', 10.0)
        # segurança
        p('target_timeout', 1.5)           # s sem ver o marcador -> LOST
        p('obstacle_stop_distance', 0.9)
        p('obstacle_half_angle_deg', 35.0)
        # A guarda precisa ignorar a pessoa que está sendo seguida. Um setor angular fixo não
        # basta: a 1 m, ±20° cobre só 36 cm, e os ombros ficam de fora. O setor passa a ser
        # calculado pela largura da pessoa na distância em que ela está, e só vale para
        # retornos que estejam **à mesma distância** do alvo.
        p('target_mask_deg', 20.0)         # setor mínimo em volta do alvo
        p('target_width_m', 0.9)           # largura considerada da pessoa
        p('target_mask_range_m', 0.6)      # tolerância de distância para considerar "é o alvo"
        p('scan_timeout', 1.5)
        p('require_joy', True)
        p('joy_timeout', 1.0)
        p('joy_start_button', 9)
        p('joy_stop_button', 1)
        p('joy_enable_buttons', [4, 5])
        p('detect_rate', 10.0)             # detecções por segundo (a imagem chega a ~30)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.marker_id = int(g('marker_id'))
        self.marker_size = float(g('marker_size_m'))
        self.cam_xyz = [float(v) for v in g('camera_xyz')]
        self.cam_yaw = math.radians(float(g('camera_yaw_deg')))
        self.target_d = float(g('target_distance'))
        self.dead_d = float(g('distance_deadband'))
        self.dead_b = math.radians(float(g('bearing_deadband_deg')))
        self.k_lin, self.k_ang = float(g('k_linear')), float(g('k_angular'))
        self.max_lin, self.max_ang = float(g('max_linear')), float(g('max_angular'))
        self.min_lin = float(g('min_linear'))
        self.allow_reverse = bool(g('allow_reverse'))
        self.target_timeout = float(g('target_timeout'))
        self.obs_stop = float(g('obstacle_stop_distance'))
        self.obs_half = math.radians(float(g('obstacle_half_angle_deg')))
        self.mask = math.radians(float(g('target_mask_deg')))
        self.target_w = float(g('target_width_m'))
        self.mask_range = float(g('target_mask_range_m'))
        self.scan_timeout = float(g('scan_timeout'))
        self.require_joy, self.joy_timeout = bool(g('require_joy')), float(g('joy_timeout'))
        self.btn_start, self.btn_stop = int(g('joy_start_button')), int(g('joy_stop_button'))
        self.btn_enable = {int(b) for b in g('joy_enable_buttons')}

        # ---------------------------------------------------------------- estado
        self.state = IDLE
        self.reason = ''
        # Pausa por comando (serviço) tem de GRUDAR: só sai com um start explícito.
        # Pausa por condição (joystick segurado, alvo sumiu, sem scan) volta sozinha
        # quando a condição some — é isso que se espera de um bypass de operador.
        self.paused_by_command = False
        self.K = self.D = None
        self.target = None            # (x, y, dist, bearing) em base_link
        self.target_time = 0.0
        self.scan = None
        self.scan_time = 0.0
        self.joy_time = 0.0
        self.joy_enable_held = False
        self.prev_buttons = []
        self.frames = self.detections = 0
        self.detect_period = 1.0 / max(1e-3, float(g('detect_rate')))
        self.last_detect = 0.0

        dict_name = str(g('marker_dict'))
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, params)
        # cantos do marcador no próprio referencial, para o solvePnP
        h = self.marker_size / 2.0
        self.obj_pts = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float32)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, str(g('image_topic')), self.on_image, sensor_qos)
        self.create_subscription(CameraInfo, str(g('camera_info_topic')), self.on_info, sensor_qos)
        self.create_subscription(LaserScan, str(g('scan_topic')), self.on_scan, sensor_qos)
        self.create_subscription(Joy, 'joy_teleop/joy', self.on_joy, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.debug_pub = self.create_publisher(Image, '~/debug_image', 1)
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/pause', self.srv_pause)
        self.create_service(Trigger, '~/stop', self.srv_stop)
        self.create_timer(1.0 / float(g('control_rate')), self.control)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info(
            'follow-me pronto: %s, id %s, marcador de %.0f cm | distância alvo %.1f m | %s'
            % (dict_name, 'qualquer' if self.marker_id < 0 else self.marker_id,
               self.marker_size * 100, self.target_d, 'aguardando Options no joystick'))

    # ------------------------------------------------------------------ entradas
    def now(self):
        return time.monotonic()

    def on_info(self, msg: CameraInfo):
        if self.K is None and msg.k[0] > 0:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('calibração recebida: fx=%.1f cx=%.1f' % (self.K[0, 0], self.K[0, 2]))

    @staticmethod
    def image_to_numpy(msg: Image):
        """Converte sensor_msgs/Image para numpy sem o cv_bridge.

        O cv_bridge desta distribuição é compilado contra o OpenCV 4.6 do sistema; carregá-lo
        junto com o OpenCV 4.10 do pip no mesmo processo quebra a ABI e derruba o nó com
        falha de segmentação. A conversão manual evita isso e ainda é mais rápida.
        """
        if msg.encoding not in ('bgr8', 'rgb8'):
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if buf.size < expected:
            return None
        img = buf[:expected].reshape(msg.height, msg.step)[:, :msg.width * 3]
        img = img.reshape(msg.height, msg.width, 3)
        return img[:, :, ::-1] if msg.encoding == 'rgb8' else img

    def on_image(self, msg: Image):
        if self.K is None:
            return
        now = self.now()
        if now - self.last_detect < self.detect_period:
            return
        self.last_detect = now
        img = self.image_to_numpy(msg)
        if img is None:
            return
        img = np.ascontiguousarray(img)
        self.frames += 1
        corners, ids, _ = self.detector.detectMarkers(img)
        if ids is None or len(ids) == 0:
            return
        best = None
        for c, i in zip(corners, ids.flatten()):
            if self.marker_id >= 0 and int(i) != self.marker_id:
                continue
            ok, rvec, tvec = cv2.solvePnP(self.obj_pts, c.reshape(4, 2).astype(np.float32),
                                          self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            t = tvec.ravel()
            # óptico (x direita, y baixo, z frente) -> base_link (x frente, y esquerda)
            xc, yc = float(t[2]), float(-t[0])
            cy, sy = math.cos(self.cam_yaw), math.sin(self.cam_yaw)
            x = self.cam_xyz[0] + xc * cy - yc * sy
            y = self.cam_xyz[1] + xc * sy + yc * cy
            d = math.hypot(x, y)
            if best is None or d < best[2]:
                best = (x, y, d, math.atan2(y, x), int(i))
        if best is None:
            return
        self.detections += 1
        self.target = best
        self.target_time = self.now()
        if self.debug_pub.get_subscription_count() > 0:
            cv2.aruco.drawDetectedMarkers(img, corners, ids)
            cv2.putText(img, 'id %d  d=%.2f m  ang=%+.0f deg' % (best[4], best[2], math.degrees(best[3])),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            out = Image()
            out.header = msg.header
            out.height, out.width = img.shape[0], img.shape[1]
            out.encoding = 'bgr8'
            out.is_bigendian = 0
            out.step = img.shape[1] * 3
            out.data = img.tobytes()
            self.debug_pub.publish(out)

    def on_scan(self, msg: LaserScan):
        self.scan = msg
        self.scan_time = self.now()

    def on_joy(self, msg: Joy):
        self.joy_time = self.now()
        b = list(msg.buttons)
        self.joy_enable_held = any(i < len(b) and b[i] == 1 for i in self.btn_enable)
        pressed = lambda i: (i < len(b) and b[i] == 1  # noqa: E731
                             and (i >= len(self.prev_buttons) or self.prev_buttons[i] == 0))
        if pressed(self.btn_stop):
            self.stop('botão do joystick')
        elif pressed(self.btn_start):
            self.start('botão do joystick')
        self.prev_buttons = b

    # ------------------------------------------------------------------ segurança
    def obstacle_distance(self):
        """Menor distância no setor frontal, ignorando o setor onde o alvo está."""
        if self.scan is None:
            return math.inf
        s = self.scan
        best = math.inf
        ang = s.angle_min
        t_ang = t_rng = None
        if self.target is not None and self.now() - self.target_time < self.target_timeout:
            t_ang, t_rng = self.target[3], self.target[2]
            # setor que cobre a largura da pessoa na distância em que ela está
            half = max(self.mask, math.atan2(self.target_w / 2.0, max(0.3, t_rng)))
        for r in s.ranges:
            a = wrap(ang)
            ang += s.angle_increment
            if not (s.range_min <= r <= s.range_max) or not math.isfinite(r):
                continue
            if abs(a) > self.obs_half:
                continue
            if t_ang is not None and abs(wrap(a - t_ang)) < half and abs(r - t_rng) < self.mask_range:
                continue          # é a própria pessoa que estamos seguindo
            best = min(best, r)
        return best

    def health(self):
        t = self.now()
        if self.require_joy and t - self.joy_time > self.joy_timeout:
            return 'joystick ausente'
        if self.joy_enable_held:
            return 'teleop do joystick ativo'
        if self.K is None:
            return 'sem calibração da câmera'
        if t - self.scan_time > self.scan_timeout:
            return 'sem scan do lidar'
        return None

    # ------------------------------------------------------------------ controle
    def control(self):
        if self.state in (IDLE, ABORTED):
            return
        if self.state == PAUSED and self.paused_by_command:
            self.send(0.0, 0.0)
            return
        problem = self.health()
        if problem:
            self.pause(problem)
            return
        if self.target is None or self.now() - self.target_time > self.target_timeout:
            if self.state != LOST:
                self.get_logger().warn('alvo perdido')
            self.state, self.reason = LOST, 'marcador não visível'
            self.send(0.0, 0.0)
            return

        x, y, dist, bearing = self.target[0], self.target[1], self.target[2], self.target[3]
        obs = self.obstacle_distance()
        if obs < self.obs_stop:
            self.state, self.reason = BLOCKED, 'obstáculo a %.2f m' % obs
            self.send(0.0, 0.0)
            return

        err_d = dist - self.target_d
        w = 0.0 if abs(bearing) < self.dead_b else max(-self.max_ang, min(self.max_ang, self.k_ang * bearing))
        v = 0.0
        if abs(err_d) > self.dead_d and abs(bearing) < math.radians(60.0):
            v = max(-self.max_lin, min(self.max_lin, self.k_lin * err_d))
            if v > 0:
                v = max(v, self.min_lin)
            elif v < 0:
                v = min(v, -self.min_lin) if self.allow_reverse else 0.0
        self.state, self.reason = FOLLOWING, ''
        self.send(v, w)

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.cmd_pub.publish(m)

    # ------------------------------------------------------------------ comandos
    def start(self, why):
        self.paused_by_command = False
        self.state, self.reason = FOLLOWING, why
        self.get_logger().info('follow-me START (%s)' % why)
        return True

    def pause(self, why):
        if self.state in (FOLLOWING, LOST, BLOCKED):
            self.state, self.reason = PAUSED, why
            self.get_logger().warn('follow-me PAUSADO: %s' % why)
        self.send(0.0, 0.0)

    def stop(self, why):
        if self.state not in (IDLE, ABORTED):
            self.get_logger().warn('follow-me ABORTADO: %s' % why)
        self.state, self.reason = ABORTED, why
        self.send(0.0, 0.0)

    def srv_start(self, req, res):
        res.success, res.message = self.start('serviço'), self.state
        return res

    def srv_pause(self, req, res):
        self.pause('serviço')
        self.paused_by_command = True      # só um start explícito tira desta pausa
        self.state = PAUSED                # mesmo que a pausa venha de IDLE
        res.success, res.message = True, self.state
        return res

    def srv_stop(self, req, res):
        self.stop('serviço'); res.success, res.message = True, self.state
        return res

    def publish_status(self):
        t = self.target
        fresh = t is not None and self.now() - self.target_time < self.target_timeout
        obs = self.obstacle_distance()
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': self.reason,
            'target_visible': bool(fresh),
            'marker_id': t[4] if t else None,
            'distance_m': round(t[2], 2) if t else None,
            'bearing_deg': round(math.degrees(t[3]), 1) if t else None,
            'target_distance': self.target_d,
            'obstacle_m': round(obs, 2) if math.isfinite(obs) else None,
            'frames': self.frames, 'detections': self.detections,
            'joy_ok': self.now() - self.joy_time <= self.joy_timeout,
            'paused_by_command': self.paused_by_command,
            'health': self.health(),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = FollowMe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
