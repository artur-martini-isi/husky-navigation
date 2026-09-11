#!/usr/bin/env python3
"""goto_point — navega até marcadores colocados no mapa.

Serve para o caso mais direto do uso indoor: o operador olha o mapa do SLAM no Foxglove, clica
num lugar e o robô vai até lá. É o mesmo miolo da exploração autônoma (A* sobre o mapa +
perseguição de ponto à frente + parada pelo scan), só que **quem escolhe o destino é a pessoa**,
não o algoritmo de fronteiras.

    Foxglove (painel 3D, ferramenta de publicar) ──► goal_pose / clicked_point
    ~/set_goal (String JSON, linha de comando / interface web) ──┤
                                                   ▼
    map (OccupancyGrid do SLAM) ──► A* ──► caminho ──► perseguição ──► cmd_vel
    scan (MID360)               ──► parada de emergência ─────────────┘
    joy_teleop/joy              ──► segurança (círculo aborta, L1/R1 devolve o controle)

Cada marcador recebido entra numa **fila**, então clicar em três lugares cria uma rota de três
waypoints, visitada na ordem. `goal_mode: replace` troca esse comportamento por "vá para o
último clique e esqueça o resto".

O destino pode cair em cima de uma parede ou dentro da margem de segurança (é um clique, não uma
medição): nesse caso o planejador desliza para a célula livre mais próxima em vez de recusar.

Tópicos publicados: `~/path` (caminho atual), `~/markers` (fila, numerada, para o painel 3D),
`~/status` (JSON com estado, fila e saúde).
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from agrobot_husky_nav.grid_planner import GridPlanner
from agrobot_husky_nav.path_follow import PurePursuit

IDLE, RUNNING, BLOCKED, PAUSED, ARRIVED, ABORTED = (
    'IDLE', 'RUNNING', 'BLOCKED', 'PAUSED', 'ARRIVED', 'ABORTED')


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GotoPoint(Node):
    def __init__(self):
        super().__init__('goto_point')
        p = self.declare_parameter
        p('map_topic', 'map'); p('scan_topic', 'scan')
        p('map_frame', 'map'); p('base_frame', 'base_link')
        p('goal_pose_topic', 'goal_pose')        # geometry_msgs/PoseStamped (Foxglove: "Publish pose")
        p('clicked_point_topic', 'clicked_point')  # geometry_msgs/PointStamped (Foxglove: "Publish point")
        # nomes absolutos, fora do namespace: o painel do Foxglove costuma publicar na raiz
        p('extra_goal_topics', ['/move_base_simple/goal', '/goal_pose'])
        p('extra_point_topics', ['/clicked_point'])
        p('goal_mode', 'append')                 # append = fila de waypoints | replace = só o último
        # planejamento
        p('robot_radius', 0.45)
        p('clearance_margin', 0.35)
        p('plan_resolution', 0.10)
        p('allow_unknown', False)                # indoor com mapa pronto: não atravessa o desconhecido
        p('unknown_penalty', 4.0)
        p('replan_period', 2.0)
        p('plan_fail_limit', 5)                  # tentativas antes de desistir do marcador
        p('smooth_path', True)                   # tira a escada de 45 graus do A*
        p('smooth_iterations', 2)
        p('sample_step', 0.15)
        # chegada
        p('goal_tolerance', 0.35)
        p('align_final_yaw', True)               # gira no lugar para o rumo do marcador, se houver
        p('yaw_tolerance', 0.15)
        p('goal_timeout', 180.0)
        p('hold_time', 1.0)                      # parado sobre o waypoint antes de seguir
        # seguimento (pure pursuit por curvatura, ver path_follow.py)
        p('lookahead_min', 0.5)                  # ponto perseguido parado
        p('lookahead_gain', 1.2)                 # ... e quanto ele se afasta por m/s
        p('lookahead_max', 1.2)
        p('max_linear', 0.3)
        p('min_linear', 0.08)
        p('max_angular', 0.7)
        p('k_angular', 1.3)
        p('turn_in_place_angle', 0.7)
        p('turn_resume_angle', 0.25)             # histerese: entra em 0,7 rad, sai em 0,25
        p('max_linear_accel', 0.4)               # rampa: degrau de velocidade vira solavanco
        p('max_angular_accel', 1.5)
        p('curvature_slowdown', 1.2)             # curva fechada, mais devagar
        p('control_rate', 10.0)
        # segurança
        p('obstacle_stop_distance', 0.6)
        p('obstacle_slow_distance', 1.5)
        p('obstacle_half_angle_deg', 35.0)
        p('blocked_timeout', 20.0)               # tempo parado contra obstáculo antes de replanejar
        p('progress_distance', 0.3)
        p('progress_time', 20.0)
        p('scan_timeout', 1.5); p('map_timeout', 15.0)
        p('require_joy', True); p('joy_timeout', 1.0)
        p('auto_start', True)                    # marcador novo já manda andar
        p('joy_start_button', 9); p('joy_stop_button', 1)
        p('joy_enable_buttons', [4, 5])
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.map_frame, self.base_frame = str(g('map_frame')), str(g('base_frame'))
        self.modo = str(g('goal_mode')).lower()
        self.planner = GridPlanner(robot_radius=float(g('robot_radius')),
                                   clearance_margin=float(g('clearance_margin')),
                                   plan_resolution=float(g('plan_resolution')),
                                   allow_unknown=bool(g('allow_unknown')),
                                   unknown_penalty=float(g('unknown_penalty')),
                                   smooth=bool(g('smooth_path')),
                                   smooth_iterations=int(g('smooth_iterations')),
                                   sample_step=float(g('sample_step')))
        self.follower = PurePursuit(
            max_linear=float(g('max_linear')), min_linear=float(g('min_linear')),
            max_angular=float(g('max_angular')), k_angular=float(g('k_angular')),
            lookahead_min=float(g('lookahead_min')), lookahead_gain=float(g('lookahead_gain')),
            lookahead_max=float(g('lookahead_max')),
            turn_in_place_angle=float(g('turn_in_place_angle')),
            turn_resume_angle=float(g('turn_resume_angle')),
            max_linear_accel=float(g('max_linear_accel')),
            max_angular_accel=float(g('max_angular_accel')),
            curvature_slowdown=float(g('curvature_slowdown')),
            control_rate=float(g('control_rate')))
        self.replan_period = float(g('replan_period'))
        self.plan_fail_limit = int(g('plan_fail_limit'))
        self.goal_tol = float(g('goal_tolerance'))
        self.align_yaw, self.yaw_tol = bool(g('align_final_yaw')), float(g('yaw_tolerance'))
        self.goal_timeout, self.hold_time = float(g('goal_timeout')), float(g('hold_time'))
        self.max_lin, self.min_lin = float(g('max_linear')), float(g('min_linear'))
        self.max_ang, self.k_ang = float(g('max_angular')), float(g('k_angular'))
        self.obs_stop, self.obs_slow = float(g('obstacle_stop_distance')), float(g('obstacle_slow_distance'))
        self.obs_half = math.radians(float(g('obstacle_half_angle_deg')))
        self.blocked_timeout = float(g('blocked_timeout'))
        self.prog_d, self.prog_t = float(g('progress_distance')), float(g('progress_time'))
        self.scan_timeout, self.map_timeout = float(g('scan_timeout')), float(g('map_timeout'))
        self.require_joy, self.joy_timeout = bool(g('require_joy')), float(g('joy_timeout'))
        self.auto_start = bool(g('auto_start'))
        self.btn_start, self.btn_stop = int(g('joy_start_button')), int(g('joy_stop_button'))
        self.btn_enable = {int(b) for b in g('joy_enable_buttons')}

        self.state, self.reason = IDLE, 'sem marcador'
        self.paused_by_command = False
        self.map = self.scan = None
        self.map_time = self.scan_time = self.joy_time = 0.0
        self.joy_enable_held = False
        self.prev_buttons = []
        self.fila = []                     # [(x, y, yaw|None, rótulo)]
        self.path = []
        self.goal_time = self.last_plan = self.blocked_since = 0.0
        self.hold_until = 0.0
        self.progress_ref = None
        self.plan_fails = 0
        self.alinhando = False
        self.recebidos = self.alcancados = self.descartados = 0
        self.ultimo_erro = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, str(g('map_topic')), self.on_map, map_qos)
        self.create_subscription(LaserScan, str(g('scan_topic')), self.on_scan, sensor_qos)
        self.create_subscription(Joy, 'joy_teleop/joy', self.on_joy, 10)
        self.create_subscription(PoseStamped, str(g('goal_pose_topic')), self.on_goal_pose, 10)
        self.create_subscription(PointStamped, str(g('clicked_point_topic')), self.on_clicked, 10)
        for t in list(g('extra_goal_topics')):
            self.create_subscription(PoseStamped, str(t), self.on_goal_pose, 10)
        for t in list(g('extra_point_topics')):
            self.create_subscription(PointStamped, str(t), self.on_clicked, 10)
        self.create_subscription(String, '~/set_goal', self.on_set_goal, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.path_pub = self.create_publisher(Path, '~/path', 1)
        self.mark_pub = self.create_publisher(MarkerArray, '~/markers', 1)
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/pause', self.srv_pause)
        self.create_service(Trigger, '~/stop', self.srv_stop)
        self.create_service(Trigger, '~/clear', self.srv_clear)
        self.create_service(Trigger, '~/skip', self.srv_skip)
        self.create_timer(1.0 / float(g('control_rate')), self.control)
        self.create_timer(0.5, self.publish_status)
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(
            'ir-até-marcador pronto: modo %s, tolerância %.2f m, vel. máx %.2f m/s | '
            'clique em %s (PoseStamped) ou %s (PointStamped)'
            % (self.modo, self.goal_tol, self.max_lin,
               str(g('goal_pose_topic')), str(g('clicked_point_topic'))))

    # ------------------------------------------------------------------ entradas
    def now(self):
        return time.monotonic()

    def on_map(self, msg):
        self.map, self.map_time = msg, self.now()

    def on_scan(self, msg):
        self.scan, self.scan_time = msg, self.now()

    def on_joy(self, msg):
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

    def robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception:  # noqa: BLE001
            return None
        tr = t.transform
        return (tr.translation.x, tr.translation.y, yaw_de(tr.rotation))

    def para_o_mapa(self, frame, x, y, yaw=None):
        """Converte um clique feito em qualquer frame para o frame do mapa."""
        frame = (frame or self.map_frame).lstrip('/')
        if frame in ('', self.map_frame.lstrip('/')):
            return x, y, yaw
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, frame, rclpy.time.Time(),
                                                timeout=Duration(seconds=0.3)).transform
        except Exception as e:  # noqa: BLE001
            self.ultimo_erro = 'sem TF %s -> %s: %s' % (self.map_frame, frame, e)
            self.get_logger().warn(self.ultimo_erro)
            return None
        th = yaw_de(t.rotation)
        return (t.translation.x + x * math.cos(th) - y * math.sin(th),
                t.translation.y + x * math.sin(th) + y * math.cos(th),
                None if yaw is None else wrap(yaw + th))

    # ------------------------------------------------------------------ marcadores
    def add_goal(self, x, y, yaw, origem):
        self.recebidos += 1
        alvo = (float(x), float(y), yaw, origem)
        if self.modo == 'replace':
            self.fila = [alvo]
        else:
            self.fila.append(alvo)
        self.path, self.progress_ref, self.plan_fails, self.alinhando = [], None, 0, False
        self.goal_time = self.last_plan = self.now()
        self.hold_until = 0.0
        if self.state in (IDLE, ARRIVED, ABORTED) and self.auto_start:
            self.state, self.reason, self.paused_by_command = RUNNING, 'marcador recebido', False
        self.get_logger().info('marcador %d em (%.2f, %.2f)%s via %s | fila: %d'
                               % (self.recebidos, x, y,
                                  '' if yaw is None else ' rumo %.0f°' % math.degrees(yaw),
                                  origem, len(self.fila)))
        self.publish_markers()

    def on_goal_pose(self, msg):
        r = self.para_o_mapa(msg.header.frame_id, msg.pose.position.x, msg.pose.position.y,
                             yaw_de(msg.pose.orientation))
        if r:
            self.add_goal(r[0], r[1], r[2], 'goal_pose')

    def on_clicked(self, msg):
        r = self.para_o_mapa(msg.header.frame_id, msg.point.x, msg.point.y, None)
        if r:
            self.add_goal(r[0], r[1], None, 'clicked_point')

    def on_set_goal(self, msg):
        """JSON da interface web: {"x":1.0,"y":2.0} ou {"x":..,"y":..,"yaw_deg":90,"mode":"replace"}."""
        try:
            d = json.loads(msg.data)
        except Exception as e:  # noqa: BLE001
            self.ultimo_erro = 'set_goal inválido: %s' % e
            self.get_logger().error(self.ultimo_erro)
            return
        pontos = d if isinstance(d, list) else [d]
        if str(pontos[0].get('mode', '')).lower() == 'replace':
            self.fila = []
        for q in pontos:
            yaw = q.get('yaw')
            if yaw is None and q.get('yaw_deg') is not None:
                yaw = math.radians(float(q['yaw_deg']))
            r = self.para_o_mapa(q.get('frame', self.map_frame), float(q['x']), float(q['y']),
                                 None if yaw is None else float(yaw))
            if r:
                self.add_goal(r[0], r[1], r[2], 'set_goal')

    # ------------------------------------------------------------------ saídas
    def publish_path(self):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def publish_markers(self):
        arr = MarkerArray()
        agora = self.get_clock().now().to_msg()
        limpa = Marker()
        limpa.header.frame_id, limpa.header.stamp = self.map_frame, agora
        limpa.ns, limpa.action = 'goto', Marker.DELETEALL
        arr.markers.append(limpa)
        for i, (x, y, yaw, _) in enumerate(self.fila):
            cor = ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9) if i == 0 else ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.7)
            m = Marker()
            m.header.frame_id, m.header.stamp = self.map_frame, agora
            m.ns, m.id, m.type, m.action = 'goto', i * 2, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 2.0 * self.goal_tol
            m.scale.z = 0.1
            m.color = cor
            arr.markers.append(m)
            txt = Marker()
            txt.header.frame_id, txt.header.stamp = self.map_frame, agora
            txt.ns, txt.id, txt.type, txt.action = 'goto', i * 2 + 1, Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = float(x), float(y), 0.6
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.35
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            txt.text = str(i + 1)
            arr.markers.append(txt)
        self.mark_pub.publish(arr)

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    # ------------------------------------------------------------------ segurança
    def front_distance(self):
        s = self.scan
        if s is None:
            return math.inf
        ranges = np.asarray(s.ranges, dtype=np.float32)
        ang = s.angle_min + np.arange(len(ranges), dtype=np.float32) * s.angle_increment
        ok = np.isfinite(ranges) & (ranges >= s.range_min) & (ranges <= s.range_max)
        a = (ang[ok] + math.pi) % (2 * math.pi) - math.pi
        r = ranges[ok]
        frente = np.abs(a) <= self.obs_half
        return float(r[frente].min()) if frente.any() else math.inf

    def health(self):
        t = self.now()
        if self.require_joy and t - self.joy_time > self.joy_timeout:
            return 'joystick ausente'
        if self.joy_enable_held:
            return 'teleop do joystick ativo'
        if t - self.scan_time > self.scan_timeout:
            return 'sem scan do lidar'
        if self.map is None or t - self.map_time > self.map_timeout:
            return 'sem mapa do SLAM'
        if self.robot_pose() is None:
            return 'sem TF %s -> %s' % (self.map_frame, self.base_frame)
        return None

    # ------------------------------------------------------------------ controle
    def descarta_atual(self, porque):
        x, y = self.fila[0][0], self.fila[0][1]
        self.fila.pop(0)
        self.descartados += 1
        self.path, self.progress_ref, self.plan_fails, self.alinhando = [], None, 0, False
        self.goal_time = self.now()
        self.ultimo_erro = '%s: marcador (%.2f, %.2f) descartado' % (porque, x, y)
        self.get_logger().warn(self.ultimo_erro)
        self.follower.reset()
        self.send(0.0, 0.0)
        self.publish_markers()

    def chegou(self):
        x, y = self.fila[0][0], self.fila[0][1]
        self.fila.pop(0)
        self.alcancados += 1
        self.path, self.progress_ref, self.plan_fails, self.alinhando = [], None, 0, False
        self.goal_time = self.now()
        self.hold_until = self.now() + self.hold_time
        self.follower.reset()
        self.send(0.0, 0.0)
        self.publish_markers()
        if self.fila:
            self.get_logger().info('marcador (%.2f, %.2f) alcançado | faltam %d' % (x, y, len(self.fila)))
        else:
            self.state, self.reason = ARRIVED, 'fila vazia'
            self.get_logger().info('marcador (%.2f, %.2f) alcançado | %d no total, fila vazia'
                                   % (x, y, self.alcancados))

    def control(self):
        if self.state in (IDLE, ABORTED, ARRIVED):
            return
        if self.state == PAUSED and self.paused_by_command:
            self.send(0.0, 0.0)
            return
        if not self.fila:
            self.state, self.reason = ARRIVED, 'fila vazia'
            self.send(0.0, 0.0)
            return
        problem = self.health()
        if problem:
            self.pause(problem)
            return
        t = self.now()
        if t < self.hold_until:
            self.send(0.0, 0.0)
            return
        pose = self.robot_pose()
        gx, gy, gyaw, _ = self.fila[0]
        dist = math.hypot(gx - pose[0], gy - pose[1])

        if t - self.goal_time > self.goal_timeout:
            self.descarta_atual('tempo esgotado')
            return

        # chegada: primeiro a posição, depois (opcional) o rumo pedido no marcador
        if dist < self.goal_tol or self.alinhando:
            if self.align_yaw and gyaw is not None:
                erro_yaw = wrap(gyaw - pose[2])
                if abs(erro_yaw) > self.yaw_tol:
                    self.alinhando = True
                    self.state, self.reason = RUNNING, 'alinhando rumo'
                    self.send(0.0, max(-self.max_ang, min(self.max_ang, self.k_ang * erro_yaw)))
                    return
            self.chegou()
            return

        # progresso: precisa **aproximar-se do marcador**, não só se mexer
        if self.progress_ref is None:
            self.progress_ref = (dist, t)
        else:
            d0, t0 = self.progress_ref
            if t - t0 > self.prog_t:
                if d0 - dist < self.prog_d:
                    self.descarta_atual('sem progresso')
                    return
                self.progress_ref = (dist, t)

        if t - self.last_plan > self.replan_period or not self.path:
            self.last_plan = t
            novo = self.planner.plan(self.map, pose, (gx, gy))
            if novo and len(novo) > 1:
                self.path, self.plan_fails = novo, 0
                self.publish_path()
            else:
                self.plan_fails += 1
                self.reason = 'sem caminho até o marcador (%d)' % self.plan_fails
                if self.plan_fails >= self.plan_fail_limit:
                    self.descarta_atual('sem caminho possível')
                    return
                if not self.path:
                    self.send(0.0, 0.0)
                    return

        alvo, self.path = self.follower.ponto(pose, self.path)
        if alvo is None:
            self.path = []
            return
        d_front = self.front_distance()

        if d_front < self.obs_stop:
            # obstáculo que o mapa ainda não tinha: para, gira para o lado do caminho e replaneja
            if self.state != BLOCKED:
                self.blocked_since = t
            self.state, self.reason = BLOCKED, 'obstáculo a %.2f m' % d_front
            if t - self.blocked_since > self.blocked_timeout:
                self.descarta_atual('bloqueado por %.0f s' % (t - self.blocked_since))
                return
            self.last_plan = 0.0
            erro = wrap(math.atan2(alvo[1] - pose[1], alvo[0] - pose[0]) - pose[2])
            self.send(*self.follower.rampa(0.0, math.copysign(self.max_ang * 0.5,
                                                              erro if erro else 1.0)))
            return

        # o scan só modula a velocidade; a geometria da curva é do seguidor
        escala = (1.0 if d_front >= self.obs_slow
                  else max(0.0, (d_front - self.obs_stop) / (self.obs_slow - self.obs_stop)))
        v, w, self.path = self.follower.comando(pose, self.path, dist_goal=dist, escala=escala)
        self.state, self.reason = RUNNING, ''
        self.send(v, w)

    # ------------------------------------------------------------------ comandos
    def start(self, why):
        self.paused_by_command = False
        if not self.fila:
            self.state, self.reason = IDLE, 'sem marcador'
            return False
        self.state, self.reason = RUNNING, why
        self.goal_time, self.last_plan = self.now(), 0.0
        self.progress_ref = None
        self.get_logger().info('ir-até-marcador START (%s) | fila: %d' % (why, len(self.fila)))
        return True

    def pause(self, why):
        if self.state in (RUNNING, BLOCKED):
            self.state, self.reason = PAUSED, why
            self.get_logger().warn('ir-até-marcador PAUSADO: %s' % why)
        self.follower.reset()
        self.send(0.0, 0.0)

    def stop(self, why):
        if self.state not in (IDLE, ABORTED):
            self.get_logger().warn('ir-até-marcador ABORTADO: %s' % why)
        self.state, self.reason = ABORTED, why
        self.path = []
        self.follower.reset()
        self.send(0.0, 0.0)

    def srv_start(self, req, res):
        res.success, res.message = self.start('serviço'), self.state
        return res

    def srv_pause(self, req, res):
        self.pause('serviço'); self.paused_by_command = True; self.state = PAUSED
        res.success, res.message = True, self.state
        return res

    def srv_stop(self, req, res):
        self.stop('serviço'); res.success, res.message = True, self.state
        return res

    def srv_clear(self, req, res):
        n = len(self.fila)
        self.fila, self.path = [], []
        self.state, self.reason = IDLE, 'fila limpa'
        self.send(0.0, 0.0)
        self.publish_markers()
        self.publish_path()
        res.success, res.message = True, '%d marcadores removidos' % n
        self.get_logger().info(res.message)
        return res

    def srv_skip(self, req, res):
        if not self.fila:
            res.success, res.message = False, 'fila vazia'
            return res
        self.descarta_atual('pulado pelo operador')
        res.success, res.message = True, '%d restantes' % len(self.fila)
        return res

    def publish_status(self):
        pose = self.robot_pose()
        d_front = self.front_distance()
        alvo = self.fila[0] if self.fila else None
        dist = math.hypot(alvo[0] - pose[0], alvo[1] - pose[1]) if (alvo and pose) else None
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': self.reason,
            'goal': [round(alvo[0], 2), round(alvo[1], 2)] if alvo else None,
            'goal_yaw_deg': (round(math.degrees(alvo[2]), 1) if alvo and alvo[2] is not None else None),
            'goal_distance_m': round(dist, 2) if dist is not None else None,
            'queue': len(self.fila),
            'queue_points': [[round(q[0], 2), round(q[1], 2)] for q in self.fila[:10]],
            'path_points': len(self.path),
            'lookahead_m': round(self.follower.ld, 2),
            'cmd': [round(self.follower.v, 2), round(self.follower.w, 2)],
            'turning_in_place': self.follower.girando,
            'path_length_m': round(GridPlanner.comprimento(self.path), 1) if len(self.path) > 1 else 0.0,
            'received': self.recebidos, 'reached': self.alcancados, 'dropped': self.descartados,
            'plan_fails': self.plan_fails,
            'obstacle_m': round(d_front, 2) if math.isfinite(d_front) else None,
            'pose': [round(pose[0], 2), round(pose[1], 2), round(math.degrees(pose[2]), 1)] if pose else None,
            'joy_ok': self.now() - self.joy_time <= self.joy_timeout,
            'paused_by_command': self.paused_by_command,
            'last_error': self.ultimo_erro,
            'health': self.health(),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = GotoPoint()
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
