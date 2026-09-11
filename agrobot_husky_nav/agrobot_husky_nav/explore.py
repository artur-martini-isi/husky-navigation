#!/usr/bin/env python3
"""explore — exploração autônoma por fronteiras, com planejamento de caminho.

O `slam_toolbox` diz o que já é conhecido; este nó decide **para onde ir para conhecer mais**,
**planeja um caminho** até lá sobre o próprio mapa e segue esse caminho. O mesmo MID360 serve às
duas pontas: a planta escolhe o destino e o caminho, e o `scan` (10 Hz) é a última barreira contra
o que o mapa ainda não sabe.

    map (OccupancyGrid do SLAM) ──► fronteiras ──► destino
                                 └► grade de custo ──► A* ──► caminho
    TF map->base_link            ──►                            │ perseguição de ponto à frente
    scan (MID360)                ──► parada de emergência ──────┼──► cmd_vel
    joy_teleop/joy               ──► segurança ─────────────────┘

**Por que planejar**: a primeira versão era só reativa (rumo ao destino somado à repulsão do scan) e
ficou presa contra uma parede, oscilando, porque repulsão local não contorna obstáculo côncavo.
O A* sobre a grade resolve: ele já entrega um caminho que passa longe das paredes.

**Fronteira** é célula livre encostada em desconhecido. Destinos sem caminho possível entram numa
lista negra por um tempo, para não insistir no impossível.
"""
import json
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from agrobot_husky_nav.grid_planner import GridPlanner

IDLE, EXPLORING, BLOCKED, PAUSED, DONE, ABORTED = 'IDLE', 'EXPLORING', 'BLOCKED', 'PAUSED', 'DONE', 'ABORTED'


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Explore(Node):
    def __init__(self):
        super().__init__('explore')
        p = self.declare_parameter
        p('map_topic', 'map'); p('scan_topic', 'scan')
        p('map_frame', 'map'); p('base_frame', 'base_link')
        # fronteiras e planejamento
        p('robot_radius', 0.45)            # raio rígido: nada de caminho mais perto que isso
        p('clearance_margin', 0.35)        # folga extra preferida (custo, não proibição)
        p('plan_resolution', 0.10)         # planeja numa grade mais grossa que a do mapa
        p('min_frontier_cells', 8)
        p('goal_tolerance', 0.7)
        p('goal_timeout', 60.0)
        p('replan_period', 2.0)
        p('max_goal_distance', 15.0)
        p('blacklist_time', 90.0)          # quanto tempo um destino inalcançável fica descartado
        p('max_candidates', 6)             # fronteiras planejadas antes de escolher
        p('allow_unknown', True)           # permite caminho por área desconhecida (com penalidade)
        p('unknown_penalty', 2.5)
        # seguimento de caminho
        p('lookahead', 0.8)                # ponto perseguido à frente, no caminho
        p('max_linear', 0.3)
        p('min_linear', 0.08)
        p('max_angular', 0.7)
        p('k_angular', 1.3)
        p('turn_in_place_angle', 0.7)
        p('control_rate', 10.0)
        # segurança
        p('obstacle_stop_distance', 0.6)
        p('obstacle_slow_distance', 1.5)
        p('obstacle_half_angle_deg', 35.0)
        p('progress_distance', 0.3)        # avanço mínimo rumo ao destino em progress_time
        p('progress_time', 15.0)
        p('scan_timeout', 1.5); p('map_timeout', 15.0)
        p('require_joy', True); p('joy_timeout', 1.0)
        p('joy_start_button', 9); p('joy_stop_button', 1)
        p('joy_enable_buttons', [4, 5])
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.map_frame, self.base_frame = str(g('map_frame')), str(g('base_frame'))
        self.radius, self.clearance = float(g('robot_radius')), float(g('clearance_margin'))
        self.plan_res = float(g('plan_resolution'))
        self.min_cells = int(g('min_frontier_cells'))
        self.goal_tol, self.goal_timeout = float(g('goal_tolerance')), float(g('goal_timeout'))
        self.replan_period = float(g('replan_period'))
        self.max_goal_d = float(g('max_goal_distance'))
        self.blacklist_time = float(g('blacklist_time'))
        self.max_candidates = int(g('max_candidates'))
        self.allow_unknown, self.unknown_pen = bool(g('allow_unknown')), float(g('unknown_penalty'))
        self.lookahead = float(g('lookahead'))
        self.max_lin, self.min_lin = float(g('max_linear')), float(g('min_linear'))
        self.max_ang, self.k_ang = float(g('max_angular')), float(g('k_angular'))
        self.turn_in_place = float(g('turn_in_place_angle'))
        self.obs_stop, self.obs_slow = float(g('obstacle_stop_distance')), float(g('obstacle_slow_distance'))
        self.obs_half = math.radians(float(g('obstacle_half_angle_deg')))
        self.prog_d, self.prog_t = float(g('progress_distance')), float(g('progress_time'))
        self.scan_timeout, self.map_timeout = float(g('scan_timeout')), float(g('map_timeout'))
        self.require_joy, self.joy_timeout = bool(g('require_joy')), float(g('joy_timeout'))
        self.btn_start, self.btn_stop = int(g('joy_start_button')), int(g('joy_stop_button'))
        self.btn_enable = {int(b) for b in g('joy_enable_buttons')}

        self.state, self.reason = IDLE, ''
        self.paused_by_command = False
        self.map = self.scan = None
        self.map_time = self.scan_time = self.joy_time = 0.0
        self.joy_enable_held = False
        self.prev_buttons = []
        self.goal = None
        self.goal_time = self.last_plan = 0.0
        self.path = []                     # [(x, y)] no frame do mapa
        self.blacklist = []                # [(x, y, t_expira)]
        self.progress_ref = None           # (distancia_ao_destino, t)
        self.goals_done = self.frontiers = self.plan_fails = 0
        self.explored_m2 = 0.0

        self.planner = GridPlanner(robot_radius=self.radius, clearance_margin=self.clearance,
                                   plan_resolution=self.plan_res, allow_unknown=self.allow_unknown,
                                   unknown_penalty=self.unknown_pen)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, str(g('map_topic')), self.on_map, map_qos)
        self.create_subscription(LaserScan, str(g('scan_topic')), self.on_scan, sensor_qos)
        self.create_subscription(Joy, 'joy_teleop/joy', self.on_joy, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.path_pub = self.create_publisher(Path, '~/path', 1)
        self.create_service(Trigger, '~/start', self.srv_start)
        self.create_service(Trigger, '~/pause', self.srv_pause)
        self.create_service(Trigger, '~/stop', self.srv_stop)
        self.create_timer(1.0 / float(g('control_rate')), self.control)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info('exploração com planejamento: raio %.2f m, grade de plano %.2f m, '
                               'vel. máx %.2f m/s | aguardando start'
                               % (self.radius, self.plan_res, self.max_lin))

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
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    # ------------------------------------------------------------------ grade de custo
    # O planejamento (grade de custo + A*) vive em grid_planner.py, compartilhado com o nó
    # goto_point: os dois precisam enxergar o mundo com o mesmo raio e a mesma folga, senão um
    # caminho aceitável para um seria proibido para o outro.

    # ------------------------------------------------------------------ fronteiras
    def find_goal(self, pose):
        m = self.map
        res = m.info.resolution
        grid = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        self.explored_m2 = float((grid >= 0).sum()) * res * res
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        occ = (grid == 100).astype(np.uint8)
        unk = (grid == -1).astype(np.uint8)
        free = (grid == 0).astype(np.uint8)
        r = max(1, int(round(self.radius / res)))
        forbidden = cv2.dilate(occ, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1,) * 2))
        near_unknown = cv2.dilate(unk, np.ones((3, 3), np.uint8))
        frontier = ((free > 0) & (forbidden == 0) & (near_unknown > 0)).astype(np.uint8)
        n, _, stats, cents = cv2.connectedComponentsWithStats(frontier, connectivity=8)
        t = self.now()
        self.blacklist = [b for b in self.blacklist if b[2] > t]
        cands = []
        self.frontiers = 0
        for i in range(1, n):
            size = stats[i, cv2.CC_STAT_AREA]
            if size < self.min_cells:
                continue
            self.frontiers += 1
            gx, gy = ox + (cents[i][0] + 0.5) * res, oy + (cents[i][1] + 0.5) * res
            d = math.hypot(gx - pose[0], gy - pose[1])
            if d > self.max_goal_d or d < self.goal_tol:
                continue
            if any(math.hypot(gx - bx, gy - by) < 1.0 for bx, by, _ in self.blacklist):
                continue
            cands.append((size / (1.0 + d * d), gx, gy))
        cands.sort(reverse=True)
        return cands                                  # melhores primeiro

    # ------------------------------------------------------------------ A*
    def plan(self, pose, goal):
        caminho = self.planner.plan(self.map, pose, goal)
        self.explored_m2 = self.planner.explored_m2
        return caminho

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
    comprimento = staticmethod(GridPlanner.comprimento)

    def novo_destino(self, pose):
        """Escolhe a fronteira pelo **caminho mais curto**, não pela distância em linha reta.

        Medido no laboratório: a fronteira a 9,3 m em linha reta exigia 45,7 m de caminho (estava
        atrás de uma parede), enquanto uma a 22,8 m precisava de 24,8 m. Ordenar por linha reta
        mandava o robô justamente para a pior escolha. Por isso aqui se planeja para as melhores
        candidatas e decide-se pelo comprimento real, ponderado pelo tamanho da fronteira.
        """
        cands = self.find_goal(pose)[:self.max_candidates]
        avaliadas = []
        for _, gx, gy in cands:
            caminho = self.plan(pose, (gx, gy))
            if caminho and len(caminho) > 1:
                L = self.comprimento(caminho)
                avaliadas.append((L, gx, gy, caminho))
            else:
                self.plan_fails += 1
                self.blacklist.append((gx, gy, self.now() + self.blacklist_time))
        if not avaliadas:
            return False
        avaliadas.sort(key=lambda a: a[0])          # caminho mais curto primeiro
        L, gx, gy, caminho = avaliadas[0]
        self.goal, self.path = (gx, gy), caminho
        self.goal_time = self.now()
        self.progress_ref = (math.hypot(gx - pose[0], gy - pose[1]), self.now())
        self.publish_path()
        self.get_logger().info(
            'destino (%.2f, %.2f): %.1f m de caminho (%.1f m em linha reta) | '
            '%d candidatas avaliadas, %d fronteiras'
            % (gx, gy, L, math.hypot(gx - pose[0], gy - pose[1]), len(avaliadas), self.frontiers))
        return True

    def ponto_perseguido(self, pose):
        """Ponto do caminho a `lookahead` à frente; descarta o que já ficou para trás."""
        if not self.path:
            return None
        d = [math.hypot(x - pose[0], y - pose[1]) for x, y in self.path]
        i0 = int(np.argmin(d))
        self.path = self.path[i0:]
        acc = 0.0
        for i in range(1, len(self.path)):
            acc += math.hypot(self.path[i][0] - self.path[i - 1][0],
                              self.path[i][1] - self.path[i - 1][1])
            if acc >= self.lookahead:
                return self.path[i]
        return self.path[-1]

    def control(self):
        if self.state in (IDLE, ABORTED, DONE):
            return
        if self.state == PAUSED and self.paused_by_command:
            self.send(0.0, 0.0)
            return
        problem = self.health()
        if problem:
            self.pause(problem)
            return
        pose, t = self.robot_pose(), self.now()

        if self.goal is None or t - self.goal_time > self.goal_timeout:
            if self.goal is not None:
                self.blacklist.append((self.goal[0], self.goal[1], t + self.blacklist_time))
                self.get_logger().warn('tempo esgotado no destino: descartado')
            self.goal = None
            if not self.novo_destino(pose):
                if self.frontiers == 0:
                    self.state, self.reason = DONE, 'nenhuma fronteira restante'
                    self.get_logger().info('exploração concluída: %.1f m2 em %d destinos'
                                           % (self.explored_m2, self.goals_done))
                else:
                    self.reason = 'nenhuma fronteira alcançável'
                self.send(0.0, 0.0)
                return

        dist = math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
        if dist < self.goal_tol:
            self.goals_done += 1
            self.get_logger().info('destino alcançado (%d) | %.1f m2 mapeados'
                                   % (self.goals_done, self.explored_m2))
            self.goal, self.path = None, []
            self.send(0.0, 0.0)
            return

        # progresso medido pela **aproximação do destino**, não por deslocamento qualquer
        if self.progress_ref is not None:
            d0, t0 = self.progress_ref
            if t - t0 > self.prog_t:
                if d0 - dist < self.prog_d:
                    self.get_logger().warn('sem progresso rumo ao destino: trocando')
                    self.blacklist.append((self.goal[0], self.goal[1], t + self.blacklist_time))
                    self.goal, self.path, self.progress_ref = None, [], None
                    self.send(0.0, 0.0)
                    return
                self.progress_ref = (dist, t)

        if t - self.last_plan > self.replan_period:
            self.last_plan = t
            novo = self.plan(pose, self.goal)
            if novo and len(novo) > 1:
                self.path = novo
                self.publish_path()
            elif not self.path:
                self.blacklist.append((self.goal[0], self.goal[1], t + self.blacklist_time))
                self.goal = None
                return

        alvo = self.ponto_perseguido(pose)
        if alvo is None:
            self.goal = None
            return
        erro = wrap(math.atan2(alvo[1] - pose[1], alvo[0] - pose[0]) - pose[2])
        d_front = self.front_distance()

        if d_front < self.obs_stop:
            # o mapa não conhecia: para e gira para o lado do caminho
            self.state, self.reason = BLOCKED, 'obstáculo a %.2f m' % d_front
            self.send(0.0, math.copysign(self.max_ang * 0.5, erro if erro else 1.0))
            return

        w = max(-self.max_ang, min(self.max_ang, self.k_ang * erro))
        if abs(erro) > self.turn_in_place:
            v = 0.0
        else:
            v = self.max_lin * max(0.0, 1.0 - abs(erro) / self.turn_in_place)
            if d_front < self.obs_slow:
                v *= max(0.0, (d_front - self.obs_stop) / (self.obs_slow - self.obs_stop))
            v = max(self.min_lin, min(self.max_lin, v)) if v > 0 else 0.0
        self.state, self.reason = EXPLORING, ''
        self.send(v, w)

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(m)

    # ------------------------------------------------------------------ comandos
    def start(self, why):
        self.paused_by_command = False
        self.state, self.reason = EXPLORING, why
        self.goal, self.path, self.blacklist = None, [], []
        self.get_logger().info('exploração START (%s)' % why)
        return True

    def pause(self, why):
        if self.state in (EXPLORING, BLOCKED):
            self.state, self.reason = PAUSED, why
            self.get_logger().warn('exploração PAUSADA: %s' % why)
        self.send(0.0, 0.0)

    def stop(self, why):
        if self.state not in (IDLE, ABORTED):
            self.get_logger().warn('exploração ABORTADA: %s' % why)
        self.state, self.reason = ABORTED, why
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

    def publish_status(self):
        pose = self.robot_pose()
        d_front = self.front_distance()
        dist = (math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])
                if (self.goal and pose) else None)
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state, 'reason': self.reason,
            'goal': [round(self.goal[0], 2), round(self.goal[1], 2)] if self.goal else None,
            'goal_distance_m': round(dist, 2) if dist is not None else None,
            'path_points': len(self.path),
            'frontiers': self.frontiers, 'goals_done': self.goals_done,
            'blacklisted': len(self.blacklist), 'plan_fails': self.plan_fails,
            'explored_m2': round(self.explored_m2, 1),
            'obstacle_m': round(d_front, 2) if math.isfinite(d_front) else None,
            'pose': [round(pose[0], 2), round(pose[1], 2)] if pose else None,
            'joy_ok': self.now() - self.joy_time <= self.joy_timeout,
            'paused_by_command': self.paused_by_command,
            'health': self.health(),
        })))


def main(args=None):
    rclpy.init(args=args)
    node = Explore()
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
