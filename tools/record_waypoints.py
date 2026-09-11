#!/usr/bin/env python3
"""record_waypoints.py — roda NO ROBÔ. Grava waypoints GNSS enquanto você dirige.

A pose vem da mesma fonte que o `gps_waypoint_follower` usa em `pose_source: dual_gnss`
(posição = ponto médio das duas antenas do Fixposition, rumo = linha GNSS1→GNSS2 + offset),
então o gravador funciona com ou sem a stack de navegação no ar.

Marcar um ponto (qualquer uma das formas):
  - botão do joystick  .............. qualquer botão de face (X, O, △, □ = índices 0..3 por padrão),
                                      porque o mapeamento varia conforme o driver do controle
  - ENTER neste terminal ............ só quando rodando em terminal interativo
  - tópico .......................... ros2 topic pub --once <ns>/waypoint_recorder/mark std_msgs/msg/Empty "{}"
Desfazer o último ponto:
  - botão Share (8) por padrão, ou <ns>/waypoint_recorder/undo

L1/R1 (4 e 5) NÃO marcam: são o enable/turbo do teleop. Todo botão apertado é registrado no
log com o seu índice, o que serve para descobrir o mapeamento real do controle em campo.

O arquivo YAML é reescrito a cada ponto (nada se perde se o processo cair) e sai no
formato de missão do seguidor:

    waypoints:
      gps_points:
        - latitude: -29.79719660
          longitude: -51.15094940
          tolerance: 0.6
          heading_deg: 12.3      # referência; o seguidor ignora

Uso:
  record_waypoints.py [saida.yaml] [--tolerance 0.6] [--min-spacing 0.3] [--min-fix 7]
                      [--mark-button 2] [--undo-button 3] [--namespace a300_00096]
                      [--yaw-offset-deg 90] [--force]
"""
import argparse
import datetime
import math
import os
import select
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Empty

try:
    from fixposition_driver_msgs.msg import FpaGnsscorr
except ImportError:  # sem o driver, grava sem checar qualidade do fix
    FpaGnsscorr = None

EARTH_R = 6371008.8


def enu_offset(lat0, lon0, lat1, lon1):
    """Deslocamento leste/norte em metros de (lat0,lon0) para (lat1,lon1)."""
    lat0r = math.radians(lat0)
    return (math.radians(lon1 - lon0) * math.cos(lat0r) * EARTH_R,
            math.radians(lat1 - lat0) * EARTH_R)


def dist_bearing(lat0, lon0, lat1, lon1):
    e, n = enu_offset(lat0, lon0, lat1, lon1)
    return math.hypot(e, n), math.atan2(n, e)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WaypointRecorder(Node):
    def __init__(self, args):
        super().__init__('waypoint_recorder', namespace=args.namespace)
        self.args = args
        self.points = []            # [{lat, lon, heading_deg, fix, ts}]
        self.ant = [None, None]     # (lat, lon, t) por antena
        self.gnss_fix = [None, None]
        self.gnss_fix_time = 0.0
        self.lat = self.lon = self.yaw = None
        self.pose_time = 0.0
        self.prev_buttons = []
        self.joy_time = 0.0
        self.yaw_offset = math.radians(args.yaw_offset_deg)
        parse = lambda s: {int(x) for x in str(s).replace(' ', '').split(',') if x != ''}
        self.mark_buttons = parse(args.mark_buttons)
        self.undo_buttons = parse(args.undo_buttons)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(NavSatFix, 'sensors/ins_0/gps_0/fix',
                                 lambda m: self.on_ant(0, m), qos)
        self.create_subscription(NavSatFix, 'sensors/ins_0/gps_1/fix',
                                 lambda m: self.on_ant(1, m), qos)
        if FpaGnsscorr is not None:
            self.create_subscription(FpaGnsscorr, 'sensors/ins_0/fixposition/fpa/gnsscorr',
                                     self.on_gnsscorr, qos)
        self.create_subscription(Joy, 'joy_teleop/joy', self.on_joy, 10)
        self.create_subscription(Empty, 'waypoint_recorder/mark', lambda m: self.mark('tópico'), 10)
        self.create_subscription(Empty, 'waypoint_recorder/undo', lambda m: self.undo('tópico'), 10)

        self.create_timer(2.0, self.print_status)
        if sys.stdin.isatty():
            self.create_timer(0.2, self.poll_stdin)

        self.log('gravador pronto. arquivo: %s' % args.output)
        self.log('marcar: botões %s | desfazer: botões %s | ENTER também marca'
                 % (sorted(self.mark_buttons), sorted(self.undo_buttons)))

    # ------------------------------------------------------------------ util
    def log(self, msg):
        print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)

    def now(self):
        return time.monotonic()

    # ------------------------------------------------------------------ callbacks
    def on_ant(self, idx, msg: NavSatFix):
        if msg.status.status < 0 or msg.latitude == 0.0:
            return
        self.ant[idx] = (msg.latitude, msg.longitude, self.now())
        a1, a2 = self.ant
        if a1 is None or a2 is None or abs(a1[2] - a2[2]) > 0.5:
            return
        self.lat = (a1[0] + a2[0]) / 2.0
        self.lon = (a1[1] + a2[1]) / 2.0
        _, bearing = dist_bearing(a1[0], a1[1], a2[0], a2[1])
        self.yaw = wrap(bearing + self.yaw_offset)
        self.pose_time = self.now()

    def on_gnsscorr(self, msg):
        self.gnss_fix = [int(msg.gnss1_fix), int(msg.gnss2_fix)]
        self.gnss_fix_time = self.now()

    def on_joy(self, msg: Joy):
        self.joy_time = self.now()
        b = list(msg.buttons)
        # toda borda de subida é registrada: revela o mapeamento real do controle em campo
        edges = [i for i, v in enumerate(b)
                 if v == 1 and (i >= len(self.prev_buttons) or self.prev_buttons[i] == 0)]
        self.prev_buttons = b
        for i in edges:
            role = 'marcar' if i in self.mark_buttons else ('desfazer' if i in self.undo_buttons else 'sem função')
            self.log('botão %d pressionado (%s)' % (i, role))
        for i in edges:
            if i in self.mark_buttons:
                self.mark('joystick botão %d' % i)
                return
            if i in self.undo_buttons:
                self.undo('joystick botão %d' % i)
                return

    def poll_stdin(self):
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            line = sys.stdin.readline()
            if line == '':
                return
            self.undo('teclado') if line.strip().lower() in ('u', 'undo') else self.mark('teclado')

    # ------------------------------------------------------------------ saúde
    def problem(self):
        t = self.now()
        if self.lat is None:
            return 'sem pose (antenas do Fixposition sem fix)'
        if t - self.pose_time > 2.0:
            return 'pose parada há %.1f s' % (t - self.pose_time)
        if FpaGnsscorr is not None and self.gnss_fix[0] is not None:
            if t - self.gnss_fix_time > 5.0:
                return 'status de correção antigo'
            if min(self.gnss_fix) < self.args.min_fix:
                return 'fix fraco (gnss1=%d gnss2=%d, precisa >=%d)' % (
                    self.gnss_fix[0], self.gnss_fix[1], self.args.min_fix)
        return None

    # ------------------------------------------------------------------ ações
    def mark(self, source):
        problem = self.problem()
        if problem and not self.args.force:
            self.log('IGNORADO (%s): %s' % (source, problem))
            return
        if self.points:
            p = self.points[-1]
            d, _ = dist_bearing(p['lat'], p['lon'], self.lat, self.lon)
            if d < self.args.min_spacing:
                self.log('IGNORADO (%s): %.2f m do ponto %d (mínimo %.2f m)'
                         % (source, d, len(self.points), self.args.min_spacing))
                return
        else:
            d = 0.0
        self.points.append({
            'lat': self.lat, 'lon': self.lon,
            'heading_deg': round(math.degrees(self.yaw), 1),
            'fix': list(self.gnss_fix),
            'ts': datetime.datetime.now().isoformat(timespec='seconds'),
        })
        self.write_file()
        self.log('MARK %d lat=%.8f lon=%.8f rumo=%.1f fix=%s dist_ant=%.2fm (%s)%s'
                 % (len(self.points), self.lat, self.lon, math.degrees(self.yaw),
                    self.gnss_fix, d, source, '  [FORÇADO: %s]' % problem if problem else ''))

    def undo(self, source):
        if not self.points:
            self.log('nada para desfazer (%s)' % source)
            return
        p = self.points.pop()
        self.write_file()
        self.log('DESFEITO ponto %d (lat=%.8f lon=%.8f) (%s)'
                 % (len(self.points) + 1, p['lat'], p['lon'], source))

    def write_file(self):
        lines = [
            '# gravado por record_waypoints.py em %s' % datetime.datetime.now().isoformat(timespec='seconds'),
            '# fonte de pose: dual_gnss (Fixposition, ponto médio das antenas; rumo pela baseline)',
            '# heading_deg é referência do momento da gravação; o seguidor ignora esse campo',
            'waypoints:',
            '  gps_points:',
        ]
        for p in self.points:
            lines += ['    - latitude: %.8f' % p['lat'],
                      '      longitude: %.8f' % p['lon'],
                      '      tolerance: %.2f' % self.args.tolerance,
                      '      heading_deg: %.1f' % p['heading_deg'],
                      '      # gravado %s, fix %s' % (p['ts'], p['fix'])]
        tmp = self.args.output + '.tmp'
        with open(tmp, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.replace(tmp, self.args.output)

    # ------------------------------------------------------------------ status
    def print_status(self):
        problem = self.problem()
        joy = 'ok' if self.now() - self.joy_time < 2.0 else 'SEM JOYSTICK'
        if self.lat is None:
            self.log('pontos=%d | sem pose | joy=%s | %s' % (len(self.points), joy, problem or ''))
            return
        self.log('pontos=%d | lat=%.7f lon=%.7f rumo=%.1f | fix=%s | joy=%s%s'
                 % (len(self.points), self.lat, self.lon, math.degrees(self.yaw),
                    self.gnss_fix, joy, ' | %s' % problem if problem else ''))

    def summary(self):
        self.log('--- resumo: %d pontos em %s ---' % (len(self.points), self.args.output))
        total = 0.0
        for i, p in enumerate(self.points):
            extra = ''
            if i:
                d, brg = dist_bearing(self.points[i - 1]['lat'], self.points[i - 1]['lon'], p['lat'], p['lon'])
                total += d
                extra = '  (+%.2f m, rumo do trecho %.0f°)' % (d, math.degrees(brg))
            self.log('  %d: %.8f, %.8f%s' % (i + 1, p['lat'], p['lon'], extra))
        if len(self.points) > 1:
            d, _ = dist_bearing(self.points[-1]['lat'], self.points[-1]['lon'],
                                self.points[0]['lat'], self.points[0]['lon'])
            self.log('  percurso total %.2f m | volta ao ponto 1: %.2f m' % (total, d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('output', nargs='?', default='/home/robot/mission_recorded.yaml')
    ap.add_argument('--namespace', default='a300_00096')
    ap.add_argument('--tolerance', type=float, default=0.6)
    ap.add_argument('--min-spacing', type=float, default=0.3, help='distância mínima entre pontos (m)')
    ap.add_argument('--min-fix', type=int, default=7, help='7 = RTK float, 8 = RTK fixed')
    ap.add_argument('--mark-buttons', default='0,1,2,3',
                    help='índices que marcam um ponto (os 4 botões de face, qualquer mapeamento)')
    ap.add_argument('--undo-buttons', default='8', help='índices que desfazem o último ponto')
    ap.add_argument('--yaw-offset-deg', type=float, default=90.0)
    ap.add_argument('--force', action='store_true', help='grava mesmo com fix fraco/pose antiga')
    args = ap.parse_args()

    rclpy.init()
    node = WaypointRecorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
