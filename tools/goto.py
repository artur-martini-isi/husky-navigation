#!/usr/bin/env python3
"""goto.py — manda o Husky a um ponto do mapa sem depender do Foxglove. Roda NO ROBÔ.

    ./goto.py 3.5 -1.2              vai para (3.5, -1.2) no frame do mapa
    ./goto.py 3.5 -1.2 --yaw 90     chega e gira para 90 graus
    ./goto.py 2 0 --frame base_link dois metros à frente de onde o robô está agora
    ./goto.py --replace 5 5         esquece a fila e vai só para este ponto
    ./goto.py --status              mostra o estado atual
    ./goto.py --clear               limpa a fila e para
    ./goto.py --skip                pula o marcador atual

Vários pontos de uma vez viram uma rota:

    ./goto.py 1 0 --then 2 2 --then 0 3
"""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('x', nargs='?', type=float)
    ap.add_argument('y', nargs='?', type=float)
    ap.add_argument('--then', nargs=2, type=float, action='append', metavar=('X', 'Y'),
                    help='mais um ponto na rota (pode repetir)')
    ap.add_argument('--yaw', type=float, help='rumo desejado na chegada, em graus')
    ap.add_argument('--frame', default='map', help='frame do ponto (map, base_link, odom...)')
    ap.add_argument('--replace', action='store_true', help='descarta a fila antes de enfileirar')
    ap.add_argument('--namespace', default='a300_00096')
    ap.add_argument('--node', default='goto_point')
    ap.add_argument('--status', action='store_true')
    ap.add_argument('--clear', action='store_true')
    ap.add_argument('--skip', action='store_true')
    ap.add_argument('--stop', action='store_true')
    ap.add_argument('--start', action='store_true')
    a = ap.parse_args()

    base = '/%s/%s' % (a.namespace, a.node)
    rclpy.init()
    n = Node('goto_cli')

    if a.status:
        visto = []
        n.create_subscription(String, base + '/status', lambda m: visto.append(m.data), 10)
        fim = n.get_clock().now().nanoseconds + 3_000_000_000
        while rclpy.ok() and not visto and n.get_clock().now().nanoseconds < fim:
            rclpy.spin_once(n, timeout_sec=0.2)
        print(json.dumps(json.loads(visto[-1]), indent=2, ensure_ascii=False) if visto
              else 'sem status em %s/status (o nó está no ar?)' % base)
        rclpy.shutdown()
        return 0 if visto else 1

    for flag, srv in (('clear', 'clear'), ('skip', 'skip'), ('stop', 'stop'), ('start', 'start')):
        if getattr(a, flag):
            cli = n.create_client(Trigger, '%s/%s' % (base, srv))
            if not cli.wait_for_service(timeout_sec=3.0):
                print('serviço %s/%s indisponível' % (base, srv)); rclpy.shutdown(); return 1
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
            r = fut.result()
            print('%s: %s' % (srv, r.message if r else 'sem resposta'))
            rclpy.shutdown()
            return 0

    if a.x is None or a.y is None:
        ap.print_help()
        rclpy.shutdown()
        return 2

    pontos = [{'x': a.x, 'y': a.y, 'frame': a.frame}]
    if a.yaw is not None:
        pontos[0]['yaw_deg'] = a.yaw
    for x, y in (a.then or []):
        pontos.append({'x': x, 'y': y, 'frame': a.frame})
    if a.replace:
        pontos[0]['mode'] = 'replace'

    pub = n.create_publisher(String, base + '/set_goal', 10)
    fim = n.get_clock().now().nanoseconds + 3_000_000_000
    while rclpy.ok() and pub.get_subscription_count() == 0 and n.get_clock().now().nanoseconds < fim:
        rclpy.spin_once(n, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print('ninguém escutando %s/set_goal (o nó goto_point está no ar?)' % base)
        rclpy.shutdown()
        return 1
    pub.publish(String(data=json.dumps(pontos)))
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    print('enviado: %s' % json.dumps(pontos, ensure_ascii=False))
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
