#!/usr/bin/env python3
"""urdf_beacon — repete a descrição do robô num tópico próprio, para o Foxglove.

O `robot_state_publisher` publica o URDF **uma única vez**, num tópico retido
(`transient_local`): quem assina depois recebe a cópia guardada. Isso funciona para qualquer
assinante que peça `transient_local` — e falha para quem assina como `volatile`, porque para esse
a mensagem já passou e nunca mais vem outra.

É exatamente o que acontece com a ponte do Foxglove. Ela decide a QoS da assinatura no momento em
que o **primeiro cliente** pede o tópico; se nesse instante o publicador ainda não tiver sido
descoberto, ela cai no padrão `volatile` e **fica assim enquanto houver algum cliente assinando**.
Como a assinatura ROS é uma só, compartilhada por todos os clientes, um navegador que conectou cedo
demais estraga a visualização para todos os outros, e o sintoma é o robô sumir do painel 3D. Já
aconteceu duas vezes aqui (2026-09-11 e 2026-09-15), e o contorno era fechar e reabrir o Foxglove.

Este nó guarda o URDF e o republica a cada `period` segundos num tópico separado. Assinante
volátil recebe a próxima repetição; assinante `transient_local` recebe na hora. Ele publica num
**tópico diferente** de propósito: o `controller_manager` também assina `robot_description` e
reage a cada mensagem nova, então republicar ali poderia reinicializar o hardware.

    <ns>/robot_description  ──(uma vez, retido)──►  urdf_beacon  ──(a cada 5 s)──►  <ns>/robot_description_foxglove

No painel 3D do Foxglove, a camada de URDF deve apontar para `robot_description_foxglove`.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class UrdfBeacon(Node):
    def __init__(self):
        super().__init__('urdf_beacon')
        p = self.declare_parameter
        p('source_topic', 'robot_description')
        p('output_topic', 'robot_description_foxglove')
        p('period', 5.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.urdf = None
        self.publicadas = 0
        retido = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(String, str(g('output_topic')), retido)
        self.create_subscription(String, str(g('source_topic')), self.on_urdf, retido)
        self.create_timer(float(g('period')), self.repetir)
        self.create_timer(60.0, self.resumo)
        self.get_logger().info('repetindo %s em %s a cada %.0f s (esperando a descrição)'
                               % (g('source_topic'), g('output_topic'), float(g('period'))))

    def on_urdf(self, msg):
        if msg.data and msg.data != self.urdf:
            self.urdf = msg.data
            self.get_logger().info('descrição recebida: %d bytes, %d links'
                                   % (len(self.urdf), self.urdf.count('<link')))
            self.repetir()

    def repetir(self):
        if self.urdf:
            self.pub.publish(String(data=self.urdf))
            self.publicadas += 1

    def resumo(self):
        if self.urdf:
            self.get_logger().info('%d repetições, %d bytes cada' % (self.publicadas, len(self.urdf)))
        else:
            self.get_logger().warn('ainda sem descrição: o robot_state_publisher está no ar?')


def main(args=None):
    rclpy.init(args=args)
    node = UrdfBeacon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
