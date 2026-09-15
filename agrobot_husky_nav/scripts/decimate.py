#!/usr/bin/env python3
"""Republica /livox/lidar decimado em /livox/lidar_lite (cabe em WiFi lento)."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

STEP = int(sys.argv[1]) if len(sys.argv) > 1 else 5   # mantem 1 a cada STEP pontos


class Decimator(Node):
    def __init__(self):
        super().__init__("livox_decimator")
        qos_in = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=2,
            durability=DurabilityPolicy.VOLATILE)
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PointCloud2, "/livox/lidar_lite", qos_out)
        self.sub = self.create_subscription(PointCloud2, "/livox/lidar", self.cb, qos_in)
        self.n = 0
        self.get_logger().info(f"decimando /livox/lidar 1:{STEP} -> /livox/lidar_lite")

    def cb(self, msg: PointCloud2):
        ps = msg.point_step
        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, ps)
        out = np.ascontiguousarray(buf[::STEP])
        new = PointCloud2()
        new.header = msg.header
        new.height = 1
        new.width = out.shape[0]
        new.fields = msg.fields
        new.is_bigendian = msg.is_bigendian
        new.point_step = ps
        new.row_step = ps * out.shape[0]
        new.is_dense = msg.is_dense
        new.data = out.tobytes()
        self.pub.publish(new)
        self.n += 1
        if self.n % 50 == 0:
            self.get_logger().info(
                f"{self.n} frames | {msg.width} -> {new.width} pts "
                f"({new.row_step/1024:.0f} KB)")


def main():
    rclpy.init()
    node = Decimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
