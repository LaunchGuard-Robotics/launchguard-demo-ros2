#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        msg = Twist()
        msg.linear.x = 0.2
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
