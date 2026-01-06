#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = Twist()

        if self.count < 30:          # 3 seconds forward
            msg.linear.x = 0.5
        elif self.count < 50:        # 2 seconds rotate
            msg.angular.z = 1.0
        else:
            self.count = -1          # restart loop

        self.publisher_.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
