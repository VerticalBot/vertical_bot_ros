#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class Test(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('test')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.pub = self.create_publisher(Float32MultiArray, '/cmd_point', 10)

        msg = Float32MultiArray()

        msg.data = [350.0, 0.0, 0.0]
        self.pub.publish(msg)

    def listener_callback(self, msg):
        print(msg)


def main():
    node = Test()

if __name__ == '__main__':
    main()