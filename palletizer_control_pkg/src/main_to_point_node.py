#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from RoboticArmPalletizerClass import RoboticArm
from sensor_msgs.msg import JointState

class ToPointNode(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('to_point_node')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.roboticArm = RoboticArm()

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/cmd_point',
            self.listener_callback,
            10)

        self.joint_state_publisher = self.create_publisher(JointState, '/cmd_joint_state', 10)

    def listener_callback(self, msg):
        cmd_point = msg.data
        if len(cmd_point)<3:
            print("invalid coordinate")
            return
        
        availJointState,goalJointState = self.roboticArm.InversProblem(cmd_point[0],cmd_point[1],cmd_point[2], 0.0)
        print(availJointState, goalJointState)
        msg = JointState()
        msg.position = goalJointState
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_publisher.publish(msg)

def main():
    node = ToPointNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
