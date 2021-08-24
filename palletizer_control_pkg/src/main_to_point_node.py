#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory
from RoboticArmPalletizerClass import RoboticArm
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET

import os
import json

class ToPointNode(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('to_point_node')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.loadParam()

        self.roboticArm = RoboticArm(self.scale)

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
        # self.get_logger().info("{0}".format(cmd_point))
        availJointState,goalJointState = self.roboticArm.InversProblem(cmd_point[0],cmd_point[1],cmd_point[2], 0.0)
        # self.get_logger().info("{0}".format(goalJointState))
        if availJointState:
            msg = JointState()
            msg.position = goalJointState
            msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_publisher.publish(msg)
    
    
    def loadParam(self):
        config_file = os.path.join(get_package_share_directory('palletizer_model_pkg'),
                        'urdf', 'config.xacro')

        tree = ET.parse(config_file)
        param = tree.getroot()[0]
        self.scale = float(param.attrib['value'])

def main():
    node = ToPointNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
