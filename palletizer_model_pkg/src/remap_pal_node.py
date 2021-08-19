#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PalletizerJointRemap(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('palletizer_joint_remap')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.joints_name = ['pal_joint0', 'pal_joint1',\
                            'pal_joint2', 'pal_joint3',\
                            'pal_joint4', 'pal_joint10',\
                            'pal_joint5', 'pal_joint6',\
                            'pal_joint7', 'pal_joint8']

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states_remap',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        position = [0.0]*10
        base_pos = msg.position
        msg.name = self.joints_name
        position[0] = base_pos[0]
        position[1] = base_pos[1]
        position[2] = base_pos[2]
        position[3] = base_pos[3]
        position[4] = base_pos[2]
        position[5] = -base_pos[1]
        position[6] = base_pos[1]
        position[7] = -base_pos[2]
        position[8] = -base_pos[2]-base_pos[1]
        position[9] = base_pos[2]+base_pos[1]
        msg.position = position
        self.js_pub.publish(msg)


def main():
    node = PalletizerJointRemap()
    rclpy.spin(node)  

if __name__ == '__main__':
    main()
