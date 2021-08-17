#!/usr/bin/env python3

from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.update_pobot_pose)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.subscription = self.create_subscription(
            Pose,
            'robot_pose',
            self.listener_callback,
            10)
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'world_1'
        self.odom_trans.child_frame_id = 'world'
        self.msg = Pose()
        self.subscription

    def listener_callback(self, msg):
        self.msg = msg
        print(msg)

    def update_pobot_pose(self):
        # if self.msg != None:
        now = self.get_clock().now()
        # print(now)
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = self.msg.position.x
        self.odom_trans.transform.translation.y = self.msg.position.y
        self.odom_trans.transform.translation.z = self.msg.position.z
        self.odom_trans.transform.rotation = self.msg.orientation
        self.broadcaster.sendTransform(self.odom_trans)

def main():
    node = StatePublisher()
    rclpy.spin(node)  

if __name__ == '__main__':
    main()