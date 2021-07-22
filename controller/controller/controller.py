from time import sleep, time
from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState

class Controller(Node):
    def __init__(self):	
        super().__init__('controller')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.joints_states_pub = self.create_publisher(JointState, "/joint_states", 4)
        self.pose_pub = self.create_publisher(Pose, "/robot_pose", 4)
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = 0.0
        self.msg = JointState()
        self.msg.name.append("joint_1")
        self.msg.name.append("joint_2")
        self.msg.name.append("joint_1_1")
        self.msg.name.append("joint_1_2")
        self.msg.name.append("joint_2_1")
        self.msg.name.append("joint_2_2")
        self.msg.position.append(0)
        self.msg.position.append(0)
        self.msg.position.append(0)
        self.msg.position.append(0)
        self.msg.position.append(0)
        self.msg.position.append(0)
        
        self.create_timer(0.5, self.timer_callback)
        self.create_timer(1, self.timer1_callback)        
        self.state_of_walk = 0

    def cmd_cb(self, data):
        a = 1
    def timer1_callback(self):
        self.pose_pub.publish(self.pose)
        print(self.pose.position.x)
    def timer_callback(self):
        if self.state_of_walk == 0:
            self.control_x("front")
            self.control_x_height("down")
            self.control_y_height("up")
            self.state_of_walk = 1
        elif self.state_of_walk == 1:
            self.control_x("back")
            self.control_x_height("up")
            self.control_y_height("down")
            self.state_of_walk = 2
        elif self.state_of_walk == 2:
            self.control_x("back")
            self.control_x_height("down")
            self.control_y_height("up")
            self.pose.position.x+=0.1
            self.state_of_walk = 0

    def control_x(self, side):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        if side == 'front':
            self.msg.position[0] = 0.05
        elif side == 'back':
            self.msg.position[0] = -0.05
        self.joints_states_pub.publish(self.msg)
    
    def control_x_height(self, side):
        if side == 'up':
            self.msg.position[2] = 0.0
            self.msg.position[3] = 0.0
        elif side == 'down':
            self.msg.position[2] = 0.04
            self.msg.position[3] = 0.04

    def control_y_height(self, side):
        if side == 'up':
            self.msg.position[4] = 0.0
            self.msg.position[5] = 0.0
        elif side == 'down':
            self.msg.position[4] = 0.04
            self.msg.position[5] = 0.0
def main():
	rclpy.init()
	controller = Controller()
	rclpy.spin(controller)
	rclpy.shutdown()
if __name__ == '__main__':
    main()
    