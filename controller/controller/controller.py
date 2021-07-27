from time import sleep, time
from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class Controller(Node):
    def __init__(self):	
        super().__init__('controller')
        self.sub = self.create_subscription(String, '/cmd_move', self.cmd_cb, 10)
        self.joints_states_pub = self.create_publisher(JointState, "/joint_states", 4)
        self.pose_pub = self.create_publisher(Pose, "/robot_pose", 4)
        self.up_robot_z = 0.08
        self.down_robot_z = 0.04
        self.pose = Pose()
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        self.pose.position.z = self.up_robot_z
        self.joint1_limit = [-0.05, 0.05]
        self.joint2_limit = [-0.05, 0.05]
        self.joint1_magnit_limit = [0.0, 0.04]
        self.joint2_magnit_limit = [0.0, 0.04]
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
        self.msg.position.append(self.joint2_magnit_limit[1])
        self.msg.position.append(self.joint2_magnit_limit[1])

        self.joint_speed = 0.005
        
        self.create_timer(0.1, self.timer_move_callback_callback)
        self.create_timer(0.1, self.timer_joint_callback)
        self.create_timer(0.1, self.robot_joint_controller)
        self.state_of_walk = None

    def robot_joint_controller(self):
        if self.state_of_walk == "front":
            if self.msg.position[2] == self.joint1_magnit_limit[0] and self.msg.position[4] == self.joint2_magnit_limit[1]:
                self.pose.position.x -= abs(self.joint_speed)
                self.msg.position[1] += self.joint_speed
                if self.msg.position[1] > self.joint2_limit[1]:
                    self.up_down_magnit(True)
                
            else:
                self.msg.position[1] -= self.joint_speed
                if self.msg.position[1] < self.joint2_limit[0]:
                    self.up_down_magnit(False)
            
        if self.state_of_walk == "back":
            if self.msg.position[2] == self.joint1_magnit_limit[0] and self.msg.position[4] == self.joint2_magnit_limit[1]:
                self.pose.position.x += abs(self.joint_speed)
                self.msg.position[1] -= self.joint_speed
                if self.msg.position[1] < self.joint2_limit[0]:
                    self.up_down_magnit(True)
                
            else:
                self.msg.position[1] += self.joint_speed
                if self.msg.position[1] > self.joint2_limit[1]:
                    self.up_down_magnit(False)


        if self.state_of_walk == "left":
            if self.msg.position[2] == self.joint1_magnit_limit[1] and self.msg.position[4] == self.joint2_magnit_limit[0]:
                self.pose.position.y += abs(self.joint_speed)
                self.msg.position[0] -= self.joint_speed
                if self.msg.position[0] < self.joint2_limit[0]:
                    self.up_down_magnit(False)
                
            else:
                self.msg.position[0] += self.joint_speed
                if self.msg.position[0] > self.joint2_limit[1]:
                    self.up_down_magnit(True)

        if self.state_of_walk == "right":
            if self.msg.position[2] == self.joint1_magnit_limit[1] and self.msg.position[4] == self.joint2_magnit_limit[0]:
                self.pose.position.y -= abs(self.joint_speed)
                self.msg.position[0] += self.joint_speed
                if self.msg.position[0] > self.joint2_limit[1]:
                    self.up_down_magnit(False)
                
            else:
                self.msg.position[0] -= self.joint_speed
                if self.msg.position[0] < self.joint2_limit[0]:
                    self.up_down_magnit(True)

    def timer_move_callback_callback(self):
        self.pose_pub.publish(self.pose)

    def timer_joint_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.joints_states_pub.publish(self.msg)

    def cmd_cb(self, data):
        self.state_of_walk = data.data
    
    def up_down_magnit(self, flag):

        self.msg.position[2] = self.joint1_magnit_limit[int(flag)]
        self.msg.position[3] = self.joint1_magnit_limit[int(flag)]
        self.msg.position[4] = self.joint2_magnit_limit[int(not flag)]
        self.msg.position[5] = self.joint2_magnit_limit[int(not flag)]

def main():
	rclpy.init()
	controller = Controller()
	rclpy.spin(controller)
	rclpy.shutdown()

if __name__ == '__main__':
    main()
    