#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from math import fabs
import numpy as np
import time
from threading import Thread, currentThread

class ToJointNode(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('to_joint_node')

        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.subscription = self.create_subscription(
            JointState,
            '/cmd_joint_state',
            self.move_of_trapeze_principle,
            10)

        self.currentJointState = [0.0,0.0,0.0,0.0]
        self.maxVelocity = 1.0
        self.countOfJoint = 4
        self.iter_time = 0.01
        self.q = None
        self.robot_state = 0

        self.joint_publisher = self.create_publisher(JointState, '/joint_states_remap', 10)
        self.state_robot_publisher = self.create_publisher(Int32, '/palletizer_robot_state', 10)
        self.create_timer(0.01, self.publish_joint_state)
        self.create_timer(0.1, self.publish_robot_state)
        self.thread = Thread(target=self.thread_realize_joint_state)
        self.thread.start()
        self.threading = False
    
    def move_of_trapeze_principle(self, msg):
        self.threading = False
        joint_state = msg.position
        way_list = self.way_compute(joint_state)
        max_way_new = self.max_way(way_list)
        if(not max_way_new == 0):
            time_arr = 3*fabs(max_way_new)/(2*self.maxVelocity)
            self.realize_of_principle(joint_state, time_arr)

    def way_compute(self, joint_state):
        wayList = []
        for i in range(len(joint_state)):
            wayList.append(joint_state[i]-self.currentJointState[i])
        return wayList

    def max_way(self, wayList):
        maxWay = wayList[-1]
        for i in range(len(wayList)-1):
            if (fabs(wayList[i])>fabs(maxWay)):
                maxWay = wayList[i]
            else:
                continue
        return maxWay

    def realize_of_principle(self, goalJointStates, time_arrOfWay):
        q = []; time_arr = []
        time_arr.append(0)
        i = 0; k=0
        start = np.array(self.currentJointState)
        end = np.array(goalJointStates)
        const = (end-start)
        q.append(start)
        while(i<=time_arrOfWay):
            i+=self.iter_time
            if(i<time_arrOfWay/3):
                vel = 9*i*const/(2*time_arrOfWay**2)
            elif(i>2*time_arrOfWay/3):
                vel = (-9/(2*time_arrOfWay**2)*i+9/(2*time_arrOfWay))*const
            else:
                vel = 3/(2*time_arrOfWay)*const
            q.append(q[k] + vel*self.iter_time)
            k+=1
            time_arr.append(i)
        
        self.q = q
        self.threading = True

    def publish_joint_state(self):
        msg = JointState()
        msg.position = self.currentJointState
        msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_publisher.publish(msg)
    
    def publish_robot_state(self):
        msg = Int32()
        msg.data = self.robot_state
        self.state_robot_publisher.publish(msg)
    
    def thread_realize_joint_state(self):
        while True:
            if not self.q is None:
                self.robot_state = 1
                for el in self.q:
                    if self.threading:
                        self.currentJointState = el.tolist()
                        time.sleep(self.iter_time)
                    else:
                        break
                self.q = None
                self.robot_state = 0
            else:
                time.sleep(0.01)        

def main():
    node = ToJointNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
