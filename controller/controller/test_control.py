import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import numpy as np

class TestControlPub(Node):
    def __init__(self):
        super().__init__('test_node')
        self.pub = self.create_publisher(String, "/cmd_move", 4)
        self.pub_manip = self.create_publisher(Float32MultiArray, "/cmd_point", 4)
    def line(self,x, y_start, y_stop):
        
        msg_manip = Float32MultiArray()
        msg_manip.data = (float(x), float(y_start), 0.0)
        self.pub_manip.publish(msg_manip)
        time.sleep(0.5)
        if y_start < 0:
            arr = np.arange(y_start, y_stop, 8)
        else:
            arr = np.arange(y_stop, y_start, 8)
            arr = np.flip(arr)
        print(arr)
        for y in arr:
            msg_manip.data = (float(x), float(y), 0.0)
            self.pub_manip.publish(msg_manip)
            time.sleep(0.053)
def main():
    time_move = 10
    rclpy.init()
    t_node = TestControlPub()
    msg = String()
    msg_manip = Float32MultiArray()
    msg_manip.data = (450.0 ,-400.0,0.0)
    t_node.pub_manip.publish(msg_manip)
    time.sleep(2)
    msg.data = 'left'
    t_node.pub.publish(msg)
    time.sleep(2.5)    
    try:
        while True:
            # time.sleep(time_move)
            t_node.line(450, -400, 400)
            t_node.line(300, 400, -400)
            
            # 
            # msg_manip.data = (400.0,-400.0,0.0)
            # t_node.pub_manip.publish(msg_manip)
            # time.sleep(0.5)
            # msg.data = 'left'
            # t_node.pub.publish(msg)
            # time.sleep(time_move)
            # msg.data = 'back'
            # t_node.pub.publish(msg)
            # time.sleep(time_move)
            # msg.data = 'right'
            # t_node.pub.publish(msg)
            # time.sleep(time_move)
    except Exception as e:
        print(e)
        exit()

if __name__=="__main__":
    main()