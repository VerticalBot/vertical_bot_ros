import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time

class TestControlPub(Node):
    def __init__(self):
        super().__init__('test_node')
        self.pub = self.create_publisher(String, "/cmd_move", 4)
        self.pub_manip = self.create_publisher(Float32MultiArray, "/cmd_point", 4)

def main():
    time_move = 10
    rclpy.init()
    t_node = TestControlPub()
    msg = String()
    msg_manip = Float32MultiArray()
    msg_manip.data = (400.0 ,400.0,0.0)
    t_node.pub_manip.publish(msg_manip)
    time.sleep(1)
    msg.data = 'left'
    t_node.pub.publish(msg)
                    
    try:
        while True:
            # time.sleep(time_move)
            msg_manip.data = (300.0,-300.0,0.0)
            t_node.pub_manip.publish(msg_manip)
            time.sleep(0.5)
            msg_manip.data = (300.0,300.0,0.0)
            t_node.pub_manip.publish(msg_manip)
            time.sleep(0.5)
            msg_manip.data = (400.0,400.0,0.0)
            t_node.pub_manip.publish(msg_manip)
            time.sleep(0.5)
            
            msg_manip.data = (400.0,-400.0,0.0)
            t_node.pub_manip.publish(msg_manip)
            time.sleep(0.5)
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