import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TestControlPub(Node):
    def __init__(self):
        super().__init__('test_node')
        self.pub = self.create_publisher(String, "/cmd_move", 4)


def main():
    time_move = 10
    rclpy.init()
    t_node = TestControlPub()
    msg = String()
    try:
        while True:
            msg.data = 'front'
            t_node.pub.publish(msg)
            time.sleep(time_move)
            msg.data = 'left'
            t_node.pub.publish(msg)
            time.sleep(time_move)
            msg.data = 'back'
            t_node.pub.publish(msg)
            time.sleep(time_move)
            msg.data = 'right'
            t_node.pub.publish(msg)
            time.sleep(time_move)
    except Exception as e:
        print(e)
        exit()

if __name__=="__main__":
    main()