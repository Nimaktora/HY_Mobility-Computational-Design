import rclpy
from rclpy.node import Node
import numpy as np
import os


from std_msgs.msg import String, Int32

class ALARM_SUB(Node):        
    def __init__(self):
        super().__init__("example4_sub_node")

        self.randnum_sub =  self.create_subscription(
            String,
            'randnum_pub_topic',
            self.listener_callback1,
            10
        )

        self.alarm_sub = self.create_subscription(
            Int32,
            'alarm_topic',
            self.listener_callback2,
            10
        )
        self.notice_pub = self.create_publisher(
            String,
            'notice_topic',
            10
        )
    def listener_callback1(self, msg):
        self.get_logger().info('Subscribing: "%s"' % msg.data)

    def listener_callback2(self, msg1):
        if msg1.data < 3:
            self.W = 'Normal'
        elif msg1.data >= 4 and msg1.data <7:
            self.W = 'Caution'
        else:
            self.W = 'Abnormal'

        
        self.get_logger().info('Subscribing: "%d"' % msg1.data)
        self.get_logger().info('Subscribing: "%s"' % self.W)

        msg2 = String()
        msg2.data = self.W

        self.notice_pub.publish(msg2)



def main(args=None):
    rclpy.init(args=args)
    node = ALARM_SUB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    

if __name__ == '__main__':
    main()