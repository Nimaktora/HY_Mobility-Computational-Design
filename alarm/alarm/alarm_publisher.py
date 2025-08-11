import rclpy
from rclpy.node import Node
import numpy as np
import os
import random

from std_msgs.msg import String
from std_msgs.msg import Int32

#from geometry.msgs.msg import Twist

class ALARM_PUB(Node):        
    def __init__(self):
        super().__init__("alarm_pub_node")
        self.randnum_pub = self.create_publisher(
            String,
            'randnum_pub_topic', 
            10
            )
        self.alarm_pub = self.create_publisher(
            Int32,
            'alarm_topic', 
            10
            )

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        self.num1 = np.random.randint(1,11)

        msg = String()
        msg.data = 'Random number: %d' % self.num1
        self.randnum_pub.publish(msg)

        msg1 = Int32()
        msg1.data = self.num1
        self.alarm_pub.publish(msg1)


        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.get_logger().info('Publishing: "%d"' % msg1.data)
        


def main(args=None):
    rclpy.init(args=args)
    node = ALARM_PUB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    

    

if __name__ == '__main__':
    main()
    