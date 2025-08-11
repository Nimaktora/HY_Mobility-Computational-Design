import rclpy
from rclpy.node import Node
import numpy as np
import os
import random

from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math

class Emergency(Node):        
    def __init__(self):
        super().__init__("emergency_node")
        self.emergency_pub = self.create_publisher(
            String,
            'emergency_topic',
            10
            )
        timer_period = 1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0
    def timer_callback(self):
        msg = String()
        self.t += 1
        if self.t >= 10:
            self.num1 = np.random.randint(1,20)
            if self.num1 <= 1:
                msg.data = 'Emergency'
            else:
                msg.data = 'Normal'

        self.emergency_pub.publish(msg)
        self.get_logger().info('pub_signal: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Emergency()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    

if __name__ == '__main__':
    main()