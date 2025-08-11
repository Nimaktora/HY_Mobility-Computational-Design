import rclpy
from rclpy.node import Node
import numpy as np
import os
import random

from std_msgs.msg import String
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist

class emergency(Node):
    def __init__(self):
        super().__init__("emergency_node")

        self.emergency_sub = self.create_subscription(
            String,
            'emergency_topic',
            self.callback,
            10
        )

        self.cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def callback(self, msg2):
        cmd_vel_msg = Twist()
        if msg2.data == 'Normal':
            cmd_vel_msg.linear.x = 1.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
        elif msg2.data == 'Caution':
            cmd_vel_msg.linear.x = -1.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
        else:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
        
        self.cmd_vel_node.publish(cmd_vel_msg)



def main(args=None):
    rclpy.init(args=args)
    node = emergency()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    

    

if __name__ == '__main__':
    main()
    
