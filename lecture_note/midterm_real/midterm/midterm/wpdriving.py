import rclpy
from rclpy.node import Node
import numpy as np
import os

from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math

class wpDriving(Node):        
    def __init__(self):
        super().__init__("wpdriving_node")

        self.emergency_sub =  self.create_subscription(
            String,
            'emergency_topic',
            self.listener_callback,
            10
        )
        self.driving = self.create_subscription(
            Odometry,
            '/skidbot/odom',
            self.odom_callback,
            10
        )
        self.cmd_vel_node = self.create_publisher(
            Twist,
            '/skidbot/cmd_vel',
            10
        )
        self.target_x = [0, -4, -4, -6, -6]
        self.target_y = [5, 5, 1, 1, 8]
        self.a = 0
        self.hold = 1

    def listener_callback(self, msg):
        self.get_logger().info('sub_signal: "%s"' % msg.data)
        if msg.data == 'Emergency':
            self.hold = 0
            self.timer = self.create_timer(3.0, self.timer_callback)
        
    def timer_callback(self):
        self.hold = 1
        self.destroy_timer(self.timer)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        r = R.from_quat([q.x, q.y, q.z, q.w])
        _, _, self.current_yaw = r.as_euler('xyz', degrees=False)
    
        

        angDif = math.atan2(((self.target_y[self.a] - self.current_y)), ((self.target_x[self.a] - self.current_x))) - self.current_yaw
        while angDif > math.pi:
            angDif -= 2 * math.pi
        while angDif < -math.pi:
            angDif += 2 * math.pi
        distance = math.hypot(self.target_x[self.a] - self.current_x, self.target_y[self.a] - self.current_y)
        cmd_vel_msg = Twist()

        if self.hold == 0:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.cmd_vel_node.publish(cmd_vel_msg) 
            return
        
        if (self.a == 5):
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.get_logger().info('Finish')
            self.cmd_vel_node.publish(cmd_vel_msg)
            return

        if distance > 0.1:
            if angDif >= 0.2:
                cmd_vel_msg.linear.x = 0.25
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0
                cmd_vel_msg.angular.z = 7.0 * angDif + 1.0
            elif angDif <= - 0.2:
                cmd_vel_msg.linear.x = 0.25
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0
                cmd_vel_msg.angular.z = 7.0 * angDif - 1.0
            else:
                cmd_vel_msg.angular.z = 0.0
                cmd_vel_msg.linear.x = 2.0 * distance + 0.2

        elif distance <= 0.1:
            self.a += 1
            self.get_logger().info('waypoint: "%d"' % self.a)

        
        self.cmd_vel_node.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = wpDriving()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    

if __name__ == '__main__':
    main()