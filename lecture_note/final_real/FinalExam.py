import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

class Depth_Image(Node):
    def __init__(self):
        super().__init__('AutoDriving')            

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/depth_camera/image_raw',
            self.image_callback,
            10
        )

        self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.auto_driving_pub = self.create_publisher(
            Image,
            '/auto_driving',
            10
        )
        
        # QOS set
        QOS = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)
        
        # subscribe & synchronize
        self.depth_image_rgb = Subscriber(self, Image, '/depth_camera/image_raw')
        self.depth_image_raw = Subscriber(self, Image, '/depth_camera/depth/image_raw')

        self.sync = ApproximateTimeSynchronizer(
            [self.depth_image_rgb, self.depth_image_raw],
            queue_size=100,
            slop=1.0)
        self.sync.registerCallback(self.depth_callback_sync)

        # initial value set - PID
        self.error = 0
        self.ierror = 0
        self.derror = 0

        # initial value set - state
        self.state = 'IDLE'
        self.duration_red = 0
        self.duration_green = 0

        # timer create
        self.timer_period = 0.1  # 0.1 seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # time set
        self.start_time = 0.0
        self.elapsed_time = 0.0

        #self.stop_time = 0.0
        self.stop_elapsed_time = 5.0
        self.stop_pass_time = 7.5

        #self.turning = 0.0
        self.turning_elapsed_time = 10.0
        self.turning_pass_time = 14.5

    # red and green car state check based on duratin
    def timer_callback(self):
        self.elapsed_time += self.timer_period
        if self.state == 'CHECK_RED':
            self.duration_red += self.timer_period
            if self.duration_red >= self.stop_elapsed_time:
                self.state = 'PASS_RED'
                #print ("PASS_RED")
        if self.state == 'PASS_RED':
            self.duration_red += self.timer_period
            if self.duration_red >= self.stop_pass_time:
                self.state = 'IDLE'
                #print ('IDLE')
                self.duration_red = 0
        if self.state == 'CHECK_GREEN':
            self.duration_green += self.timer_period
            if self.duration_green >= self.turning_elapsed_time:
                self.state = 'PASS_GREEN'
                #print ("PASS_GREEN")
        if self.state == 'PASS_GREEN':
            self.duration_green += self.timer_period
            if self.duration_green >= self.turning_pass_time:
                self.state = 'IDLE'
                #print ('IDLE')
                self.duration_green = 0

    # color mask -> edge line -> circle
    def get_color(self, color_name):
        color_map = {
            'red': (64, 64, 166),
            'green': (64, 166, 64),
            'yellow': (79, 255, 255),
            'white': (255, 255, 255)
        }
        return np.array(color_map[color_name])
    def get_color_mask(self, img, color_name,noise):
        color = self.get_color(color_name)
        upper = color + noise
        lower = color - noise
        mask =  cv2.inRange(img, lower, upper)
        return mask  
    def get_edge_line(self, mask):
        edges = cv2.Canny(mask, 50, 150, apertureSize=3, L2gradient=True)
        return edges
    # circle -> light distance
    def get_light_distance(self, img, color, edges, dist_m): #dist_m = monitor limit
        circles = cv2.HoughCircles(edges ,cv2.HOUGH_GRADIENT,1.2,150,param1=15,param2=30,minRadius=1,maxRadius=100)
        dist = dist_m + 1
        if circles is not None:
          circle = np.around(circles[0][0]).astype(int)
          center_x = circle[0]
          center_y = circle[1]
          dist = img[center_y, center_x]
        
        if color == 'red':
            if (dist <= dist_m) and (self.state == 'IDLE'):
                print(f"적신호와의 거리: {dist:.5f}")
                self.flag_red = True
            if (self.state == 'CHECK_RED') and (self.flag_red == True):
                print(f"-> Stop: {self.stop_elapsed_time} second")
                self.flag_red = False
            
        if color == 'green':
            if (dist <= dist_m) and (self.state == 'IDLE'):
                print(f"초록신호와의 거리: {dist:.5f}")
                self.flag_green = True
            if (self.state == 'CHECK_GREEN') and (self.flag_green == True):
                print(f"-> Turning: {self.turning_elapsed_time} second")
                self.flag_green = False

        return dist
    
    # distance -> light on/off status
    def get_distance2status(self, dist, dist_lim):  
        status = "OFF"
        if dist <= dist_lim:
            status = "ON"
        else:
          status = "OFF"
        return status
    # light status -> driving
    def status2driving(self, img):
        cmd = Twist()

        yellow_mask = self.get_color_mask(img, 'yellow', 0.5)
        white_mask = self.get_color_mask(img, 'white', 0.5)
        mask = yellow_mask + white_mask

        cv_shape = img.shape
        height, width = cv_shape[:2]

        canny_yellow = cv2.Canny(yellow_mask, 50, 150, apertureSize=3, L2gradient=True)
        canny_all = cv2.Canny(mask, 50, 150, apertureSize=3, L2gradient=True)

        lines = cv2.HoughLines(canny_yellow, 1, np.pi/180, 100)

        inclinations = []
        points = []
        # center = []
    
        if lines is not None:
            for i in range(len(lines)):
                for rho, theta in lines[i]:
                    if (np.deg2rad(10) < theta < np.deg2rad(90)) or (np.deg2rad(90) < theta < np.deg2rad(170)):
                        a = np.cos(theta)
                        b = np.sin(theta)

                        x0 = a*rho
                        y0 = b*rho
                        x1 = int(x0 + 2000*(-b))
                        y1 = int(y0 + 2000*(a))
                        x2 = int(x0 - 2000*(-b))
                        y2 = int(y0 - 2000*(a))                        
                        ok, pt1, pt2 = cv2.clipLine((0, 0, width, height), (x1, y1), (x2, y2))

                        inclination = (y1 - y2) / (x1 - x2)
                        inclinations.append(inclination)
                        points.append(pt1)
                        points.append(pt2)

                        # cv2.line(cv_rgb_image,(x1,y1),(x2,y2),(0,0,255),2)
                        cv2.line(img, pt1, pt2, (0,0,255), 2)

                        # cv2.line(cv_rgb_image, pt1, pt2, (255,0,0),2)
        if len(inclinations) >= 2:
            max_inclination_ind = np.argmax(inclinations)
            max_inclination = inclinations[max_inclination_ind]

            min_inclination_ind = np.argmin(inclinations)
            min_inclination = inclinations[min_inclination_ind]

            points = np.array(points).reshape(-1, 2, 2)

            max_point = points[max_inclination_ind]
            min_point = points[min_inclination_ind]

            cv2.line(img, max_point[0], max_point[1], (255,0,0), 2)
            cv2.line(img, min_point[0], min_point[1], (0,255,0), 2)
            
            left_x = np.min(min_point[:,0])
            right_x = np.max(max_point[:,0])
            
            
            self.lane_center_x = (left_x + right_x) // 2
            image_center_x = width // 2

            # PID controller
            kp = 0.002
            ki = 0.000
            kd = 0.0001
            kp_g = 0.01
            error_prev = self.error
            self.error = self.lane_center_x - image_center_x
            
            self.derror = (self.error - error_prev)
            self.ierror += self.error

            steering = -(kp * self.error + ki * self.ierror + kd * self.derror)
        
            cv2.line(img, (self.lane_center_x, height), (self.lane_center_x, height - 50), (0, 255, 0), 2)
            cv2.line(img, (image_center_x, height), (image_center_x, height - 50), (255, 255, 0), 2)
            cmd = Twist()
            
            if self.state == 'IDLE':
                cmd.angular.z = steering
                cmd.linear.x = 6.5 - 30.5 * math.pow(abs(steering),2)
            elif self.state == 'CHECK_RED':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            elif self.state == 'PASS_RED':
                cmd.linear.x = 4.5
                cmd.angular.z = steering
            elif self.state == 'CHECK_GREEN':
                
                alpha = 0.4
                beta = 0.4
                cmdlinearx_f = cmd.linear.x
                cmd.linear.x = 1.0 - 2.55 * math.pow(abs(steering),1)
                cmdlinearx_f = alpha * cmd.linear.x + (1.0-alpha) * cmdlinearx_f #low pass filter
                if cmdlinearx_f < 0:
                    cmd.linear.x = np.clip(cmdlinearx_f, -3.0, -2.9)
                elif cmdlinearx_f >= 0:
                    cmd.linear.x = np.clip(cmdlinearx_f, 2.6, 10000)
                cmdangularz_f = cmd.angular.z
                cmd.angular.z = 0.3 * steering -0.0* math.pow(self.duration_green,2) - kp_g * math.pow((20.5 - self.dist_green), 3)
                cmd.angular.z = beta * cmd.angular.z + (1-beta) * cmdangularz_f
                
                    
            elif self.state == 'PASS_GREEN':
                cmd.linear.x = 0.8
                cmd.angular.z = steering

            cmd.linear.x = np.clip(cmd.linear.x, -3.0, 10000)
            cmd.angular.z = np.clip(cmd.angular.z, -1.0, 1.0)

            #print(f"Error: {self.error}, Steering: {cmd.angular.z:.3f}, State: {self.state}")
            
            self.speed = cmd.linear.x
              
            self.cmd_vel_pub.publish(cmd)

    # car state judge
    def depth_callback_sync(self, rgb_msg, depth_msg):
        depth_rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8") # np.darray
        
        depth_raw_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough') # np.darray
        depth_normal = cv2.normalize(depth_raw_img, None, 0, 255, cv2.NORM_MINMAX)
        depth_normal = depth_normal.astype(np.uint8)
        #cv2.imshow("depth_rgb",depth_rgb_img)
        #cv2.imshow("depth_raw", depth_normal)
        
        ## red
        red_mask = self.get_color_mask(depth_rgb_img, 'red', 10)
        edges_red = self.get_edge_line(red_mask)
        self.dist_red = self.get_light_distance(depth_raw_img, 'red', edges_red, 40)
        self.status_red = self.get_distance2status(self.dist_red, 15)
        
        ## green
        green_mask = self.get_color_mask(depth_rgb_img, 'green', 30)
        edges_green = self.get_edge_line(green_mask)
        self.dist_green = self.get_light_distance(depth_raw_img, 'green', edges_green, 40)
        self.status_green = self.get_distance2status(self.dist_green, 11)
        # cv2.imshow("green_mask", green_mask)

        # car state switch
        if self.state == 'IDLE' and self.status_red == 'ON':
            self.state = 'CHECK_RED'
            #print("<RED DETECTED>")

        if self.state == 'IDLE' and self.status_green == 'ON':
            self.state = 'CHECK_GREEN'
            #print("<GREEN DETECTED>")
        cv2.waitKey(1)


    def image_callback(self, image_msg):
        cv_rgb_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8") # np.darray

        self.status2driving(cv_rgb_image)

        info_image = self.car_info_image(cv_rgb_image)
        info_image_msg = self.bridge.cv2_to_imgmsg(info_image, encoding="bgr8")
        self.auto_driving_pub.publish(info_image_msg)

        #cv2.imshow("Info image", info_image)
        cv2.waitKey(1)
    

    def depth_callback(self, depth_msg):
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough') # np.darray

    # car information on image
    def car_info_image(self, image):


        if (self.state == 'IDLE') or (self.state == 'PASS_GREEN') or (self.state == 'PASS_RED'):
            speed_text = f"Speed: {self.speed:.3f} m/s"
            cv2.putText(image, speed_text, (70, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (64, 166, 64), 2, cv2.LINE_AA) #current spped
        elif self.state == "CHECK_RED":
            stop_text = f"Stop elapsed: {self.duration_red:.3f} / {self.stop_elapsed_time:.3f} s"
            cv2.putText(image, stop_text, (70, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (64, 64, 166), 2, cv2.LINE_AA) #stop elapsed time
        elif self.state == "CHECK_GREEN":
            turning_text = (f"Turning elapsed: {self.duration_green:.3f} / {self.turning_elapsed_time:.3f} s, Speed: {self.speed:.3f} m/s")
            cv2.putText(image, turning_text, (70, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (64, 64, 166), 2, cv2.LINE_AA) #turning elapsed time & speed

        time_text = f"Elapsed time: {self.elapsed_time:.3f} s"
        cv2.putText(image, time_text, (image.shape[1]-350, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (64, 166, 64), 2, cv2.LINE_AA) #elapsed time

        return image



def main(args=None):
    rclpy.init(args=args)

    node = Depth_Image()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()    

    

if __name__ == '__main__':
    main()
    

