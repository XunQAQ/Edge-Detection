import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class HandPublisher(Node):

    def __init__(self):
        super().__init__('hand_publisher')
        self.subscription = self.create_subscription(
            Image,
            'video_frames',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.found_red = False

    def listener_callback(self, msg):
        angle = 0.0
        linear = 0.0

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # converting BGR to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([5,50,50])
        upper_red = np.array([15,255,255])
      
        # create a red HSV colour boundary and
        # threshold HSV image
        mask = cv2.inRange(hsv, lower_red, upper_red)
  
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image,image, mask= mask)
        
        # finds edges in the input image image and
        # marks them in the output map edges
        edges = cv2.Canny(image,100,200)
        
        

        # Determine if red edge is found
        if cv2.countNonZero(edges) >  0:
            self.found_red = True
        else:
            self.found_red = False


        if self.found_red:
            self.get_logger().info('red')
            linear = 0.0
            self.get_logger().info('right')
            angle = 0.5
#            self.direction *= -1  # Reverse direction
        else:
            self.get_logger().info('no red')
            linear = 0.5
            angle = 0.0

        # Move robot and turn if red edge is found
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angle

        # Publish twist message
        if self.vel_publisher.get_subscription_count() > 0:
            self.vel_publisher.publish(twist)
        else:
            self.get_logger().info('waiting for subcriber')#add something


def main(args=None):
    rclpy.init(args=args)
    hand_publisher = HandPublisher()
    rclpy.spin(hand_publisher)
    hand_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

