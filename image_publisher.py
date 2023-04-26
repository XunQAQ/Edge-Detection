import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert the image to grayscale and apply edge detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 200)

        # Determine if there are any edges in the center of the image
        height, width = edges.shape
        center = int(width / 2)
        center_slice = edges[:, center-10:center+10]
        has_edge = np.any(center_slice)

        if has_edge:
            # If there is an edge, turn the robot 90 degrees and continue forward
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 1.57 # 90 degrees in radians
            self.publisher_.publish(twist)
        else:
            # Otherwise, continue forward
            twist = Twist()
            twist.linear.x = 0.5
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
