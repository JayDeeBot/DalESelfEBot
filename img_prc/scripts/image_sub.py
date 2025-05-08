import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription1 = self.create_subscription(
            Image,
            'webcam_image',
            self.listener_callback1,
            10)
        
        self.subscription2 = self.create_subscription(
            Image,
            'edge_image',
            self.listener_callback2,
            10)
        self.subscription1  
        self.bridge = CvBridge()

    def listener_callback1(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        # Display the image
        cv2.imshow('Received Image1', cv_image)
        cv2.waitKey(1)

    def listener_callback2(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        
        # Display the image
        cv2.imshow('Received Image2', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
