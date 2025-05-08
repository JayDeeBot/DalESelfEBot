import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import img_prc.image_processing as image_processing

class EdgeDetectionProcessor(Node):
    def __init__(self):
        super().__init__('edge_detection_processor')
        self.subscription = self.create_subscription(
            Image,
            'webcam_image',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'edge_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        edges = image_processing.face_edge_detection(cv_image)
        edge_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
        self.publisher_.publish(edge_msg)
        

        

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
