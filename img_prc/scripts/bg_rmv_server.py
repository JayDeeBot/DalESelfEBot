from rclpy.action import ActionServer
from img_prc_interface.action import Img
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
from rembg import remove
from PIL import Image as PILImage
import numpy as np
import img_prc.image_processing as image_processing

class BackgroundRemovalServer(Node):
    def __init__(self):
        super().__init__('background_removal_server')
        self._action_server = ActionServer(
            self,
            Img,
            'process_edge_image',
            self.process_image_callback)
        self.bridge = CvBridge()

    def process_image_callback(self, goal_handle):
        # Process image once goal is received and execute background removal
        self.get_logger().info('Processing background removal...')
        
        # Get the image from the goal message
        image_msg = goal_handle.request.image
        
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            
            # Convert the frame to PIL image
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

            # Remove the background
            output_image = remove(pil_image)

            # Convert the output image back to OpenCV format
            output_image = cv2.cvtColor(np.array(output_image), cv2.COLOR_RGB2BGR)

            output_image = image_processing.face_edge_detection(output_image)  # Turn into edge image
            # Prepare image message
            msg = self.bridge.cv2_to_imgmsg(output_image, encoding='mono8')

            # Send processed image back as result
            goal_handle.succeed()
            result = Img.Result()
            result.processed_image = msg
            self.get_logger().info('Done background removal...')
            return result
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            goal_handle.abort()
            return Img.Result()

def main(args=None):
    rclpy.init(args=args)
    node = BackgroundRemovalServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
