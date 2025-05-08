from rclpy.action import ActionClient
from img_prc_interface.action import Img
from sensor_msgs.msg import Image
import rclpy
import cv2
from cv_bridge import CvBridge

class ImgClient:
    def __init__(self):
        self.node = rclpy.create_node('edge_detection_client')
        self.client = ActionClient(self.node, Img, 'process_edge_image')
        self.bridge = CvBridge()  # Convert ROS Image messages to OpenCV format
        self.subscription = self.node.create_subscription(
            Image,
            'webcam_image',
            self.listener_callback,
            10)
        
        # Timer to send a goal every 5 seconds
        self.timer = self.node.create_timer(5.0, self.send_goal_timer_callback)
        self.latest_image = None
        
    def send_goal(self):
        if self.latest_image is not None:
            goal_msg = Img.Goal()
            goal_msg.image = self.latest_image  # Set the latest image in the goal message

            # Send goal to the action server asynchronously
            send_goal_future = self.client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            
            # When the goal is completed, the result is handled by add_done_callback
            send_goal_future.add_done_callback(self.result_callback)
        else:
            print("No image received yet.")

    def send_goal_timer_callback(self):
        print("Sending goal to action server.")
        self.send_goal()

    def feedback_callback(self, feedback_msg):
        print(f"Feedback: {feedback_msg.status}")

    def result_callback(self, future):
        goal_handle = future.result()

        # Check if the goal was successfully accepted and completed
        if goal_handle is not None:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(self.get_result_callback)
        else:
            print("Goal handle was not received.")

    def get_result_callback(self, future):
        result = future.result().result  # Get the result from the future
        if result:
            print("Processing result received.")
            processed_image_msg = result.processed_image  # Access the processed image
            
            try:
                # Convert the processed image from ROS Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(processed_image_msg, desired_encoding='bgr8')
                cv2.imshow('client Image', cv_image)
                cv2.waitKey(1)
            except Exception as e:
                print(f"Error converting image: {e}")
        else:
            print("The result was empty, action failed or was preempted.")

    def wait_for_action_server(self):
        # Wait for the action server to be available
        while not self.client.wait_for_server(timeout_sec=1.0):
            print("Waiting for action server to become available...")

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        self.latest_image = msg

def main(args=None):
    rclpy.init(args=args)
    edge_detection_client = ImgClient()
    
    # Wait for the action server to be available
    edge_detection_client.wait_for_action_server()
    
    print("Action client is now ready and sending requests every 5 seconds.")
    
    # Start spinning the node to process the timer callbacks
    rclpy.spin(edge_detection_client.node)
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
