import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.action import ActionServer
from img_prc_interface.action import Img  # Import action definition, change this during integration
from rembg import remove
from PIL import Image as PILImage
import sys

def edge_detection_color(image, lower, upper):
    # Split the image into its color channels
    b_channel, g_channel, r_channel = cv2.split(image)
    
    # Apply Canny edge detection to each channel
    edges_b = cv2.Canny(b_channel, lower, upper)
    edges_g = cv2.Canny(g_channel, lower, upper)
    edges_r = cv2.Canny(r_channel, lower, upper)
    
    # Combine the edges from each channel
    combined_edges = cv2.bitwise_or(edges_b, edges_g)
    combined_edges = cv2.bitwise_or(combined_edges, edges_r)
    
    return combined_edges

def clean_edges(edge_image, min_edge_length=10):
    """
    Remove short, disconnected edges from the edge-detected image using connected component analysis.
    
    Parameters:
    - edge_image: Input edge-detected image.
    - min_edge_length: Minimum length of edges to keep.
    
    Returns:
    - cleaned_edges: Edge-detected image with short edges removed.
    """
    # Ensure the edge image is single-channel (grayscale)
    if len(edge_image.shape) > 2:
        edge_image = cv2.cvtColor(edge_image, cv2.COLOR_BGR2GRAY)

    # Find all connected components (edges)
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(edge_image, connectivity=8)
    
    # Create an output image to store the cleaned edges
    cleaned_edges = np.zeros_like(edge_image)
    
    # Iterate through all detected components
    for i in range(1, num_labels):  # Start from 1 to skip the background
        if stats[i, cv2.CC_STAT_AREA] >= min_edge_length:
            cleaned_edges[labels == i] = 255
    
    return cleaned_edges

def clarify(image, alpha=1.2, beta=50):
    """
    Brighten and clarify the input image.
    
    Parameters:
    - image: Input image.
    - alpha: Contrast control (1.0-3.0).
    - beta: Brightness control (0-100).
    
    Returns:
    - result: Brightened and clarified image.
    """
    # Adjust brightness and contrast
    brightened = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    
    # Apply histogram equalization
    #equalized = cv2.equalizeHist(brightened)
    
    # Convert back to BGR
    result = brightened
    
    return result

class ImageProcessor(Node):
    def __init__(self, camera_index):
        self.window_width =  500
        self.window_height = 500
        super().__init__('image_processor')
        self.edge_publisher_ = self.create_publisher(Image, 'edge_image', 10)
        self.cam_publisher = self.create_publisher(Image, 'webcam_image', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(camera_index)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self._action_server = ActionServer(self, Img, 'process_edge_image', self.process_image_callback)

        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

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

            output_image = self.face_edge_detection(output_image)  # Turn into edge image
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
        
    def face_edge_detection(self, frame):
        edges_head = edge_detection_color(frame, 230, 240)
        frame = clarify(frame, 1.2, 0)
        # cv2.imshow("gray", edges_head)
        # Detect faces in the image
        faces = self.face_cascade.detectMultiScale(frame, scaleFactor=1.05, minNeighbors=3, minSize=(int(self.window_width/6), int((self.window_height)/6)))
        
        # Process each detected face
        for (x, y, w, h) in faces:
            # Calculate the new height (1.3 times the original height)
            h2 = int(h * 0.8)
            # Adjust the y-coordinate to keep the region within the image boundaries
            y2 = max(0, y - (h2 - h) // 2)
            # Ensure the new height does not exceed the image boundaries
            h2 = min(h2, frame.shape[0] - y2)
            y2 = y
            h2 = h
            # Extract the face region with the new height
            face_region = frame[y2:y2+h2, x:x+w]
            # cv2.imshow("face", face_region)
            edges = edge_detection_color(face_region, 160, 200)       
            edges_head[y2:y2+h2, x:x+w] = edges

            # Detect eyes in the face region
            eyes = self.eye_cascade.detectMultiScale(face_region, scaleFactor=1.10, minNeighbors=3)
            for (ex, ey, ew, eh) in eyes:
                eye_region = clarify(face_region[ey:ey+eh, ex:ex+ew], 1.3, 20)
                # cv2.imshow("eyes", eye_region)
                eye_edges = edge_detection_color(eye_region, 150, 160)
                edges_head[y2+ey:y2+ey+eh, x+ex:x+ex+ew] = eye_edges
        
        edges_head = clean_edges(edges_head, int(self.window_height/30))
        edges_head = cv2.resize(edges_head, (500, 500))
        return edges_head

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        # Resize the frame to improve processing speed
        frame = cv2.resize(frame, (self.window_width, self.window_height))
        # cv2.imshow("webcam", frame)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.cam_publisher.publish(msg)
        
        edges_head = self.face_edge_detection(frame)
        msg = self.bridge.cv2_to_imgmsg(edges_head, encoding='mono8')
        self.edge_publisher_.publish(msg)  # Publish the processed image
        
        # Display the resulting frame
        cv2.imshow('Face and Eye Detection with Edges', edges_head)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    # Get the camera index from command line arguments
    camera_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    
    image_processor = ImageProcessor(camera_index)
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
