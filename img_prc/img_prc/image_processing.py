
import numpy as np
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.action import ActionServer
from img_prc_interface.action import Img  # Import action definition, change this during integration
from PIL import Image as PILImage
import cv2.data

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
window_width =  500
window_height = 500


def face_edge_detection(frame):
    edges_head = edge_detection_color(frame, 230, 240)
    frame = clarify(frame, 1.2, 0)
    # cv2.imshow("gray", edges_head)
    # Detect faces in the image
    faces = face_cascade.detectMultiScale(frame, scaleFactor=1.05, minNeighbors=3, minSize=(int(window_width/6), int((window_height)/6)))
    
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
        eyes = eye_cascade.detectMultiScale(face_region, scaleFactor=1.10, minNeighbors=3)
        for (ex, ey, ew, eh) in eyes:
            eye_region = clarify(face_region[ey:ey+eh, ex:ex+ew], 1.3, 20)
            # cv2.imshow("eyes", eye_region)
            eye_edges = edge_detection_color(eye_region, 150, 160)
            edges_head[y2+ey:y2+ey+eh, x+ex:x+ex+ew] = eye_edges
    
    edges_head = clean_edges(edges_head, int(window_height/30))
    edges_head = cv2.resize(edges_head, (500, 500))
    return edges_head

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