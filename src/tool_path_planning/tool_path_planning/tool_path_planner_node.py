#!/usr/bin/env python3
"""
ROS2 Node for Tool Path Planning in the Selfie-Drawing Robot project
Author: Amrith David
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import json
import yaml
import os
import cv2

# Import path planner class
from .path_planner import PathPlanner

class ToolPathPlannerNode(Node):
    """ROS2 node for tool path planning in the Selfie-Drawing Robot project"""
    
    def __init__(self):
        """Initialise the node"""
        super().__init__('tool_path_planner')
        
        # Set up CvBridge instance to convert between ROS images and OpenCV format
        self.bridge = CvBridge()
        
        # Create a publisher for sending toolpath data to /toolpath topic
        self.toolpath_publisher = self.create_publisher(String, '/toolpath', 10)
        
        # Subscribe to receive edge images from pixel map topic
        self.subscription = self.create_subscription(
            Image,
            '/pixel_map',
            self.pixel_map_callback,
            10)
        
        # Define path to localisation YAML using home directory
        home_dir = os.path.expanduser('~')
        self.yaml_path = os.path.join(home_dir, 'ros2_ws', 'src', 'DalESelfEBot', 'ur3_localisation', 'config', 'params.yaml')
        
        # Set up Canvas margins constants (inward from edges) (specified by Jarred)
        self.x_margin_mm = 73.75*1.05  # mm inward from x-axis edges
        self.y_margin_mm = 52.5*1.05  # mm inward from y-axis edges
        
        # Initialise a path planner instance
        self.path_planner = PathPlanner()
        
        # Initialise canvas corners
        self.canvas_corners = None
        
        self.get_logger().info('Tool Path Planner Node initialised')
    
    def load_canvas_corners(self):
        """Load canvas corner positions from YAML file"""
        try:
            if not os.path.exists(self.yaml_path):
                self.get_logger().error(f'Canvas corners YAML file not found at {self.yaml_path}')
                return False
                
            with open(self.yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                
            if not data or 'corner_positions' not in data or len(data['corner_positions']) != 4:
                self.get_logger().error('YAML file does not contain proper corner data')
                return False
                
            self.canvas_corners = []
            for corner in data['corner_positions']:
                self.canvas_corners.append({
                    'x': corner['x'],
                    'y': corner['y'],
                    'z': corner['z']
                })
            
            self.get_logger().info('Successfully loaded canvas corners from YAML file')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error loading canvas corners: {str(e)}')
            return False
    
    def pixel_map_callback(self, msg):
        """Process received pixel map (edge image) from the GUI"""
        self.get_logger().info('Received pixel map from GUI, processing...')
        
        try:
            # 1. Convert ROS Image to OpenCV format
            # Use 'mono8' encoding to match what's published by the GUI
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            
            # Make sure it's a binary image
            _, binary_image = cv2.threshold(cv_image, 127, 255, cv2.THRESH_BINARY)
            
            # 2. Process through our path planner
            self.get_logger().info('Extracting vector paths from edge image...')
            paths = self.path_planner.extract_paths_from_image(binary_image)
            self.get_logger().info(f'Extracted {len(paths)} vector paths')
            
            # 3. Load canvas corners if needed
            if not self.canvas_corners:
                self.get_logger().info('Loading canvas corners for coordinate transformation...')
                if not self.load_canvas_corners():
                    self.get_logger().error('Failed to load canvas corners, cannot transform coordinates')
                    return
            
            # 4. Transform coordinates to robot workspace
            self.get_logger().info('Transforming coordinates to robot workspace...')
            transformed_paths = self.path_planner.transform_coordinates_with_corners(self.canvas_corners)
            self.get_logger().info('Coordinate transformation complete')
            
            # 5. Optimize paths to minimize drawing time
            self.get_logger().info('Optimising paths to minimise drawing time...')
            optimized_paths = self.path_planner.optimise_paths()
            self.get_logger().info('Path optimisation complete')
            
            # 6. Format paths as JSON for the control subsystem
            # Structure as per Jarred's requirements:
            # - splines with sequential IDs (0, 1, 2, etc.)
            # - Each spline has waypoints with [x, y] coordinates (z is handled by control)
            
            splines = []
            for i, path in enumerate(optimized_paths):
                if not path:  # Skip empty paths
                    continue
                    
                waypoints = []
                for point in path:
                    # Only include x, y (z is handled by control)
                    waypoints.append([point[0], point[1]])
                
                # Add the spline with sequential ID
                splines.append({
                    "id": i,  # Sequential IDs starting from 0 as requested
                    "waypoints": waypoints
                })
            
            # Create the complete toolpath structure
            toolpath_data = {
                "splines": splines
            }
            
            # Convert to JSON string
            toolpath_json = json.dumps(toolpath_data)
            
            # Publish to /toolpath topic
            msg = String()
            msg.data = toolpath_json
            self.toolpath_publisher.publish(msg)
            
            self.get_logger().info(f'Published toolpath with {len(splines)} splines to /toolpath')
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)  # Sets up ros2 initialisation
    
    node = ToolPathPlannerNode()  # Creates an instance of the node
    
    rclpy.spin(node)  # Starts ros2 event loop
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()