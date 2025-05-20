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

# Import path planner class
from .path_planner import PathPlanner

class ToolPathPlannerNode(Node): # defines ROS2 node class ToolPathPlannerNode that inherits from Node class
    def __init__(self): # initialises node with name tool_path_planner
        super().__init__('tool_path_planner')
        
        # sets up CvBridge instance to covert between ROS images and OpenCV format
        self.bridge = CvBridge()
        
        # Create a publisher for sending toolpath data to /toolpath topic
        self.toolpath_publisher = self.create_publisher(String, '/toolpath', 10)
        
        # Subscribes to recieve edge images from pixel map topic
        self.subscription = self.create_subscription(
            Image,
            '/pixel_map',
            self.pixel_map_callback,
            10)
        
        # defines path to localisation YAML using home directory
        home_dir = os.path.expanduser('~')
        self.yaml_path = os.path.join(home_dir, 'ros2_ws', 'src', 'ur3_localisation', 'config', 'params.yaml')
        
        # sets up Canvas margins constants (inward from edges) (specified by jarred)
        self.x_margin_mm = 29.5  # mm inward from x-axis edges
        self.y_margin_mm = 21.0  # mm inward from y-axis edges
        
        # Initialises a path planner instance
        self.path_planner = PathPlanner()
        
        # Initialise canvas corners 
        self.canvas_corners = None
        
        self.get_logger().info('Tool Path Planner Node initialized')
    
    def pixel_map_callback(self, msg): # method called when an image is recieved 
        """Minimal callback for received pixel map (edge image)"""
        self.get_logger().info('Received pixel map (stub implementation)')
        
        # real implementation:
        # 1. Convert ROS Image to OpenCV
        # 2. Load canvas corners if needed
        # 3. Process the image through our path planner
        # 4. Generate and publish toolpath

def main(args=None):
    rclpy.init(args=args) # sets up ros2 initialisation
    
    node = ToolPathPlannerNode() # creates an instance of the node
    
    rclpy.spin(node) # starts ros2 event loop
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()