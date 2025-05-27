#!/usr/bin/env python3
"""
Test script for Path Planner - uses saved edge images instead of webcam
Author: Amrith David
"""
import cv2
import os
import sys
import json

# Add the path to import your path planner
sys.path.append('/home/amrithdavid/ros2_ws/src/DalESelfEBot/src/tool_path_planning/tool_path_planning')
from path_planner import PathPlanner

class PathPlannerTester:
    """Test class for path planner using saved images"""
    
    def __init__(self):
        """Initialize the tester"""
        self.path_planner = PathPlanner()
        
        # Define test images directory
        self.test_images_dir = os.path.join(
            os.path.dirname(__file__), 
            'test_images'
        )
        
        # Mock canvas corners (you can adjust these based on your setup)
        self.mock_canvas_corners = [
            {'x': 0.4, 'y': -0.2, 'z': 0.1},    # Bottom Left
            {'x': 0.6, 'y': -0.2, 'z': 0.1},    # Bottom Right  
            {'x': 0.6, 'y': 0.0, 'z': 0.1},     # Top Right
            {'x': 0.4, 'y': 0.0, 'z': 0.1}      # Top Left
        ]
    
    def load_test_image(self, image_name):
        """Load a test image from the test_images directory"""
        image_path = os.path.join(self.test_images_dir, image_name)
        
        if not os.path.exists(image_path):
            print(f"Error: Test image not found at {image_path}")
            return None
            
        # Load image in grayscale
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        
        if image is None:
            print(f"Error: Could not load image {image_path}")
            return None
            
        print(f"Successfully loaded test image: {image_name}")
        print(f"Image dimensions: {image.shape}")
        
        return image
    
    def process_test_image(self, image_name):
        """Process a single test image through the complete pipeline"""
        print(f"\n{'='*50}")
        print(f"Processing test image: {image_name}")
        print(f"{'='*50}")
        
        # 1. Load the test image
        image = self.load_test_image(image_name)
        if image is None:
            return None
            
        # 2. Extract paths from the image
        print("\n1. Extracting vector paths...")
        paths = self.path_planner.extract_paths_from_image(image)
        print(f"   Extracted {len(paths)} paths")
        
        # 3. Transform coordinates using mock canvas corners
        print("\n2. Transforming coordinates to robot workspace...")
        transformed_paths = self.path_planner.transform_coordinates_with_corners(
            self.mock_canvas_corners
        )
        print(f"   Transformed {len(transformed_paths)} paths")
        
        # 4. Optimize paths
        print("\n3. Optimizing path sequence...")
        optimized_paths = self.path_planner.optimise_paths()
        print(f"   Optimized {len(optimized_paths)} paths")
        
        # 5. Generate toolpath JSON (same format as your ROS node)
        print("\n4. Generating toolpath JSON...")
        splines = []
        for i, path in enumerate(optimized_paths):
            if not path:
                continue
                
            waypoints = []
            for point in path:
                waypoints.append([point[0], point[1]])
            
            splines.append({
                "id": i,
                "waypoints": waypoints
            })
        
        toolpath_data = {
            "splines": splines
        }
        
        print(f"   Generated {len(splines)} splines")
        
        # 6. Save results
        output_file = f"toolpath_{image_name.split('.')[0]}.json"
        with open(output_file, 'w') as f:
            json.dump(toolpath_data, f, indent=2)
        
        print(f"   Saved toolpath to: {output_file}")
        
        # 7. Create visualization
        print("\n5. Creating visualization...")
        try:
            fig = self.path_planner.visualize_paths(show_optimisation=True)
            if fig:
                viz_file = f"visualization_{image_name.split('.')[0]}.png"
                fig.savefig(viz_file, dpi=150, bbox_inches='tight')
                print(f"   Saved visualization to: {viz_file}")
                
                # Show the plot
                import matplotlib.pyplot as plt
                plt.show()
        except Exception as e:
            print(f"   Visualization error: {e}")
        
        return toolpath_data
    
    def test_all_images(self):
        """Process all test images in the test_images directory"""
        if not os.path.exists(self.test_images_dir):
            print(f"Error: Test images directory not found: {self.test_images_dir}")
            print("Please create the directory and add your edge images.")
            return
        
        # Get all image files
        image_files = [f for f in os.listdir(self.test_images_dir) 
                      if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
        
        if not image_files:
            print(f"No image files found in {self.test_images_dir}")
            return
        
        print(f"Found {len(image_files)} test images: {image_files}")
        
        # Process each image
        results = {}
        for image_file in image_files:
            result = self.process_test_image(image_file)
            if result:
                results[image_file] = result
        
        print(f"\n{'='*50}")
        print("TESTING COMPLETE")
        print(f"{'='*50}")
        print(f"Successfully processed {len(results)} images")
        
        return results

def main():
    """Main function"""
    print("Path Planner Tester")
    print("This script tests your path planning subsystem using saved edge images")
    print("=" * 60)
    
    tester = PathPlannerTester()
    
    # Check if test images directory exists
    if not os.path.exists(tester.test_images_dir):
        print(f"Creating test images directory: {tester.test_images_dir}")
        os.makedirs(tester.test_images_dir)
        print("Please add your edge images to this directory and run the script again.")
        return
    
    # Test all images
    results = tester.test_all_images()
    
    if results:
        print("\nTesting completed successfully!")
        print("Check the generated JSON files and visualizations.")
    else:
        print("\nNo images were processed. Please check your test images.")

if __name__ == '__main__':
    main()
