#!/usr/bin/env python3
"""
Test script for path planner module.
Author: Amrith David
"""
import os
import sys
import matplotlib.pyplot as plt
import cv2
import numpy as np

# Add the source directory to the path
sys.path.append('src')

# Import the path planner
from tool_path_planning.path_planner import PathPlanner

def create_sample_image(): # creates 500x500 test image
    """Create a face for demonstration"""
    img = np.zeros((500, 500), dtype=np.uint8)
    
    # Draw a simple face with thicker lines to ensure contour detection
    cv2.circle(img, (250, 250), 150, 255, 2)  # Face outline
    cv2.circle(img, (200, 200), 30, 255, 2)   # Left eye
    cv2.circle(img, (300, 200), 30, 255, 2)   # Right eye
    cv2.ellipse(img, (250, 280), (80, 70), 0, 0, 180, 255, 2)  # Smile
    
    # Add some processing to make the edges more detectable
    kernel = np.ones((3, 3), np.uint8)
    img = cv2.dilate(img, kernel, iterations=1)
    
    return img

def test_path_extraction():
    """Test vector path extraction functionality"""
    print("\nTesting Vector Path Extraction")
    print("-----------------------------")
    
    # Create a path planner instance
    planner = PathPlanner()
    
    # Create a sample image
    image = create_sample_image()
    
    # Create output directory
    output_dir = 'output'
    os.makedirs(output_dir, exist_ok=True)
    
    # Save the sample image
    cv2.imwrite(os.path.join(output_dir, 'sample_image.png'), image)
    
    # Visualize the contours directly using OpenCV for debugging
    debug_image = cv2.cvtColor(image.copy(), cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(debug_image, contours, -1, (0, 255, 0), 1)
    cv2.imwrite(os.path.join(output_dir, 'debug_contours.png'), debug_image)
    print(f"Debug contour image saved to {output_dir}/debug_contours.png")
    
    # Extract paths
    paths = planner.extract_paths_from_image(image)
    print(f"Extracted {len(paths)} paths")
    
    # Create and save a visualization of the paths
    fig = planner.visualize_paths(show_optimisation=False)
    if fig:
        fig.savefig(os.path.join(output_dir, 'extracted_paths.png'))
        plt.close(fig)
        print(f"Visualization saved to {output_dir}/extracted_paths.png")
    
    # Print some statistics
    if paths:
        total_points = sum(len(path) for path in paths)
        avg_points = total_points / len(paths)
        print(f"Total points: {total_points}")
        print(f"Average points per path: {avg_points:.1f}")
    
    return paths

def test_with_real_edge_image(image_path):
    """Test vector path extraction with a real edge image"""
    print("\nTesting Vector Path Extraction with Real Edge Image")
    print("--------------------------------------------------")
    
    # Create a path planner instance
    planner = PathPlanner()
    
    # Load the edge image
    print(f"Loading image from {image_path}")
    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    if image is None:
        print(f"Error: Could not load image from {image_path}")
        return None
        
    # Ensure it's a binary image
    _, image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
    
    # Create output directory
    output_dir = 'output'
    os.makedirs(output_dir, exist_ok=True)
    
    # Save the input image
    cv2.imwrite(os.path.join(output_dir, 'input_image.png'), image)
    
    # Visualize the contours directly using OpenCV for debugging
    debug_image = cv2.cvtColor(image.copy(), cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(debug_image, contours, -1, (0, 255, 0), 1)
    cv2.imwrite(os.path.join(output_dir, 'debug_contours_real.png'), debug_image)
    print(f"Debug contour image saved to {output_dir}/debug_contours_real.png")
    
    # Extract paths
    paths = planner.extract_paths_from_image(image)
    print(f"Extracted {len(paths)} paths")
    
    # Create and save a visualization of the paths
    fig = planner.visualize_paths(show_optimisation=False)
    if fig:
        fig.savefig(os.path.join(output_dir, 'extracted_paths_real.png'))
        plt.close(fig)
        print(f"Visualization saved to {output_dir}/extracted_paths_real.png")
    
    # Print some statistics
    if paths:
        total_points = sum(len(path) for path in paths)
        avg_points = total_points / len(paths)
        print(f"Total points: {total_points}")
        print(f"Average points per path: {avg_points:.1f}")
    
    return paths

def test_coordinate_transformation():
    """Test coordinate transformation functionality"""
    print("\nTesting Coordinate Transformation")
    print("-------------------------------")
    
    # Create a path planner instance
    planner = PathPlanner()
    
    # Load or create a sample image
    if os.path.exists("sample_edge_image.png"):
        image = cv2.imread("sample_edge_image.png", cv2.IMREAD_GRAYSCALE)
        _, image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
    else:
        print("Error: Sample edge image not found")
        return None
    
    # Extract paths
    paths = planner.extract_paths_from_image(image)
    print(f"Extracted {len(paths)} paths")
    
    # Load canvas corners from the YAML file
    yaml_path = "ur3_localisation/config/params.yaml"
    
    if not os.path.exists(yaml_path):
        print(f"Error: YAML file not found at {yaml_path}")
        return None
        
    print(f"Loading canvas corners from {yaml_path}")
    try:
        import yaml
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            
        if not data or 'corner_positions' not in data or len(data['corner_positions']) != 4:
            print("Error: YAML file does not contain proper corner data")
            return None
            
        canvas_corners = []
        for corner in data['corner_positions']:
            canvas_corners.append({
                'x': corner['x'],
                'y': corner['y'],
                'z': corner['z']
            })
        print("Successfully loaded canvas corners from YAML file")
    except Exception as e:
        print(f"Error loading YAML file: {e}")
        return None
    
    # Save original paths for comparison
    original_paths = []
    for path in planner.paths:
        original_paths.append(path.copy())
    
    # Create output directory
    output_dir = 'output'
    os.makedirs(output_dir, exist_ok=True)
    
    # Save visualization of original paths
    fig_before = planner.visualize_paths(show_optimisation=False)
    if fig_before:
        fig_before.savefig(os.path.join(output_dir, 'paths_before_transform.png'))
        plt.close(fig_before)
        print(f"Original paths saved to {output_dir}/paths_before_transform.png")
    
    # Transform coordinates
    transformed_paths = planner.transform_coordinates_with_corners(canvas_corners)
    
    # Save visualization of transformed paths
    fig_after = planner.visualize_paths(show_optimisation=False)
    if fig_after:
        fig_after.savefig(os.path.join(output_dir, 'paths_after_transform.png'))
        plt.close(fig_after)
        print(f"Transformed paths saved to {output_dir}/paths_after_transform.png")
    
    # Print canvas corner coordinates
    print("\nCanvas corners used for transformation:")
    for i, corner in enumerate(canvas_corners):
        label = ["BL", "BR", "TR", "TL"][i]
        print(f"{label}: ({corner['x']:.3f}, {corner['y']:.3f}, {corner['z']:.3f})")
    
    # Verify transformation results
    # Check if paths are within canvas boundaries
    if transformed_paths:
        all_x = [point[0] for path in transformed_paths for point in path]
        all_y = [point[1] for path in transformed_paths for point in path]
        
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)
        
        canvas_min_x = min(c['x'] for c in canvas_corners)
        canvas_max_x = max(c['x'] for c in canvas_corners)
        canvas_min_y = min(c['y'] for c in canvas_corners)
        canvas_max_y = max(c['y'] for c in canvas_corners)
        
        x_margin = 29.5 / 1000.0  # m
        y_margin = 21.0 / 1000.0  # m
        
        expected_min_x = canvas_min_x + x_margin
        expected_max_x = canvas_max_x - x_margin
        expected_min_y = canvas_min_y + y_margin
        expected_max_y = canvas_max_y - y_margin
        
        print(f"\nTransformed bounds: X: {min_x:.3f} to {max_x:.3f}, Y: {min_y:.3f} to {max_y:.3f}")
        print(f"Expected bounds with margins: X: {expected_min_x:.3f} to {expected_max_x:.3f}, Y: {expected_min_y:.3f} to {expected_max_y:.3f}")
        
        # Check if aspect ratio is maintained
        original_aspect_ratio = (max(p[0] for path in original_paths for p in path) - min(p[0] for path in original_paths for p in path)) / \
                               (max(p[1] for path in original_paths for p in path) - min(p[1] for path in original_paths for p in path))
                               
        transformed_aspect_ratio = (max_x - min_x) / (max_y - min_y)
        
        print(f"Original aspect ratio: {original_aspect_ratio:.3f}")
        print(f"Transformed aspect ratio: {transformed_aspect_ratio:.3f}")
        print(f"Aspect ratio difference: {abs(original_aspect_ratio - transformed_aspect_ratio):.3f}")
        
        # Check pass criteria
        if (min_x >= expected_min_x and max_x <= expected_max_x and 
            min_y >= expected_min_y and max_y <= expected_max_y):
            print("✓ Drawing fits within canvas boundaries with margins")
        else:
            print("✗ Drawing exceeds canvas boundaries with margins")
        
        if abs(original_aspect_ratio - transformed_aspect_ratio) < 1:
            print("✓ Aspect ratio is maintained")
        else:
            print("✗ Aspect ratio is not maintained")
    
    return transformed_paths

def test_path_optimisation():
    """Test path optimisation functionality"""
    print("\nTesting Path Optimisation")
    print("------------------------")
    
    # Create a path planner instance
    planner = PathPlanner()
    
    # Load or create a test image
    if os.path.exists("sample_edge_image.png"):
        image = cv2.imread("sample_edge_image.png", cv2.IMREAD_GRAYSCALE)
        _, image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
    else:
        image = create_sample_image()
    
    # Extract paths
    planner.extract_paths_from_image(image)
    print(f"Extracted {len(planner.paths)} paths")
    
    # Save visualisation of unoptimised paths
    output_dir = 'output'
    os.makedirs(output_dir, exist_ok=True)
    
    fig_before = planner.visualize_paths(show_optimisation=False)
    if fig_before:
        fig_before.savefig(os.path.join(output_dir, 'paths_before_optimisation.png'))
        plt.close(fig_before)
        print(f"Original paths saved to {output_dir}/paths_before_optimisation.png")
    
    # Run optimisation
    planner.optimise_paths()
    
    # Save visualisation of optimised paths
    fig_after = planner.visualize_paths(show_optimisation=True)
    if fig_after:
        fig_after.savefig(os.path.join(output_dir, 'paths_after_optimisation.png'))
        plt.close(fig_after)
        print(f"Optimised paths saved to {output_dir}/paths_after_optimisation.png")
    
    return planner.paths

def main():
    print("Testing Path Planner")
    print("===================")
    
    # Path to the real edge image
    real_image_path = "sample_edge_image.png"
    
    # Test path extraction
    if os.path.exists(real_image_path):
        print("\n--- TESTING PATH EXTRACTION ---")
        paths = test_with_real_edge_image(real_image_path)
    else:
        print(f"Real image not found at {real_image_path}")
        print("Testing with sample image instead...")
        paths = test_path_extraction()
        
    # Test coordinate transformation
    print("\n--- TESTING COORDINATE TRANSFORMATION ---")
    transformed_paths = test_coordinate_transformation()

    # Test path optimisation
    print("\n--- TESTING PATH OPTIMISATION ---")
    optimised_paths = test_path_optimisation()
    
    print("\nTest complete!")

if __name__ == "__main__":
    main()
