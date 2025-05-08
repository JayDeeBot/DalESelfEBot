#!/usr/bin/env python3
"""
Path Planner module for the Selfie-Drawing Robot
Author: Amrith David
"""
import cv2 # for image processing
import numpy as np # for numerical operations

class PathPlanner: # handles path planning functionality
    """Tool path planning class"""
    
    def __init__(self): # method initialises class with empty paths list
        """Initialise the path planner"""
        self.paths = []
        self.original_image = None
    
    def extract_paths_from_image(self, image): # processes edge images to extract vector paths
        """Extract vector paths from an edge image"""
        # Store the original image for reference (used for visualisation)
        self.original_image = image.copy()
        
        # Make sure we're working with a proper binary image by converting image to strict binary image
        # any pixel below 127 becomes 0 (black) and any value above becomes 255 (white)
        _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
        
        # Debug: Print image properties
        print(f"Image shape: {binary_image.shape}")
        print(f"Image type: {binary_image.dtype}")
        print(f"Image min/max values: {np.min(binary_image)}/{np.max(binary_image)}")
        
        # using opencv's findContours to detect continuous edges in the image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        print(f"Number of raw contours found: {len(contours)}")
        
        # Convert contours to simple paths
        self.paths = [] # resets path list
        
        for contour in contours: # loops through all contours found
            # Skip very small contours (less than 10 points)
            if len(contour) < 10:
                continue
                
            # Simplify the contour using Douglas-Peucker algorithm
            # This reduces the number of points while preserving the shape
            epsilon = 0.002 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Only keep contours with enough points after simplification
            if len(approx) < 3:
               continue
                
            points = []

            for point in approx: # each contour is stored as a 3D array (nested list)
                x = point[0][0] # extracts x coordinate
                y = point[0][1] # extracts y coordinate
                points.append((x,y)) # adding the (x,y) tuple to the points list
            
            self.paths.append(points) # add all the points of the contour that's stored in points list as an element
        
        print(f"Number of filtered paths: {len(self.paths)}")
        
        return self.paths # returns a list of paths, each path is a list of points, each point is a tuple
    
    def transform_coordinates_with_corners(self, canvas_corners): 
        """Transform image coordinates to robot workspace coordinates using canvas corners"""
        if not self.paths or not canvas_corners: # safety check - if no paths to transform or canvas corners
            print("No paths or canvas corners to transform") # print warning and return original paths
            return self.paths
         # prints how many canvas corners recieved (should be 4)   
        print(f"Transform: Canvas corners received with {len(canvas_corners)} points")
        
        # Extract canvas corner positions
        # Corner order: [BL, BR, TR, TL]
        if len(canvas_corners) < 4: # if less than 4 canvas corners, print error msg and return original paths
            print("Error: Need 4 canvas corners for transformation")
            return self.paths
            # assigning 4 corner pnts to variables bl,br,tr,tl
        bl = canvas_corners[0]
        br = canvas_corners[1]
        tr = canvas_corners[2]
        tl = canvas_corners[3]
        
        # margin adjustments (inward from edges) (converting from mm to m)
        x_margin_m = 29.5 / 1000.0  # Convert mm to m
        y_margin_m = 21.0 / 1000.0  # Convert mm to m
        
        # calculates which way the bottom and left edges of canvas are pointing
        bottom_vector = [br['x'] - bl['x'], br['y'] - bl['y']]
        left_vector = [tl['x'] - bl['x'], tl['y'] - bl['y']]
        
        # Calculate lengths of the bottom and left edges using pythag to calculate unit vectors
        bottom_length = (bottom_vector[0]**2 + bottom_vector[1]**2)**0.5
        left_length = (left_vector[0]**2 + left_vector[1]**2)**0.5

        # Calculate unit vectors by dividing x,y components of each vector by edge length
        bottom_unit = [bottom_vector[0]/bottom_length, bottom_vector[1]/bottom_length]
        left_unit = [left_vector[0]/left_length, left_vector[1]/left_length]
        
        # Calculate new corner positions with margins
        # creating dictionaries called bl_adjusted, br_adjusted etc
        bl_adjusted = {
            'x': bl['x'] + x_margin_m * bottom_unit[0] + y_margin_m * left_unit[0],
            'y': bl['y'] + x_margin_m * bottom_unit[1] + y_margin_m * left_unit[1],
            'z': bl['z']
        }
        
        br_adjusted = {
            'x': br['x'] - x_margin_m * bottom_unit[0] + y_margin_m * left_unit[0],
            'y': br['y'] - x_margin_m * bottom_unit[1] + y_margin_m * left_unit[1],
            'z': br['z']
        }
        
        tr_adjusted = {
            'x': tr['x'] - x_margin_m * bottom_unit[0] - y_margin_m * left_unit[0],
            'y': tr['y'] - x_margin_m * bottom_unit[1] - y_margin_m * left_unit[1],
            'z': tr['z']
        }
        
        tl_adjusted = {
            'x': tl['x'] + x_margin_m * bottom_unit[0] - y_margin_m * left_unit[0],
            'y': tl['y'] + x_margin_m * bottom_unit[1] - y_margin_m * left_unit[1],
            'z': tl['z']
        }
        
        print(f"Adjusted canvas corners with margins: x={x_margin_m*1000}mm, y={y_margin_m*1000}mm")
        
        # Find bounding box of image paths by finding max and min x and y coordinates out of all paths in image
        # safety check to see if paths available
        if not self.paths:
            return self.paths
        
        # find bounding box of image paths
        min_x = float('inf')  # Starting with infinity
        max_x = float('-inf')  # Start with negative infinity
        min_y = float('inf')
        max_y = float('-inf')

        # Iterate through each path and point
        for path in self.paths:
            for point in path:
                x, y = point[0], point[1]
        
            # Update min and max values
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
        # calculating image width and height using max and min values 
        img_width = max_x - min_x
        img_height = max_y - min_y
        # if image width and height (lengths) are negative its invalid
        if img_width <= 0 or img_height <= 0:
            print("Error: Invalid image dimensions")
            return self.paths
        # print image bounds
        print(f"Image bounds: X: {min_x} to {max_x}, Y: {min_y} to {max_y}")
        
        # Transform each path
        transformed_paths = []
        for path in self.paths:
            transformed_path = []
            for point in path:
                # Normalise coordinates to [0,1] range
                norm_x = (point[0] - min_x) / img_width
                norm_y = (point[1] - min_y) / img_height
                
                # Invert Y coordinate (image Y is top to bottom, robot Y is bottom to top)
                norm_y = 1.0 - norm_y
                
                # Bilinear interpolation to map to robot coordinates
                robot_x = (1 - norm_x) * (1 - norm_y) * bl_adjusted['x'] + \
                          norm_x * (1 - norm_y) * br_adjusted['x'] + \
                          (1 - norm_x) * norm_y * tl_adjusted['x'] + \
                          norm_x * norm_y * tr_adjusted['x']
                          
                robot_y = (1 - norm_x) * (1 - norm_y) * bl_adjusted['y'] + \
                          norm_x * (1 - norm_y) * br_adjusted['y'] + \
                          (1 - norm_x) * norm_y * tl_adjusted['y'] + \
                          norm_x * norm_y * tr_adjusted['y']
                
                # Use average Z height of corners for Z coordinate
                robot_z = (bl_adjusted['z'] + br_adjusted['z'] + tr_adjusted['z'] + tl_adjusted['z']) / 4.0
                
                transformed_path.append((robot_x, robot_y))
            
            transformed_paths.append(transformed_path)
        
        self.paths = transformed_paths
        
        print(f"Transformation complete: {len(self.paths)} paths transformed")
        return self.paths
    
    def optimise_paths(self):
        """Optimise paths to minimise travel distance and drawing time"""
        if not self.paths: # safety check - if no paths to optimise
            print("No paths to optimise")
            return self.paths
            
        print(f"Optimising {len(self.paths)} paths")
        
        # Save original paths for comparison and calculation of improvements
        original_paths = []
        for path in self.paths:
            original_paths.append(path.copy())
        
        # Calculate total path length before optimisation
        original_length = self._calculate_total_path_length(original_paths)
        print(f"Original path length: {original_length:.3f} units")
        
        # Starting point (origin or a specified starting position)
        current_point = (0, 0)  # Assume the robot starts at origin
        
        # List to store optimised paths
        optimised_paths = []
        
        # Keep track of which paths we've used
        remaining_paths = []
        for i, path in enumerate(self.paths):
            remaining_paths.append({
                'index': i,
                'path': path.copy(),
                'used': False
            })
        
        # Process all paths using nearest neighbour algorithm
        while any(not path_info['used'] for path_info in remaining_paths):
            best_path_info = None
            best_distance = float('inf')
            best_reverse = False
            
            # Find the closest unused path (nearest neighbour approach)
            for path_info in remaining_paths:
                if path_info['used']: # skip paths we've already used
                    continue
                    
                path = path_info['path']
                if not path: # skip empty paths
                    path_info['used'] = True
                    continue
                
                # Check distance to start of path
                start_distance = self._point_distance(current_point, path[0])
                if start_distance < best_distance:
                    best_path_info = path_info
                    best_distance = start_distance
                    best_reverse = False
                
                # Check distance to end of path (if we draw it in reverse)
                end_distance = self._point_distance(current_point, path[-1])
                if end_distance < best_distance:
                    best_path_info = path_info
                    best_distance = end_distance
                    best_reverse = True
            
            # If we found a path, add it to the optimised paths
            if best_path_info:
                best_path = best_path_info['path']
                best_path_info['used'] = True
                
                # If it's better to draw in reverse, reverse the path direction
                if best_reverse:
                    best_path = best_path[::-1]  # Reverse the path using slicing
                
                optimised_paths.append(best_path)
                
                # Update current point to the end of this path for next iteration
                current_point = best_path[-1]
        
        # Calculate total path length after optimisation
        optimised_length = self._calculate_total_path_length(optimised_paths)
        print(f"Optimised path length: {optimised_length:.3f} units")
        
        # Calculate percent reduction to verify meeting the 20% requirement
        if original_length > 0:
            reduction_percent = (original_length - optimised_length) / original_length * 100
            print(f"Optimisation reduced path length by {reduction_percent:.2f}%")
            
            if reduction_percent >= 20:
                print("✓ Achieved at least 20% reduction in path length")
            else:
                print("✗ Did not achieve 20% reduction in path length")
        
        # Update paths with the optimised version
        self.paths = optimised_paths
        return self.paths

    def _calculate_total_path_length(self, paths):
        """Calculate the total length of all paths including travel between paths"""
        if not paths:
            return 0
        
        total_length = 0
        current_point = (0, 0)  # Assume starting from origin
        
        for path in paths:
            if not path:
                continue
            
            # Add travel distance to the start of the path
            total_length += self._point_distance(current_point, path[0])
            
            # Add length of the path itself
            for i in range(1, len(path)):
                total_length += self._point_distance(path[i-1], path[i])
            
            # Update current point to end of this path
            current_point = path[-1]
        
        return total_length

    def _point_distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5
    
    def identify_facial_features(self):
        """Placeholder for feature identification"""
        print("Identify: Features identification called")
        return
    
    def prioritise_features(self):
        """Placeholder for feature prioritisation"""
        print("Prioritise: Feature prioritisation called")
        return self.paths
        
    def visualize_paths(self, show_optimisation=True):
        """Create a visualisation of the extracted paths with optimisation info"""
        if not self.paths:
            return None
            
        import matplotlib.pyplot as plt
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Show original image in the background (if available)
        if self.original_image is not None:
            ax.imshow(self.original_image, cmap='gray', alpha=0.3)
        
        # Show each path in a different colour
        colours = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        
        # Draw paths with arrows to show direction and order
        for i, path in enumerate(self.paths):
            if not path:
                continue
                
            colour = colours[i % len(colours)]
            
            # Plot path
            x_values = [p[0] for p in path]
            y_values = [p[1] for p in path]
            ax.plot(x_values, y_values, color=colour, linewidth=2, alpha=0.7)
            
            # Mark start point
            ax.plot(path[0][0], path[0][1], 'o', color=colour, markersize=6)
            
            # Mark end point
            ax.plot(path[-1][0], path[-1][1], 's', color=colour, markersize=6)
            
            # Add path number
            ax.annotate(str(i), (path[0][0], path[0][1]), 
                       fontsize=10, color=colour, weight='bold')
            
            # Draw arrows along the path to show direction
            if len(path) > 5:
                idx = len(path) // 2  # Middle of path
                dx = path[idx+1][0] - path[idx-1][0]
                dy = path[idx+1][1] - path[idx-1][1]
                length = (dx**2 + dy**2)**0.5
                if length > 0:
                    ax.arrow(path[idx][0], path[idx][1], dx/length*5, dy/length*5, 
                            head_width=5, head_length=5, fc=colour, ec=colour)
        
        # Draw lines showing travel between paths if optimisation visualisation is enabled
        if show_optimisation and len(self.paths) > 1:
            current_point = (0, 0)  # Origin
            for path in self.paths:
                if not path:
                    continue
                    
                # Draw travel line
                ax.plot([current_point[0], path[0][0]], [current_point[1], path[0][1]], 
                       'k--', linewidth=1, alpha=0.5)
                
                # Update current point
                current_point = path[-1]
        
        ax.set_aspect('equal')
        ax.set_title(f'Paths ({len(self.paths)} paths)')
        
        return fig