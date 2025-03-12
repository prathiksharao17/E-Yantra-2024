#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BitMapGenerator(Node):
    def __init__(self):
        super().__init__('bit_map_generator')
        self.width = 1000
        self.height = 1000
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.detected_markers = []
        self.inflation_radius = 30  # Pixels for safety margin

    def get_aruco_dict_and_params(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        return aruco_dict, parameters

    def detect_markers(self, gray):
        aruco_dict, parameters = self.get_aruco_dict_and_params()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        return corners, ids

    def create_bitmap(self, frame):
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ArUco detection
        corners, ids = self.detect_markers(gray)

        if ids is not None and len(ids) == 4:
            self.detected_markers = ids.flatten().tolist()

            # Dictionary to map ArUco IDs to specific corners
            id_to_corner = {
                80: 'top_left',     
                85: 'top_right',    
                95: 'bottom_right', 
                90: 'bottom_left'   
            }

            # Initialize corners for perspective transform
            corners_dict = {
                'top_left': None,
                'top_right': None,
                'bottom_left': None,
                'bottom_right': None
            }

            # Assign corners based on IDs
            for corner, marker_id in zip(corners, ids.flatten()):
                corner_points = corner[0]
                corner_type = id_to_corner.get(marker_id)
                if corner_type:
                    if corner_type == 'top_left':
                        corners_dict['top_left'] = corner_points[0]
                    elif corner_type == 'top_right':
                        corners_dict['top_right'] = corner_points[1]
                    elif corner_type == 'bottom_right':
                        corners_dict['bottom_left'] = corner_points[3]
                    elif corner_type == 'bottom_left':
                        corners_dict['bottom_right'] = corner_points[2]

            if all(corners_dict[c] is not None for c in corners_dict):
                # Define source and destination points
                src_points = np.array([
                    corners_dict['top_left'],
                    corners_dict['top_right'],
                    corners_dict['bottom_left'],
                    corners_dict['bottom_right']
                ], dtype=np.float32)

                dst_points = np.array([
                    [0, 0],
                    [self.width-1, 0],
                    [0, self.height-1],
                    [self.width-1, self.height-1]
                ], dtype=np.float32)

                # Apply Perspective Transform
                matrix = cv2.getPerspectiveTransform(src_points, dst_points)
                transformed_image = cv2.warpPerspective(frame, matrix, (self.width, self.height))

                # Save the original transformed image
                cv2.imwrite('arena_original.png', transformed_image)

                # Convert to grayscale and enhance contrast
                gray_transformed = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2GRAY)
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
                enhanced_gray = clahe.apply(gray_transformed)
                
                # Apply Gaussian blur to reduce noise
                blurred = cv2.GaussianBlur(enhanced_gray, (5, 5), 0)
                
                # Use both adaptive thresholding and Otsu's method
                adaptive_thresh = cv2.adaptiveThreshold(
                    blurred,
                    255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY_INV,
                    21,  # Increased block size
                    5    # Increased constant
                )
                
                # Apply Otsu's thresholding
                _, otsu_thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                
                # Combine both thresholds
                binary_threshold = cv2.bitwise_or(adaptive_thresh, otsu_thresh)
                
                # Apply morphological operations to clean up the binary image
                kernel = np.ones((5,5), np.uint8)
                binary_threshold = cv2.morphologyEx(binary_threshold, cv2.MORPH_CLOSE, kernel)
                binary_threshold = cv2.morphologyEx(binary_threshold, cv2.MORPH_OPEN, kernel)

                # Find contours with hierarchy
                contours, hierarchy = cv2.findContours(binary_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Create clean binary image (white background)
                binary_image = np.ones((self.height, self.width), dtype=np.uint8) * 255
                
                # Draw and fill contours
                min_area = 1000
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > min_area:
                        # Fill the contour
                        cv2.drawContours(binary_image, [contour], -1, 0, -1)
                        # Draw the boundary
                        cv2.drawContours(binary_image, [contour], -1, 0, 2)

                # Create inflation layer
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.inflation_radius, self.inflation_radius))
                inflated_binary = cv2.erode(binary_image, kernel)

                # Create a colored version of the inflation layer for visualization
                inflation_visualization = transformed_image.copy()
                # Draw original obstacles in black
                mask = binary_image == 0
                inflation_visualization[mask] = [0, 0, 0]
                # Draw inflation layer in red
                inflation_mask = (inflated_binary == 0) & (binary_image == 255)
                inflation_visualization[inflation_mask] = [0, 0, 255]

                # Save all images
                cv2.imwrite('arena_with_inflation.png', inflation_visualization)
                cv2.imwrite('2D_bit_map.png', inflated_binary)
                
                print("Bitmap generation complete. Files saved: arena_original.png, arena_with_inflation.png, 2D_bit_map.png")
                self.get_logger().info('Bitmap generated successfully')
                
                # Shutdown the node after successful bitmap generation
                self.destroy_node()
                rclpy.shutdown()
            else:
                self.get_logger().error('Not all required ArUco markers were detected')
        else:
            self.get_logger().error(f'Could not detect exactly 4 ArUco markers. Detected: {len(ids) if ids is not None else 0}')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.create_bitmap(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    bitmap_generator = BitMapGenerator()
    rclpy.spin(bitmap_generator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()