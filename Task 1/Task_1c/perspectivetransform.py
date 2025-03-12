import cv2
import numpy as np
import argparse
import os


class Arena:
    def __init__(self, image_path):
        self.width = 1000
        self.height = 1000
        self.image_path = image_path
        self.detected_markers = []
        self.obstacles = 0
        self.total_area = 0

    def get_aruco_dict_and_params(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        return aruco_dict, parameters

    def detect_markers(self, gray):
        aruco_dict, parameters = self.get_aruco_dict_and_params()

        # OpenCV 4.7+ style of Aruco detection
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
       
        return corners, ids

    def sort_corners_by_position(self, corners):
        """
        Sort corners based on their position in the image.
        Assumes 4 markers have been detected and sorts them as:
        top-left, top-right, bottom-left, bottom-right
        """
        # Calculate the center of each detected ArUco marker
        centers = [(np.mean(corner[0][:, 0]), np.mean(corner[0][:, 1])) for corner in corners]

        # Sort by y-coordinate first (top to bottom), then by x-coordinate (left to right)
        sorted_indices = np.argsort([center[1] for center in centers])  # Sort by y (top to bottom)
        top_markers = sorted_indices[:2]  # Two top markers
        bottom_markers = sorted_indices[2:]  # Two bottom markers

        # Sort top markers by x-coordinate (left to right)
        top_left, top_right = sorted(top_markers, key=lambda idx: centers[idx][0])

        # Sort bottom markers by x-coordinate (left to right)
        bottom_left, bottom_right = sorted(bottom_markers, key=lambda idx: centers[idx][0])

        # Return the sorted corners in the order: top-left, top-right, bottom-left, bottom-right
        return [corners[top_left], corners[top_right], corners[bottom_left], corners[bottom_right]]

    def identification(self):
        # Read the image
        frame = cv2.imread(self.image_path)
        if frame is None:
            print(f"Error: Could not read the image at {self.image_path}")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
       
        # ArUco detection
        corners, ids = self.detect_markers(gray)

        # Draw detected markers
        if ids is not None and len(ids) == 4:
            self.detected_markers = ids.flatten().tolist()

            # Sort corners by their spatial positions
            sorted_corners = self.sort_corners_by_position(corners)

            # Define source points for the perspective transform
            src_points = np.array([
                sorted_corners[0][0][0],  # Top-left corner
                sorted_corners[1][0][1],  # Top-right corner
                sorted_corners[2][0][3],  # Bottom-left corner
                sorted_corners[3][0][2],  # Bottom-right corner
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

            # Create a mask for non-white areas
            lower_bound = np.array([88.7, 88.7, 88.7], dtype=np.uint8)
            upper_bound = np.array([192, 192, 192], dtype=np.uint8)
            mask = cv2.inRange(transformed_image, lower_bound, upper_bound)

            # Find contours on the mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS )

            # Filter contours based on area and shape
            min_area = 1000
            relevant_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > min_area:
                    relevant_contours.append(contour)

            self.obstacles = len(relevant_contours)
            self.total_area = sum(cv2.contourArea(contour) for contour in relevant_contours)

            # Draw all detected contours on the transformed image
            cv2.drawContours(transformed_image, relevant_contours, -1, (0, 255, 0), 2)
           
            # Save the transformed image to a file
            output_path = "transformed_image.png"
            cv2.imwrite(output_path, transformed_image)
            print(f"Transformed image saved to {output_path}")
        else:
            print(f"Error: Could not detect exactly 4 ArUco markers in the image. Detected: {len(ids) if ids is not None else 0}")

    def text_file(self):
        file_path = os.path.abspath("obstacles.txt")
        with open("obstacles.txt", "w") as file:
            file.write(f"Aruco ID: {self.detected_markers}\n")
            file.write(f"Obstacles: {self.obstacles}\n")
            file.write(f"Area: {self.total_area}\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process an image to detect ArUco markers and obstacles.')
    parser.add_argument('--image', type=str, required=True, help='Path to the input image')
    args = parser.parse_args()

    arena = Arena(args.image)
    arena.identification()
    arena.text_file()