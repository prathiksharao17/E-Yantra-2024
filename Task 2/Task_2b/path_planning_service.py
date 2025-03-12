#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from waypoint_navigation.srv import PathPlanning
from geometry_msgs.msg import Pose
import cv2

class PathPlanningService(Node):
    def __init__(self):
        super().__init__('path_planning_service')
        self.srv = self.create_service(PathPlanning, 'path_planning', self.path_planning_callback)
        self.arena_width = 1000
        self.arena_height = 1000
        self.bit_map = None
        self.load_bit_map()

    def load_bit_map(self):
        """
        Load the 2D bit map of the arena from the file.
        """
        self.bit_map = cv2.imread('2D_bit_map.png', cv2.IMREAD_GRAYSCALE)
        if self.bit_map is None:
            self.get_logger().error("Failed to load the 2D bit map.")

    def path_planning_callback(self, request, response):
        self.get_logger().info(f"Received request for path planning from waypoint ({request.waypoint.position.x}, {request.waypoint.position.y}, {request.waypoint.position.z})")
        
        # Your path planning algorithm goes here
        start_point = np.array([request.start_position.x, request.start_position.y, request.start_position.z])
        end_point = np.array([request.waypoint.position.x, request.waypoint.position.y, request.waypoint.position.z])
        
        # Use the A* algorithm on the 2D bit map
        path = self.a_star_search(start_point, end_point)
        
        # Populate the response with the planned path
        response.path = [Pose() for _ in range(len(path))]
        for i, point in enumerate(path):
            response.path[i].position.x = point[0]
            response.path[i].position.y = point[1]
            response.path[i].position.z = point[2]
        
        self.get_logger().info(f"Path planning completed. Number of waypoints in the path: {len(path)}")
        return response

    def a_star_search(self, start, goal):
        """
        Implement A* search algorithm to find the shortest path between start and goal.
        """
        # Heuristic function (Manhattan distance)
        def heuristic(a, b):
            (x1, y1, z1), (x2, y2, z2) = a, b
            return abs(x1 - x2) + abs(y1 - y2) + abs(z1 - z2)

        # A* search
        open_set = set()
        closed_set = set()
        came_from = {}
        g_score = {tuple(start): 0}
        f_score = {tuple(start): heuristic(start, goal)}

        open_set.add(tuple(start))

        while open_set:
            current = min(open_set, key=lambda x: f_score[x])
            if np.array_equal(current, goal):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            open_set.remove(current)
            closed_set.add(current)

            for dx, dy, dz in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]:
                neighbor = (current[0] + dx, current[1] + dy, current[2] + dz)

                # Check if the neighbor is within the arena and not inside an obstacle
                if (0 <= neighbor[0] < self.arena_width and
                    0 <= neighbor[1] < self.arena_height and
                    0 <= neighbor[2] < 50 and
                    not self.is_point_in_obstacle(neighbor)):
                    tentative_g_score = g_score[current] + 1
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                        open_set.add(neighbor)

        return []  # No path found

    def is_point_in_obstacle(self, point):
        """
        Check if a given point is inside an obstacle in the 2D bit map.
        """
        x, y, _ = point
        if self.bit_map is not None and self.bit_map[int(y), int(x)] == 0:
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    path_planning_service = PathPlanningService()

    try:
        rclpy.spin(path_planning_service)
    except KeyboardInterrupt:
        path_planning_service.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        path_planning_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()