#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose, PoseArray
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints, PathPlanning
import math
import time
import numpy as np

class WayPointServer(Node):
    def __init__(self):
        super().__init__('waypoint_server')
        
        # Callback groups for concurrent execution
        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        
        # Starting position in image coordinates (500, 500)
        self.drone_start_position = self.image_to_whycon(500, 500, 27.0)
        
        # Initialize waypoints list
        self.waypoints = []
        
        # Create service and action server
        self.srv = self.create_service(GetWaypoints, 'waypoints', self.waypoint_callback)
        self.path_planning_client = self.create_client(PathPlanning, 'path_planning')
        
        # Subscribe to random points topic
        self.random_points_sub = self.create_subscription(
            PoseArray,
            'random_points',
            self.random_points_callback,
            10
        )
        
        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback,
            callback_group=self.action_callback_group
        )
        
        self.get_logger().info('WayPoint Server has been initialized')

    def image_to_whycon(self, x, y, z):
        """
        Convert image coordinates to WhyCon coordinates
        Implement the conversion logic from whycon_mapper.py here
        """
        # These conversion factors should match your whycon_mapper.py
        x_whycon = (x - 500) / 100.0  # Adjust these conversion factors
        y_whycon = (y - 500) / 100.0  # based on your whycon_mapper.py
        return [x_whycon, y_whycon, z]

    def whycon_to_image(self, x, y, z):
        """
        Convert WhyCon coordinates to image coordinates
        """
        # Inverse of the image_to_whycon conversion
        x_image = x * 100.0 + 500
        y_image = y * 100.0 + 500
        return [x_image, y_image, z]

    def random_points_callback(self, msg):
        """
        Callback for receiving random waypoints
        """
        self.waypoints = []
        for pose in msg.poses:
            self.waypoints.append([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])
        self.get_logger().info(f'Received new waypoints: {self.waypoints}')

    def waypoint_callback(self, request, response):
        self.get_logger().info(f"Received request to get waypoints: {request.get_waypoints}")
        
        if request.get_waypoints and self.waypoints:
            self.get_logger().info("Processing waypoints request")
            response.waypoint = [Pose() for _ in range(len(self.waypoints))]
            
            for i, waypoint in enumerate(self.waypoints):
                response.waypoint[i].position.x = waypoint[0]
                response.waypoint[i].position.y = waypoint[1]
                response.waypoint[i].position.z = waypoint[2]
        else:
            self.get_logger().info("No waypoints available or request rejected")
            
        return response

    def execute_callback(self, goal_handle):
        """
        Execute the navigation to waypoint action
        """
        self.get_logger().info('Executing goal...')
        
        # Convert start position to WhyCon coordinates if needed
        start_position = self.drone_start_position
        
        end_position = [
            goal_handle.request.waypoint.position.x,
            goal_handle.request.waypoint.position.y,
            goal_handle.request.waypoint.position.z
        ]

        # Create path planning request
        request = PathPlanning.Request()
        request.start_position = Pose()
        request.start_position.position.x = start_position[0]
        request.start_position.position.y = start_position[1]
        request.start_position.position.z = start_position[2]
        
        request.waypoint = Pose()
        request.waypoint.position.x = end_position[0]
        request.waypoint.position.y = end_position[1]
        request.waypoint.position.z = end_position[2]

        # Wait for path planning service to be available
        while not self.path_planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Path planning service not available, waiting...')

        # Request path planning
        future = self.path_planning_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is None:
            self.get_logger().error('Path planning service call failed')
            goal_handle.abort()
            return NavToWaypoint.Result()

        response = future.result()
        
        # Execute the planned path
        feedback_msg = NavToWaypoint.Feedback()
        
        for waypoint in response.path:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavToWaypoint.Result()
                
            feedback_msg.current_waypoint = waypoint
            goal_handle.publish_feedback(feedback_msg)
            
            # Hover for 3 seconds only at the target waypoint
            if waypoint.position.x == end_position[0] and \
               waypoint.position.y == end_position[1] and \
               waypoint.position.z == end_position[2]:
                self.hover_for_3_seconds(waypoint)

        goal_handle.succeed()
        return NavToWaypoint.Result()

    def hover_for_3_seconds(self, waypoint):
        """
        Hover the drone at the given waypoint for 3 seconds.
        """
        self.get_logger().info(f'Hovering at waypoint for 3 seconds: ({waypoint.position.x}, {waypoint.position.y}, {waypoint.position.z})')
        time.sleep(3.0)

def main(args=None):
    rclpy.init(args=args)
    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        executor.shutdown()
        waypoint_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()