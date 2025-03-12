#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints, PathPlanning

class WayPointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self.goals = []
        self.goal_index = 0
        self.drone_start_position = None  # Initialize as None
        
        # Create action client
        self.action_client = ActionClient(self, NavToWaypoint, 'waypoint_navigation')
        
        # Create service clients
        self.waypoint_cli = self.create_client(GetWaypoints, 'waypoints')
        self.path_planning_cli = self.create_client(PathPlanning, 'path_planning')
        
        # Wait for services
        while not self.waypoint_cli.wait_for_service(timeout_sec=1.0) or \
              not self.path_planning_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        # Setup timer to periodically check for waypoints
        self.create_timer(1.0, self.check_and_receive_goals)
        
    def check_and_receive_goals(self):
        """Periodically check for new waypoints if none are available"""
        if not self.goals:
            self.receive_goals()
        
    def send_goal(self, waypoint):
        if not waypoint:
            self.get_logger().warn('No valid waypoint provided')
            return
            
        # Request the path planning service
        request = PathPlanning.Request()
        request.start_position = Pose()
        request.start_position.x = self.drone_start_position[0]
        request.start_position.y = self.drone_start_position[1]
        request.start_position.z = self.drone_start_position[2]
        
        request.waypoint = Pose()
        request.waypoint.position.x = waypoint[0]
        request.waypoint.position.y = waypoint[1]
        request.waypoint.position.z = waypoint[2]

        # Call path planning service
        future = self.path_planning_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response is None:
            self.get_logger().error('Path planning service call failed')
            return

        # Send goal to action server with planned path
        goal_msg = NavToWaypoint.Goal()
        goal_msg.waypoint.position.x = waypoint[0]
        goal_msg.waypoint.position.y = waypoint[1]
        goal_msg.waypoint.position.z = waypoint[2]
        goal_msg.path = response.path

        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.hov_time))
        self.goal_index += 1
        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Current position: ({feedback.current_waypoint.pose.position.x:.2f}, '
            f'{feedback.current_waypoint.pose.position.y:.2f}, '
            f'{feedback.current_waypoint.pose.position.z:.2f})'
        )

    def receive_goals(self):
        self.get_logger().info('Requesting waypoints...')
        future = self.send_waypoint_request()
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response is None:
            self.get_logger().error('Failed to receive waypoints')
            return
            
        # Clear existing goals and process new ones
        self.goals.clear()
        for pose in response.waypoint:
            waypoint = [pose.position.x, pose.position.y, pose.position.z]
            self.goals.append(waypoint)
            self.get_logger().info(f'Received waypoint: {waypoint}')
            
        if self.goals:
            self.get_logger().info(f'Total waypoints received: {len(self.goals)}')
            if self.drone_start_position is None:
                self.drone_start_position = self.goals[0]
                self.send_goal(self.goals[0])
        else:
            self.get_logger().warn('No waypoints received, will retry later')

    def send_waypoint_request(self):
        request = GetWaypoints.Request()
        request.get_waypoints = True
        return self.waypoint_cli.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    waypoint_client = WayPointClient()
    
    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()