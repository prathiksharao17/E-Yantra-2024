import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
import math
import time

class DroneDrawer(Node):
    def __init__(self):
        super().__init__('drone_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.client_teleport = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.client_set_pen = self.create_client(SetPen, '/turtle1/set_pen')
        
        while not self.client_teleport.wait_for_service(timeout_sec=1.0) or not self.client_set_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.srv_teleport = self.client_teleport
        self.srv_set_pen = self.client_set_pen

        self.original_pen = (255, 255, 255, 3)  # Default color white, thickness 3, pen on
        self.draw_drone()  # Call to draw_drone inside constructor

    def teleport_turtle(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta
        self.srv_teleport.call_async(request)
        self.get_logger().info(f'Teleporting to: x={x}, y={y}, theta={theta}')  # Log the coordinates and angle

    def set_pen(self, r, g, b, width, off):
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        self.srv_set_pen.call_async(request)

    def draw_circle(self, radius):
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 1.0 / radius  # Adjust the angular speed to make a circle
        for _ in range(62):  # Draw the circle by sending commands repeatedly
            self.publisher.publish(twist)
            time.sleep(0.1)

    def draw_square(self):
        vertices = [
            (3.0, 5.0),  # Start at first vertex
            (5.0, 7.0),  # Second vertex
            (7.0, 5.0),  # Third vertex
            (5.0, 3.0)   # Fourth vertex
        ]

        # Iterate over each pair of consecutive vertices
        for i in range(len(vertices)):
            start = vertices[i]
            end = vertices[(i + 1) % len(vertices)]  # Next vertex, looping back to first

            # Calculate the angle to face the next vertex
            angle = math.atan2(end[1] - start[1], end[0] - start[0])
            distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)

            # Teleport to the start position of the side
            self.teleport_turtle(start[0], start[1], angle)

            # Move in a straight line to the next vertex
            twist = Twist()
            twist.linear.x = 2.0  # Move forward
            twist.angular.z = 0.0

            time_to_move = distance / twist.linear.x
            self.publisher.publish(twist)
            time.sleep(time_to_move)  # Move for the calculated time

            # Stop after reaching the vertex
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            time.sleep(0.1)  # Short pause after completing a side

        # Ensure the turtle returns to the starting point
        self.teleport_turtle(vertices[0][0], vertices[0][1], 0.0)
        self.set_pen(255, 255, 255, 3, 1)

    def draw_lines_to_square_edges(self):
        """Draw lines from the centers of the four circles to the centers of the square edges."""
        # Define the centers of the circles
        circle_centers = [
            (2.0, 2.0),  # Bottom-left circle
            (2.0, 8.0),  # Top-left circle
            (8.0, 8.0),  # Top-right circle
            (8.0, 2.0)   # Bottom-right circle
        ]

        # Define the midpoints of the square's edges
        square_edge_midpoints = [
            (6.0, 6.0),  # Midpoint of top edge
            (6.0, 4.0),  # Midpoint of right edge
            (4.0, 4.0),  # Midpoint of bottom edge
            (4.0, 6.0)   # Midpoint of left edge
        ]

        for i in range(len(circle_centers)):
            start = circle_centers[i]
            end = square_edge_midpoints[i]

            # Calculate the angle to face the square edge
            angle = math.atan2(end[1] - start[1], end[0] - start[0])
            distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)

            # Teleport to the circle center
            self.set_pen(255, 255, 255, 3, 1)  # Pen off while teleporting
            self.teleport_turtle(start[0], start[1], angle)

            # Set pen down to draw the line
            self.set_pen(255, 255, 255, 3, 0)  # Pen on

            # Move to the square edge midpoint
            twist = Twist()
            twist.linear.x = 2.815  # Move forward
            twist.angular.z = 0.0

            time_to_move = distance / twist.linear.x
            self.publisher.publish(twist)
            time.sleep(time_to_move)  # Move for the calculated time

            # Stop after reaching the midpoint
            twist.linear.x = 0.0
            self.publisher.publish(twist)
            time.sleep(0.1)  # Short pause

    def draw_drone(self):
        # Store the original pen state
        original_color = (255, 255, 255)  # White
        original_width = 3
        
        # Lift the pen before teleporting
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=1)  # Pen is off
        
        # Teleport to first circle's center and draw
        self.teleport_turtle(2.0, 1.0, 0.0)
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=0)  # Pen is on
        self.draw_circle(1.0)

        # Lift the pen before teleporting to the next position
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=1)  # Pen is off
        self.teleport_turtle(2.0, 7.0, 0.0)
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=0)  # Pen is on
        self.draw_circle(1.0)

        # Lift the pen before teleporting to the next position
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=1)  # Pen is off
        self.teleport_turtle(8.0, 7.0, 0.0)
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=0)  # Pen is on
        self.draw_circle(1.0)

        # Lift the pen before teleporting to the next position
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=1)  # Pen is off
        self.teleport_turtle(8.0, 1.0, 0.0)
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=0)  # Pen is on
        self.draw_circle(1.0)

        # Now teleport to start of square
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=1)  # Pen is off
        self.teleport_turtle(5.0, 3.0, 0.0)
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=0)  # Pen is on
        self.draw_square()

        # Draw lines from circles to square edges
        self.draw_lines_to_square_edges()

        # Finally, teleport to the center
        self.set_pen(r=original_color[0], g=original_color[1], b=original_color[2], width=original_width, off=1)  # Pen is off
        self.teleport_turtle(5.0, 5.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    drawer = DroneDrawer()
    rclpy.spin(drawer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



