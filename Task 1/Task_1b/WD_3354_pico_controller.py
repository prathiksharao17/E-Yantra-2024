#!/usr/bin/env python3

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node

class SwiftPico(Node):
    def __init__(self):
        super().__init__('pico_controller')
        # self.cmd.rc_throttle = 1500

        # Setpoint
        self.drone_position = [0.0, 0.0, 0.0]
        self.setpoint = [2.0, 2.0, 19.0]

        # Command initialization
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1550
        self.cmd.rc_throttle = 1532

        # PID gains (start with conservative values)
        self.Kp = [0.50, 0.51, 0.50]  # Roll, Pitch, Throttle
        self.Ki = [0.0, 0.0, 0.00006]  # Roll, Pitch, Throttle
        self.Kd = [0.70, 0.70, 1.28]  # Roll, Pitch, Throttle

        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]

        self.max_values = [500, 500, 50]
        self.min_values = [-500, -500, -50]

        self.sample_time = 0.01

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        # Subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

        # Timer for PID loop
        self.timer = self.create_timer(self.sample_time, self.pid_control_loop)

        # Initialize drone
        self.arm()

        # Log initialization
        self.get_logger().info('Swift Pico PID controller initialized and armed.')

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_throttle = 1000
        self.command_pub.publish(self.cmd)
        self.get_logger().info('Drone disarmed.')

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1532
        self.command_pub.publish(self.cmd)
        self.get_logger().info('Drone armed.')

    def whycon_callback(self, msg):
        if msg.poses:
            pose = msg.poses[0]
            self.drone_position[0] = pose.position.x
            self.drone_position[1] = pose.position.y
            self.drone_position[2] = pose.position.z
            self.get_logger().info(f'Drone position updated: {self.drone_position}')

    def altitude_set_pid(self, msg):
        self.Kp[2] = msg.kp
        self.Ki[2] = msg.ki
        self.Kd[2] = msg.kd
        self.get_logger().info(f'Altitude PID updated: Kp={self.Kp[2]}, Ki={self.Ki[2]}, Kd={self.Kd[2]}')

    def pitch_set_pid(self, msg):
        self.Kp[1] = msg.kp
        self.Ki[1] = msg.ki
        self.Kd[1] = msg.kd
        self.get_logger().info(f'Pitch PID updated: Kp={self.Kp[1]}, Ki={self.Ki[1]}, Kd={self.Kd[1]}')

    def roll_set_pid(self, msg):
        self.Kp[0] = msg.kp
        self.Ki[0] = msg.ki
        self.Kd[0] = msg.kd
        self.get_logger().info(f'Roll PID updated: Kp={self.Kp[0]}, Ki={self.Ki[0]}, Kd={self.Kd[0]}')        

    # def altitude_set_pid(self, msg):
    #     self.Kp[2] = msg.kp * 0.01
    #     self.Ki[2] = msg.ki * 0.00001
    #     self.Kd[2] = msg.kd * 0.01
    #     self.get_logger().info(f'Altitude PID updated: Kp={self.Kp[2]}, Ki={self.Ki[2]}, Kd={self.Kd[2]}')

    # def pitch_set_pid(self, msg):
    #     self.Kp[1] = msg.kp * 0.01
    #     self.Ki[1] = msg.ki * 0.0001
    #     self.Kd[1] = msg.kd * 0.01
    #     self.get_logger().info(f'Pitch PID updated: Kp={self.Kp[1]}, Ki={self.Ki[1]}, Kd={self.Kd[1]}')

    # def roll_set_pid(self, msg):
    #     self.Kp[0] = msg.kp * 0.01
    #     self.Ki[0] = msg.ki * 0.0001
    #     self.Kd[0] = msg.kd * 0.01
    #     self.get_logger().info(f'Roll PID updated: Kp={self.Kp[0]}, Ki={self.Ki[0]}, Kd={self.Kd[0]}')        

    def pid_control_loop(self):
        self.pid_roll()
        self.pid_pitch()
        self.pid_throttle()
        self.command_pub.publish(self.cmd)
        self.pid_error_pub.publish(self.create_pid_error_msg())
        
        self.get_logger().info(f'Publishing command: Roll={self.cmd.rc_roll}, Pitch={self.cmd.rc_pitch}, Throttle={self.cmd.rc_throttle}')

    def pid_roll(self):
        error = -1 * (self.drone_position[0] - self.setpoint[0])
        p = self.Kp[0] * error
        self.error_sum[0] += error * self.sample_time

        if self.Ki[0] != 0:  # Avoid division by zero
            self.error_sum[0] = min(max(self.error_sum[0], -self.max_values[0] / self.Ki[0]), self.max_values[0] / self.Ki[0])
            i_term = self.Ki[0] * self.error_sum[0]
        else:
            i_term = 0

        d = self.Kd[0] * (error - self.prev_error[0]) / self.sample_time
        output = p + i_term + d
        output = max(self.min_values[0], min(self.max_values[0], output))
        
        # Adjust rc_roll based on output
        new_roll = self.cmd.rc_roll
        new_roll += int(output)
        self.cmd.rc_roll = min(max(new_roll, 1490), 1510)

        self.prev_error[0] = error
        self.get_logger().info(f'Roll PID: Error={error}, P={p}, I={i_term}, D={d}, Output={output}')

    def pid_pitch(self):
        error = self.drone_position[1] - self.setpoint[1]
        p = self.Kp[1] * error
        self.error_sum[1] += error * self.sample_time

        if self.Ki[1] != 0:  # Avoid division by zero
            self.error_sum[1] = min(max(self.error_sum[1], -self.max_values[1] / self.Ki[1]), self.max_values[1] / self.Ki[1])
            i_term = self.Ki[1] * self.error_sum[1]
        else:
            i_term = 0

        d = self.Kd[1] * (error - self.prev_error[1]) / self.sample_time
        output = p + i_term + d
        output = max(self.min_values[1], min(self.max_values[1], output))
        
        # Adjust rc_pitch based on output
        new_pitch = self.cmd.rc_pitch
        new_pitch += int(output)
        self.cmd.rc_pitch = min(max(new_pitch, 1490), 1510)

        self.prev_error[1] = error
        self.get_logger().info(f'Pitch PID: Error={error}, P={p}, I={i_term}, D={d}, Output={output}')

    def pid_throttle(self):
        
        error = self.drone_position[2] - self.setpoint[2]
        p = self.Kp[2] * error
        d = self.Kd[2] * (error - self.prev_error[2]) / self.sample_time
        self.error_sum[2] += error * self.sample_time
        i = self.Ki[2] * self.error_sum[2]
        output = p + i + d
        new_throttle = self.cmd.rc_throttle
        new_throttle += int(output)
        self.cmd.rc_throttle = min(max(new_throttle, 1496), 1560)
        self.prev_error[2] = error
        self.get_logger().info(f'Throttle PID: Error={error}, P={p}, I={i}, D={d}, Output={output}')

    def create_pid_error_msg(self):
        pid_error = PIDError()
        pid_error.roll_error = self.drone_position[0] - self.setpoint[0]
        pid_error.pitch_error = self.drone_position[1] - self.setpoint[1]
        pid_error.throttle_error = self.drone_position[2] - self.setpoint[2]
        pid_error.yaw_error = 0.0 
        return pid_error

def main(args=None):
    rclpy.init(args=args)
    swift_pico = SwiftPico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()