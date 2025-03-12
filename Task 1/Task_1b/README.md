# Task 1b - Swift Pico Drone PID Controller

## Overview

This project implements a **PID controller** for the **Swift Pico drone** within a **Gazebo simulation**. The controller enables the drone to maintain a stable position at the setpoint **[2, 2, 19]** while maintaining an **error margin of ±0.4** for at least **10 seconds**.
# SwiftPico PID Controller - Implementation Breakdown

## 1. Initialization (`__init__()`)
- Node: `pico_controller`
- Setpoint: `[2.0, 2.0, 19.0]`
- PID Gains: `Kp=[0.50, 0.51, 0.50]`, `Ki=[0.0, 0.0, 0.00006]`, `Kd=[0.70, 0.70, 1.28]`
- Publishers: `/drone_command`, `/pid_error`
- Subscribers: `/whycon/poses`, `/throttle_pid`, `/pitch_pid`, `/roll_pid`
- Control Loop: 100 Hz (0.01s interval)

## 2. Drone Control
- **`arm()`** - Arms the drone (`rc_throttle = 1532`)
- **`disarm()`** - Disarms the drone (`rc_throttle = 1000`)

## 3. Callbacks
- **`whycon_callback(msg)`** - Updates drone position from `/whycon/poses`
- **`altitude_set_pid(msg)`**, **`pitch_set_pid(msg)`**, **`roll_set_pid(msg)`** - Update PID gains dynamically

## 4. PID Control Loop (`pid_control_loop()`)
Calls:
- **`pid_roll()`** - Computes roll correction (`rc_roll ∈ [1490, 1510]`)
- **`pid_pitch()`** - Computes pitch correction (`rc_pitch ∈ [1490, 1510]`)
- **`pid_throttle()`** - Computes throttle correction (`rc_throttle ∈ [1496, 1560]`)
- Publishes: `/drone_command`, `/pid_error`

## 5. PID Error Message (`create_pid_error_msg()`)
- Creates and publishes a `PIDError` message

## 6. Execution (`main()`)
- Initializes ROS 2 (`rclpy.init()`)
- Creates `SwiftPico` instance
- Runs node (`rclpy.spin()`)
- Destroys node on exit

