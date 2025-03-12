# Task 2: Autonomous Navigation & Package Identification 

## Objective 
Develop and implement a system that enables the **Swift Pico Drone** to:
- Autonomously **traverse** the warehouse.
- **Plan paths** within the Gazebo environment.
- **Identify packages** accurately.

## Task 2 Breakdown
### Task 2A - Waypoint Navigation in Gazebo
- Navigate **Swift Pico Drone** through specified **setpoints**.
- Use **PID controller** from **Task 1B**.
- Utilize **ROS 2** features: 
  - Action servers & clients.
  - ROS 2 services.

### Task 2B - Path Planning in Gazebo Warehouse
- **Create a 2D bitmap** of the arena.
- Use the **map** to plan a path between two points.
- Implement a **path planning algorithm** of choice.

### Task 2C - Inventory Validation in Gazebo 
- Navigate the **Swift Pico Drone** through the warehouse.
- **Avoid collisions** with racks.
- Identify **two packages** at known locations.
