# Task: 2D Bitmap Generation and Path Planning

## Objective

The objective of this task is twofold:

1. Create a **2D bitmap** of the arena.
2. Use that map to **plan a path** between two given points in the arena using a path planning algorithm of your choice with the Swift Pico drone.

---

## Task Breakdown

### 1. **Create a 2D Bitmap of the Arena**
- Use **4 ArUco markers** as the corners of the arena and crop the image accordingly, similar to what was done in Task 1C.
- The **image dimensions** should be set to **1000x1000 pixels**.
- Detect the **racks** within the arena using **contour detection**.
- Add an **inflation layer** around the racks to account for the drone's size and safety margins.
- Convert the detected racks into **black regions**, and the rest of the arena into **white**, resulting in a **bit image** of the arena.


---

### 2. **Path Planning Algorithm**
Develop a **path planning algorithm** to allow the Swift Pico drone to navigate from one waypoint to another in the **shortest time possible**.

#### **Navigation Requirements**
- The drone should navigate through the given points using the developed path planning algorithm while maintaining an **error margin** of **±0.6 meters** in x, y, and z coordinates.
- The drone should maintain a **fixed height of 27** on the z-axis of the **WhyCon system**.
- The drone will start at a **fixed position** with image coordinates **(500, 500)**.
- **Two random waypoints** will be provided for navigation.
- The drone must **hover at the starting point** and each of the two waypoints for **3 seconds** before proceeding.

---
