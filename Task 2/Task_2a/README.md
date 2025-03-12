# Implementation Breakdown: 2D Bitmap Generation for Swift Pico Drone 

## Overview
This implementation processes images from the **ROS 2 /image_raw topic** to:
1. Detect **ArUco markers** for perspective transformation.
2. Generate a **2D bitmap** of the warehouse.
3. Identify **obstacles** and apply an **inflation layer** for safe navigation.

---

## 1. Node Setup and Image Subscription
- The `BitMapGenerator` node subscribes to `/image_raw` (ROS 2 topic).
- Uses `CvBridge` to convert ROS `Image` messages to OpenCV format.

```python
self.subscription = self.create_subscription(
    Image,
    '/image_raw',
    self.image_callback,
    10
)
```

---

## 2. ArUco Marker Detection
- Uses **cv2.aruco** to detect **4 ArUco markers** (`DICT_4X4_250`).
- These markers define **reference points** for **perspective transformation**.

```python
aruco_dict, parameters = self.get_aruco_dict_and_params()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
corners, ids, _ = detector.detectMarkers(gray)
```

---

## 3. Perspective Transformation
- The four detected ArUco markers are mapped to **predefined corners**.
- A **transformation matrix** warps the input image to a **top-down view**.

```python
matrix = cv2.getPerspectiveTransform(src_points, dst_points)
transformed_image = cv2.warpPerspective(frame, matrix, (self.width, self.height))
```

- Saves the **transformed image**:  
   `arena_original.png`

---

## 4. Obstacle Detection 
- Converts the **transformed image** to **grayscale**.
- Uses **adaptive thresholding** and **Otsu's method** for **binarization**.
- Applies **morphological operations** to refine the **binary map**.

```python
_, otsu_thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
binary_threshold = cv2.bitwise_or(adaptive_thresh, otsu_thresh)
```

- Extracts **obstacles** using **contours**.
- Saves **bitmap** of detected obstacles:  
   `2D_bit_map.png`

---

## 5. Inflation Layer for Safe Navigation 
- Expands obstacles using **morphological dilation** (`cv2.erode`).
- Marks **inflated areas** in **red** for visualization.

```python
inflation_visualization[inflation_mask] = [0, 0, 255]
```

- Saves **inflated obstacle visualization**:  
   `arena_with_inflation.png`

---

## 6. ROS 2 Node Lifecycle 
- Processes incoming **image messages** from ROS.
- Shuts down **after successful bitmap generation**.

```python
self.destroy_node()
rclpy.shutdown()
```

---

## Generated Output Files 
| File Name                | Description                          |
|--------------------------|--------------------------------------|
| `arena_original.png`     | Warped **top-down** view of the arena |
| `2D_bit_map.png`         | **Binary** map of detected obstacles |
| `arena_with_inflation.png` | **Obstacle inflation** for safe path planning |

---

## Conclusion 
This implementation enables the **Swift Pico Drone** to:
- Generate a **2D map** from camera input.
- Detect **obstacles** and **expand** them for safety.
- Provide a clear **binary bitmap** for path planning.
