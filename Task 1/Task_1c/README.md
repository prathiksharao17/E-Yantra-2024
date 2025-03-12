# Task 1C: Image Processing for 2D Map 

## Objective 
Develop image processing skills to create a **2D map** for drone navigation.

## **Apply Perspective Transform**
1. **Load Image:** Import `task1c_image.jpg` into the environment.
2. **Detect Aruco Markers:** Identify four markers as reference points.
3. **Define Destination Points:** Map markers to a rectangular output.
4. **Transform Image:** Compute and apply the **perspective transformation**.

## **Find Obstacles**
1. **Process Transformed Image:** Use OpenCV to detect obstacles.
2. **Generate Output (.txt):**  
   - **Detected Aruco markers**  
   - **Number of obstacles**  
   - **Total obstacle area**  
