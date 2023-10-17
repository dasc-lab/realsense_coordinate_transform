
# Coordinate Transformation of a Ground Robot from Vicon Frame to Realsense Camera Pixel Space
[comment]: ## realsense_coordinate_transform
## Components
1. [Vicon Coordinate System](https://www.vicon.com/) as world frame
2. [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html) for transmitting coordinate data
3. Real Sense Camera and Nvidia Jetson Xavier attached to the bottom of a drone
4. A ground robot whose coordinates will be used as inputs for the algorithms

## Prerequisites
1. Python
2. Vicon Coordinate System tracker and broadcaster
3. [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html)
4. OpenCV for image processing and writing
5. Realsense libraries for Camera intrinsics and video streaming to Jetson Xavier

## System Overview
* This portion of the system first takes two pieces of information of the ground robot from the Vicon system -its Quaternion and Vicon Frame coordinates - and broadcasts them via a ROS publisher.
* A ROS subcriber subsumed in the python script would process the broadcasted quaternion and translation in real time on Jetson Xavier strapped onto the drone.
* The script then expresses the quaternion in terms of a rotation matrix and subsequently performs homogeneous transformation on the ground robot Vicon coordinates to compute the camera coordinates of the ground robot.
* Since the camera is fixed on the bottom of the drone, the coordinate transformation between drone frame and camera frame is constant and trivial.
* With the help of extensive realsense libraries, the script utilizes [rs2_project_point_to_pixel(...)](https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0) to subsequently obtain the camera intrinsics and calculates the predicted pixel coordinates of the ground robot in pixel space based on the camera coordinates of the said ground robot. 


## Coordinate Transformation System Diagram
World Coordinates and Quaternion of the ground robot -> 4x4 Homogeneous matrix transform -> Coordinates of the ground robot in camera frame -> Project 3D point coordinates to 2D pixel coordinates -> robot appears on the expected coordinates of the image

## Potential Applications
This script can be useful for robotic systems that involve aerial video streaming or image capturing in an indoor setting. For instance, our in-door system involves an overhead drone hovering at a 2-meter altitude, which is the current altitude limit for the Vicon system. The quaternions and coordinates of both the drone and the ground robot will be broadcasted by the Vicon system. A homogeneous transformation will be performed to predict the pixel coordinates of the ground robot in the camera space. Owing to the constant evolving nature of this project, we may update this page if we find a way to apply this system to an outdoor setting.
