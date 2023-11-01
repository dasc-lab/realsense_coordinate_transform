
# Coordinate Transformation of a Ground Robot from Vicon Frame to Realsense Camera Pixel Space
[comment]: ## realsense_coordinate_transform
## Integral Components
1. [Vicon Coordinate System](https://www.vicon.com/) as world frame
3. [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html) for transmitting coordinate data
4. Real Sense Camera and Nvidia Jetson Xavier attached to a drone
5. A ground robot whose coordinates will be used as inputs for the algorithms

## Software Prerequisites
1. Python 3
2. Vicon Coordinate System tracker and broadcaster
3. Set up [Vicon bridge](https://github.com/dasc-lab/ros2-vicon-bridge/tree/main)
4. [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/foxy/index.html)
5. OpenCV2 for image processing and writing
6. [Realsense libraries](https://github.com/IntelRealSense/librealsense) for Camera intrinsics and video streaming to Jetson Xavier
7. [Numpy](https://numpy.org/install/) for transforming quaternions to rotation matrices and other basic operations


## System Overview
* This portion of the system first takes two pieces of information of the ground robot from the Vicon system -its Quaternion and Vicon Frame coordinates - and broadcasts them via a ROS publisher.
* A ROS subcriber subsumed in the python script would process the broadcasted quaternion and translation in real time on Jetson Xavier strapped onto the drone.
* The script then expresses the quaternion in terms of a rotation matrix and subsequently performs homogeneous transformation on the ground robot Vicon coordinates to compute the camera coordinates of the ground robot.
* Since the camera is fixed on the bottom of the drone, the coordinate transformation between drone frame and camera frame is constant and trivial.
* With the help of realsense libraries, the script utilizes [rs2_project_point_to_pixel(...)](https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0) to subsequently obtain the camera intrinsics and calculates the predicted pixel coordinates of the ground robot in pixel space based on the camera coordinates of the said ground robot. 

## System Details
* Initializes Vicon coordinate frame and start tracking your robot.
  * At least three Vicon dots will need to be placed on the robot for the tracker to initialize the robot in the system.
  * Initialize the robot in the system by selecting the three Vicon dots attached to the robot in the 'Object' tab of the interface and enter a name for it.
  * Click 'Track' to start broadcasting the pose and coordinate of the robot through ROS2
* Set up Vicon Bridge
  * Open up a terminal window and in your project `/ros2_ws/src` folder,  enter `source install/setup.bash`
  * Run Vicon bridge with the command `ros2 launch vicon_bridge all_segments.yaml`
  * (Optional) In a new terminal, run the command `ros2 topic list` to see the current available topics
  * In a new terminal, run the command `ros2 topic echo /your_topic_name' to visualize the broadcasted information
* `cd` into the folder that contains `coordinate_transform.py`, run the file using `python3 run coordinate_transform.py`
  
## Coordinate Transformation System Diagram
![alt text](https://github.com/dasc-lab/realsense_coordinate_transform/blob/main/system_diagram.drawio%20(1).png)


World Coordinates and Quaternion of the ground robot -> 4x4 Homogeneous matrix transform -> Coordinates of the ground robot in camera frame -> Project 3D point coordinates to 2D pixel coordinates -> robot appears on the expected coordinates of the image

## Potential Applications
This script can be useful for robotic systems that involve aerial video streaming or image capturing in an indoor setting. For instance, our in-door system involves an overhead drone hovering at a 2-meter altitude, which is the current altitude limit for the Vicon system. The quaternions and coordinates of both the drone and the ground robot will be broadcasted by the Vicon system. A homogeneous transformation will be performed to predict the pixel coordinates of the ground robot in the camera space. Owing to the constant evolving nature of this project, we may update this page if we find a way to apply this system to an outdoor setting.
