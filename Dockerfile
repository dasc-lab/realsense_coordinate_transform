FROM ros:humble-perception

#VICON Bridge set up
#RUN sudo apt-get install libboost-thread-dev libboost-date-time-dev
#RUN sudo apt-get install ros-humble-diagnostic-updater
#WORKDIR /home/nvidia/Documents/workspace/ros2_ws
#RUN colcon build --symlink-install


#pyrealsense camera set up
RUN sudo mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \ 
RUN sudo tee /etc/apt/sources.list.d/librealsense.list
RUN sudo apt-get update


RUN sudo apt-get install librealsense2-dkms
RUN sudo apt-get install librealsense2-utils

