FROM ros:noetic-ros-core-focal
SHELL ["/bin/bash", "-c"]

#install tools
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install -y build-essential cmake git libpoco-dev libeigen3-dev apt-utils nano pip terminator gedit spyder3 \
    ros-noetic-turtlesim bash-completion -y \
    python3-rosdep \
    python3-pip
	
RUN apt update && apt install -y ros-noetic-turtlebot3
    
RUN source /opt/ros/noetic/setup.sh
    
        

