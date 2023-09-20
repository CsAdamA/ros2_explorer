FROM althack/ros2:humble-dev 

# Install additional packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
        libgl1-mesa-dev \
        libglew-dev \
        libeigen3-dev \
        libboost-all-dev \
        libopencv-dev \
        libbackward-cpp-dev \
        ros-humble-rviz2 \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        ros-humble-vision-opencv \
        ros-humble-example-interfaces \
        ros-humble-realsense2-* \
        ros-humble-depthai-ros \
        ros-humble-gazebo-* \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-turtlebot3-msgs \
        ros-humble-dynamixel-sdk \
        ros-humble-turtlebot3 \
        ros-humble-geometry2 \
        ros-humble-rqt* \
        ros-dev-tools \
        ros-humble-turtlebot4-simulator \
        nano \
        udev \
    # Clean up
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog
CMD ["bash", "/scripts/udev_reload.sh"]

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

