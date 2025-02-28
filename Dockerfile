# Use the official ROS 2 Humble image for ARM64 (Raspberry Pi)
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=0 \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Update package list and install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up the ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to launch a ROS 2 container shell
CMD ["/bin/bash"]

