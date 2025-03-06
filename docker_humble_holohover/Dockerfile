# Use the official ROS 2 Humble image for ARM64 (Raspberry Pi)
FROM ros:humble-ros-base AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=0 \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

# Define the username as a build argument (fixing UndefinedVar issue)
ARG USERNAME=testing
ENV USERNAME=${USERNAME}

# Update package list and install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory for the user
WORKDIR /home/${USERNAME}/dev_ws/src

# Copy project files into the container
COPY . .

WORKDIR /home/$USERNAME/dev_ws/src

RUN rosdep update \
    && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

# Set the default command to launch a ROS 2 container shell
CMD ["/bin/bash"]


