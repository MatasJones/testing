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
RUN apt-get update && apt-get install --no-install-recommends -y \
    libbox2d-dev \
    libeigen3-dev \
    libmatio-dev \
    libgpiod-dev \
    libyaml-cpp-dev \
    ros-humble-rosbag2-storage-mcap \
    cmake \
    g++ \
    git \
 && apt-get clean autoclean \
 && apt-get autoremove --yes \
 && rm -rf /var/lib/apt/lists/* /var/lib/dpkg/* /var/cache/* /var/log/* \
 && echo "source /opt/ros/humble/setup.bash; source /home/${USERNAME}/dev_ws/install/local_setup.sh" > /root/source.sh \
 && echo "source /root/source.sh" >> /root/.bashrc \
 && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && git clone https://github.com/google/flatbuffers.git /opt/flatbuffers \
 && cd /opt/flatbuffers \
 && cmake -B build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build --target install


# Set the working directory for the user
WORKDIR /home/${USERNAME}/dev_ws/src

# Copy project files into the container
COPY . .

WORKDIR /home/$USERNAME/dev_ws

RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# RUN rosdep update \
#     && rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y -r

# Set the default command to launch a ROS 2 container shell
CMD ["/bin/bash"]




