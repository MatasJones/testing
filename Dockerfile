FROM ros:humble-ros-base

WORKDIR /root/ros2_ws

COPY ./bin/install.sh /root/install.sh

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    libbox2d-dev \
    libeigen3-dev \
    libmatio-dev \
    libgpiod-dev \
    ros-humble-rosbag2-storage-mcap \
    && apt-get clean autoclean \
    && apt-get autoremove --yes \
    && rm -rf /var/lib/{apt,dpkg,cache,log}/ \
    && echo "source /opt/ros/humble/setup.bash; source /root/ros2_ws/install/local_setup.sh" > /root/source.sh \
    && echo "source /root/source.sh" >> /root/.bashrc \
    && /root/install.sh  \
    && /ros_entrypoint.sh

COPY ./ws/src /root/ros2_ws/src
