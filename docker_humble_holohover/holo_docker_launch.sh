#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

# CMD="docker run -it --rm --privileged --network host --name holo_testing_container holo_testing_image"
CMD="docker run -it --rm --privileged --network host --name holo_testing_container \
    holo_testing_image \
    bash -c "source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py""


bash -c "$CMD"

# CONTAINER_NAME=holo_testing_container

# until [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME 2>/dev/null)" == "true" ]; do
#     echo "Waiting for container '$CONTAINER_NAME' to be running..."
#     sleep 1
# done

# CMD="docker exec -it holo_testing_container bash -c 'source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py'"

# bash -c "$CMD"