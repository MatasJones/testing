#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CMD="docker run -it --rm --privileged --network host -v $parent_dir/testing_logs:/home/testing/dev_ws/src/testing_logs --name main_testing_container main_testing_image"

bash -c "$CMD"

CONTAINER_NAME=main_testing_container

until [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME 2>/dev/null)" == "true" ]; do
    echo "Waiting for container '$CONTAINER_NAME' to be running..."
    sleep 1
done

CMD="docker exec -it main_testing_container bash -c 'source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_talker talker_launch.py'"

bash -c "$CMD"