#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CMD="docker run -it --rm --privileged --network host -v $parent_dir/testing_logs:/home/testing/dev_ws/src/testing_logs --name main_testing_container main_testing_image"

bash -c "$CMD"

CMD="docker exec -it main_testing_container bash -c 'source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_talker talker_launch.py'"

bash -c "$CMD"