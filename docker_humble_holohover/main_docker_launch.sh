#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CMD="docker run -it --rm --privileged --network host -v $parent_dir/testing_logs:/home/testing/dev_ws/src/testing_logs --name main_testing_container main_testing_image"

bash -c "$CMD"