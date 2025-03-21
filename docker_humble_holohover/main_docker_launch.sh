#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CMD="docker run -it --rm --privileged --network host -v $parent_dir:/home/testing/dev_ws/src --name main_testing_image main_testing_image"

bash -c "$CMD"