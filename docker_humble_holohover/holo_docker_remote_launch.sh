#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CMD="docker run -dt --rm --privileged --network host -v $parent_dir:/home/testing/dev_ws/src --name holo_testing_container holo_testing_image"

bash -c "$CMD"