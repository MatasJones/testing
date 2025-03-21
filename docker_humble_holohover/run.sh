#!/bin/sh

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

# Select Docker container parameters
# --rm is used to automatically remove the container when it is closed
# --privileged is used to allow the container to have access to the hardware devices
# --net=host allows the container to use the host's network
# -v /dev:/dev gives the container access to all hardware devices

docker run -it \
    --rm \
    --privileged \
    -v /dev:/dev \
    -v $parent_dir:/home/testing/dev_ws/src \
    --name main_testing_image localhost:5001/main_testing_image
   
