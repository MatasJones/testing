#!/bin/sh

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CONTAINER_NAME="latency_test_container"

# Select Docker container parameters
# --rm is used to automatically remove the container when it is closed
# --privileged is used to allow the container to have access to the hardware devices
# --net=host allows the container to use the host's network
# -v /dev:/dev gives the container access to all hardware devices

docker run -it \
    --name $CONTAINER_NAME \
    --rm \
    --privileged \
    -v /dev:/dev \
    -h testing_image:latest \
     testing_image:latest 
    #-v $parent_dir/testing_logs:/home/testing/dev_ws/src \
   
