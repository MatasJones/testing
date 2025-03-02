#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

IMAGE_NAME="latency_test_image"
CONTAINER_NAME="latency_test_container"

# Stop and remove any images with the same name to avoid conflicts
docker stop $CONTAINER_NAME 2>/dev/null || true
docker rm $CONTAINER_NAME 2>/dev/null || true

# Select docker container
docker run -it \
    --name latency_test \
    --rm \
    --privileged \
    --net=host \
    -v /dev:/dev \


# --rm is used to automatically remove the container when it is closed
# --privileged is used to allow the container to have access to the hardware devices
# --net=host allows the contain to use the host's network
# -v /dev:/dev gives the container access to all of the hardware
