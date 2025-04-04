#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CONTAINER_NAME="holo_testing_container"

# Run container in detached mode, keep it alive using 'tail -f /dev/null'
docker run -it --privileged --network host --name "$CONTAINER_NAME" holo_testing_image tail -f /dev/null

# Capture the container ID for cleanup
CONTAINER_ID=$(docker ps -q -f name="$CONTAINER_NAME")

# Trap to handle Ctrl+C and clean up
trap 'echo "Stopping and removing container..."; docker stop "$CONTAINER_NAME" > /dev/null; docker rm "$CONTAINER_NAME" > /dev/null; exit' INT

# Wait for the container to be running
until [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" == "true" ]; do
    echo "Waiting for container '$CONTAINER_NAME' to be running..."
    sleep 1
done

# Now, execute everything inside the container directly
docker exec -d "$CONTAINER_NAME" bash -c "source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py"

# Wait for the container process (this helps with trapping Ctrl+C and ensures cleanup)
wait


