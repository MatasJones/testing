#!/bin/bash

# Get the current working directory
current_dir=$(pwd)

# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")

CONTAINER_NAME="holo_testing_container"

# Run container in detached mode, keep it alive using 'tail -f /dev/null'
docker run -d --privileged --network host --name "$CONTAINER_NAME" holo_testing_image tail -f /dev/null

# Capture the container ID for cleanup
CONTAINER_ID=$(docker ps -q -f name="$CONTAINER_NAME")

# Trap to handle Ctrl+C and clean up
trap 'echo "Stopping and removing container..."; docker stop "$CONTAINER_NAME" > /dev/null; docker rm "$CONTAINER_NAME" > /dev/null; exit' INT

# Wait for the container to be running
until [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" == "true" ]; do
    echo "Waiting for container '$CONTAINER_NAME' to be running..."
    sleep 1
done

# Exec into the container and launch the node
docker exec -it "$CONTAINER_NAME" bash -c "source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py"

# Keep the script running to allow Ctrl+C to trigger trap
wait $!

