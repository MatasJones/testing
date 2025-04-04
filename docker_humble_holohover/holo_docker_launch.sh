#!/bin/bash

# # Get the current working directory
# current_dir=$(pwd)

# # Use dirname to get the parent directory
# parent_dir=$(dirname "$current_dir")

# CMD="docker run -dit --privileged --network host --name holo_testing_container holo_testing_image tail -f /dev/null"

# bash -c "$CMD"

# CONTAINER_NAME=holo_testing_container

# trap 'echo "Stopping and removing container..."; docker rm "$CONTAINER_NAME" --force > /dev/null; docker rm "$CONTAINER_NAME" > /dev/null; exit' INT

# until [ "$(docker inspect -f '{{.State.Running}}' $CONTAINER_NAME 2>/dev/null)" == "true" ]; do
#     echo "Waiting for container '$CONTAINER_NAME' to be running..."
#     sleep 1
# done

# CMD="docker exec -it holo_testing_container bash -c 'source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py'"

# bash -c "$CMD"

set -e  # Exit on any error

current_dir=$(pwd)
parent_dir=$(dirname "$current_dir")
CONTAINER_NAME=holo_testing_container

# Start container in background (detached) with 'tail -f /dev/null' to keep it alive
docker run -dit --privileged --network host --name "$CONTAINER_NAME" holo_testing_image tail -f /dev/null

# Capture the Docker container's ID (for cleanup)
CONTAINER_ID=$(docker ps -q -f name="$CONTAINER_NAME")

# Trap Ctrl+C (INT) and clean up the container
trap 'echo "Stopping and removing container..."; docker stop "$CONTAINER_NAME" > /dev/null; docker rm "$CONTAINER_NAME" > /dev/null; exit' INT

# Wait for the container to be running
until [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" == "true" ]; do
    echo "Waiting for container '$CONTAINER_NAME' to be running..."
    sleep 1
done

# Exec into the container and launch the node
docker exec -it "$CONTAINER_NAME" bash -c "source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py"

# Keep the script running so the trap works (you can manually Ctrl+C to exit)
wait $!
