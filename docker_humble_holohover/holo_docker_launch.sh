#!/bin/bash
# Get the current working directory
current_dir=$(pwd)
# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")
CONTAINER_NAME="holo_testing_container"

# Remove container if it already exists
docker rm -f "$CONTAINER_NAME" &>/dev/null || true

# Run container in detached mode
docker run -d --privileged --network host --name "$CONTAINER_NAME" holo_testing_image tail -f /dev/null

# Wait for the container to be running
until [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" == "true" ]; do
  echo "Waiting for container '$CONTAINER_NAME' to be running..."
  sleep 1
done

# Execute ROS2 node in interactive mode (not detached)
echo "Starting ROS2 node..."
docker exec -it "$CONTAINER_NAME" bash -c "source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py"

# This will execute after the ROS2 node exits (either normally or via Ctrl+C)
echo "Stopping and removing container..."
docker stop "$CONTAINER_NAME" > /dev/null
docker rm "$CONTAINER_NAME" > /dev/null