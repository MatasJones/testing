#!/bin/bash
# Get the current working directory
current_dir=$(pwd)
# Use dirname to get the parent directory
parent_dir=$(dirname "$current_dir")
CONTAINER_NAME="main_testing_container"

# Remove container if it already exists
docker rm -f "$CONTAINER_NAME" &>/dev/null || true

# Run container in detached mode
echo "$parent_dir"
docker run -d --privileged --network host -v $parent_dir/testing_logs:/home/testing/dev_ws/src/testing_logs --name "$CONTAINER_NAME" main_testing_image tail -f /dev/null

# Wait for the container to be running
until [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" == "true" ]; do
  echo "Waiting for container '$CONTAINER_NAME' to be running..."
  sleep 1
done

if [ -z "$1" ]; then
  ARG1="100"
else
  ARG1="$1"
fi

# Execute ROS2 node in interactive mode (not detached)
echo "Starting ROS2 node..."
docker exec -it "$CONTAINER_NAME" bash -c "source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_talker talker_launch.py spacing_ms:='$ARG1'"

# This will execute after the ROS2 node exits (either normally or via Ctrl+C)
echo "Stopping and removing container..."
docker stop "$CONTAINER_NAME" > /dev/null
docker rm "$CONTAINER_NAME" > /dev/null