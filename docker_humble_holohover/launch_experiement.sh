#!/bin/bash

source config.sh
echo "Starting latency experiment.."

# SSH to listeners and launch nodes
if ping -c 1 ${REMOTE_IPS[1]} &> /dev/null; then 

    # Verify that the container is running
    status=$(ssh ${USERS[1]}@${REMOTE_IPS[1]} "docker inspect -f '{{.State.Status}}' holo_testing_container")

    # If running, continue
    if [ "$status" = "running" ]; then
        echo "Container of holo:${REMOTE_IPS[1]} is running."

    # If not running, launch the container
    else
        echo "Container of holo:${REMOTE_IPS[1]} is not running, launching container..."
        ssh ${USERS[1]}@${REMOTE_IPS[1]} "bash /home/ubuntu/testing/docker_humble_holohover/holo_docker_remote_launch.sh"
        echo "Container of holo:${REMOTE_IPS[1]} is launched."
    fi

    CMD="docker exec -t holo_testing_container bash -c 'if [ -f /home/testing/dev_ws/install/latency_test_listener/share/latency_test_listener/launch/listener_launch.py ]; then echo \"File exists\"; else echo \"File does not exist\"; fi'"

    ssh ${USERS[1]}@${REMOTE_IPS[1]} "$CMD"


    CMD="docker exec holo_testing_container bash -c 'source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py > /dev/null 2>&1 &'"

    ssh ${USERS[1]}@${REMOTE_IPS[1]} "$CMD"

    echo "Listener node launched on holo:${REMOTE_IPS[1]}."

else
    echo "Ping to holo:${REMOTE_IPS[1]} failed..."

fi # Used to indicated the end of an if statement block


# SSH to talker and launch node
if ping -c 1 ${SERVER_IP} &> /dev/null; then 

    # Verify that the container is running
    status=$(ssh ${USERS[1]}@${SERVER_IP} "docker inspect -f '{{.State.Status}}' main_testing_container")

    # If running, continue
    if [ "$status" = "running" ]; then
        echo "Container of main:${SERVER_IP} is running."

    # If not running, launch the container
    else
        echo "Container of main:${SERVER_IP} is not running, launching container..."
        ssh ${USERS[1]}@${SERVER_IP} "bash /home/ubuntu/testing/docker_humble_holohover/main_docker_remote_launch.sh"
        echo "Container of main:${SERVER_IP} is launched."
    fi

    CMD="docker exec main_testing_container bash -c 'source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_talker talker_launch.py > /dev/null '" #2>&1 &'"

    ssh ${USERS[1]}@${SERVER_IP} "$CMD"

    echo "Talker node launched on main:${SERVER_IP}."

else
    echo "Ping to ${SERVER_IP} failed..."

fi # Used to indicated the end of an if statement block

echo "All nodes launched!"

# Launch test

# Retrieve logs

# Graph results

# Terminate experiment

