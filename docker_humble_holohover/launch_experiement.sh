#!/bin/bash

source config.sh
echo "Starting latency experiment.."

# SSH to listeners and launch nodes
if ping -c 1 ${REMOTE_IPS[1]} &> /dev/null; then 

    # Verify that the container is running
    status=$(ssh ${USERS[1]}@${REMOTE_IPS[1]} "docker inspect -f '{{.State.Status}}' holo_testing_container")

    # If running, continue
    if [ "$status" = "running" ]; then
        echo "Container of ${REMOTE_IPS[1]} is running."

    # If not running, launch the container
    else
        echo "Container of ${REMOTE_IPS[1]} is not running, launching container..."
        ssh ${USERS[1]}@${REMOTE_IPS[1]} "bash /home/ubuntu/testing/docker_humble_holohover/holo_docker_remote_launch.sh"
        echo "Container of ${REMOTE_IPS[1]} is launched."
    fi

    CMD="docker exec -t holo_testing_container bash -c 'if [ -f /home/testing/dev_ws/install/latency_test_listener/share/latency_test_listener/launch/listener_launch.py ]; then echo \"File exists\"; else echo \"File does not exist\"; fi'"

    ssh ${USERS[1]}@${REMOTE_IPS[1]} "$CMD"


    CMD="docker exec holo_testing_container bash -c 'source /home/testing/dev_ws/install/setup.bash && ros2 launch latency_test_listener listener_launch.py'"
     #> /dev/null 2>&1 &'"

    ssh ${USERS[1]}@${REMOTE_IPS[1]} "$CMD"

    echo "Listener node launched on ${REMOTE_IPS[1]}."

else
    echo "Ping to ${REMOTE_IPS[1]} failed..."

fi # Used to indicated the end of an if statement block


# SSH to talker and launch node



# Launch test

# Retrieve logs

# Graph results

# Terminate experiment

