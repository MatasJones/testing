#!/bin/bash

source config.sh
echo "Starting experiment latency.."

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
        ssh ${USERS[1]}@${REMOTE_IPS[1]} "bash /home/ubuntu/testing/docker_humble_holohover/holo_docker_launch.sh"
    fi

    #CMD="docker exec -it holo_testing_container bash -c 'source /home/ubuntu/holohover_ws/install/setup.bash && ros2 launch holohover_launch holohover.launch.py'"
    
    # Send command to devices
    #ssh ${USERS[1]}@${REMOTE_IPS[1]} $CMD

else
    echo "Ping to $SERVER_IP failed..."

fi # Used to indicated the end of an if statement block


# SSH to talker and launch node



# Launch test

# Retrieve logs

# Graph results

# Terminate experiment

