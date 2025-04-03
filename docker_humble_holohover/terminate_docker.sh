#!/bin/bash
source config.sh

# SSH to listeners and launch nodes
if ping -c 1 ${REMOTE_IPS[1]} &> /dev/null; then 

    # Verify that the container is running
    status=$(ssh ${USERS[1]}@${REMOTE_IPS[1]} "docker stop holo_testing_container")
    echo "Container of ${REMOTE_IPS[1]} is stopped."

fi # Used to indicated the end of an if statement block
