#!/bin/bash

REMOTE_IPS=("192.168.0.131" "192.168.0.122" "192.168.0.136" "192.168.0.108" "192.168.0.109")
USERS=("ubuntu" "ubuntu" "ubuntu" "ubuntu" "ubuntu")
MAIN_IP=("192.168-0.130")
MAIN_PORT=("5001")
IMAGE_NAME=("testing_image:latest")

if ping -c 1 ${REMOTE_IPS[i]} &> /dev/null; then # ping -c 1 ip_address -> sends a ping to the ip address and &> /dev/null redirects any output to /dev/null effectively hiding it
    # Create a command to the devices: to pull the change, retag the image so that it is easier to use localy and prune all unnecessary images
    CMD = "docker pull $MAIN_IP:$MAIN_PORT/$IMAGE_NAME && docker tag $MAIN_IP:$MAIN_PORT/$IMAGE_NAME $IMAGE_NAME && docker image prune -f"
    echo ssh ${USERS[1]@$REMOTE_IPS[1]} $CMD
    
    # Send command to devices
    ssh ${USERS[1]@$REMOTE_IPS[1]} $CMD

else
    echo "Ping to $REMOTE_IPS[2] failed..."

fi # Used to indicated the end of an if statement block


