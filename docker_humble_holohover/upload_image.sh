#!/bin/bash

# /**
#  * @brief Important, if you get issues with HTTP/HTTPS, you need to add your
#  * IP:PORT to the daemon.json file in /etc/docker/daemon.json as "insecure-registries" : ["IP:PORT"]
#  *
#  */

source config.sh

echo "Starting upload.."

# REMOTE_IPS[1] corresponds to the black holohover
if ping -c 1 ${REMOTE_IPS[1]} &> /dev/null; then # ping -c 1 ip_address -> sends a ping to the ip address and &> /dev/null redirects any output to /dev/null effectively hiding it
    # Create a command to the devices: to pull the change, retag the image so that it is easier to use localy and prune all unnecessary images
    CMD="docker pull $MAIN_IP:$MAIN_PORT/$IMAGE_NAME && docker tag $MAIN_IP:$MAIN_PORT/$IMAGE_NAME $IMAGE_NAME && docker image prune -f"
    
    echo ssh ${USERS[1]}@${REMOTE_IPS[1]} $CMD
    
    # Send command to devices
    ssh ${USERS[1]}@${REMOTE_IPS[1]} $CMD

else
    echo "Ping to $REMOTE_IPS[1] failed..."

fi # Used to indicated the end of an if statement block


