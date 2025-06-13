#!/bin/bash

# /**
#  * @brief Important, if you get issues with HTTP/HTTPS, you need to add your
#  * IP:PORT to the daemon.json file in /etc/docker/daemon.json as "insecure-registries" : ["IP:PORT"]
#  *
#  */

source config.sh

echo "Starting upload.."
for i in {0..3}; do
# REMOTE_IPS[1] corresponds to the black holohover
    if ping -c 1 ${REMOTE_IPS[i]} &> /dev/null; then # ping -c i ip_address -> sends a ping to the ip address and &> /dev/null redirects any output to /dev/null effectively hiding it
        # Create a command to the devices: to pull the change, retag the image so that it is easier to use localy and prune all unnecessary images
        CMD="docker pull $MAIN_IP:$MAIN_PORT/$HOLO_IMAGE_NAME && docker tag $MAIN_IP:$MAIN_PORT/$HOLO_IMAGE_NAME $HOLO_IMAGE_NAME && docker image prune -f"
        
        echo ssh ${USERS[i]}@${REMOTE_IPS[i]} $CMD
        
        # Send command to devices
        ssh ${USERS[i]}@${REMOTE_IPS[i]} $CMD

        if [ $? -ne 0 ]; then
            echo "Error: Failed to upload image to ${REMOTE_IPS[i]}"
            exit 1  # Exit the script if an error occurs
        fi

        echo "Image uploaded to ${REMOTE_IPS[i]} successfully."

    else
        echo "Ping to $REMOTE_IPS[i] failed..."

    fi # Used to indicated the end of an if statement block
done