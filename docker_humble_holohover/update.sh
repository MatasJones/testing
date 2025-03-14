#!/bin/bash

source config.sh

echo "Updating image.."

# Rebuild the image to incorporate the changes
CMD="docker build -t localhost:$MAIN_PORT/$IMAGE_NAME -f $DOCKERFILE_PATH/Dockerfile $IMAGE_SRC_PATH"

bash -c "$CMD" # -c flag is used to tell the bash to treat the quoted string as a command and execute it

# Push the new image to the local registory
CMD="docker push localhost:$MAIN_PORT/$IMAGE_NAME"

bash -c "$CMD"