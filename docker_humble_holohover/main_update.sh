#!/bin/bash

source config.sh

echo "Updating main image.."

# Rebuild the image to incorporate the changes
CMD="docker buildx build --platform linux/amd64 -t localhost:$MAIN_PORT/$MAIN_IMAGE_NAME -f $DOCKERFILE_PATH/Dockerfile $IMAGE_SRC_PATH"
bash -c "$CMD" # -c flag is used to tell the bash to treat the quoted string as a command and execute it

# Tag the new image
CMD="docker tag $MAIN_IMAGE_NAME localhost:$MAIN_PORT/$MAIN_IMAGE_NAME"

# Push the new image to the local registory
CMD="docker push localhost:$MAIN_PORT/$MAIN_IMAGE_NAME"

bash -c "$CMD"

#CMD="echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc"