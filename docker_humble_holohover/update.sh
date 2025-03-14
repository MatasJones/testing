#!/bin/bash

source config.sh

echo "Updating image.."
CMD="docker build -t localhost:$MAIN_PORT/$IMAGE_NAME -f $DOCKERFILE_PATH/Dockerfile .."

bash -c "$CMD" # -c flag is used to tell the bash to treat the quoted string as a command and execute it

CMD="docker push localhost:$MAIN_PORT/$IMAGE_NAME"

bash -c "$CMD"