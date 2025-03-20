#!/bin/bash

echo "updating and pushing new images.."

CMD="bash main_update.sh"

bash -c "$CMD"

CMD="bash main_upload_image.sh"

bash -c "$CMD"

CMD="bash holo_update.sh"

bash -c "$CMD"

CMD="bash holo_upload_image.sh"

bash -c "$CMD"