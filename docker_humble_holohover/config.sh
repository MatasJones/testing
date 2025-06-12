#!/bin/bash

REMOTE_IPS=("192.168.0.131" "192.168.0.122" "192.168.0.108" "192.168.0.109" "192.168.0.72")
USERS=("ubuntu" "ubuntu" "ubuntu" "ubuntu" "ubuntu" "ubuntu")
MAIN_IP=("192.168.0.130")
SERVER_IP=("192.168.0.72")
MAIN_PORT=("5001")
MAIN_IMAGE_NAME=("main_testing_image:latest")
HOLO_IMAGE_NAME=("holo_testing_image:latest")
HOLO_CONTAINER_NAME=("holo_testing_container")
DOCKERFILE_PATH=(".")
IMAGE_SRC_PATH=("..")