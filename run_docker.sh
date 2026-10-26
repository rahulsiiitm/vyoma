#!/bin/bash

# Allow X11 forwarding (so Gazebo and Rviz can open windows on your Arch host)
xhost +local:docker

# Build the docker image
echo "Building Docker image..."
docker build -t drone_ros2_env -f Dockerfile .

# Run the docker container
echo "Starting Docker container..."
docker run -it --rm \
    --name=vyoma-dev \
    --net=host \
    --ipc=host \
    --device=/dev/video0:/dev/video0 \
    --device=/dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/home/drone_user/workspace:rw" \
    drone_ros2_env bash
