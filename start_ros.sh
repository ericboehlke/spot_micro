#!/bin/sh
export UID=$(id -u)
export GID=$(id -g)
docker run -it --net=host \
  --privileged \
  -e USER=$USER \
  -e CONTAINER_NAME=ros-container \
  --runtime=nvidia \
  --rm \
  -v "/home/$USER/:/home/spot/" \
  --name=ros2-foxy \
  spot:foxy
