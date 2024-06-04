#!/bin/bash

# Find the most idle GPU
idle_gpu=0

carla_cmd="./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=5000 -quality-level=Epic && /bin/bash"

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

VOLUMES="--volume=$XSOCK:$XSOCK:rw --volume=$XAUTH:$XAUTH:rw --volume=$SHDIR:$SHDIR:rw"

docker run --name="carla-$USER" \
  -d --rm \
  --privileged \
  --gpus "device=$idle_gpu" \
  --env="DISPLAY=${DISPLAY}" \
  --env="XAUTHORITY=${XAUTH}" \
  $VOLUMES \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.14 \
  $carla_cmd