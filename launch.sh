#!/bin/bash

function help {
  echo "usage: ./launch.sh [--no-graphics]"
  exit 1
}


if [ $# -ne 0 ]; then
  if [ $1 = "--help" ]; then
  	help
  fi
  
  if [ $1 = "--no-graphics" ]; then
    echo "Running in no-graphics mode"
    DOCKER_GPU_ARGS=""
    shift
    DOCKER_APP="/bin/bash"
  fi
else
    echo "Running in graphics mode"
    export containerId=$(docker ps -l -q)
    xhost +local:root # to allow GUI
    DOCKER_APP="terminator"
fi

XAUTH=/tmp/.docker.xauth

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

# Settings required for having graphic mode
DOCKER_GPU_ARGS="--env DISPLAY --env QT_X11_NO_MITSHM=1 --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --ipc=host --privileged --env="XAUTHORITY=$XAUTH""
DOCKER_COMMAND="docker run -it"
DOCKER_NETWORK_ARGS="--net=host"
DOCKER_VOLUME_ARGS="-v $PWD:/home -e HOME=/home -w /home"
DOCKER_IMAGE="ros_ws:noetic"
set -x
$DOCKER_COMMAND \
$DOCKER_NETWORK_ARGS \
$DOCKER_VOLUME_ARGS \
$DOCKER_GPU_ARGS \
$DOCKER_IMAGE \
$DOCKER_APP


