#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)
USER_NAME=aiformula
HOST_NAME=aiformula

docker run -it \
    -v ${SCRIPT_DIR}/..:/home/${USER_NAME}/workspace/ros/src/aiformula \
    --add-host ${HOST_NAME}:127.0.0.1 \
    --hostname ${HOST_NAME} \
    --ipc host \
    -e XMODIFIERS \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v /media:/media \
    -u ${USER_NAME} \
    -w /home/${USER_NAME}/workspace/ros \
    --gpus all \
    --privileged \
    --net host \
    --name aiformula_foxy_amd \
    aiformula:foxy_amd
