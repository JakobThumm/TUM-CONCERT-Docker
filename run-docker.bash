#!/bin/bash

nvidia-docker run -it --gpus all \
 --env="DISPLAY=$DISPLAY" \
 --net=host \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="$HOME/.ssh:/home/user/.ssh:ro" \
 --mount type=bind,source=$SSH_AUTH_SOCK,target=/ssh-agent \
 --env SSH_AUTH_SOCK=/ssh-agent \
 --name concert_description_april2024 \
 jakobthumm/tum-concert:latest \
 x-terminal-emulator
