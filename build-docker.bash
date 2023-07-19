#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

docker build --tag jakobthumm/tum-concert:latest . -f $DIR/Dockerfile "$@"
