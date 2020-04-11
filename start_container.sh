#!/bin/bash
if [[ $1 == "cudagl" ]]; then
    docker run -it --rm \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume="`pwd`:/workspace" \
        --gpus=all \
        --name=atlas \
        jiangengdong/openrave:cudagl
elif [[ $1 == "cudavnc" ]]; then 
    docker run -it --rm \
        -p=5900:5900 \
        --volume="`pwd`:/workspace" \
        --gpus=all \
        --name=atlas \
        jiangengdong/openrave:cudavnc
else
    docker run -it --rm \
        --volume="`pwd`:/workspace" \
        --gpus=all \
        --name=atlas \
        jiangengdong/openrave:cudagl
fi