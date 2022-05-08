#!/bin/bash

image="pc_downsampling"
tag="latest"

docker build . \
    -t $image:$tag \
    --build-arg CACHEBUST=$(date +%s)