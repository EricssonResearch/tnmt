#!/bin/bash

nvidia-docker run --net=host --rm -v /home/ekonvan:/data --name=rl-ekonvan -it -p 9090:9090 tnmt_image
