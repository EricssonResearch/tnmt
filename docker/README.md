## How to build using Docker

1. instal docker - https://docs.docker.com/install/linux/docker-ce/ubuntu/
2. docker build -t tmnt_image .
3. docker run -it --entrypoint=/bin/bash tmnt_image

Known issues ->

1. scan topic not always running
2. scan topic may not contain any data
3. docker build may fail -> to fix increase memory and swap file used by the docker daemon
