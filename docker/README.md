## How to build using Docker

1. instal docker - https://docs.docker.com/install/linux/docker-ce/ubuntu/
2. docker build -t tmnt_image .
3. docker run -it --entrypoint=/bin/bash tmnt_image
4. /root/startup.sh (or /root/startup.sh >> /root/ros.out & to start it in the background)

Known issues ->

1. docker build may fail -> to fix increase memory and swap file used by the docker daemon
