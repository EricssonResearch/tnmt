## How to build using Docker

1. instal docker - https://docs.docker.com/install/linux/docker-ce/ubuntu/
2. docker build -t tmnt_image .
3. docker run -it --entrypoint=/bin/bash tmnt_image
4. /root/startup.sh (or /root/startup.sh >> /root/ros.out & to start it in the background)

## Pull from docker.hub

```
docker pull kappavita/projectcs18:latest
```

## To test

```
rostopic list # you should see channels odom and scan
```

```
# these channels should be active

rostopic echo /odom 
rostopic echo /scan
```
