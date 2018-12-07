## How to build using Docker

1. instal docker - https://docs.docker.com/install/linux/docker-ce/ubuntu/
2. docker build -t tmnt_image .

## How to use

3. docker run -d -p:2022:22 tmnt_image (startup script will now run automatically)
4. You can access the image via ssh -p 2022 root@localhost (password is root)

## Persistent storage

saved_model is the directory that is being used to store the models produced by tf. In order to keep these models even after the container is done you should use a volume mount such as:

```
-v /home/my_user/mymodels:/root/turtlebot2i/src/turtlebot2i_deep_qlearning/dqn/saved_model
```


## Pull from hub.docker.com

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
