# Build Docker

```
docker build -t a1_ros2 .
```

## Run Docker
example: 
```
xhost +local:root

docker run -it \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="PATH/TO/ros2_ws:/root/ros2_ws:rw" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --privileged \
    --network=host \
    --name="a1_ros2" \
    unitree_ros2 \
    /bin/bash
```
before run it you should have created your own ros2_ws/src with packages
