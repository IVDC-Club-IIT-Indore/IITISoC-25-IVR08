
#!/bin/bash
if [ "$(docker ps -aq -f name=px4_2-container)" ]; then
    echo "Starting existing px4_2-container..."
    docker start -ai px4_2-container
else
    echo "Creating and starting new px4_2-container..."
    docker run -it \
        --name px4_2-container \
        --device /dev/dri \
        -v $(pwd)/catkin_ws:/root/catkin_ws \
        -v $(pwd)/data:/data \
        -v $(pwd)/px4-dev:/root/px4-dev \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e QT_X11_NO_MITSHM=1 \
        --privileged \
        dual_drone_final2:latest
fi
