


#xhost + >/dev/null
xhost +local:root

# 先在宿主机运行一次，授权 Docker root 用户访问 X11
# xhost +SI:localuser:root
# --cap-add=SYS_PTRACE --security-opt seccomp=unconfined \
# 然后运行容器
docker run --rm -it \
    --gpus all \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--env="NVIDIA_VISIBLE_DEVICES=all" \
    --cap-add=SYS_PTRACE --security-opt seccomp=unconfined \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $XAUTHORITY:/root/.Xauthority:rw \
    -v /etc/localtime:/etc/localtime:ro \
    -v /home/jzm/workspace/turtlebox_learn/carto_learn/cartographer_ros:/workspace \
    -e http_proxy="http://127.0.0.1:7890" \
    -e https_proxy="http://127.0.0.1:7890" \
    -e no_proxy="localhost,127.0.0.1" \
    -e LOCAL_UID=$(id -u) \
    -e LOCAL_GID=$(id -g) \
    -e LOCAL_USER=$(id -un) \
    -e LOCAL_GROUP=$(id -gn) \
    my_ros:ros_noetic_cartographer \
    bash -c "cd /workspace && exec bash"


