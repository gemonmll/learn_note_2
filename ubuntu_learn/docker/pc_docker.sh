#!/usr/bin/env bash
set -e

NAME="hds_pc_docker"
IMAGE="hds_pc_sim:v0.1"

usage() {
  echo "Usage: $0 [clean]"
  echo "  clean  : remove container $NAME (if exists)"
}

clean() {
  if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "[clean] removing container: $NAME"
    docker rm -f "$NAME" >/dev/null
  else
    echo "[clean] container not found: $NAME"
  fi
}

case "${1:-}" in
  clean)
    clean
    exit 0
    ;;
  "" )
    ;;
  * )
    usage
    exit 1
    ;;
esac

# X11 授权（按你现在的习惯）
xhost +local:root >/dev/null

if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  exec docker exec -it "$NAME" bash
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  docker start "$NAME" >/dev/null
  exec docker exec -it "$NAME" bash
fi

# 容器不存在：创建并进入
exec docker run -it \
    --name "$NAME" \
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
    -v /home/jzm/workspace/vision_work/c801_hds_work/c801_pc_code:/codepath  \
    "$IMAGE" \
    bash
