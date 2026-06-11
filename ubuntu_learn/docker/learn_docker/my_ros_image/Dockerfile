# 第1步：告诉 Docker 我们的“基础食谱”是什么。
FROM ros:noetic

# 第2步：在构建时，一次性完成所有安装
RUN apt-get update && \
    apt-get install -y \
        ros-noetic-rviz \
        libnvidia-gl-535 \
    && rm -rf /var/lib/apt/lists/*
