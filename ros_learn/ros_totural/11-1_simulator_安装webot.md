好的，这篇文章是关于如何在 Ubuntu 系统上安装 ROS 2 Jazzy 版本的 `webots_ros2` 仿真器接口。我会为你详细讲解，并附上关键的源码/命令。

这篇文章的目标是：在 Ubuntu 上安装 `webots_ros2` 软件包，并运行一个仿真示例。

-----

### 1\. 背景 (Background)

`webots_ros2` 软件包是 ROS 2 和 Webots 仿真器之间的一个接口。它包含几个子包，其中 `webots_ros2_driver` 允许你启动 Webots 并与之通信。这篇教程将指导你如何安装它，并运行一个示例。

### 2\. 先决条件 (Prerequisites)

1.  你需要了解 ROS 2 的基本概念，比如如何创建工作空间（workspace）和功能包（package）。
2.  你需要先安装 Webots 仿真器软件。
      * (或者，`webots_ros2` 也可以在启动时自动帮你下载并安装 Webots，如果它没有检测到已安装的版本。)

### 3\. 多 Webots 版本处理

如果你安装了多个 Webots 版本，`webots_ros2` 会按照以下顺序查找：

1.  `ROS2_WEBOTS_HOME` 环境变量指定的路径。
2.  `WEBOTS_HOME` 环境变量指定的路径。
3.  默认安装路径 (如 `/usr/local/webots` 或 `/snap/webots/current/usr/share/webots`)。
4.  如果都找不到，它会提示你自动安装。

-----

### 4\. 任务1：安装 webots\_ros2

你有两种安装方式：

#### 方式一：通过 apt 包管理器安装 (推荐)

这是最简单的方式，直接安装官方发布的 deb 包。

```bash
# 源码 (命令)
sudo apt-get install ros-jazzy-webots-ros2
```

#### 方式二：从源码编译安装 (获取最新版本)

如果你需要最新的功能或进行开发，可以选择从 Github 源码编译。

**a. 创建 ROS 2 工作空间**

```bash
# 源码 (命令)
mkdir -p ~/ros2_ws/src
```

**b. Source ROS 2 环境**

```bash
# 源码 (命令)
source /opt/ros/jazzy/setup.bash
```

**c. 克隆源码**
进入你的工作空间，克隆 `webots_ros2` 仓库（注意 `--recurse-submodules` 参数用于拉取所有子模块）。

```bash
# 源码 (命令)
cd ~/ros2_ws
git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git src/webots_ros2
```

**d. 安装依赖**
你需要安装 `pip`, `rosdep` 和 `colcon`，然后使用 `rosdep` 来安装所有依赖包。

```bash
# 源码 (命令) - 安装构建工具
sudo apt install python3-pip python3-rosdep python3-colcon-common-extensions

# 源码 (命令) - 初始化并更新 rosdep
sudo rosdep init
rosdep update

# 源码 (命令) - 安装工作空间内的依赖
rosdep install --from-paths src --ignore-src --rosdistro jazzy
```

**e. 编译工作空间**
使用 `colcon` 编译。

```bash
# 源码 (命令)
colcon build
```

**f. Source 你的工作空间**
编译完成后，你需要 source 本地工作空间的配置文件。

```bash
# 源码 (命令)
source install/local_setup.bash
```

-----

### 5\. 任务2：启动 webots\_ros2\_universal\_robot 示例

安装完成后，你可以运行一个示例来测试。

**a. Source ROS 2 环境**
(如果你是通过 apt 安装的，或者新开了一个终端)

```bash
# 源码 (命令)
source /opt/ros/jazzy/setup.bash
```

(如果你是从源码安装的，并且在上一步已经 source 过，可以跳过。如果新开了终端，则需要 source 你的本地工作空间)

```bash
# 源码 (命令) - 对应源码安装
cd ~/ros2_ws
source install/local_setup.bash
```

**b. (可选) 指定 Webots 路径**
如果你想使用特定位置的 Webots，可以设置环境变量：

```bash
# 源码 (命令)
export WEBOTS_HOME=/usr/local/webots
```

**c. 启动示例**
使用 `ros2 launch` 命令启动 `webots_ros2_universal_robot` 包中的 `multirobot_launch.py` 启动文件。

```bash
# 源码 (命令)
ros2 launch webots_ros2_universal_robot multirobot_launch.py
```

-----

总结完毕。这篇文章主要讲解了如何通过 `apt` 或源码两种方式安装 `webots_ros2` 接口，以及如何运行一个示例来验证安装。