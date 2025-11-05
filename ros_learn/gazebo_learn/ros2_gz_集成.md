很好，我可以帮你整一个 **完整的可运行示例项目**，用于 ros_gz_bridge（ROS 2 ↔ Ignition Gazebo (Fortress)）桥接。下面我先整理该教程的完整翻译概要，然后附加一个示例仓库结构 + launch 文件 +桥接配置，你可以拷贝运行。

---

## 一、教程完整翻译概要

以下是官方文档 “Ignition Gazebo (Fortress) — ROS 2 Integration” 的翻译概要，附带注释。来源：官方。 ([Gazebo][1])

### 1. 介绍

本教程说明如何将 ROS 2 系统与 Ignition Gazebo (Fortress) 集成，使二者之间可以**交换数据或命令**。你可以从 ROS 接收命令／数据并应用到仿真中，也可以将仿真中的结果发送到 ROS。 ([Gazebo][1])
关键工具是 ros_gz_bridge。

### 2. ros_gz_bridge

ros_gz_bridge 提供了一个网络桥（network bridge），允许 ROS 2 和 Ignition Transport（Ignition 内部的消息系统）之间进行消息交换。 ([Gazebo][1])
**注意**：目前支持的消息类型是有限的。你需要查看其 README 来确认你希望桥接的类型是否被支持。 ([Gazebo][1])

### 3. 要求（Requirements）

* 你需要已安装 ROS 2 环境 + Ignition Gazebo (Fortress) 环境。 ([Gazebo][1])
* 遵循 “Install Gazebo and ROS” 文档确保兼容性。

### 4. 双向通信（Bidirectional communication）

你可以初始化一个双向桥：ROS → Ignition，或者 Ignition → ROS，或者双向。语法格式如下：

```
ros2 run ros_gz_bridge parameter_bridge /TOPIC@ROS_MSG@IGN_MSG
```

其中：

* `/TOPIC` 是 Ignition 内部话题名称。
* `ROS_MSG` 是 ROS 消息类型。
* `IGN_MSG` 是 Ignition 消息类型。 ([Gazebo][1])
  消息方向标识：
* `@` 表示双向。 ([Gazebo][1])
  举例：

```
ros2 run ros_gz_bridge parameter_bridge /keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32
```

这个例子将关键词 “keypress” 的话题从 Ignition ↔ ROS 进行桥接。

### 5. 备注

教程中还提到可以查看 `ros_gz_sim_demos` 示例仓库，其中包含桥接各种 actuator（执行器）和 sensor（传感器）类型的 demo launch 文件。 ([Gazebo][1])

---

## 二、示例项目结构与配置

下面我给出一个 **简单可运行**的示例项目，用于 ROS 2 + Ignition Fortress 桥接键盘输入。你可以按此结构创建自己项目。

### 项目结构（示例）

```
ros2_ign_bridge_example/
├── world/
│   └── simple_world.sdf
├── launch/
│   └── bridge_launch.py
└── README.md
```

### 文件内容

#### `world/simple_world.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="simple_world">
    <physics name="default_physics" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"/>

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz-gui>
          <title>3D View</title>
        </gz-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <camera_pose>-6 0 6 0 0.5 0</camera_pose>
      </plugin>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- 包含键盘输入插件 -->
    <plugin filename="libignition-gazebo-keyboard-publisher-system.so"
            name="ignition::gazebo::systems::KeyboardPublisher">
      <topic>/keyboard/keypress</topic>
    </plugin>

  </world>
</sdf>
```

#### `launch/bridge_launch.py`

这是 ROS 2 的 launch 文件，用于启动桥接节点。

```python
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 启动 Ignition Gazebo 世界
    ign_world = ExecuteProcess(
        cmd=['ign', 'gazebo', os.path.join(
            os.getcwd(), 'world', 'simple_world.sdf')],
        output='screen'
    )

    # 启动 ros_gz_bridge 桥接命令
    bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/keyboard/keypress@std_msgs/msg/Int32@ignition.msgs.Int32'],
        output='screen'
    )

    return LaunchDescription([
        ign_world,
        bridge
    ])
```

#### `README.md`

````markdown
# ROS2 ↔ Ignition Bridge Example

## 前提条件
- Ubuntu 22.04（示例）
- ROS 2 Humble 安装
- Ignition Gazebo Fortress 安装
- 安装 ros_gz_bridge：`sudo apt install ros-humble-ros-gz-bridge`

## 使用方法
1. 克隆本项目：
   ```bash
   git clone <this-repo>
   cd ros2_ign_bridge_example
````

2. 启动:

   ```bash
   ros2 launch launch/bridge_launch.py
   ```
3. 在 Ignition 窗口中添加 “Key Publisher” 插件（从菜单 Plugins → Key Publisher）。
4. 在另一个终端中监听 ROS 话题：

   ```bash
   ros2 topic echo /keyboard/keypress
   ```
5. 在 GUI 窗口按键，你应当在 ROS 终端看到 Int32 数据被打印出来。

## 扩展

你可以修改桥接的话题、类型，比如：

```
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist
```

然后通过 ROS 2 发 `geometry_msgs/msg/Twist` 消息控制仿真中的机器人。

```

---

## 三、运行步骤总结  
1. 安装 ROS 2 Humble + Ignition Gazebo Fortress，并安装 `ros-humble-ros-gz-bridge`.  
2. 构建或放置上述项目结构。  
3. 启动 launch: `ros2 launch launch/bridge_launch.py`。  
4. 在 Ignition 界面添加 KeyPublisher 插件，按键触发消息。  
5. 在 ROS 终端监听 `/keyboard/keypress` 话题。  
6. 确认 ROS 端接收到数据。

---

如果你愿意，我可以 **打包一个 GitHub 仓库链接**（包含完整源码、说明、Dockerfile）并 **提供可选的机器人控制桥接示例**（如 `/cmd_vel` 控制差速驱动机器人）—你要先从哪个开始？
::contentReference[oaicite:11]{index=11}
```

[1]: https://gazebosim.org/docs/fortress/ros2_integration/?utm_source=chatgpt.com "ROS 2 Integration — Gazebo fortress documentation"
