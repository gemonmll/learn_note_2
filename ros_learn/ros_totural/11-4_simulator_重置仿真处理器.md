好的，我们来学习这篇关于 "Reset Handler" (仿真重置处理器) 的教程。

这篇文章的目标是：**当用户在 Webots 界面点击“重置仿真”按钮时，自动重启相关的 ROS 2 节点，使仿真能继续运行。**

-----

### 背景 (Background)

在之前的教程中，我们启动了 `webots` (仿真器)、`my_robot_driver` (插件驱动) 和 `obstacle_avoider` (避障节点)。

**问题在于：** 当你在 Webots 图形界面中点击“重置”按钮时，Webots 会杀死并重启它的控制器。在 ROS 2 体系中，这意味着 `WebotsController` 节点 (`my_robot_driver`) 会被终止。然而，`obstacle_avoider` 这样的标准 ROS 2 节点**不会**被终止。

这会导致一个“断开”的状态：`obstacle_avoider` 节点仍在运行并发布 `/cmd_vel`，但 `my_robot_driver` 已经死了（或者即使重启了，`obstacle_avoider` 也是旧的实例），仿真无法继续。

本教程提供了几种解决此问题的策略。

-----

### 策略1：简单情况 (只有控制器)

如果你的启动文件只启动了 `Webots` 和 `WebotsController` (即我们的 `my_robot_driver`，没有 `obstacle_avoider` 节点)，那么解决方案非常简单。

**解决方案：** 只需在 `WebotsController` 节点中添加 `respawn=True` 参数。

```python
# 源码 (robot_launch.py - 简单情况)
# ...
my_robot_driver = WebotsController(
    robot_name='my_robot',
    parameters=[
        {'robot_description': robot_description_path}
    ],
    # 关键点：当此节点退出时（例如 Webots 重置时），自动重新启动它
    respawn=True
)

return LaunchDescription([
    webots,
    my_robot_driver
])
```

-----

### 策略2：多节点，无需关闭 (适用于我们的情况)

**这正是我们 `my_package` (基础篇 + 高级篇) 所处的情况。** 我们有 `my_robot_driver` (控制器) 和 `obstacle_avoider` (普通节点)。两个都需要在重置时重启。

**解决方案：**

1.  `WebotsController` (`my_robot_driver`) 仍然设置 `respawn=True`。
2.  我们创建一个辅助函数 (例如 `get_nodes_to_respawn`)，它返回一个包含所有*其他*需要重启的节点（即 `obstacle_avoider`）的列表。
3.  我们使用一个事件处理器 `RegisterEventHandler`，它会监听 `my_robot_driver` 节点的 `OnProcessExit` (进程退出) 事件。
4.  当 `my_robot_driver` 退出时（即 Webots 重置时），`reset_handler` 会被触发，并调用 `get_nodes_to_respawn` 函数，从而重新启动 `obstacle_avoider` 节点。
5.  这个函数返回的节点列表也必须在 `LaunchDescription` 中返回，以便在*初始*启动时运行它们。

**修改 `my_package/launch/robot_launch.py` 文件：**

(这个方案**不需要**创建新的 Python 节点文件，也**不需要**修改 `setup.py`，它纯粹是 `launch` 文件的改动。)

```python
# 源码 (robot_launch.py - 适用于我们的高级教程)

import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
# 导入事件处理相关的模块
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

# 1. 创建一个函数，返回所有需要重置时重启的 ROS 节点
# (除了 WebotsController 之外)
def get_nodes_to_respawn(*args):
    obstacle_avoider_node = Node(
        package='my_package',
        executable='obstacle_avoider',
    )
    # 返回一个列表
    return [ obstacle_avoider_node ]

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt')

    # 2. 启动 Webots (不变)
    webots = WebotsLauncher(
        world=world_path
    )

    # 3. 启动机器人驱动，并设置 respawn=True
    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True  # 关键点 1: 确保驱动自我重启
    )

    # 4. 注册事件处理器
    reset_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=my_robot_driver,  # 监听 my_robot_driver 节点的退出事件
            on_exit=get_nodes_to_respawn,   # 当它退出时，调用此函数来启动新节点
        )
    )

    # 5. 返回 LaunchDescription
    return LaunchDescription([
        webots,
        my_robot_driver,
        reset_handler  # 添加事件处理器
    ] + get_nodes_to_respawn()) # 关键点 2: 初始启动时也调用此函数
```

-----

### 策略3：需要关闭节点的复杂情况 (如 Nav2, RViz)

如果你的系统非常复杂，包含像 Nav2 或 RViz 这样不能简单“重启”的节点，教程推荐使用手动重启的方案。

**解决方案 (概念):**

1.  **启动文件 A (`webots_launch.py`)：** 只启动 Webots。
2.  **启动文件 B (`ros_stack_launch.py`)：** 启动所有其他节点（`WebotsController` (不带 `respawn=True`)、Nav2、RViz 等）。
3.  在**启动文件 B** 中添加一个 `OnProcessExit` 处理器，监听 `WebotsController` 的退出。
4.  当它退出时，处理器触发一个**全局关闭事件 (`Shutdown()`)**，这将关闭启动文件 B 中的所有节点（Nav2, RViz 等）。

**工作流程：**

1.  终端1: `ros2 launch my_package webots_launch.py`
2.  终端2: `ros2 launch my_package ros_stack_launch.py`
3.  (在 Webots 中点击 "Reset")
4.  终端2 中的所有进程关闭。
5.  用户必须在**终端2**中**手动**按 "上箭头" 重新运行 `ros2 launch my_package ros_stack_launch.py`。

-----

### 总结与测试 (针对策略2)

对于我们当前的 `my_package`，策略2 是最合适的。

**测试：**

1.  返回工作空间根目录 (`~/ros2_ws`)。
2.  编译：`colcon build`
3.  Source: `source install/local_setup.bash`
4.  启动：`ros2 launch my_package robot_launch.py`
5.  机器人应该会开始自动避障。
6.  点击 Webots 窗口中的“重置仿真”按钮（通常是一个“返回开头”的图标）。
7.  观察：机器人应该会回到初始位置，然后**再次开始自动避障**，证明 `my_robot_driver` 和 `obstacle_avoider` 都已成功重启。