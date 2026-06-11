
好的，我们来学习这篇教程，这是关于 Webots 仿真的高级主题：`Ros2Supervisor` 节点。

这篇文章的目标是：**使用 `Ros2Supervisor` 节点，从一个标准的 ROS 2 节点（在机器人控制器*之外*) 来监视和控制仿真世界。**

-----

### 背景 (Background)

在之前的教程中，我们创建的 `WebotsController` 节点 (`my_robot_driver`) 是作为机器人*内部*的控制器运行的。

而 `webots_ros2` 包还提供了一个特殊的节点叫 `Ros2Supervisor`。这个节点利用了 Webots 的 "Supervisor API"，它拥有对整个仿真世界的“上帝视角”权限。

`Ros2Supervisor` 节点启动后，会提供一系列 ROS 2 服务 (Services)，允许你：

  * 获取仿真世界中*任何*对象（Node）的句柄。
  * 获取或设置这些对象的位置、旋转等属性（Field）。
  * 控制仿真时间，或者重置整个世界。

本教程将教你如何启动 `Ros2Supervisor` 节点，并创建我们自己的 ROS 2 节点（一个客户端）来调用它提供的服务，以获取机器人的实时位置。

### 先决条件 (Prerequisites)

  * 你已经完成了“基础篇”教程，并拥有 `my_package` 工作空间。

-----

### 任务1：创建 Supervisor 客户端节点

我们将创建一个全新的 Python 节点，它不控制机器人，只负责与 `Ros2Supervisor` 通信。

在你的包路径下创建新文件 `my_package/my_package/my_supervisor_node.py`：

```python
# 源码 (my_package/my_package/my_supervisor_node.py)
# 这是一个标准的 ROS 2 节点
import rclpy
from rclpy.node import Node
from webots_ros2_driver_webots.srv import (
    GetSelf, 
    GetNode, 
    GetField
)

class MySupervisor(Node):
    def __init__(self):
        super().__init__('my_supervisor_node')
        
        # 1. 初始化服务客户端
        # 这些客户端将调用 Ros2Supervisor 提供的服务
        # 注意：服务名称被硬编码为 '/my_robot/...' 
        # (我们稍后需要在 launch 文件中处理这个命名空间)
        self.__robot_node_client = self.create_client(GetSelf, '/my_robot/get_self')
        self.__robot_field_client = self.create_client(GetField, '/my_robot/get_field')
        
        # 等待服务上线
        while not self.__robot_node_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('\'get_self\' service not available, waiting...')
        
        # 2. 创建一个定时器，每 100ms 查询一次机器人位置
        self.create_timer(0.1, self.__timer_callback)
        
        # 用于存储机器人句柄 (ID)
        self.__robot_node = None
        self.__translation_field = None

    def __timer_callback(self):
        # 这个回调函数启动一个异步调用链来获取位置
        
        # 步骤 A: 获取机器人自己的句柄 (Node ID)
        if self.__robot_node is None:
            self.__robot_node_client.call_async(GetSelf.Request()) \
                .add_done_callback(self.__robot_node_callback)
            return # 等待回调完成

        # 步骤 B: 获取 "translation" 属性的句柄 (Field ID)
        if self.__translation_field is None:
            request = GetField.Request()
            request.node = self.__robot_node
            request.field_name = 'translation'
            self.__robot_field_client.call_async(request) \
                .add_done_callback(self.__robot_field_callback)
            return # 等待回调完成
        
        # 步骤 C: 获取 "translation" 属性的 *值*
        # (Webots API 设计如此，需要用同一个 GetField 服务，但传入 Field ID)
        request = GetField.Request()
        request.field = self.__translation_field
        self.__robot_field_client.call_async(request) \
            .add_done_callback(self.__robot_translation_callback)

    def __robot_node_callback(self, future):
        # 步骤 A 的回调：存储机器人句柄
        self.__robot_node = future.result().node
        self.get_logger().info('Robot node handle acquired.')

    def __robot_field_callback(self, future):
        # 步骤 B 的回调：存储 "translation" 属性的句柄
        self.__translation_field = future.result().field
        self.get_logger().info('Translation field handle acquired.')

    def __robot_translation_callback(self, future):
        # 步骤 C 的回调：获取并打印值
        translation = future.result().value.vec3f
        self.get_logger().info(f'Robot position: [{translation.x:.3f}, {translation.y:.3f}, {translation.z:.3f}]')


def main(args=None):
    rclpy.init(args=args)
    supervisor_node = MySupervisor()
    rclpy.spin(supervisor_node)
    supervisor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**代码精炼讲解：**
这段代码的逻辑是一个异步调用链，因为我们不能阻塞 `spin()`：

1.  `__init__`：创建服务客户端，并启动一个定时器。
2.  `__timer_callback` (第一次调用)：触发 `get_self` 服务调用，以获取机器人自身的句柄 (一个内部 ID)。
3.  `__robot_node_callback` (回调1)：存储这个句柄 (`self.__robot_node`)。
4.  `__timer_callback` (第二次调用)：触发 `get_field` 服务调用 (传入机器人句柄和 "translation" 字符串)，以获取 "translation" *属性*的句柄。
5.  `__robot_field_callback` (回调2)：存储这个属性句柄 (`self.__translation_field`)。
6.  `__timer_callback` (后续调用)：触发 `get_field` 服务调用 (传入属性句柄)，以获取该属性的*实际值*（一个 3D 向量）。
7.  `__robot_translation_callback` (回调3)：打印这个 3D 向量值。

-----

### 任务2：编辑 `setup.py`

我们需要为这个新节点添加一个 `entry_point` (入口点)，以便 `ros2 launch` 可以找到它。

打开 `my_package/setup.py`，在 `console_scripts` 列表中添加新的一行：

```python
# 源码 (setup.py - 'entry_points' 部分)
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_package.my_robot_driver:main',
            'obstacle_avoider = my_package.obstacle_avoider:main',
            # 新增下面这行
            'my_supervisor_node = my_package.my_supervisor_node:main',
        ],
    },
```

-----

### 任务3：编辑启动文件 (`robot_launch.py`)

这是本教程最关键的一步。我们需要在 `my_package/launch/robot_launch.py` 中添加**两个**新节点：

1.  **`Ros2Supervisor` 节点**：这是由 `webots_ros2_driver` 提供的*服务服务器*。
2.  **`my_supervisor_node` 节点**：这是我们刚刚编写的*服务客户端*。

**重要提示 (教程陷阱)：**
我们编写的客户端 (`my_supervisor_node.py`) 硬编码了服务名称为 `/my_robot/get_self`。
但是，`Ros2Supervisor` 节点默认发布的服务名称是 `/Ros2Supervisor/get_self`。

因此，在启动我们的客户端节点时，**必须使用 `remappings` (重映射)**，将 `/my_robot` 映射到 `/Ros2Supervisor`，否则我们的客户端将永远找不到服务。

(注：教程的 Python 示例*遗漏*了 `remappings`，导致无法运行；而 C++ 示例包含了 `remappings`。我们这里采用正确的方式。)

打开 `my_package/launch/robot_launch.py`，将其修改为如下内容 (基于上一篇 "Reset Handler" 教程的最终版本)：

```python
# 源码 (my_package/launch/robot_launch.py)

import os
import launch
from launch_ros.actions import Node # 确保 Node 已导入
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

# (Reset Handler 函数不变)
def get_nodes_to_respawn(*args):
    obstacle_avoider_node = Node(
        package='my_package',
        executable='obstacle_avoider',
        output='screen' # 建议添加 output
    )
    return [ obstacle_avoider_node ]

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt')

    # 1. 启动 Webots (不变)
    webots = WebotsLauncher(
        world=world_path
    )

    # 2. 启动机器人驱动 (不变, 保持 respawn=True)
    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
            {'use_sim_time': True} # 最好统一使用仿真时间
        ],
        respawn=True
    )
    
    # 3. (新增) 启动 Webots 官方的 Supervisor 服务节点
    ros2_supervisor = Node(
        package='webots_ros2_driver',
        executable='ros2_supervisor',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # 4. (新增) 启动我们自己的 Supervisor 客户端节点
    my_supervisor_node = Node(
        package='my_package',
        executable='my_supervisor_node', # 对应 setup.py
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        # 关键：将我们代码中的 /my_robot/... 映射到 supervisor 提供的 /Ros2Supervisor/...
        remappings=[
            ('/my_robot/get_self', '/Ros2Supervisor/get_self'),
            ('/my_robot/get_field', '/Ros2Supervisor/get_field'),
            # (如果使用其他服务，也需要在这里添加映射)
        ]
    )

    # 5. Reset Handler (不变)
    reset_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=my_robot_driver,
            # 注意：我们的 supervisor 客户端也应该在重置时重启
            on_exit=get_nodes_to_respawn + [my_supervisor_node], 
        )
    )
    
    # (教程中没有处理 supervisor 的重置，这里我为你改进了)
    # (更稳健的方法是将 my_supervisor_node 也加入 get_nodes_to_respawn 函数)

    return LaunchDescription([
        webots,
        my_robot_driver,
        ros2_supervisor,      # 添加 Supervisor 服务
        my_supervisor_node,   # 添加 Supervisor 客户端
        reset_handler
    ] + get_nodes_to_respawn()) # 启动避障节点
```

*(注意：为了在重置时也重启 `my_supervisor_node`，我稍微修改了 `reset_handler`。教程原文并未处理这一点。)*

-----

### 任务4：测试代码

现在，编译并运行。

**a. 编译 (回到工作空间根目录 `~/ros2_ws`)**

```bash
# 源码 (命令)
cd ~/ros2_ws
colcon build
```

**b. Source 环境**

```bash
# 源码 (命令)
source install/local_setup.bash
```

**c. 运行启动文件**

```bash
# 源码 (命令)
ros2 launch my_package robot_launch.py
```

**预期输出：**
在启动的终端中，你应该会看到机器人自动开始避障，同时，`my_supervisor_node` 节点的日志会开始滚动，**持续打印机器人的 3D 坐标**：

```
[INFO] [my_supervisor_node]: Robot node handle acquired.
[INFO] [my_supervisor_node]: Translation field handle acquired.
[INFO] [my_supervisor_node]: Robot position: [0.000, 0.000, 0.025]
[INFO] [my_supervisor_node]: Robot position: [0.003, 0.000, 0.025]
[INFO] [my_supervisor_node]: Robot position: [0.006, 0.000, 0.025]
...
```

这证明了我们的 `my_supervisor_node` 节点（一个外部节点）已成功通过 `Ros2Supervisor` 节点（服务）获取了仿真中机器人的实时位置。