好的，这篇教程是上一篇“基础篇”的延续。

这篇文章的目标是：**扩展上一篇教程中的机器人仿真，增加一个独立的 ROS 2 节点来实现避障功能。**

这篇教程的核心在于演示如何使用 `webots_ros2_driver` 自动为 Webots 中的传感器（如距离传感器）创建 ROS 2 接口（话题），然后编写一个普通的 ROS 2 节点来订阅这些话题并发布控制指令。

-----

### 先决条件 (Prerequisites)

  * 你必须已经完成了上一篇“基础篇”教程 (Setting up a robot simulation (Basic))，因为本教程是在那个 `my_package` 包的基础上进行修改。

-----

### 任务1：更新 `my_robot.urdf`

在“基础篇”中，URDF 文件只是用来加载我们的自定义插件。现在，我们要利用 `webots_ros2_driver` 的一个强大功能：自动为 Webots 设备创建 ROS 接口。

`webots_ros2_driver` 会自动查找 Webots 世界中机器人的设备（如摄像头、雷达、距离传感器）。如果我们不在 URDF 中配置它们，它会使用默认值（默认话题名等）创建接口。

但在这里，我们希望自定义这些接口。

打开 `my_package/resource/my_robot.urdf` 文件，用以下内容**替换**它：

```xml
<?xml version="1.0" ?>
<robot name="My robot">
    <webots>
        <device reference="ds0" type="DistanceSensor">
            <ros>
                <topicName>/left_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        
        <device reference="ds1" type="DistanceSensor">
            <ros>
                <topicName>/right_sensor</topicName>
                <alwaysOn>true</alwaysOn>
            </ros>
        </device>
        
        <plugin type="my_package.my_robot_driver.MyRobotDriver" />
        
        </webots>
</robot>
```

  * `<device reference="ds0" type="DistanceSensor">`：
      * `reference="ds0"`：这必须与 Webots 世界文件 (`.wbt`) 中机器人节点下的设备 `name` 字段完全匹配。
      * `type="DistanceSensor"`：告诉 `webots_ros2_driver` 这是什么类型的设备。
  * `<ros>` 标签：
      * `<topicName>/left_sensor</topicName>`：指定 ROS 话题名称。`webots_ros2_driver` 会自动将 `ds0` 的数据（`sensor_msgs/msg/Range` 类型）发布到这个话题。

现在，`webots_ros2_driver` 启动时，会做两件事：

1.  加载我们的 `MyRobotDriver` 插件（它订阅 `/cmd_vel`）。
2.  自动创建两个发布者，将 `ds0` 和 `ds1` 传感器数据分别发布到 `/left_sensor` 和 `/right_sensor` 话题。

-----

### 任务2：创建避障 ROS 节点

现在我们不再需要上一篇教程中手动的 `ros2 topic pub` 了。我们将创建一个**标准**的 ROS 2 节点，它订阅传感器话题并发布到 `/cmd_vel`，形成一个完整的控制闭环。

(同样，我们以 **Python** 为例进行精简讲解。)

在 `my_package/my_package/` 文件夹下，创建**新文件** `obstacle_avoider.py`：

```python
# 源码 (Python 节点: obstacle_avoider.py)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

MAX_RANGE = 0.15 # 触发避障的距离阈值

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # 1. 创建 /cmd_vel 发布者 (与上一教程中插件的订阅匹配)
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        
        # 2. 订阅 URDF 中定义的传感器话题
        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)
        
        # 初始化传感器值
        self.__left_sensor_value = MAX_RANGE
        self.__right_sensor_value = MAX_RANGE

    # 左侧传感器回调
    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range

    # 右侧传感器回调 (逻辑核心)
    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range
        
        command_message = Twist()
        
        # 默认前进
        command_message.linear.x = 0.1 
        
        # 检查是否需要转向
        if self.__left_sensor_value < 0.9 * MAX_RANGE or \
           self.__right_sensor_value < 0.9 * MAX_RANGE:
            # 遇到障碍物，向右转 (负的角速度)
            command_message.angular.z = -2.0 
        
        # 发布控制指令
        self.__publisher.publish(command_message)

def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    
    avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

  * 这个节点完全独立于 Webots 插件。它是一个标准的 ROS 节点。
  * 它订阅 `/left_sensor` 和 `/right_sensor`。
  * 在 `__right_sensor_callback` 中（为了简化，教程把所有逻辑都放在这里），它检查两侧传感器的值。
  * 如果任一传感器检测到障碍物（小于阈值 `MAX_RANGE` 的 90%），它就发布一个带旋转速度的 `Twist` 消息；否则，它只发布前进的速度。
  * 这个 `/cmd_vel` 消息会被上一教程中创建的 `MyRobotDriver` 插件接收，并转换成电机的转速。

-----

### 任务3：更新额外文件

我们需要告诉构建系统这个新节点的存在，并修改启动文件来启动它。

**a. 更新 `setup.py` (Python)**

打开 `my_package/setup.py`，找到 `entry_points` 字典，在 `console_scripts` 列表中添加新的 `obstacle_avoider` 入口：

```python
# 源码 (setup.py - 精简)
    entry_points={
        'console_scripts': [
            # 保留上一教程的插件入口
            'my_robot_driver = my_package.my_robot_driver:main',
            # 添加新的避障节点入口
            'obstacle_avoider = my_package.obstacle_avoider:main'
        ],
    },
```

**(C++ 版本需要修改 `CMakeLists.txt` 来 `add_executable` 并 `install` 这个新节点。)**

**b. 更新 `robot_launch.py` (启动文件)**

打开 `my_package/launch/robot_launch.py`，我们需要添加 `Node` 动作来启动新的避障节点。

```python
# 源码 (Launch file: robot_launch.py)
import os
import launch
from launch_ros.actions import Node # 需要导入 Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')
    world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt') # 教程中遗漏了这行，但它是必须的

    # 1. 启动 Webots (不变)
    webots = WebotsLauncher(
        world=world_path # 教程中示例代码拼写错误，应为 world=
    )

    # 2. 启动机器人驱动 (不变)
    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )

    # 3. (新增) 启动我们的避障节点
    obstacle_avoider = Node(
        package='my_package',
        executable='obstacle_avoider', # 对应 setup.py 中的入口点
    )

    # 4. 事件处理 (不变)
    event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    # 返回所有要启动的动作
    return LaunchDescription([
        webots,
        my_robot_driver,
        obstacle_avoider, # 添加新节点
        event_handler
    ])
```

-----

### 任务4：测试代码 (Test the code)

现在，我们重新编译并运行。

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

现在，Webots 启动后，机器人应该会**自动**开始前进。当你手动在 Webots 中将它移动到墙边时，它会自动向右转弯以避开障碍物。你不再需要手动发布 `/cmd_vel` 话题了。