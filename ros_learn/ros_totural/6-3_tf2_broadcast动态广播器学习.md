好的，这篇文章是一篇关于 **ROS 2 (Jazzy) 中如何使用 C++ 编写 tf2 广播器 (broadcaster)** 的中级教程。

帮您详细梳
理一下这篇文章的学习内容：

### 教程目标

学习如何编写一个 C++ 节点，该节点可以将机器人的状态（即坐标变换）广播到 `tf2` 坐标变换树中。

### 背景知识

这篇教程是 `tf2` 系列教程的一部分。它假设您已经了解 `tf2` 的基本概念，并完成了前置教程，特别是：

  * [tf2 简介](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
  * [编写静态广播器 (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html)

本教程将重用在“静态广播器”教程中创建的 `learning_tf2_cpp` 功能包。

-----

### 详细步骤解析

教程主要分为 4 个任务：

#### 任务 1：编写广播器节点

这是本教程的核心。您需要创建一个 C++ 文件 `src/turtle_tf2_broadcaster.cpp`。

##### 1.1 关键代码 (`turtle_tf2_broadcaster.cpp`)

这个节点会订阅一只乌龟（turtlesim）的姿态（Pose）信息，并将其转换为 `tf2` 变换（Transform），然后广播出去。

```cpp
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp" // 包含坐标变换消息
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h" // 包含 tf2 的四元数库
#include "tf2_ros/transform_broadcaster.h" // 包含 tf2 广播器
#include "turtlesim/msg/pose.hpp" // 包含乌龟的姿态消息

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("turtle_tf2_frame_publisher")
  {
    // 1. 声明并获取一个参数 "turtlename"，用于指定是哪只乌龟
    turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

    // 2. 初始化 TransformBroadcaster
    // 这是发布 tf 变换的核心对象
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 3. 构建订阅的话题名，例如 "/turtle1/pose"
    std::ostringstream stream;
    stream << "/" << turtlename_.c_str() << "/pose";
    std::string topic_name = stream.str();

    // 4. 创建订阅者，订阅乌龟的姿态话题
    // 并绑定回调函数 handle_turtle_pose
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      topic_name, 10,
      std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
  }

private:
  void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
  {
    geometry_msgs::msg::TransformStamped t;

    // ----- 开始填充变换消息 (TransformStamped) -----

    // 5. 填充 Header (消息头)
    t.header.stamp = this->get_clock()->now(); // 时间戳：使用当前节点时间
    t.header.frame_id = "world"; // 父坐标系 (Parent Frame): world
    t.child_frame_id = turtlename_.c_str(); // 子坐标系 (Child Frame): 乌龟的名字 (例如 "turtle1")

    // 6. 填充 Translation (平移)
    // 乌龟在 2D 中，所以 z 坐标设为 0
    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    // 7. 填充 Rotation (旋转)
    // 乌龟只绕 Z 轴旋转 (theta)，所以 x 和 y 轴的旋转为 0
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta); // RPY: Roll, Pitch, Yaw
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // ----- 填充完毕 -----

    // 8. 发送（广播）这个变换
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
```

##### 1.2 关键概念总结

1.  **`tf2_ros::TransformBroadcaster`**：这是向 `tf2` 系统发布坐标变换的专用工具。您只需要创建它的一个实例，并调用其 `sendTransform()` 方法。
2.  **`geometry_msgs::msg::TransformStamped`**：这是 `tf2` 变换的标准消息格式。它包含了所有关键信息：
      * `header.stamp`：时间戳，告诉 `tf2` 这个变换是在哪个时间点有效的。
      * `header.frame_id`：父坐标系的名称。
      * `child_frame_id`：子坐标系的名称。
      * `transform.translation`：平移（x, y, z）。
      * `transform.rotation`：旋转（使用四元数 x, y, z, w）。
3.  **订阅与发布**：节点的逻辑是“订阅一个消息，发布一个变换”。它订阅 `turtlesim/msg/Pose`，并在回调函数中处理数据，然后发布 `geometry_msgs/msg::TransformStamped` 消息（通过 `TransformBroadcaster`）。

##### 1.3 修改 `CMakeLists.txt`

为了编译这个新的 C++ 文件，您需要修改 `CMakeLists.txt`：

```cmake
# 添加可执行文件
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)

# 添加依赖
ament_target_dependencies(
  turtle_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)

# 安装可执行文件
install(TARGETS
  turtle_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})
```

-----

#### 任务 2：编写启动文件 (Launch File)

为了方便地同时启动模拟器和广播器节点，教程指导您创建一个 `launch/turtle_tf2_demo_launch.py` 文件。

##### 2.1 启动文件代码

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动 turtlesim 模拟器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # 2. 启动我们刚编写的广播器
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            # 3. 传入参数，指定乌龟的名字为 'turtle1'
            # 这会匹配 C++ 代码中声明的 "turtlename" 参数
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
    ])
```

##### 2.2 修改 `package.xml`

因为启动文件使用了 `launch` 和 `launch_ros`，所以需要添加它们作为执行依赖：

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

##### 2.3 修改 `CMakeLists.txt`

为了让 ROS 2 能找到这个 `launch` 文件夹，还需要在 `CMakeLists.txt` 中添加安装规则：

```cmake
# 安装 launch 目录
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
```

-----

#### 任务 3：构建

1.  在工作空间的根目录检查并安装依赖：
    ```bash
    rosdep install -i --from-path src --rosdistro jazzy -y
    ```
2.  构建功能包：
    ```bash
    colcon build --packages-select learning_tf2_cpp
    ```

-----

#### 任务 4：运行

1.  打开一个新终端，`source` 您的工作空间：

    ```bash
    . install/setup.bash
    ```

2.  运行启动文件。这将同时启动 `turtlesim` 和 `turtle_tf2_broadcaster`：

    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_demo_launch.py
    ```

3.  打开第二个终端，`source` 工作空间，然后启动键盘遥控节点：

    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```

    现在您可以使用方向键移动乌龟。

4.  **(验证)** 打开第三个终端，`source` 工作空间，使用 `tf2_echo` 工具来查看 `world` 和 `turtle1` 之间的坐标变换是否正在被广播：

    ```bash
    ros2 run tf2_ros tf2_echo world turtle1
    ```

    如果您能看到类似下面的输出，并且当您移动乌龟时平移（Translation）和旋转（Rotation）数据在变化，那么说明您的广播器工作正常！

    ```
    At time 1625137663.912474878
    - Translation: [5.276, 7.930, 0.000]
    - Rotation: in Quaternion [0.000, 0.000, 0.934, -0.357]
    ```

### 总结

这篇教程带您完成了**动态坐标变换广播器**的编写。您学会了如何订阅一个数据源（乌龟的 `/pose` 话题），将其转换为 `TransformStamped` 消息，并使用 `TransformBroadcaster` 将这个动态变化的坐标关系发布到 `tf2` 系统中。

接下来的教程（[Writing a listener (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html)）将教您如何编写一个“监听器”节点来使用这个广播出来的变换。