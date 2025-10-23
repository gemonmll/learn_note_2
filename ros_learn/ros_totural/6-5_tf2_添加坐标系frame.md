好的，这篇文章是 ROS 2 Jazzy 版本关于\*\*如何使用 C++ 向 tf2 添加坐标系（frame）\*\*的中级教程。

这篇教程的目标是教你如何向 tf2 坐标变换树中添加一个额外的坐标系。

以下是这篇教程的详细步骤和内容分解：

### 背景知识 (Background)

教程首先指出，在之前的教程中，你已经学习了如何创建 tf2 的广播者（broadcaster）和监听者（listener）。本教程将在此基础上，教你如何添加**固定坐标系**和**动态坐标系**。

在机器人系统中，为每个传感器、连杆或关节定义一个局部坐标系通常会让问题变得更简单（例如，在激光雷达的中心来处理激光扫描数据）。tf2 允许你定义这些局部坐标系，并且当你需要在不同坐标系之间转换数据时，tf2 会自动处理所有中间的坐标变换。

### tf2 坐标树 (tf2 tree)

tf2 构建的是一个**树形结构**的坐标系。

  * 它不允许出现闭环（即一个坐标系不能是自己的祖先）。
  * 一个坐标系（frame）只能有一个父坐标系（parent）。
  * 一个坐标系可以有多个子坐标系（children）。

在当前的turtlesim示例中，已经有三个坐标系：`world`, `turtle1` 和 `turtle2`。`turtle1` 和 `turtle2` 都是 `world` 的子坐标系。

-----

### 任务1：编写一个固定坐标系广播者 (Write the fixed frame broadcaster)

这个任务的目标是添加一个新的坐标系 `carrot1`，并使其成为 `turtle1` 的子坐标系。这个 `carrot1` 将作为第二只乌龟（turtle2）的新目标。

#### 1.1 检查代码 (Examine the code)

你需要在你的工作空间中（教程假设是 `learning_tf2_cpp` 包）的 `src` 目录下创建一个名为 `fixed_frame_tf2_broadcaster.cpp` 的文件。

**核心 C++ 代码解析：**

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp" // 用于变换消息
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"        // tf2 广播者

using namespace std::chrono_literals;

class FixedFrameBroadcaster : public rclcpp::Node
{
public:
  FixedFrameBroadcaster()
  : Node("fixed_frame_tf2_broadcaster")
  {
    // 1. 初始化 TransformBroadcaster
    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 2. 创建一个定时器，周期性调用广播回调函数
    timer_ = this->create_wall_timer(
      100ms, std::bind(&FixedFrameBroadcaster::broadcast_timer_callback, this));
  }

private:
  void broadcast_timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;

    // 3. 填充变换消息 (TransformStamped)
    t.header.stamp = this->get_clock()->now(); // 设置时间戳
    t.header.frame_id = "turtle1";            // 设置父坐标系
    t.child_frame_id = "carrot1";             // 设置子坐标系

    // 4. 定义 'carrot1' 相对于 'turtle1' 的变换
    //    这里 'carrot1' 在 'turtle1' 的 y 轴正方向 2.0 米处
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.0;
    
    // 5. 定义旋转（这里没有旋转，使用单位四元数）
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    // 6. 发送（广播）这个变换
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
```

这段代码创建了一个节点，该节点以 10Hz（每100ms一次）的频率广播一个*固定*的变换：`carrot1` 坐标系始终位于 `turtle1` 坐标系前方 2 米处。

#### 1.2 修改 CMakeLists.txt

为了编译这个新节点，你需要编辑 `CMakeLists.txt` 文件：

1.  添加可执行文件：
    ```cmake
    add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
    ```
2.  链接依赖：
    ```cmake
    ament_target_dependencies(
      fixed_frame_tf2_broadcaster
      geometry_msgs
      rclcpp
      tf2_ros
    )
    ```
3.  添加安装规则，以便 `ros2 run` 可以找到它：
    ```cmake
    install(TARGETS
      fixed_frame_tf2_broadcaster
      DESTINATION lib/${PROJECT_NAME})
    ```

#### 1.3 编写 Launch 文件

在 `launch` 目录下创建一个新的 `turtle_tf2_fixed_frame_demo_launch.py` 文件。

**Launch 文件代码解析：**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. 包含之前教程的 launch 文件 (启动 turtlesim 和 turtle1, turtle2)
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('learning_tf2_cpp'),
                'launch',
                'turtle_tf2_demo_launch.py'
            ])
        ),
        # 2. 启动我们新编写的固定坐标系广播者
        Node(
            package='learning_tf2_cpp',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
        ),
    ])
```

#### 1.4 构建 (Build)

在工作空间根目录运行 `colcon build` 来编译你的包。

#### 1.5 运行 (Run)

1.  Source 你的工作空间：`. install/setup.bash`
2.  运行 launch 文件：`ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo_launch.py`

此时，`carrot1` 坐标系已经添加。但 `turtle2` 仍然在跟随 `turtle1`。

**让 `turtle2` 跟随 `carrot1`：**
你需要修改 `turtle2` 的监听者节点的目标坐标系。教程提供了两种方法：

1.  **命令行修改：** 在启动时传入参数。
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_fixed_frame_demo_launch.py target_frame:=carrot1
    ```
2.  **修改 Launch 文件：** 更新 `turtle_tf2_fixed_frame_demo_launch.py`，在 `IncludeLaunchDescription` 中添加 `launch_arguments`。
    ```python
    def generate_launch_description():
        demo_nodes = IncludeLaunchDescription(
            ...,
            # 将 'carrot1' 作为 target_frame 参数传递给被包含的 launch 文件
            launch_arguments={'target_frame': 'carrot1'}.items(),
        )
        return LaunchDescription([
            demo_nodes,
            Node(...),
        ])
    ```

修改后重新构建并运行，你会看到 `turtle2` 开始跟随 `carrot1`。由于 `carrot1` 相对于 `turtle1` 是固定的，所以 `turtle2` 会试图保持在 `turtle1` 前方 2 米的位置。

-----

### 任务2：编写一个动态坐标系广播者 (Write the dynamic frame broadcaster)

这个任务的目标是让 `carrot1` 坐标系相对于 `turtle1` *动态*变化（移动）。

#### 2.1 检查代码 (Examine the code)

创建一个新文件 `dynamic_frame_tf2_broadcaster.cpp`。

**核心 C++ 代码解析 (与固定广播者的区别)：**

```cpp
// ... (includes 和类定义类似) ...

class DynamicFrameBroadcaster : public rclcpp::Node
{
  // ... (构造函数和 tf_broadcaster_ 初始化类似) ...
private:
  void broadcast_timer_callback()
  {
    // 1. 获取当前时间
    rclcpp::Time now = this->get_clock()->now();
    // 2. 根据时间计算一个周期性变化的值
    double x = now.seconds() * 3.141592653589793238463; // PI

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = "turtle1";
    t.child_frame_id = "carrot1";

    // 3. 使用 sin() 和 cos() 来动态改变 'carrot1' 的位置
    //    这将使 'carrot1' 绕着 'turtle1' 做圆周运动
    t.transform.translation.x = 10 * sin(x);
    t.transform.translation.y = 10 * cos(x);
    t.transform.translation.z = 0.0;

    // ... (旋转不变, 广播不变) ...
    tf_broadcaster_->sendTransform(t);
  }
  // ...
};

// ... (main 函数不变) ...
```

关键区别在于 `broadcast_timer_callback` 函数。现在，`carrot1` 的 X 和 Y 平移量是根据当前时间 `now.seconds()` 的 `sin` 和 `cos` 值来计算的，这使得 `carrot1` 会绕着 `turtle1` 做半径为 10 的圆周运动。

#### 2.2 修改 CMakeLists.txt

你需要为这个新的可执行文件添加编译和安装规则，就像任务1中一样：

```cmake
add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
  dynamic_frame_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
)
install(TARGETS
  dynamic_frame_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})
```

#### 2.3 编写 Launch 文件

创建一个新的 `turtle_tf2_dynamic_frame_demo_launch.py` 文件。

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. 包含基础 demo，并确保 'turtle2' 跟踪 'carrot1'
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('learning_tf2_cpp'),
                'launch',
                'turtle_tf2_demo_launch.py'
            ]),
            launch_arguments={'target_frame': 'carrot1'}.items(),
        ),
        # 2. 启动动态坐标系广播者
        Node(
            package='learning_tf2_cpp',
            executable='dynamic_frame_tf2_broadcaster',
            name='dynamic_broadcaster',
        ),
    ])
```

注意：这次我们启动的是 `dynamic_frame_tf2_broadcaster` 节点。

#### 2.4 构建 (Build) 和 2.5 运行 (Run)

步骤与任务1相同（构建、Source、Launch）。

运行 `ros2 launch learning_tf2_cpp turtle_tf2_dynamic_frame_demo_launch.py` 后，你将看到 `carrot1` 坐标系绕着 `turtle1` 旋转，而 `turtle2` 会努力地追逐这个移动的 `carrot1`。

### 总结 (Summary)

这篇教程教会了你：

1.  tf2 坐标变换树的结构（树形、单父节点）。
2.  如何使用 C++ 和 `tf2_ros::TransformBroadcaster` 来发布一个**固定**的坐标变换。
3.  如何发布一个随时间变化的**动态**坐标变换。
4.  如何修改 `CMakeLists.txt` 和创建 `launch` 文件来编译和运行新节点。

希望这个详细分解对你的学习有帮助！