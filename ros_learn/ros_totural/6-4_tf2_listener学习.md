好的，这篇教程是上一篇“编写 tf2 广播器”的续篇。它将教您如何**接收和使用** `tf2` 坐标变换。

这篇文章的核心是\*\*“tf2 监听器 (Listener)”\*\*，其目的是让一个机器人（`turtle2`）能够“看到”并跟随另一个机器人（`turtle1`）。

帮您详细梳理一下这篇文章的学习内容：

### 教程目标

学习如何使用 `tf2_ros::Buffer` 和 `tf2_ros::TransformListener` 来订阅 `tf2` 话题，并查询（lookup）两个坐标系之间的变换关系。

### 背景知识

这篇教程假设您已经完成了前两篇教程：

  * [编写静态广播器 (C++)](https://www.google.com/search?q=https://docs.ros.org/en/jazzy/TutorialS/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html)
  * [编写广播器 (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Cpp.html)

我们将继续在 `learning_tf2_cpp` 功能包中工作。

-----

### 详细步骤解析

#### 任务 1：编写监听器节点

这是本教程的核心。您需要创建一个 C++ 文件 `src/turtle_tf2_listener.cpp`。

这个节点的逻辑比广播器要复杂一点，它要做三件事：

1.  \*\*生成（Spawn）\*\*一只新的乌龟 `turtle2`。
2.  **监听** `tf2` 变换。
3.  **计算** `turtle1` 相对于 `turtle2` 的位置，并**发布**速度指令（`Twist` 消息）来控制 `turtle2` 追随 `turtle1`。

##### 1.1 关键代码 (`turtle_tf2_listener.cpp`)

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// 1. 包含必要的头文件
#include "geometry_msgs/msg/transform_stamped.hpp" // 变换消息
#include "geometry_msgs/msg/twist.hpp" // 速度指令消息
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h" // tf2 异常
#include "tf2_ros/transform_listener.h" // tf2 监听器
#include "tf2_ros/buffer.h" // tf2 缓冲区
#include "turtlesim/srv/spawn.hpp" // 生成乌龟的服务

using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("turtle_tf2_frame_listener"),
    turtle_spawning_service_ready_(false),
    turtle_spawned_(false)
  {
    // 声明一个参数 "target_frame"，默认值为 "turtle1"
    target_frame_ = this->declare_parameter<std::string>("target_frame", "turtle1");

    // 2. 初始化 tf2 缓冲区 (Buffer)
    // 缓冲区用于存储从 /tf 话题收到的所有坐标变换
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    
    // 3. 初始化 tf2 监听器 (Listener)
    // 监听器会自动订阅 /tf 话题，并将数据填充到缓冲区中
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. 创建一个服务客户端，用于调用 /spawn 服务来生成 turtle2
    spawner_ =
      this->create_client<turtlesim::srv::Spawn>("spawn");

    // 5. 创建一个发布者，用于向 /turtle2/cmd_vel 发布速度指令
    publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

    // 6. 创建一个 1 秒钟触发一次的定时器，其回调函数是 on_timer
    // 这是这个节点的主控制循环
    timer_ = this->create_wall_timer(
      1s, std::bind(&FrameListener::on_timer, this));
  }

private:
  // 定时器回调函数，每秒执行一次
  void on_timer()
  {
    // 定义源坐标系和目标坐标系
    std::string fromFrameRel = target_frame_.c_str(); // "turtle1" (要跟随的目标)
    std::string toFrameRel = "turtle2"; // "turtle2" (跟随者)

    if (turtle_spawning_service_ready_) { // 检查生成服务是否就绪
      if (turtle_spawned_) { // 检查 turtle2 是否已生成
        
        // ----- 核心的 TF2 查询逻辑 -----
        geometry_msgs::msg::TransformStamped t;
        try {
          // 7. 查询变换：从 "turtle1" 到 "turtle2"
          //    tf2::TimePointZero 表示获取"最新"的可用变换
          t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          // 8. 异常处理：如果变换不存在（例如刚启动，数据还没到），
          //    打印错误信息并返回，等待下一次定时器触发
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return;
        }
        // ----- 查询结束 -----

        // 9. 计算控制指令 (Twist 消息)
        geometry_msgs::msg::Twist msg;

        // 计算角速度：使用 atan2(y, x) 得到 turtle2 指向 turtle1 的角度
        static const double scaleRotationRate = 1.0;
        msg.angular.z = scaleRotationRate * atan2(
          t.transform.translation.y,
          t.transform.translation.x);

        // 计算线速度：使用 sqrt(x^2 + y^2) 得到 turtle2 到 turtle1 的距离
        static const double scaleForwardSpeed = 0.5;
        msg.linear.x = scaleForwardSpeed * sqrt(
          pow(t.transform.translation.x, 2) +
          pow(t.transform.translation.y, 2));

        // 10. 发布速度指令
        publisher_->publish(msg);
      
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully spawned");
        turtle_spawned_ = true; // 标记 turtle2 已生成
      }
    } else { // 如果生成服务还未就绪
      // 检查 /spawn 服务是否可用
      if (spawner_->service_is_ready()) {
        // 准备服务请求
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = 4.0;
        request->y = 2.0;
        request->theta = 0.0;
        request->name = "turtle2";

        // 异步发送服务请求，并设置回调
        auto response_received_callback = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
            auto result = future.get();
            if (strcmp(result->name.c_str(), "turtle2") == 0) {
              turtle_spawning_service_ready_ = true; // 标记服务已就绪
            } else {
              RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
            }
          };
        auto result = spawner_->async_send_request(request, response_received_callback);
      } else {
        RCLCPP_INFO(this->get_logger(), "Service is not ready");
      }
    }
  }

  // 成员变量
  bool turtle_spawning_service_ready_;
  bool turtle_spawned_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}
```

##### 1.2 关键概念总结

1.  **`tf2_ros::Buffer` (缓冲区)**：这是 `tf2` 的核心。它像一个数据库，存储着所有 `tf` 变换信息。它由 `tf2_ros::TransformListener` 自动填充。
2.  **`tf2_ros::TransformListener` (监听器)**：这是一个辅助类。您只需要创建它一次（在构造函数中），并把它和 `Buffer` 关联起来。它会在后台自动订阅 `/tf` 和 `/tf_static` 话题，并将收到的数据存入 `Buffer`。
3.  **`tf_buffer_->lookupTransform()`**：这是**最关键的函数**。您用它来向 `Buffer` 查询两个坐标系之间的变换关系。
      * `lookupTransform(target_frame, source_frame, time)`
      * 在教程中是 `lookupTransform("turtle2", "turtle1", ...)`
      * **含义是：“请告诉我，`turtle1` 在 `turtle2` 坐标系下的位置是什么？”**
      * 返回的 `t.transform.translation.x` 和 `t.transform.translation.y` 就是 `turtle1` 相对于 `turtle2` 的 x 和 y 距离。
4.  **`try...catch (tf2::TransformException)`**：**必须使用**。`lookupTransform` 可能会失败（例如，两个坐标系的数据还没到齐，或者在 `tf` 树中它们根本不连通）。这个 `try-catch` 块可以捕获这些异常，防止节点崩溃。

##### 1.3 修改 `CMakeLists.txt`

和广播器类似，您需要添加新的可执行文件并链接依赖：

```cmake
add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)

ament_target_dependencies(
  turtle_tf2_listener
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)

install(TARGETS
  turtle_tf2_listener
  DESTINATION lib/${PROJECT_NAME})
```

-----

#### 任务 2：更新启动文件

现在您有了两个广播器（一个给 `turtle1`，一个给 `turtle2`）和一个监听器。您需要更新 `launch/turtle_tf2_demo_launch.py` 来启动所有这些节点。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 启动模拟器
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # 2. 启动 turtle1 的广播器
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),

        # 3. (新增) 声明一个启动参数 "target_frame"
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),

        # 4. (新增) 启动 turtle2 的广播器
        #    注意：turtle2 是由监听器节点生成的，但它的 tf 广播器需要在这里启动
        #    广播器会订阅 /turtle2/pose 话题
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),

        # 5. (新增) 启动监听器节点
        Node(
            package='learning_tf2_cpp',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                # 将启动参数 "target_frame" 传递给监听器节点
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])
```

**关键点**：

  * 您需要**两个**广播器实例，一个用于 `turtle1`，一个用于 `turtle2`。这样 `tf2` 系统才能同时知道两只乌龟的位置。
  * 监听器节点 (`FrameListener`) 通过 `lookupTransform` 查询 `turtle1` 相对于 `turtle2` 的位置。
  * `FrameListener` 节点自己会调用服务生成 `turtle2`，而 `broadcaster2` 节点会订阅 `turtle2` 的 `/pose` 话题并广播其 `tf`。

-----

#### 任务 3：构建

1.  检查依赖：`rosdep install -i --from-path src --rosdistro jazzy -y`
2.  构建：`colcon build --packages-select learning_tf2_cpp`

-----

#### 任务 4：运行

1.  `source` 您的工作空间：`. install/setup.bash`
2.  运行新的启动文件：
    ```bash
    ros2 launch learning_tf2_cpp turtle_tf2_demo_launch.py
    ```
    您会看到 `turtlesim` 启动，并且 `turtle2` 会被自动生成。
3.  打开新终端，`source` 工作空间，启动键盘遥控：
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```
4.  **验证**：
    激活 `turtle_teleop_key` 终端，使用方向键移动 `turtle1`（默认目标）。您会看到 `turtle2` 会自动跟随 `turtle1` 移动！

### 总结

这篇教程是 `tf2` 的一个完整闭环演示。您学会了：

1.  **广播器 (Broadcaster)**：如何将机器人的姿态（Pose）**发布**到 `tf2` 系统。
2.  **监听器 (Listener)**：如何使用 `Buffer` 和 `Listener` **查询** `tf2` 系统，获取两个坐标系之间的相对变换，并利用这个信息进行控制（例如让一个机器人跟随另一个）。