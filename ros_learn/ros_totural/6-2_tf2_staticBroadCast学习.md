好的，我们一起来学习这篇 tf2 在 ROS 2 中的教程：《Writing a static broadcaster (C++)》。我先给你一个「中文要点整理+笔记版」，然后如果你愿意，可以做个「练习实现」部分。

---

## 📝 教程中文整理 + 要点

### 背景（Background）

发布静态变换（static transform）常用于定义机器人底盘与其固定传感器、非移动部件之间的关系。比如你的激光雷达固定在机器人上：你就定义一个「机器人底盘 frame → 激光雷达 frame」的静态变换。 ([ROS Docs][1])
本教程主要教你如何用 C++ 编写一个静态变换广播器（static broadcaster），以及介绍在多数情况你其实可以直接用命令行工具来完成。 ([ROS Docs][1])

### 先决条件（Prerequisites）

你需要先创建工作区（workspace）并且有基本的 ROS 2 包创建能力。 ([ROS Docs][1])

### 任务概览（Tasks）

教程分为四步：

1. 创建一个新包（package）
2. 编写静态广播节点（C++）
3. 构建（build）
4. 运行（run）
   此外还有一节“正确方式来发布静态变换”（The proper way to publish static transforms）讲述命令行工具。 ([ROS Docs][1])

---

### 1. 创建包

* 包名示例： `learning_tf2_cpp`。 ([ROS Docs][1])
* 依赖： `geometry_msgs`、`rclcpp`、`tf2`、`tf2_ros`、`turtlesim`。 ([ROS Docs][1])
* 命令示例：

  ```bash
  ros2 pkg create --build-type ament_cmake --license Apache-2.0 --dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim learning_tf2_cpp
  ```

  ([ROS Docs][1])

### 2. 编写静态广播节点（Static broadcaster node）

#### 源码说明

* 文件：`static_turtle_tf2_broadcaster.cpp`（可从 GitHub 原始链接下载） ([ROS Docs][1])
* 核心头文件：

  ```cpp
  #include "geometry_msgs/msg/transform_stamped.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "tf2/LinearMath/Quaternion.h"
  #include "tf2_ros/static_transform_broadcaster.h"
  ```

  ([ROS Docs][1])
* 类 `StaticFramePublisher` 继承自 `rclcpp::Node`。构造函数中新建 `StaticTransformBroadcaster`，并调用 `make_transforms()`。 ([ROS Docs][1])
* `make_transforms()` 方法做以下操作：

  * 创建 `geometry_msgs::msg::TransformStamped t`。 ([ROS Docs][1])
  * 填 `t.header.stamp = this->get_clock()->now()`；即用当前时刻戳。 ([ROS Docs][1])
  * 设置父坐标系 `t.header.frame_id = "world"`。 ([ROS Docs][1])
  * 设置子坐标系 `t.child_frame_id = transformation[1]`（即命令行参数传进来的名字） ([ROS Docs][1])
  * 填平移值 `x = atof(transformation[2])`、`y = …`、`z = …`。 ([ROS Docs][1])
  * 使用 `tf2::Quaternion q; q.setRPY(roll, pitch, yaw)` 来生成四元数表示旋转。然后赋值给 `t.transform.rotation.x/y/z/w`。 ([ROS Docs][1])
  * 最后调用 `tf_static_broadcaster_->sendTransform(t);` 将变换广播出去。 ([ROS Docs][1])
* `main()` 函数流程：

  * 检查命令行参数数目是否等于 8（包名、child_frame_name、x、y、z、roll、pitch、yaw）否则输出使用说明并退出。 ([ROS Docs][1])
  * 检查子 frame 名称是否为 “world” (不允许) ([ROS Docs][1])
  * 调用 `rclcpp::init(...)`, `rclcpp::spin(...)`, `rclcpp::shutdown();` 类似典型 ROS2 节点流程。 ([ROS Docs][1])

#### 更新 package.xml 和 CMakeLists.txt

* 在 `package.xml` 中，确保填写 `<description>`、`<maintainer>`、`<license>` 等基本信息。 ([ROS Docs][1])
* 在 `CMakeLists.txt` 中加入：

  ```cmake
  add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
  ament_target_dependencies(
     static_turtle_tf2_broadcaster
     geometry_msgs
     rclcpp
     tf2
     tf2_ros
  )
  install(TARGETS
     static_turtle_tf2_broadcaster
     DESTINATION lib/${PROJECT_NAME})
  ```

  ([ROS Docs][1])

### 3. 构建（Build）

* 在工作区根目录先运行 `rosdep install -i --from-path src --rosdistro jazzy -y`（在 Linux 上）以保证缺失依赖安装。 ([ROS Docs][1])
* 使用 `colcon build --packages-select learning_tf2_cpp`（或带 `--merge-install`）进行编译。 ([ROS Docs][1])
* 编译完成后，记得 `source install/setup.bash`（或 Windows 上对应脚本）以让环境识别新包。 ([ROS Docs][1])

### 4. 运行（Run）

* 执行命令示例：

  ```bash
  ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
  ```

  这里参数解释：

  * `mystaticturtle`：子坐标系名称
  * `0 0 1`：x=0, y=0, z=1 (平移)
  * `0 0 0`：roll=0, pitch=0, yaw=0 (旋转) ([ROS Docs][1])
* 然后你可以运行查看 `/tf_static` 话题：

  ```bash
  ros2 topic echo /tf_static
  ```

  你应该会看到类似如下输出：

  ```
  transforms:
  - header:
      stamp:
        sec: ...
        nanosec: ...
    frame_id: world
    child_frame_id: mystaticturtle
    transform:
      translation:
        x: 0.0
        y: 0.0
        z: 1.0
      rotation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
  ```

  ([ROS Docs][1])

### 5. 正确方式发布静态变换（The proper way to publish static transforms）

虽然本教程演示了你用 C++ 写 broadcaster，但在实际开发中，若只是 “固定坐标系之间的变换”，你 **应该** 使用 `static_transform_publisher` 可执行工具（由 tf2_ros 提供），而不是每次写代码来做。 ([ROS Docs][1])
示例命令：

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 1 --yaw 0 --pitch 0 --roll 0 --frame-id world --child-frame-id mystaticturtle
```

或使用四元数表示法：

```bash
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 1 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id world --child-frame-id mystaticturtle
```

此外也可以在 launch 文件中调用该节点。 ([ROS Docs][1])

### 总结（Summary）

* 学会了如何用 C++ 在 ROS2 中发布一个静态的坐标系变换。
* 明白静态变换适合 “从属于机器人但不移动的部件” 的情况，比如传感器固定在机器人上。
* 了解多久变换树、坐标系管理的重要性，以及简化场景下可以直接用工具 `static_transform_publisher`。
  ([ROS Docs][1])

---

## ✅ 快速笔记（重点复习）

* 静态变换 = 一个坐标系相对于另一个坐标系 **固定不变**（即从启动或定义时即设定，不随时间变化）
* 静态广播器（StaticTransformBroadcaster）在 C++ 中使用 `tf2_ros::StaticTransformBroadcaster`
* 消息类型：`geometry_msgs::msg::TransformStamped`
* 四元数工具：`tf2::Quaternion q; q.setRPY(roll, pitch, yaw);`
* 在 ROS2 节点生命周期内，一次发送即可，或者持续但值不变。
* 对于简单场景，用 `static_transform_publisher` 更为方便。
* 在包定义中要加依赖（geometry_msgs, tf2, tf2_ros, rclcpp）并在 CMakeLists 中正确链接。

---

如果你愿意，我接下来可以帮你 **生成一个练习题目 + 示例代码**（基于这个教程）让你在本地实践。你想这样做吗？另外：你希望练习代码是 *C++* 版本还是 *Python* 版本呢？

[1]: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html "Writing a static broadcaster (C++) — ROS 2 Documentation: Jazzy  documentation"

---

# 具体部分
非常好 👍 你要看完整源码才能真正理解 `tf2_ros::StaticTransformBroadcaster` 的工作原理。
下面我会结合 ROS 2 Jazzy 官方教程
[Writing a Tf2 Static Broadcaster (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html)
为你整理一个完整的「中文讲解 + 官方源码 + 文件结构 + 构建运行说明」。

---

# 🧭 教程：编写 tf2 静态变换广播器（C++）

---

## 📘 一、背景

在 ROS 2 中，**静态变换 (static transform)** 常用于描述机器人中固定部件之间的坐标关系。
例如：

* 机器人底盘 `base_link` 与安装在上面的激光雷达 `laser_frame`。
* 世界坐标系 `world` 与固定相机 `camera_frame`。

静态变换一旦定义，就不会随时间变化。
在 ROS 2 中有两种方式发布静态变换：

1. **用 C++ / Python 自定义节点广播**（本教程）
2. **用命令行工具 static_transform_publisher**（推荐日常使用）

---

## 🧩 二、创建工程

在你的 ROS 2 工作区中执行：

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 \
--dependencies geometry_msgs rclcpp tf2 tf2_ros turtlesim learning_tf2_cpp
```

然后目录结构大致如下：

```
learning_tf2_cpp/
 ├── CMakeLists.txt
 ├── package.xml
 └── src/
     └── static_turtle_tf2_broadcaster.cpp
```

---

## 💻 三、源码：`static_turtle_tf2_broadcaster.cpp`

以下是 ROS 2 Jazzy 官方教程提供的完整 C++ 源码（含中文注释）：

```cpp
// static_turtle_tf2_broadcaster.cpp
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <memory>
#include <string>
#include <vector>

using std::placeholders::_1;

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher(std::vector<std::string> transformation)
  : Node("static_turtle_tf2_broadcaster")
  {
    // 创建一个 StaticTransformBroadcaster 对象
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    make_transforms(transformation);
  }

private:
  void make_transforms(std::vector<std::string> transformation)
  {
    geometry_msgs::msg::TransformStamped t;

    // 设置时间戳
    t.header.stamp = this->get_clock()->now();

    // 父坐标系固定为 "world"
    t.header.frame_id = "world";

    // 子坐标系名由命令行参数指定
    t.child_frame_id = transformation[1];

    // 读取平移参数
    t.transform.translation.x = std::stod(transformation[2]);
    t.transform.translation.y = std::stod(transformation[3]);
    t.transform.translation.z = std::stod(transformation[4]);

    // 读取旋转角度 (roll, pitch, yaw)
    tf2::Quaternion q;
    q.setRPY(
      std::stod(transformation[5]),
      std::stod(transformation[6]),
      std::stod(transformation[7])
    );

    // 转换为四元数
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // 发布静态变换
    tf_static_broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  // 参数格式：8 个参数（程序名 + 7 个数值）
  if (argc != 8) {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      "usage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
    return 1;
  }

  // 禁止使用 world 作为 child_frame_id
  if (std::string(argv[1]) == "world") {
    RCLCPP_INFO(
      rclcpp::get_logger("rclcpp"),
      "Your static turtle name cannot be 'world'");
    return 1;
  }

  // 初始化 ROS 节点
  rclcpp::init(argc, argv);

  // 创建节点对象
  std::vector<std::string> arguments(argv, argv + argc);
  auto node = std::make_shared<StaticFramePublisher>(arguments);

  // spin 使节点保持运行（虽然静态广播只需一次）
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

---

## ⚙️ 四、修改 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(learning_tf2_cpp)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)

ament_target_dependencies(static_turtle_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

install(TARGETS
  static_turtle_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---

## 📦 五、修改 `package.xml`

确保依赖声明完整：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>learning_tf2_cpp</name>
  <version>0.0.0</version>
  <description>Learning tf2 static broadcaster tutorial</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>geometry_msgs</depend>
  <depend>rclcpp</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>turtlesim</depend>

  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
```

---

## 🔨 六、构建项目

```bash
cd ~/ros2_ws
colcon build --packages-select learning_tf2_cpp
source install/setup.bash
```

---

## 🚀 七、运行节点

```bash
ros2 run learning_tf2_cpp static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
```

* `mystaticturtle` → 子坐标系名称
* `0 0 1` → 平移 (x, y, z)
* `0 0 0` → 旋转 (roll, pitch, yaw)

---

## 📡 八、验证广播

监听静态变换：

```bash
ros2 topic echo /tf_static
```

你应该会看到类似输出：

```yaml
transforms:
- header:
    stamp:
      sec: 1234
      nanosec: 567890000
    frame_id: world
  child_frame_id: mystaticturtle
  transform:
    translation:
      x: 0.0
      y: 0.0
      z: 1.0
    rotation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

---

## 💡 九、命令行替代方法

对于一般情况，不需要自己写节点，直接使用 ROS 2 提供的工具更方便：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 1 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id world \
  --child-frame-id mystaticturtle
```

这条命令与上面的程序输出 **完全相同**。

---

## 🧠 十、总结

| 项目   | 说明                                                            |
| ---- | ------------------------------------------------------------- |
| 广播器类 | `tf2_ros::StaticTransformBroadcaster`                         |
| 消息类型 | `geometry_msgs::msg::TransformStamped`                        |
| 功能   | 发布固定坐标系之间的变换                                                  |
| 适用场景 | 固定相机、传感器、底盘、世界坐标系关系                                           |
| 调试命令 | `ros2 topic echo /tf_static`、`ros2 run tf2_tools view_frames` |
| 推荐实践 | 简单静态关系用 `static_transform_publisher` 即可                       |

---

是否希望我接下来帮你写一个 **launch 文件**（把 `static_transform_publisher` 嵌入启动流程），
让你可以直接在 RViz 2 中看到 `world` 与 `mystaticturtle` 的关系？

# 第三部分 launch启动
太棒了 🙌
我们来完成下一步：为你的 **静态变换广播器**（`static_turtle_tf2_broadcaster`）添加一个 **Launch 文件**，让你可以：

✅ 一键启动 `turtlesim`
✅ 同时发布一个 `world → mystaticturtle` 的静态变换
✅ 最后在 RViz2 中可视化坐标系关系

---

# 🚀 教程：为静态变换添加 Launch 文件（C++ 版）

---

## 📁 一、目录结构

假设你当前在包 `learning_tf2_cpp` 内，创建目录：

```bash
cd ~/ros2_ws/src/learning_tf2_cpp
mkdir launch
```

目录结构会变成：

```
learning_tf2_cpp/
 ├── CMakeLists.txt
 ├── package.xml
 ├── src/
 │   └── static_turtle_tf2_broadcaster.cpp
 └── launch/
     └── static_broadcaster_demo.launch.py
```

---

## 🧩 二、Launch 文件内容

创建文件：
`learning_tf2_cpp/launch/static_broadcaster_demo.launch.py`

内容如下👇（含中文注释）：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1️⃣ 启动 turtlesim 节点（只是为了演示环境）
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # 2️⃣ 启动静态变换广播器
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_broadcaster',
            arguments=[
                '--x', '2.0', '--y', '2.0', '--z', '0.0',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'mystaticturtle'
            ]
        ),

        # 3️⃣ 启动 RViz2 并加载默认配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 
                       # 加载一个预设的 rviz 配置文件
                       # 如果你没有，就留空或自定义
                       # 例如：'/home/username/rviz/static_tf_demo.rviz'
                      ],
            output='screen'
        )
    ])
```

---

## 🧱 三、更新 `CMakeLists.txt`

添加 Launch 文件安装部分，以便构建后被识别：

```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
```

完整的 `CMakeLists.txt` 尾部应当类似：

```cmake
install(TARGETS
  static_turtle_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

---

## 🧩 四、构建并加载环境

```bash
cd ~/ros2_ws
colcon build --packages-select learning_tf2_cpp
source install/setup.bash
```

---

## ▶️ 五、运行 Launch 文件

```bash
ros2 launch learning_tf2_cpp static_broadcaster_demo.launch.py
```

---

## 🧠 六、验证与可视化

启动后，应该会：

1. 打开 **turtlesim 窗口**
2. 启动 `static_transform_publisher` 节点
3. 启动 **RViz2**

你可以在 RViz2 中添加：

* “TF” 显示类型
* “Axes” (坐标轴) 显示

然后就会看到：

* 世界坐标系 `world`
* 子坐标系 `mystaticturtle`
* 它们之间的固定偏移 `(x=2, y=2, z=0)`

---

## 🧩 七、扩展玩法（可选）

你还可以：

* 替换 `tf2_ros` 的节点为你自己的 `static_turtle_tf2_broadcaster`
* 例如：

```python
Node(
    package='learning_tf2_cpp',
    executable='static_turtle_tf2_broadcaster',
    name='my_static_broadcaster',
    arguments=['mystaticturtle', '2', '2', '0', '0', '0', '0']
)
```

这样就能直接使用你自己写的 C++ 静态广播器。

---

## ✅ 总结

| 步骤  | 内容                                                                     |
| --- | ---------------------------------------------------------------------- |
| 1️⃣ | 创建 `launch` 目录                                                         |
| 2️⃣ | 编写 `.launch.py` 文件                                                     |
| 3️⃣ | 在 `CMakeLists.txt` 中安装 Launch                                          |
| 4️⃣ | 构建并运行 `ros2 launch learning_tf2_cpp static_broadcaster_demo.launch.py` |
| 5️⃣ | 用 RViz2 可视化 `world` → `mystaticturtle`                                 |

---

是否希望我帮你**附上一个可用的 RViz 配置文件**（`.rviz`），
让你打开时自动显示 TF 树和坐标轴？
