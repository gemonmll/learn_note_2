非常好 👏👏👏
你找到的是 **ROS 2 官方关于“Composable Node（组件化节点）”的教程**，这正是现代 ROS 2 推荐的架构方式。
我们可以一步步帮你“吃透”这一整篇教程，让你理解 *为什么要这么做*、*怎么改*、*运行时发生了什么*。

---

## 🧩 一、背景 — 为什么要用 Composable Node？

传统 ROS 节点每个进程只能运行一个 node：

```cpp
int main() {
  rclcpp::spin(std::make_shared<MyNode>());
}
```

如果你有 10 个节点，就会产生 10 个独立进程。
这样带来两个问题：

1. **进程间通信有开销**
   每个节点通信需要通过 DDS（数据分发服务），序列化/反序列化，性能较差。

2. **资源浪费**
   每个进程都要各自初始化 ROS runtime、线程、内存。

---

### ✅ 解决方案：Composable Node + Component Container

Composable Node（组件节点）= 可以被动态加载的 Node
Component Container = 一个“容器进程”，可以同时加载多个组件。

这样：

* 多个节点共用同一个进程（零拷贝通信）
* 可在运行时加载/卸载节点（无需重启整个系统）
* 支持在 launch 文件中以 **ComposableNodeContainer** 启动

---

## 🧠 二、从传统 Node 到 Composable Node 的转化过程

假设我们有最初的代码：

```cpp
namespace palomino
{
class VincentDriver : public rclcpp::Node
{
public:
  VincentDriver()
  : Node("vincent_driver")
  {
    RCLCPP_INFO(this->get_logger(), "Hello from VincentDriver");
  }
};
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<palomino::VincentDriver>());
  rclcpp::shutdown();
}
```

---

## 🔧 第一步：修改类定义

Composable Node 要求构造函数能接收一个 `NodeOptions` 参数。
这是 ROS 2 的节点配置系统，container 会通过它传参数。

```cpp
namespace palomino
{
class VincentDriver : public rclcpp::Node
{
public:
  explicit VincentDriver(const rclcpp::NodeOptions & options)
  : Node("vincent_driver", options)
  {
    RCLCPP_INFO(this->get_logger(), "Hello from VincentDriver (Composable)");
  }
};
}
```

---

## 🧱 第二步：删除 main()，改用注册宏

我们不再写 main()。
取而代之的是一个宏，告诉 ROS “我有一个可加载组件”。

```cpp
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(palomino::VincentDriver)
```

> 宏会在编译时自动生成插件描述信息，让 container 知道如何加载这个类。

---

## ⚙️ 第三步：修改 CMakeLists.txt

### 1️⃣ 添加依赖

```cmake
find_package(rclcpp_components REQUIRED)
```

### 2️⃣ 改成编译共享库，而不是可执行文件

旧的：

```cmake
add_executable(vincent_driver src/vincent_driver.cpp)
```

新的：

```cmake
add_library(vincent_driver_component SHARED src/vincent_driver.cpp)
```

### 3️⃣ 链接组件库

```cmake
target_link_libraries(vincent_driver_component
  rclcpp_components::component
)
```

### 4️⃣ 注册你的组件

```cmake
rclcpp_components_register_node(
  vincent_driver_component
  PLUGIN "palomino::VincentDriver"
  EXECUTABLE vincent_driver
)
```

这行做了两件事：

* **注册插件**（让 ROS 知道这个类可以被动态加载）
* **自动生成一个可执行文件** `vincent_driver`（可单独运行）

---

### 5️⃣ 安装目标文件

```cmake
ament_export_targets(export_vincent_driver_component)
install(TARGETS vincent_driver_component
        EXPORT export_vincent_driver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
```

---

## 🚀 第四步：运行方式一（独立运行）

因为我们加了 `EXECUTABLE vincent_driver`，
你可以直接运行：

```bash
ros2 run palomino vincent_driver
```

这会自动加载组件并执行。

---

## 🧩 第五步：运行方式二（容器加载）

这是组件的强大之处：
不需要单独运行多个节点，而是加载到容器中：

```bash
ros2 run rclcpp_components component_container
```

然后加载组件：

```bash
ros2 component load /ComponentManager palomino palomino::VincentDriver
```

💡 现在你的 `VincentDriver` 节点和其他组件节点在**同一个进程**里运行！

---

## 🐍 第六步：Launch 文件用法

原来的写法：

```python
from launch_ros.actions import Node

Node(
    package='palomino',
    executable='vincent_driver'
)
```

替换成：

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ComposableNodeContainer(
    name='a_buncha_nodes',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',  # 多线程版本
    composable_node_descriptions=[
        ComposableNode(
            package='palomino',
            plugin='palomino::VincentDriver',
            name='vincent_driver',
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]
)
```

✅ `use_intra_process_comms=True` 表示节点之间零拷贝通信。

---

## 🔍 七、总结对比表

| 项目        | 传统 Node            | Composable Node      |
| --------- | ------------------ | -------------------- |
| 启动方式      | 独立进程（每个 Node 启动一个） | 多个节点可共用同一进程          |
| 代码结构      | 有 main()           | 无 main()，用注册宏        |
| 编译目标      | add_executable     | add_library (SHARED) |
| 通信效率      | DDS 传输（有序列化）       | Intra-process (零拷贝)  |
| 可动态加载     | 否                  | ✅ 可 load/unload      |
| launch 文件 | Node()             | ComposableNode()     |

---

## 🧠 八、你现在应该理解了

> ROS 2 组件化（rclcpp_components）= 用共享库形式组织节点 + 用 container 启动 + 更高效通信。

这是 **Autoware、Nav2、Isaac ROS** 等大型系统的标准做法。

---

# 第二部分 构建一个最小系统
---

太棒了 😎
我们来一步步构建一个**最小可运行的 ROS 2 Composable Node 示例包**。
这个示例可以在你的 ROS 2 工作区（如 `ros2_ws/src`）中直接编译并运行。

---

## 🧩 一、项目结构

在你的工作区 `src` 里创建一个包：

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_composable_demo --dependencies rclcpp rclcpp_components
```

然后你的目录会长这样：

```
my_composable_demo/
├── CMakeLists.txt
├── package.xml
└── src/
    └── my_node.cpp
```

---

## 🧠 二、源代码：`src/my_node.cpp`

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace my_composable_demo
{

class MyComposableNode : public rclcpp::Node
{
public:
  explicit MyComposableNode(const rclcpp::NodeOptions & options)
  : Node("my_composable_node", options)
  {
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Hello from composable node!");
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace my_composable_demo

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(my_composable_demo::MyComposableNode)
```

---

## ⚙️ 三、修改 `CMakeLists.txt`

替换掉自动生成的内容为：

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_composable_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# 编译为共享库
add_library(my_composable_demo_component SHARED src/my_node.cpp)

target_compile_features(my_composable_demo_component PUBLIC c_std_17 cxx_std_17)
target_include_directories(my_composable_demo_component PUBLIC include)
ament_target_dependencies(my_composable_demo_component rclcpp rclcpp_components)

# 链接 rclcpp_components::component
target_link_libraries(my_composable_demo_component rclcpp_components::component)

# 注册组件并生成可执行文件
rclcpp_components_register_node(
  my_composable_demo_component
  PLUGIN "my_composable_demo::MyComposableNode"
  EXECUTABLE my_composable_demo_node
)

install(TARGETS
  my_composable_demo_component
  EXPORT export_my_composable_demo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_my_composable_demo_component)
ament_package()
```

---

## 📦 四、修改 `package.xml`

确保包含：

```xml
<depend>rclcpp</depend>
<depend>rclcpp_components</depend>
```

完整示例（关键行已包含）：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_composable_demo</name>
  <version>0.0.1</version>
  <description>Minimal Composable Node Example</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 🏗️ 五、编译

```bash
cd ~/ros2_ws
colcon build --packages-select my_composable_demo
source install/setup.bash
```

---

## 🚀 六、运行方式 1：直接运行可执行文件

因为我们在 CMake 中加了 `EXECUTABLE my_composable_demo_node`，
你可以直接运行：

```bash
ros2 run my_composable_demo my_composable_demo_node
```

输出：

```
[INFO] [xxxx.xx] [my_composable_node]: Hello from composable node!
```

---

## 🧩 七、运行方式 2：加载到组件容器中

```bash
# 启动一个容器
ros2 run rclcpp_components component_container
```

在另一个终端：

```bash
ros2 component load /ComponentManager my_composable_demo my_composable_demo::MyComposableNode
```

输出：

```
[INFO] [xxxx.xx] [my_composable_node]: Hello from composable node!
```

你可以用以下命令确认加载情况：

```bash
ros2 component list
```

---

## 🐍 八、运行方式 3（推荐）：用 launch 文件加载多个组件

创建 `launch/my_composable_demo.launch.py`：

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # 多线程容器
        composable_node_descriptions=[
            ComposableNode(
                package='my_composable_demo',
                plugin='my_composable_demo::MyComposableNode',
                name='node1',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='my_composable_demo',
                plugin='my_composable_demo::MyComposableNode',
                name='node2',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

运行：

```bash
ros2 launch my_composable_demo my_composable_demo.launch.py
```

结果：

```
[INFO] [node1]: Hello from composable node!
[INFO] [node2]: Hello from composable node!
```

此时两个节点在同一个进程内运行（零拷贝通信）。

---

## ✅ 九、总结

| 项目   | 内容                                |
| ---- | --------------------------------- |
| 类定义  | 带 `NodeOptions` 构造函数              |
| 编译目标 | `add_library(... SHARED)`         |
| 注册宏  | `RCLCPP_COMPONENTS_REGISTER_NODE` |
| 运行方式 | 可独立运行、动态加载、launch 启动              |
| 优点   | 高性能、低延迟、可热插拔、多节点共进程               |

---

是否希望我帮你把这个例子打包成一个完整的 `.zip` 项目（包含目录结构和文件内容），
方便你直接放进 `ros2_ws/src` 编译运行？
