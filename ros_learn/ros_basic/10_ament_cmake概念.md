
---

## 🧩 一、ament_cmake 是什么？

一句话解释：

> **`ament_cmake` 是 ROS 2 的 C++ 构建系统（基于 CMake 的 ROS 扩展）。**

---

换句话说：

| 名称              | 用途                   | 属于谁          |
| --------------- | -------------------- | ------------ |
| **CMake**       | 通用的 C/C++ 构建工具       | 非 ROS，通用软件世界 |
| **ament_cmake** | ROS 2 对 CMake 的“增强版” | ROS 2 专用     |

它为 ROS 2 增加了：

* 依赖包的自动管理
* 节点安装路径标准化
* 生成环境变量脚本（`setup.bash`）
* ROS 运行工具（`ros2 run`, `ros2 launch`）识别可执行文件
* 支持 `colcon build`

---

## 🧱 二、它和 catkin 的关系？

如果你接触过 ROS 1：

| ROS 版本 | 构建系统                           | 工具                            |
| ------ | ------------------------------ | ----------------------------- |
| ROS 1  | **catkin**                     | `catkin_make`, `catkin build` |
| ROS 2  | **ament_cmake / ament_python** | `colcon build`                |

> 👉 `ament_cmake` 是 ROS 2 的 “catkin 继承者”。

---

## ⚙️ 三、为什么要有 ament_cmake？

ROS 2 的目标是跨平台、模块化、支持多语言。
原本的 catkin：

* 只支持 Linux；
* 只支持 Python2；
* 代码生成耦合严重；
* 依赖解析机制老旧。

所以 ROS 2 推出了：

* `ament_cmake` → 用于 C++ 包
* `ament_python` → 用于 Python 包
  它们共同基于新的构建工具 **colcon**。

---

## 🧰 四、ament_cmake 实际上是怎么工作的？

我们来看 `CMakeLists.txt` 的关键几行：

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp)

install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

我们拆开理解：

---

### 🧩 1️⃣ `find_package(ament_cmake REQUIRED)`

> 告诉 CMake：我使用 ROS 2 的 ament_cmake 系统。

它会导入一堆 ROS 特有的 CMake 功能，比如：

* `ament_target_dependencies()`
* `ament_export_dependencies()`
* `ament_package()`

没有这一行，CMake 就不知道 ROS 2 的编译逻辑。

---

### 🧩 2️⃣ `ament_target_dependencies(my_node rclcpp)`

> 自动帮你链接 ROS 库（例如 rclcpp）和 include 路径。

传统 CMake 你要写：

```cmake
target_link_libraries(my_node rclcpp::rclcpp)
target_include_directories(my_node PRIVATE ${rclcpp_INCLUDE_DIRS})
```

而 ROS 的写法更简单：

```cmake
ament_target_dependencies(my_node rclcpp)
```

---

### 🧩 3️⃣ `ament_package()`

> 表示“这是一个 ROS 2 包”，生成安装元数据。

这一步会：

* 生成 `package.xml` 中的依赖信息；
* 在安装时注册你的包；
* 让 `ros2 run` 能找到你的节点。

没有它的话：

```bash
ros2 run my_cpp_pkg my_node
```

就找不到执行文件。

---

### 🧩 4️⃣ `install()`

> 告诉 ROS 要把编译出来的二进制放到哪里。

例如：

```cmake
install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
```

编译后结果会放到：

```
install/my_cpp_pkg/lib/my_cpp_pkg/my_node
```

`colcon` 会自动为它生成可执行路径。

---

## 🧠 五、ament_cmake 的核心概念图

```
┌─────────────────────────────┐
│  colcon build               │
│    ↓                        │
│  调用 CMakeLists.txt        │
│    ↓                        │
│  find_package(ament_cmake)  │
│    ↓                        │
│  加载 ROS 的构建规则        │
│    ↓                        │
│  add_executable()           │
│    ↓                        │
│  ament_target_dependencies()│
│    ↓                        │
│  install() & ament_package()│
│    ↓                        │
│  生成 install/setup.bash    │
└─────────────────────────────┘
```

---

## 📦 六、ament_cmake 带来的额外功能

| 功能                   | 传统 CMake | ament_cmake |
| -------------------- | -------- | ----------- |
| 自动依赖导入               | ❌        | ✅           |
| ROS 依赖解析             | ❌        | ✅           |
| 环境变量导出               | ❌        | ✅           |
| `ros2 run` 支持        | ❌        | ✅           |
| `colcon test` 测试支持   | ❌        | ✅           |
| 多语言包集成（Python + C++） | ❌        | ✅           |

---

## 🔍 七、总结一句话

> ✅ `ament_cmake` = CMake + ROS 2 构建扩展 + colcon 集成支持

你写的 `CMakeLists.txt` 看起来像普通 CMake，
但实际上背后多了 ROS 2 特有的自动化机制。

---

## 🌱 举个完整例子

假设我们有个包 `talker_cpp`：

### 目录：

```
talker_cpp/
├── package.xml
├── CMakeLists.txt
└── src/
    └── talker.cpp
```

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.5)
project(talker_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

ament_package()
```

### `package.xml`

```xml
<package format="3">
  <name>talker_cpp</name>
  <version>0.0.1</version>
  <description>Simple publisher</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

构建：

```bash
colcon build --packages-select talker_cpp
source install/setup.bash
ros2 run talker_cpp talker
```

---

