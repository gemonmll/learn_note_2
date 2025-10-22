非常好 👍 你贴的这篇文章是 **ROS 2 官方教程之一**，标题是：

> **“Implementing custom interfaces”**
> （实现自定义接口）

它讲的是：
如何在 **一个包里** 同时 **定义消息（msg）文件** 并 **在代码中使用它**。
这和之前那种「接口单独放在 interface 包」的做法稍微不同。

---

## 🌱 一、背景：为什么要学这个

在 ROS 2 里，节点间通信依赖于 **接口 (interfaces)**，比如：

* 消息（`.msg`）—— 发布/订阅用的；
* 服务（`.srv`）—— 请求/响应用的；
* 动作（`.action`）—— 带进度和反馈的任务用的。

👉 最佳实践是：
把这些接口单独放在一个 **interface-only package** 里（如 `example_interfaces`）。

但是有时候：

* 你只是做个小 demo；
* 或者只是希望代码+消息都在一个包里。

那就可以像本教程一样：
**在同一个包里定义 msg 并使用它。**

---

## 📦 二、步骤详解

### **1️⃣ 创建一个包**

```bash
ros2 pkg create --build-type ament_cmake --license Apache-2.0 more_interfaces
mkdir more_interfaces/msg
```

这会创建一个 CMake 包（`ament_cmake`），并建一个放消息的目录。

---

### **2️⃣ 创建自定义消息**

在 `more_interfaces/msg/AddressBook.msg` 写入：

```text
uint8 PHONE_TYPE_HOME=0
uint8 PHONE_TYPE_WORK=1
uint8 PHONE_TYPE_MOBILE=2

string first_name
string last_name
string phone_number
uint8 phone_type
```

这是一个简单的通讯录结构，包含字符串字段和枚举常量。

---

### **2.1️⃣ 配置 CMake 构建消息**

在 `package.xml` 中添加消息生成依赖：

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

在 `CMakeLists.txt` 中添加：

```cmake
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AddressBook.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
```

👉 作用：

* `rosidl_generate_interfaces`：生成 C++/Python 消息代码。
* `ament_export_dependencies`：让下游包能使用这些生成的接口。

---

### **3️⃣ 在同包中使用这个消息**

创建 `src/publish_address_book.cpp`：

```cpp
#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/address_book.hpp"

class AddressBookPublisher : public rclcpp::Node {
public:
  AddressBookPublisher() : Node("address_book_publisher") {
    publisher_ = this->create_publisher<more_interfaces::msg::AddressBook>("address_book", 10);

    auto publish_msg = [this]() {
      auto message = more_interfaces::msg::AddressBook();
      message.first_name = "John";
      message.last_name = "Doe";
      message.phone_number = "1234567890";
      message.phone_type = message.PHONE_TYPE_MOBILE;

      std::cout << "Publishing Contact: " << message.first_name << " " << message.last_name << std::endl;
      publisher_->publish(message);
    };

    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::AddressBook>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

主函数：

```cpp
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddressBookPublisher>());
  rclcpp::shutdown();
}
```

---

### **3.2️⃣ 构建这个节点**

在 `CMakeLists.txt` 添加：

```cmake
find_package(rclcpp REQUIRED)

add_executable(publish_address_book src/publish_address_book.cpp)
target_link_libraries(publish_address_book rclcpp::rclcpp)
```

---

### **3.3️⃣ 链接消息类型支持**

因为消息定义和节点在同一个包中，需要手动链接类型支持：

```cmake
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(publish_address_book "${cpp_typesupport_target}")
```

> 🚨 如果消息来自外部包（例如 `std_msgs`），这步就不用写。

---

### **4️⃣ 构建并运行**

```bash
colcon build --packages-up-to more_interfaces
source install/local_setup.bash
ros2 run more_interfaces publish_address_book
```

运行后你会看到：

```
Publishing Contact
First: John  Last: Doe
```

你还可以在另一个终端里查看消息：

```bash
ros2 topic echo /address_book
```

---

### **5️⃣（进阶）使用别的包的消息作为字段**

比如我们想定义一个通讯录，包含多个“联系人”，而联系人定义在别的包 `rosidl_tutorials_msgs` 里：

```text
rosidl_tutorials_msgs/Contact[] address_book
```

那么需要在 `package.xml` 中添加依赖：

```xml
<build_depend>rosidl_tutorials_msgs</build_depend>
<exec_depend>rosidl_tutorials_msgs</exec_depend>
```

在 `CMakeLists.txt`：

```cmake
find_package(rosidl_tutorials_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES rosidl_tutorials_msgs
)
```

---

## 🧩 三、重点总结

| 概念                                | 作用                        |
| --------------------------------- | ------------------------- |
| `rosidl_default_generators`       | 编译时生成 msg/srv/action 的源代码 |
| `rosidl_default_runtime`          | 运行时使用生成的接口                |
| `rosidl_generate_interfaces()`    | 生成接口代码                    |
| `rosidl_get_typesupport_target()` | 在同包中链接消息类型支持              |
| `ament_export_dependencies()`     | 向下游包导出依赖                  |
| `ros2 topic echo`                 | 查看消息发布情况                  |

---

## 🚀 四、你学到了什么

* 如何 **在同一个包** 里定义并使用消息；
* 如何配置 CMake 和 package.xml；
* 如何发布自定义消息；
* 如何引用其他包的消息；
* ROS 2 接口生成的内部机制（rosidl pipeline）。

---

是否希望我接着帮你画一张图，说明消息从 `.msg` → C++ 类 → 节点发布的流程？
（包含 `rosidl`、`rclcpp`、`rmw` 层的调用链）

---
# ament_export_dependencies 指令解析
---

非常好的问题 👍
`ament_export_dependencies()` 是 ROS 2 的 **CMake 构建系统（ament_cmake）** 提供的一个关键函数。
它的作用是——当你这个包被**其他包依赖**时，自动把你依赖的库或接口“传递”给下游包。

---

## 🌱 一、基本概念：依赖的“传递”

在 ROS 2 中，每个包都通过 `CMakeLists.txt` 来声明自己依赖了哪些库或接口。

举个例子：

```cmake
find_package(rosidl_default_runtime REQUIRED)
```

表示当前包用到了 ROS 2 的接口运行时系统。
但是——如果你的包会被别人用（例如别人要用你定义的消息），
那“别人”也必须能找到这个依赖。

所以你需要告诉下游包：

> “嘿，我用了 `rosidl_default_runtime`，你在编译我时也要包含它。”

这就用到了：

```cmake
ament_export_dependencies(rosidl_default_runtime)
```

---

## ⚙️ 二、它到底做了什么

`ament_export_dependencies()` 做了几件事：

1. ✅ **导出 include 路径**
   确保下游包在 `find_package(your_package)` 时，自动能包含正确的头文件。

2. ✅ **导出链接库依赖**
   如果你的包链接了某个库（比如 `rclcpp` 或者生成的消息类型支持），
   下游包链接你时也会自动带上这些库。

3. ✅ **导出构建依赖信息**
   让 `colcon` 或 `ament` 构建系统知道构建顺序（比如先构建 `rosidl_default_runtime` 再构建你）。

---

## 🧩 三、在消息包中的用途

在教程里，我们看到：

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
```

这里的意思是：

* `rosidl_generate_interfaces()` 生成了消息的类型支持代码；
* 但这些生成的代码在运行时依赖 `rosidl_default_runtime`；
* 所以要把这个依赖导出出去，否则别的包使用这个消息时会报错：

  ```
  undefined reference to `rosidl_typesupport_cpp...`
  ```

---

## 📦 四、和 `ament_export_include_directories`、`ament_export_libraries` 的区别

| 函数名                                  | 作用                                 |
| ------------------------------------ | ---------------------------------- |
| `ament_export_include_directories()` | 导出头文件路径（让别的包能 `#include`）          |
| `ament_export_libraries()`           | 导出库（让别的包能 `target_link_libraries`） |
| `ament_export_dependencies()`        | 导出包级别依赖（让别的包自动 `find_package()`）   |

通常三者可以搭配使用。
在消息包这种情况中，最重要的是 **依赖导出**，因为 `.msg` 生成的代码是通过依赖的 ROSIDL 运行时访问的。

---

## 💡 五、一个完整示例

假设有个包 `my_interfaces` 定义了自定义消息：

```cmake
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MyMsg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
```

然后另一个包 `my_talker` 想使用这个消息，只需写：

```cmake
find_package(my_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker
  rclcpp
  my_interfaces
)
```

👉 由于 `my_interfaces` 里已经写了
`ament_export_dependencies(rosidl_default_runtime)`
`my_talker` 就**自动继承**到了 `rosidl_default_runtime`，
不需要再手动 `find_package(rosidl_default_runtime)`。

---

## 🔍 六、总结一句话

> `ament_export_dependencies()` 的作用是：
>
> 💬 “让下游包在使用我时，自动带上我依赖的包。”

---

是否希望我给你展示一个最小例子：
展示如果**不写** `ament_export_dependencies(rosidl_default_runtime)` 会发生什么编译错误？
（这样你能更直观看出它的必要性）


---
# 第三部分 最小示例

---
非常好 👍 这个问题问得非常深入。我们就来**一步步构建一个最小例子**，直观理解为什么 `ament_export_dependencies(rosidl_default_runtime)` 是必需的。

---

## 🎯 目标

我们要对比两种情况：

1. ✅ **正确做法**：消息包导出了依赖，下游包编译成功。
2. ❌ **错误做法**：消息包没导出依赖，下游包编译失败。

---

## 🧩 例子结构

我们创建两个包：

```
ros2_ws/src/
├── my_interfaces/      # 定义自定义消息
│   └── msg/
│       └── MyMsg.msg
└── my_talker/          # 使用上面的消息
    └── src/
        └── talker.cpp
```

---

## 🧱 Step 1: my_interfaces（定义消息）

### msg/MyMsg.msg

```text
string content
```

### package.xml

```xml
<package format="3">
  <name>my_interfaces</name>
  <version>0.0.0</version>
  <description>Minimal message example</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

### CMakeLists.txt（两种情况对比）

#### ✅ 正确写法（导出依赖）

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_interfaces)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MyMsg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

#### ❌ 错误写法（**少了**最后那行）

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_interfaces)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MyMsg.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_package()
```

---

## 🧱 Step 2: my_talker（使用消息）

### src/talker.cpp

```cpp
#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/my_msg.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("talker");
  auto pub = node->create_publisher<my_interfaces::msg::MyMsg>("chatter", 10);

  rclcpp::WallRate rate(1);
  while (rclcpp::ok()) {
    my_interfaces::msg::MyMsg msg;
    msg.content = "Hello ROS 2!";
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", msg.content.c_str());
    pub->publish(msg);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_talker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_interfaces REQUIRED)   # <-- 使用上面的接口包

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp my_interfaces)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---

## ⚙️ Step 3: 构建和结果

### ✅ 如果 **my_interfaces** 里写了：

```cmake
ament_export_dependencies(rosidl_default_runtime)
```

则：

```bash
colcon build --packages-select my_interfaces my_talker
```

✅ 编译成功！

---

### ❌ 如果 **没有写** `ament_export_dependencies(rosidl_default_runtime)`

则：

```bash
colcon build --packages-select my_interfaces my_talker
```

会报错：

```
/usr/bin/ld: CMakeFiles/talker.dir/src/talker.cpp.o: in function `rclcpp::Publisher<my_interfaces::msg::MyMsg>::Publisher(...)':
undefined reference to `rosidl_typesupport_cpp::get_message_type_support_handle<my_interfaces::msg::MyMsg>()'
collect2: error: ld returned 1 exit status
make[2]: *** [talker] Error 1
```

---

## 🧠 原因解释

* `my_talker` 想使用 `my_interfaces::msg::MyMsg`。
* 这需要链接到消息类型支持库（`rosidl_typesupport_cpp`）。
* 但如果 `my_interfaces` 没导出依赖，`my_talker` 编译时就**不知道要链接哪些库**。
* 所以链接阶段找不到符号，报 `undefined reference`。

---

## ✅ 总结

| 写法                                                    | 结果     | 原因       |
| ----------------------------------------------------- | ------ | -------- |
| ✅ `ament_export_dependencies(rosidl_default_runtime)` | ✅ 编译成功 | 依赖被正确传递  |
| ❌ 没有这行                                                | ❌ 链接失败 | 类型支持库未传递 |

---

是否希望我帮你做一个**同时演示这两种情况的最小 colcon 工程 zip 包结构**（你直接解压就能运行）？
我可以把两个版本都准备好，你能在 container 内直接编译对比。
