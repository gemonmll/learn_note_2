下面我来 **详细介绍**一下 RViz2（ROS 2 可视化工具）官方教程 “Building a Custom RViz Panel” 的内容、结构、关键点，以及在实践中你可能要关注的事项。你如果有某个章节要更深入（例如 Qt 插件结构、ROS 节点集成、界面设计等），我也可以帮你进一步解释。

---

## 一、教程背景

这个教程的定位是：如果你想在 RViz 中不仅仅是“可视化数据”，而是想在 RViz 界面里创建一个 **自定义面板（Panel）**，用于显示或交互（例如监控话题、发布按钮事件等）——那么这个教程就是为你准备的。 ([docs.ros.org][1])

具体来说，你将学习以下三件事：

* 在 RViz 中创建一个新的 Qt 面板（Panel）组件。 ([docs.ros.org][1])
* 在这个面板里订阅 ROS 话题，显示接收到的数据。 ([docs.ros.org][1])
* 在面板里设置一个按钮，当点击后通过 ROS 发布消息。 ([docs.ros.org][1])

所以这个面板既有 “监听” 功能，也有 “发出” 功能，是一个双向交互的小插件。

---

## 二、骨架代码（Boilerplate Code）

教程首先给出了最基础的面板插件结构：

### 头文件（Header）

例如 `demo_panel.hpp` 的初始内容：

```cpp
#ifndef RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_
#define RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_

#include <rviz_common/panel.hpp>

namespace rviz_panel_tutorial
{
class DemoPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit DemoPanel(QWidget * parent = 0);
  ~DemoPanel() override;
};
}  // namespace rviz_panel_tutorial

#endif  // RVIZ_PANEL_TUTORIAL__DEMO_PANEL_HPP_
```

要点包括：

* 继承自 `rviz_common::Panel`。 ([docs.ros.org][1])
* 使用 `Q_OBJECT` 宏，因为 Qt 信号／槽机制需要它。 ([docs.ros.org][1])
* 声明构造函数和析构函数。

### 源文件（Source）

例如 `demo_panel.cpp` 的初始内容：

```cpp
#include <rviz_panel_tutorial/demo_panel.hpp>

namespace rviz_panel_tutorial
{
DemoPanel::DemoPanel(QWidget* parent) : Panel(parent)
{
}

DemoPanel::~DemoPanel() = default;
}  // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_panel_tutorial::DemoPanel, rviz_common::Panel)
```

关键点：

* 构造函数调用基类 `Panel(parent)`。
* 使用 `PLUGINLIB_EXPORT_CLASS(...)` 宏，将你的类导出为 pluginlib 可识别的面板插件。 ([docs.ros.org][1])

### package.xml & 插件描述 XML

在 `package.xml` 中，需要依赖：

```xml
<depend>pluginlib</depend>
<depend>rviz_common</depend>
```

(教程中提及) ([docs.ros.org][1])

在 `rviz_common_plugins.xml`（或者你自己的插件描述文件）中，你需要添加：

```xml
<library path="demo_panel">
  <class type="rviz_panel_tutorial::DemoPanel" base_class_type="rviz_common::Panel">
    <description></description>
  </class>
</library>
```

这里 `path="demo_panel"` 对应你生成的库名，`type` 对应你的类全名。 ([docs.ros.org][1])

### CMakeLists.txt

主要流程包括：

* `find_package(ament_cmake_ros REQUIRED)`、`find_package(pluginlib REQUIRED)`、`find_package(rviz_common REQUIRED)`。 ([docs.ros.org][1])
* 启用 `CMAKE_AUTOMOC ON`（因为 Qt 的 `Q_OBJECT` 宏）和 `qt5_wrap_cpp` 用于处理 MOC 文件。 ([docs.ros.org][1])
* `add_library(demo_panel src/demo_panel.cpp ${MOC_FILES})`，设置 include 目录、链接依赖、安装目标。 ([docs.ros.org][1])
* 安装插件描述文件：

  ```cmake
  install(FILES rviz_common_plugins.xml
          DESTINATION share/${PROJECT_NAME})
  ```

  ([docs.ros.org][1])

### 测试初版插件

构建并安装后，启动 `rviz2`，在菜单栏中选择 “Panels → Add New Panel”，你应该看到“DemoPanel”选项（或你命名的面板名），添加后会显示一个空白的面板（只有标题栏，没有内容）。 ([docs.ros.org][1])

---

## 三、填充界面与 ROS 接入（Filling in the Panel）

这部分教程讲如何让面板“活”起来：包括一个标签（QLabel）显示订阅话题的数据；一个按钮（QPushButton）点击后发布消息。 ([docs.ros.org][1])

### 更新头文件

在 `demo_panel.hpp` 中新增：

```cpp
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_msgs/msg/string.hpp>
#include <QLabel>
#include <QPushButton>

namespace rviz_panel_tutorial
{
class DemoPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit DemoPanel(QWidget * parent = 0);
  ~DemoPanel() override;
  void onInitialize() override;

protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void topicCallback(const std_msgs::msg::String & msg);

  QLabel* label_;
  QPushButton* button_;

private Q_SLOTS:
  void buttonActivated();
};
}  // namespace rviz_panel_tutorial
```

要点：

* 增加 `onInitialize()` 方法，用于初始化 ROS 接口。
* 使用 `RosNodeAbstractionIface` 从 RViz 上下文获取 ROS 节点抽象。
* 声明 publisher 和 subscription。
* 声明 Qt 控件 `label_` 和 `button_`，以及槽函数 `buttonActivated()`。 ([docs.ros.org][1])

### 更新源文件

在 `demo_panel.cpp` 中更新为：

```cpp
#include <rviz_panel_tutorial/demo_panel.hpp>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

namespace rviz_panel_tutorial
{
DemoPanel::DemoPanel(QWidget* parent) : Panel(parent)
{
  const auto layout = new QVBoxLayout(this);
  label_ = new QLabel("[no data]");
  button_ = new QPushButton("GO!");
  layout->addWidget(label_);
  layout->addWidget(button_);
  QObject::connect(button_, &QPushButton::released, this, &DemoPanel::buttonActivated);
}

DemoPanel::~DemoPanel() = default;

void DemoPanel::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();
  publisher_ = node->create_publisher<std_msgs::msg::String>("/output", 10);
  subscription_ = node->create_subscription<std_msgs::msg::String>(
      "/input", 10, std::bind(&DemoPanel::topicCallback, this, std::placeholders::_1));
}

void DemoPanel::topicCallback(const std_msgs::msg::String & msg)
{
  label_->setText(QString(msg.data.c_str()));
}

void DemoPanel::buttonActivated()
{
  auto message = std_msgs::msg::String();
  message.data = "Button clicked!";
  publisher_->publish(message);
}
}  // namespace rviz_panel_tutorial

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_panel_tutorial::DemoPanel, rviz_common::Panel)
```

说明：

* 在构造函数里构建界面（布局、标签、按钮、连接槽）。
* 在 `onInitialize()` 获取节点上下文并创建 publisher/subscription。
* `topicCallback()` 将接收到的消息数据显示到标签。
* `buttonActivated()` 点击按钮即向 `/output` 话题发送一条字符串消息。
* 完成后通过 `PLUGINLIB_EXPORT_CLASS` 导出。 ([docs.ros.org][1])

### 测试交互功能

* 构建并运行 RViz2。
* 在面板中，你应能看到 “[no data]” 的标签和 “GO!” 的按钮。
* 在终端你可以发布 `/input` 话题：

  ```bash
  ros2 topic pub /input std_msgs/msg/String "{data: 'Please be kind.'}"
  ```

  面板中的标签应更新为 “Please be kind.”。 ([docs.ros.org][1])
* 点击面板按钮，终端监控 `/output` 话题：

  ```bash
  ros2 topic echo /output
  ```

  应显示 “Button clicked!”。 ([docs.ros.org][1])

---

## 四、清理／美化（Cleanup）

为了使插件更正式、更好用，教程最后建议一些“整洁”步骤：

* 在 `rviz_common_plugins.xml` 中为插件添加描述和图标。例如将 `icons/classes/DemoPanel.png` 放入对应路径。 ([docs.ros.org][1])
* 在 CMakeLists.txt 中安装图标：

  ```cmake
  install(FILES icons/classes/DemoPanel.png
          DESTINATION share/${PROJECT_NAME}/icons/classes)
  ```

  ([docs.ros.org][1])
* 这样用户在 “Add New Panel” 对话框中能看到图标与说明，更直观。

---

## 五、总结流程

制作一个自定义 RViz 面板插件的大致流程可以归纳为：

1. 创建 ROS 2 包，声明依赖 `rviz_common`、`pluginlib`。
2. 编写插件继承自 `rviz_common::Panel`，设置 Qt 容器（控件、布局、信号槽）。
3. 在插件初始化阶段 (`onInitialize()`) 获取 ROS 节点上下文并创建 publisher/subscriber。
4. 在 GUI 控件中订阅话题并将数据反应到界面；设置按钮等控件触发 ROS 发布。
5. 在 CMakeLists.txt 中处理 Qt 的 MOC、构建库、安装插件描述文件。
6. 在 `rviz_common_plugins.xml` 中声明插件库、类、描述、图标。
7. 构建并运行 RViz2，使用 “Panels → Add New Panel” 来加载你的插件。
8. 发布／订阅对应话题，验证界面响应。
9. 最后，可选地为插件添加图标／说明，以提升用户体验。

---

## 六、实践中需要注意／推荐事项

* 确保你的包使用的是与你 ROS 2 分发版（如 Jazzy）兼容的版本。教程处注明“你现在阅读的是旧版本，最新版本请查看 Kilted” 。 ([docs.ros.org][1])
* Qt 与 ROS 集成：使用 `Q_OBJECT` 宏、`CMAKE_AUTOMOC ON`、`qt5_wrap_cpp` 是必须的。忽略这几步可能导致编译失败或插件不被识别。
* 获取 ROS 节点上下文：必须通过 `getDisplayContext()->getRosNodeAbstraction().lock()` 获取节点抽象，而不是自行创建。插件必须运行在 RViz 环境中。 ([docs.ros.org][1])
* 界面设计：在构造函数中构建布局并创建控件；在 `onInitialize()` 才启动 ROS 通信。不要在构造函数中依赖 ROS 节点，因为上下文可能尚未准备。
* 发布／订阅的话题名称、类型要一致，否则可能收到无效消息或掉信息。教程示例中 `/input` 和 `/output` 使用 `std_msgs/msg/String`。
* 插件声明与安装：如果在 `rviz_common_plugins.xml` 中未正确声明或路径安装错误，RViz 无法识别你的面板。
* 用户界面体验：添加图标和说明可以让插件在 UI 中显得更专业、更易识别。
* 测试：建议在 RViz 中保存你的面板配置（*.rviz 文件），以便下次快速加载。
* 扩展：本教程只是一个简单演示，实际项目中你可以在面板中添加更多控件（滑块、下拉菜单、图像预览、自定义绘图区域等），并订阅或发布复杂消息类型。

---

如果你愿意，我可以 **帮你逐章翻译**这个教程（包括全部代码示例、注释、每一步 CMake/包配置）成中文，并加入一些 **实践示例**（比如订阅一个你自己定义的消息类型、或者在面板上显示一个图像或按钮组合）。你看要不要？

[1]: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-Custom-Panel/RViz-Custom-Panel.html "Building a Custom RViz Panel — ROS 2 Documentation: Jazzy  documentation"
