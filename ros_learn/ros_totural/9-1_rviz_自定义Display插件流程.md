下面我帮你 **详尽介绍**一下 RViz2（ROS 2 中常用的三维可视化工具）官方教程 “RViz2 — *Building a Custom RViz Display*” 的内容、结构、关键点，以及在实践中你可能要关注的事项。你如果有某个章节要更深入（例如插件结构、CMake配置、Property系统等），我也可以帮你深入解释。

---

## 一、教程背景 (Background)

教程开头指出：

* 在 RViz2 中，已经有很多 **现成的 Display 插件** 用于展示各种常见的消息类型（如激光扫描、点云、机器人模型、TF 坐标系等）。 ([docs.ros.org][1])
* 但如果你有一种 **自定义消息类型**（或已有类型但想用特殊方式可视化），而现有的 Display 插件不支持，通常有两种选择：

  1. **将消息转换成已有类型**（如 `visualization_msgs/Marker`）之后再用已有插件显示。 ([docs.ros.org][1])
  2. **编写一个自定义的 Display 插件**，专门处理你的消息类型并在 RViz2 中用你定义的方式进行渲染。教程就是讲这第二条。 ([docs.ros.org][1])
* 虽然第 1 种方法“快速且灵活”，但可能造成更多网络流量、数据转换开销、以及可视化方式受限。相对地，第 2 种方法“工作量稍大但可视化效果更强”。 ([docs.ros.org][2])

**为什么要做？**
如果你在 ROS 2 中开发系统，需要在 RViz 中显示自己定义的消息（比如某种传感器数据、算法的中间结果、特殊可视化对象等），而现有插件不能满足，那就需要写插件。这个教程正是指导如何从零开始编写一个简单的自定义 Display 插件。

---

## 二、教程所用的 “玩具消息类型” (Point2D Message)

为了演示插件的制作，教程定义了一个非常简单的消息类型：

```text
std_msgs/Header header
float64 x
float64 y
```

也就是 `Point2D.msg`（属于包 `rviz_plugin_tutorial_msgs`） ([docs.ros.org][1])
这个消息有：

* 一个 `header`，包含 frame_id + timestamp
* 两个坐标：浮点数 `x` 和 `y`

这个简单类型的优点是：逻辑简单、容易看懂；你用它来演示从接收消息 → 转换到场景图中渲染一个对象。

---

## 三、插件基本骨架 (Boilerplate for Basic Plugin)

这个部分教程讲如何建立最小可运行的自定义 Display 插件。主要内容包括：头文件 (header)、源文件 (cpp)、package.xml、插件描述 XML、CMakeLists 等。 ([docs.ros.org][2])

### 头文件 (Header File)

示例 `point_display.hpp` 大致如下：

```cpp
#ifndef RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
#define RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_

#include <rviz_common/message_filter_display.hpp>
#include <rviz_plugin_tutorial_msgs/msg/point2_d.hpp>

namespace rviz_plugin_tutorial
{
class PointDisplay
  : public rviz_common::MessageFilterDisplay<rviz_plugin_tutorial_msgs::msg::Point2D>
{
  Q_OBJECT
protected:
  void processMessage(const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg) override;
};
}  // namespace rviz_plugin_tutorial
#endif  // RVIZ_PLUGIN_TUTORIAL__POINT_DISPLAY_HPP_
```

关键要点：

* 插件类继承自 `rviz_common::MessageFilterDisplay<YourMsgType>`，即用模板将你自己的消息类型作为参数。 ([docs.ros.org][1])
* 使用了 `Q_OBJECT` 宏，这是因为 RViz 的 GUI 部分（基于 Qt）需要它才能使信号/槽机制正常。 ([docs.ros.org][1])
* 实现了 `processMessage(...)` 方法，这是你用来接收每条消息并做可视化更新的地方。

### 源文件 (Source File)

在 `point_display.cpp` 中，例如：

```cpp
#include <rviz_plugin_tutorial/point_display.hpp>
#include <rviz_common/logging.hpp>

namespace rviz_plugin_tutorial
{
void PointDisplay::processMessage(const rviz_plugin_tutorial_msgs::msg::Point2D::ConstSharedPtr msg)
{
  RVIZ_COMMON_LOG_INFO_STREAM("We got a message with frame " << msg->header.frame_id);
}
}  // namespace rviz_plugin_tutorial

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rviz_plugin_tutorial::PointDisplay,
  rviz_common::Display)
```

要点：

* 在 `processMessage` 里，最基础就是接收到消息后打印一条日志，以证明插件生命周期和消息回调是工作了。 ([docs.ros.org][1])
* 最后通过 `PLUGINLIB_EXPORT_CLASS` 宏导出这个类，使得 pluginlib／RViz 能识别加载它。 ([docs.ros.org][2])

### package.xml & plugins 描述 xml

你需要在 package.xml 中声明如下依赖（至少）：

```xml
<depend>pluginlib</depend>
<depend>rviz_common</depend>
<depend>rviz_plugin_tutorial_msgs</depend>
```

([docs.ros.org][2])

另外，在 `rviz_common_plugins.xml`（或你自己定义的 XML ）中：

```xml
<library path="point_display">
  <class type="rviz_plugin_tutorial::PointDisplay" base_class_type="rviz_common::Display">
    <description></description>
  </class>
</library>
```

要匹配你的库路径 & 类名。 ([docs.ros.org][2])

### CMakeLists.txt

CMake 脚本需要完成以下工作：

* `find_package(ament_cmake_ros REQUIRED)`、`find_package(pluginlib REQUIRED)`、`find_package(rviz_common REQUIRED)`、`find_package(rviz_plugin_tutorial_msgs REQUIRED)` 等。 ([docs.ros.org][1])
* 启用 `CMAKE_AUTOMOC ON`（因为使用 Qt 的 Q_OBJECT 宏）和使用 `qt5_wrap_cpp` 处理带 Q_OBJECT 的头。 ([docs.ros.org][1])
* `add_library(point_display src/point_display.cpp ${MOC_FILES})`、将其链接依赖目标、安装 TARGETS 等。 ([docs.ros.org][1])
* 安装 rviz_plugins.xml 或 plugins 描述 xml 到 share/${PROJECT_NAME}。 ([docs.ros.org][1])

### 测试／运行

编译好、安装好后，启动 `rviz2`，然后在 “Add” 菜单中查找你的 Display 插件并添加。最初会显示 “error” 状态，因为尚未设置消息 topic。然后你为该插件设置 topic（例如 `/point`），并在终端发布消息：

```bash
ros2 topic pub /point rviz_plugin_tutorial_msgs/msg/Point2D "{header: {frame_id: map}, x: 1, y: 2}" -r 0.5
```

你应该能在 RViz 的输出中看到 “We got a message with frame map” 的日志。 ([docs.ros.org][1])

---

## 四、真正可视化 (Actual Visualization)

到此为止，插件已经能接收到消息了，但尚未真正 “画出”什么东西。这个步骤在教程中称为 “Actual Visualization” (step2) 。 ([docs.ros.org][2])

主要新增内容：

* 增加 `rviz_rendering` 作为依赖（因为你要创建可视化对象，如 Shape 等） 。 ([docs.ros.org][1])
* 在头文件中加入：

  * `#include <rviz_rendering/objects/shape.hpp>`
  * 在类里新增成员 `std::unique_ptr<rviz_rendering::Shape> point_shape_;`
  * 在类中重载 `void onInitialize() override;` 方法。 ([docs.ros.org][1])
* 在 `onInitialize()` 中创建 `point_shape_` 实例：

  ```cpp
  point_shape_ = std::make_unique<rviz_rendering::Shape>(
      rviz_rendering::Shape::Type::Cube, scene_manager_, scene_node_);
  ```

  注意：**必须在 `onInitialize` 而**不是构造函数中创建，因为 `scene_manager_` 和 `scene_node_` 在构造函数时尚未就绪。 ([docs.ros.org][1])
* 在 `processMessage` 中更新可视化对象的位置／方向：

  ```cpp
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    // 报错或 skip
  }
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  Ogre::Vector3 point_pos;
  point_pos.x = msg->x;
  point_pos.y = msg->y;
  point_shape_->setPosition(point_pos);
  ```

  这样，从消息中读取 x、y，转换到 OGRE（RViz 的渲染引擎）空间，设置 scene_node_ 的位置／方向，从而在 RViz 中 “看到”这个形状被移动到消息定义的位置。 ([docs.ros.org][2])
* 教程还提醒：如果形状没有出现，可能是因为：没有在 topic 发布消息、消息太久没发布、或者在 RViz 中未正确设置 topic。 ([docs.ros.org][2])

---

## 五、“让用户定制”可视化属性 (It’s Nice to Have Options)

如果你希望用户（使用 RViz）能够通过 GUI 控件（属性面板）改变这个 Display 的可视化样式，比如颜色、大小、透明度等，教程进入这个步骤 (step3) 。 ([docs.ros.org][1])

主要内容包括：

* 在头文件中 `#include <rviz_common/properties/color_property.hpp>` 等你想用的属性类型。 ([docs.ros.org][1])
* 在类里新增成员属性指针，例如：

  ```cpp
  std::unique_ptr<rviz_common::properties::ColorProperty> color_property_;
  private Q_SLOTS:
    void updateStyle();
  ```
* 在 `onInitialize()` 中创建 property 对象：

  ```cpp
  color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
      "Point Color", QColor(36, 64, 142),
      "Color to draw the point.", this, SLOT(updateStyle()));
  updateStyle();
  ```

  这里 `"Point Color"` 是用户看到的属性名称，默认颜色、描述、回调槽。 ([docs.ros.org][1])
* 然后实现 `updateStyle()`，从 QColor 读取值，转换为 OGRE 颜色（使用 `qtToOgre()`），并应用到 point_shape_。例如：

  ```cpp
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_property_->getColor());
  point_shape_->setColor(color);
  ```

  ([docs.ros.org][1])
* 这样，用户在 RViz 左侧属性面板中就可以看到 “Point Color” 属性，点击改变颜色后，插件内部会响应并更新形状颜色。

---

## 六、显示状态 (Status Report)

为了使你的插件更“健壮”并且能在 RViz 中显示状态反馈（如提示错误、警告、正常状态），教程到了 step4，介绍如何使用 StatusProperty 。 ([docs.ros.org][1])

例如，在 `processMessage()` 中加入如下逻辑：

```cpp
if (msg->x < 0) {
  setStatus(StatusProperty::Warn, "Message",
            "I will complain about points with negative x values.");
} else {
  setStatus(StatusProperty::Ok, "Message", "OK");
}
```

这里：

* `StatusProperty` 定义了状态类别（Error、Warn、Ok）＋ 一个 key（这里是 `"Message"`）＋描述文本。
* 在 RViz 左侧 Display 面板中，你会看到该 Display 名称旁边有状态图标（绿色＝OK、黄色＝Warn、红色＝Error） 。
* 这使得用户能直观知道某条消息是否满足你的预期、是否存在警告等。

---

## 七、清理／打包 (Cleanup)

教程的最后一个步骤 (step5) 是如何完善插件，使得它看起来更“正式”、更 RViz‐友好。内容包括： ([docs.ros.org][1])

* 在插件描述 XML （`rviz_common_plugins.xml`）中，为 class 标签加入 `name="Point2D"`、`message_type="rviz_plugin_tutorial_msgs/msg/Point2D"` 等字段。例如：

  ```xml
  <class name="Point2D" type="rviz_plugin_tutorial::PointDisplay"
         base_class_type="rviz_common::Display">
    <description>Tutorial to display a point</description>
    <message_type>rviz_plugin_tutorial_msgs/msg/Point2D</message_type>
  </class>
  ```

  * `name` 字段：用户在 Add 菜单看到的名字。
  * `message_type`：说明该插件对应的消息类型，这样 RViz 在 “Add by Topic” 时可建议你使用这个插件。
* 为插件增加 icon（例如 `icons/classes/Point2D.png`），并在 CMake 脚本中安装它：

  ```cmake
  install(FILES icons/classes/Point2D.png
          DESTINATION share/${PROJECT_NAME}/icons/classes)
  ```
* 注意：如果你改变了插件的 name ／ type ／路径，那么之前保存的 RViz 配置可能失效。教程提醒用户注意。 ([docs.ros.org][1])

---

## 八、总结流程

综上，制作一个自定义 RViz2 Display 插件的大致流程可分为以下几个阶段：

| 阶段       | 主要任务                                                                                                   |
| -------- | ------------------------------------------------------------------------------------------------------ |
| 定义消息类型   | 自己定义一个消息（如 Point2D）或使用已有的。                                                                             |
| 创建插件骨架   | 继承 `MessageFilterDisplay<MsgType>`，实现 `processMessage()`，在 package.xml、CMakeLists.txt、plugins.xml 中配置。 |
| 编译/安装/测试 | 构建包、安装、启动 rviz2 、添加插件、订阅 topic、打印日志。                                                                   |
| 添加可视化对象  | 引入 rviz_rendering ，创建 Shape、场景节点，设置位置／方向／图形。                                                           |
| 增强属性支持   | 添加 ColorProperty、SizeProperty 等，使用户可在 RViz GUI 中定制样式。                                                  |
| 添加状态反馈   | 使用 `setStatus()` 方法，为插件提供 OK/Warn/Error 状态反馈。                                                          |
| 打包完善     | 完善插件名称、消息类型关联、图标安装、说明文字，方便用户 Add 菜单使用。                                                                 |

---

## 九、实践中需要注意／推荐事项

* 确保你的消息类型 **包含 std_msgs/Header**（即有 header.frame_id 和 stamp）时，使用 `MessageFilterDisplay` 是最简单的路径；如果没有 header ，则可能需要使用其他基类。教程默认是含 header。 ([docs.ros.org][2])
* 在 onInitialize() 里创建 scene_node_／shape 等对象，而不要放在构造函数里，因为 scene_manager_／scene_node_ 还没准备好。教程强调这一点。 ([docs.ros.org][1])
* 坐标变换：在 processMessage 中获取 msg->header 指定的 frame 到 fixed_frame 的变换，保证你的可视化对象位置正确。否则会出现 “物体没显示／位置乱” 的情况。 ([docs.ros.org][2])
* 属性（Properties）系统：你可以不仅仅支持颜色，还可以支持大小、透明度、形状类型、线条粗细等，借助不同的 Property 子类（如 FloatProperty、EnumProperty、BoolProperty 等）进行扩展。
* 状态反馈系统：除了 “x < 0 报 Warning” 这种示例，实际使用中你可以根据消息内容、订阅情况、帧变换失败、过期消息等触发 Error/Warn，帮助用户调试。
* 插件部署：制作好插件后，别忘记将对应 .so 库安装到 install 目录，并确保 plugins.xml （或 rviz_common_plugins.xml）与 package.xml 一致、 CMakeLists.txt 安装路径也正确。否则 RViz 可能找不到插件／加载失败。
* 命名／版本兼容：如果你改了插件名称／类型／图标路径，以前保存的 RViz 配置文件可能 “失联” 或找不到该插件。要么更新配置，要么兼容旧名字。教程也提到这一点。 ([docs.ros.org][1])
* 在 RViz 中测试：

  * 在 Displays 列表中看插件状态（红/黄/绿）
  * 确保 topic 名正确、消息频率合适、固定帧 (Fixed Frame) 设定正确、TF 变换可用。
  * 如果可视化对象没有出现，检查：没有发布消息、消息太旧、坐标关系不对、scene_node_／shape 尺寸太小或被遮挡。
* 最后，建议保存你定制插件对应的 rviz 配置文件（*.rviz），方便以后快速加载。

---

如果你愿意，我可以 **帮你逐章翻译**这个教程（包括全部代码示例、注释、每一步 CMake/包配置）成中文，并加入一些 **实践示例**（比如订阅一个真实消息类型，改成你自己的消息类型）。你看要不要？

[1]: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-Custom-Display/RViz-Custom-Display.html "Building a Custom RViz Display — ROS 2 Documentation: Jazzy  documentation"
[2]: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Custom-Display/RViz-Custom-Display.html?utm_source=chatgpt.com "Building a Custom RViz Display"
