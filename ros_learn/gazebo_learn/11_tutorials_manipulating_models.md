下面我来较为 **完整、详细** 地帮你学习这篇 Ignition Gazebo (Fortress) 教程 — *“Manipulating Models”*（操作模型） — 并附上关键的说明与示例代码。你若想，我也可以帮你翻译为中文（或者给出中文解说）。

---

## 一、教程概况

这篇教程是 “Docs / Gazebo Fortress LTS” 系列中的一个 GUI 操作模型的入门教程。 ([Gazebo Simulation][1])
它主要介绍如何在 Ignition Gazebo 的图形用户界面中，通过插件操作模型（选择、平移、旋转、对齐等）。

重点如下：

* 使用 “Transform Control” 插件（包括选择、平移、旋转、对齐到世界坐标系、设置“捕捉（Snap）”值）([Gazebo Simulation][1])
* 使用 “Component Inspector” 插件，用于查看模型或实体的属性（姿态、是否静态、重力、磁场等）([Gazebo Simulation][1])
* 使用 “View Angle” 插件，快速切换视角对模型观察([Gazebo Simulation][1])
* 使用 “Align Tool” 插件，将一个模型按另一个模型的包围盒（bounding box）沿特定轴对齐／贴靠([Gazebo Simulation][1])

该教程适用于已安装并能运行 Ignition Gazebo 的用户，前提是你了解 GUI 的基本使用（之前有个 “Understanding the GUI” 教程）([Gazebo Simulation][1])

---

## 二、前置条件（Prerequisites）

* 启动一个示例 world：例如运行命令

  ```
  ign gazebo shapes.sdf
  ```

  来打开一个包含基本几何体的世界。 ([Gazebo Simulation][1])
* 已具备对 GUI 的基本导航理解，如摄像机视角、选择实体、树视图、插件列表等。
* 熟悉基本术语：实体 (entity)、模型 (model)、坐标轴 (x, y, z)、局部轴 vs 世界轴、包围盒 (bounding box) 等。
* 确保你的 GUI 显示顶部工具栏、插件菜单、实体树 (Entity Tree)、右侧面板等。

---

## 三、主要模块详解 + 操作说明

下面我按模块展开，说明每个插件／功能是什么、怎么用、有什么技巧。由于原文主要是 GUI 操作，没有大量代码，但我也会补充一些可能的命令或配置示例。

### 1. Transform Control 插件

这是最核心的插件，用于在 GUI 中对模型实体进行选中、平移 (translate)、旋转 (rotate)、对齐 (snap) 操作。 ([Gazebo Simulation][1])

#### • 选择模式 (Select Mode)

* 默认模式。你点击某模型即可选中。被选中的实体会在实体树 (Entity Tree) 中被高亮显示。 ([Gazebo Simulation][1])
* 多选：按住 `Ctrl` 键并点击多个实体。 ([Gazebo Simulation][1])
* 在选择模式下不能做平移或旋转。若想进入移动／旋转模式，要切换。
* 快捷键：按 `Esc` 可以退出当前模式，返回选择模式。 ([Gazebo Simulation][1])

#### • 平移模式 (Translate Mode)

* 切换方式：点击工具栏中第二个图标（向箭头/平移图标），或在 Transform Control 插件中选择，或按快捷键 `T`。 ([Gazebo Simulation][1])
* 被选中模型会出现三个箭头（红 = x 轴, 绿 = y 轴, 蓝 = z 轴）。点击并拖拽这些箭头即可沿相应轴平移模型。 ([Gazebo Simulation][1])
* 也可以按住 `X`, `Y` 或 `Z`（或组合）键，然后点击并拖拽，这样可以固定沿那几个轴移动，无论鼠标方向如何。 ([Gazebo Simulation][1])

#### • 旋转模式 (Rotate Mode)

* 切换方式：点击第三个图标，或在插件里选，或按快捷键 `R`。 ([Gazebo Simulation][1])
* 被选中模型会出现三个代表 roll/pitch/yaw 的圆弧：红 = roll (绕 x 轴), 绿 = pitch (绕 y 轴), 蓝 = yaw (绕 z 轴) 。点击/拖拽即可绕那轴旋转。 ([Gazebo Simulation][1])

#### • 对齐到世界坐标轴 (Align to World frame)

* 当模型旋转后，其本地坐标轴可能与世界坐标系不对齐。如果你希望沿世界轴而不是模型本地轴进行移动／旋转，可按住 `Shift` 键。这样操作后会临时沿世界轴进行，但松开后又恢复到本地轴。 ([Gazebo Simulation][1])
* 注意：这是在交互过程中临时“切换”到世界轴，不是永久改变模型的本地轴方向。 ([Gazebo Simulation][1])

#### • 捕捉 (Snap) 值设定

* 在平移或旋转时，如果按住 `Ctrl` 然后拖拽，则会按照预设增量来“跳跃移动”/“跳跃旋转”——即“捕捉”效果。默认值为：平移 1 米，旋转 45°。 ([Gazebo Simulation][1])
* 捕捉平移是依据世界轴进行，旋转的捕捉是依据对象当前的方向。 ([Gazebo Simulation][1])
* 自定义捕捉值：点击工具栏中“Snap 图标”或在 Transform Control 插件里设置。 ([Gazebo Simulation][1])
* 举例：同时按住 `Shift` + `Ctrl`，可保持沿世界轴且按照捕捉增量进行移动。 ([Gazebo Simulation][1])

**总结要点**：如果你想快速把一个模型放到合适的位置／朝向，这些工具很有用。少量键盘（T, R, Shift, Ctrl）配合鼠标拖拽即可。

### 2. Component Inspector 插件

该插件用于查看选中实体（模型、链接、关节等）的属性。是一个查看工具，不直接用于移动／旋转。 ([Gazebo Simulation][1])

* 在插件菜单中选择 “Component Inspector” 启用。
* 选中一个实体后，你会在右侧面板看到其属性，如 `Pose` (位置和姿态)、是否为静态 (`static`)、是否受风作用 (wind)、世界的重力/磁场等。 ([Gazebo Simulation][1])
* 若实体在仿真中移动／旋转时，其 `Pose` 值会自动更新。你也可以 **暂停**这个更新（暂停插件刷新）以便观察固定状态。 ([Gazebo Simulation][1])
* 插件支持锁定某个实体：即便切换其它实体，仍可继续监视该实体属性。也可以同时打开多个 Component Inspector 窗口来监视多个实体。 ([Gazebo Simulation][1])

**实用提示**：在调试模型位置／姿态问题（如模型不在预期位置，或掉落）时，Component Inspector 非常有用。

### 3. View Angle 插件

该插件帮助你快速从某个实体或世界框架切换视角，以便观察模型或场景。 ([Gazebo Simulation][1])

* 在插件菜单选 “View Angle” 启动。
* 如果选中一个实体，该视角可以“面对”这个实体；如果没有选中实体，则视角是相对于世界。 ([Gazebo Simulation][1])
* 支持多选实体：你可以选择多个实体（Ctrl + 点击），然后切换视角使它们都在视野中。 ([Gazebo Simulation][1])
* “Home”按钮：恢复加载场景时的原始视角。 ([Gazebo Simulation][1])

**用途**：当你的场景比较复杂、模型很多、视角难以快速调整时，用 View Angle 快速定位非常便捷。

### 4. Align Tool 插件

这个插件是用来“按包围盒”(bounding box)将一个模型对齐或贴靠另一个实体（通常是顶层模型）——例如：把箱子贴靠在桌子边上。 ([Gazebo Simulation][1])

* 在插件菜单中选择 “Align Tool”。 ([Gazebo Simulation][1])
* 使用方法：选中两个模型（Ctrl + 点击）——第一个被认为是 “基准” 模型，第二个是要被对齐的模型。
* 在 “Relative to:” 下拉菜单里你可以选择 “First” 或 “Last” 来决定哪个是参照。 ([Gazebo Simulation][1])
* 当选定后，将鼠标移到 Align Tool 的某个轴按钮（x / y / z）上，会显示预览：第二个模型将怎样沿该轴贴靠第一个模型的包围盒边界。然后点击按钮确认对齐。 ([Gazebo Simulation][1])
* “Reverse”按钮：切换到反向贴靠——即把模型贴靠到参照模型的外部边界。这个在你想“放在旁边”而不是“重叠”时很有用。 ([Gazebo Simulation][1])
* 可以选择多个模型，一次对齐多个。 ([Gazebo Simulation][1])

**实战场景**：你有多个箱子模型，想让它们排成一列靠墙放；或者你有地板模型、家具模型，需要精确贴靠。Align Tool 就非常适用。

---

## 四、代码／命令示例

虽然教程主要是 GUI 操作，并没有大量 programmatic 的代码，但我这里补充一些可能你在实际操作或扩展时会用到的命令／脚本。例如：使用命令行、SDF 文件或服务调用来控制模型。

### 示例 1：启动示例世界

```bash
ign gazebo shapes.sdf
```

如教程所示。 ([Gazebo Simulation][1])

### 示例 2：SDF 文件片段（如果你要定义一个模型以便操作）

比如定义一个简单盒子模型 `yellow_block`：

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="yellow_block">
    <pose>0 0 0.05 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.2 1</ambient>
          <diffuse>0.7 0.7 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

（这个片段在某帮助文档中作为示例）([Intro to Robotics Reference][2])

然后你可以通过命令将其插入仿真：

```bash
ign service -s /world/default/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/path/to/yellow_block.sdf", name: "yellow_block"'
```

同理，删除模型可用：

```bash
ign service -s /world/default/remove \
  --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  --req 'name: "yellow_block"'
```

这些是较为“编程式”的操作，与 GUI 操作背后的系统接口相关。([Intro to Robotics Reference][2])

### 示例 3：使用命令行控制模型运动（扩展）

如果你想在脚本中控制模型位置／姿态，可以借助 `ign topic` 或 `ign service`。例如：

```bash
# 发布一个 Twist 控制某模型
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"
```

（在 ROS2 + Gazebo Bridge 场景）([Gistre Blog][3])

虽然不在原教程 “Manipulating Models” 篇里，但若你在 GUI 操作之外想要编程控制，这些命令是方向。

---

## 五、小结与建议

* 本教程主要偏 GUI，适合快速上手模型移动／旋转／对齐操作。
* 若你日后想做更复杂的模型生成、脚本控制或与 ROS2 集成，则需要进一步学习 SDF、Ignition Transport、ROS-Gazebo Bridge 等。
* 建议实操：打开一个简单世界（shapes.sdf），然后试着：

  1. 选定一个模型，进入平移模式，拖拽一下；
  2. 进入旋转模式，旋转一个模型；
  3. 尝试按住 `Shift`、`Ctrl` 等组合，看效果；
  4. 用 Align Tool 选两个模型，实现贴靠；
  5. 用 Component Inspector 看下模型移动后的 pose 值。
* 当操作熟练后，可以将这些 GUI 操作与脚本／服务接口结合，从而实现自动化布局、批量对齐等。

---

如果你愿意，我可以帮你把这篇教程 **完整地翻译成中文**，并 **为你摘出每个步骤对应的脚本／代码片段**（即便原教程不多，也帮你扩展），方便你学习与实践。你看要不要？

[1]: https://gazebosim.org/docs/fortress/manipulating_models/ "Manipulating Models — Gazebo fortress documentation"
[2]: https://reference-guide-cb79e5.pages.oit.duke.edu/gazebo-fortress-help/?utm_source=chatgpt.com "Gazebo Fortress Help"
[3]: https://blog.gistre.epita.fr/posts/marie.floissac-duforez-2023-09-27-introduction-to-ros2-humble-and-gazebo-fortress/?utm_source=chatgpt.com "Introduction to ROS2 Humble and Gazebo Fortress"
