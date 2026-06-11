下面我们一起系统地学习 Ignition Gazebo Fortress（通常简称 “Gazebo Fortress”）的 **GUI（图形用户界面）** 部分，内容基于官方文档并作详细整理。若你需要对应版本（例如 岩石 Fortress 6.x）的细节也可以，我这个整理以通用界面为主。

![Image](https://gazebosim.org/docs/latest/_images/entity_select.png)

![Image](https://jeremypedersen.com/images/roboblog/robo_16/left_2.png)

![Image](https://jeremypedersen.com/images/roboblog/robo_16/right_panel.png)

---

## 1. 概述

GUI 是你与仿真世界交互的重要入口，通过图形界面你可以：

* 插入模型／形状
* 选择、移动、旋转实体
* 控制仿真运行（播放／暂停／步进）
* 利用插件查看实体树、组件属性、日志、摄像机视角等

从官方文档：界面大致由工具栏、右侧插件面板、场景视图（3D）以及底部运行控制组成。 ([Gazebo][1])

---

## 2. 启动与版本提示

* 启动命令举例：如果你用 Fortress 版本，终端命令可能是：

  ```
  ign gazebo shapes.sdf
  ```

  或者为了指定版本：

  ```
  ign gazebo --force-version 6.x.x shapes.sdf
  ```

  ([Gazebo][1])
* 若你同时安装多个版本（例如 Jetty、Harmonic、Fortress 等），要确认你启动的是 Fortress 版本。 ([Gazebo][2])

---

## 3. GUI 的主要区域与功能

### 3.1 顶部工具栏

* 左上角有「文件／菜单」按钮，用于保存世界、加载／保存界面配置、风格设置等等。 ([Gazebo][1])
* 右上角有「插件」按钮（通常是三个竖点或菜单标志），点击可看到所有可用插件，并能在界面右侧打开／关闭。 ([Gazebo][1])
* 紧接下方还有一排插入形状或模型的按钮：例如插入 “Box、Sphere、Cone、Cylinder、Capsule” 等。 ([Gazebo][1])

### 3.2 插件面板（右侧）

* 启动默认会显示一些插件，例如 “Entity Tree”（实体树）和 “Component Inspector”（组件检测器）等。 ([Gazebo][1])
* **Entity Tree**：显示仿真世界中所有实体（地面、模型、光源等）。你可以展开某个实体查看其子组成（链接、碰撞、视觉、关节）。 ([Gazebo][1])
* **Component Inspector**：当你选中某个实体后，在此面板查看其属性（位置、方向、材质、传感器状态等）。 ([Jeremy Pedersen][3])
* 插件窗口通常可以右键点击调出菜单（例如关闭、设置）或从“插件”按钮重新打开。 ([Gazebo][1])

### 3.3 场景视图（3D世界）

* 在这个区域你可以查看、选取、操作仿真中的实体。功能包括：

  * **左键点击**：选择一个实体。 ([Jeremy Pedersen][3])
  * **右键点击**：弹出上下文菜单，内容如：

    * Move to：将视角聚焦至该实体。 ([Gazebo][1])
    * Follow：使相机跟随该实体。
    * Track：持续追踪该实体。
    * Remove：从仿真中移除该实体。
    * Copy/Paste：复制／粘贴实体。
    * View：子菜单可用于显示碰撞体、质量中心等。 ([Jeremy Pedersen][3])
  * **鼠标拖动／滚轮**：

    * 拖动：平移视角。
    * 滚轮：缩放。
    * 右键拖动：通常也是缩放或旋转视角。 ([Jeremy Pedersen][3])

### 3.4 底部控制条（运行控制）

* 包含播放／暂停按钮。 你可以启动仿真、暂停、或者步进（逐帧／定步长度）运行。 ([Jeremy Pedersen][3])
* 显示实时因子（Real Time Factor, RTF）：用于观察仿真是否与真实时间同步。 ([Jeremy Pedersen][3])

---

## 4. 常见操作举例

### 4.1 插入形状模型

* 点击顶部工具栏中的 “Box／Sphere…” 按钮 → 在场景地面点击放置模型。
* 放置后，该模型会自动附加到世界中，并可在右侧的实体树中看到其名字（如 `box_0`、`sphere_0`）。 ([Gazebo][1])

### 4.2 选择与移动实体

* 选择工具：点击实体或在实体树中选择。
* 移动实体：选中实体后，会出现坐标轴／拖拽手柄。通过拖拽改变位置。 ([reference-guide-cb79e5.pages.oit.duke.edu][4])
* 旋转实体：切换到旋转工具／使用圆弧拖拽方式旋转实体。 ([reference-guide-cb79e5.pages.oit.duke.edu][4])

### 4.3 插件操作

* 如果你看不到“Entity Tree”，可以点击右上“插件”菜单 → 找到 “Entity Tree” 并选择打开。
* 在实体树中右键某个实体可选择“Reset Model Pose”（重置模型姿态）或“Remove Model”。 ([reference-guide-cb79e5.pages.oit.duke.edu][4])

### 4.4 保存与加载界面配置／世界

* 文件菜单中可 “保存世界” `.sdf` 文件、保存“界面配置”（即插件的布局与样式）。 ([Gazebo][1])

---

## 5. 技巧提示 &常见问题

* **版本冲突**：如果安装了多个版本，应使用 `--force-version` 来选定。 ([Gazebo][1])
* **界面未渲染**：有用户反映在 Docker 或远程环境中 GUI 卡住：“GUI requesting list of world names.”。解决方式之一：设置环境变量 `IGN_IP=127.0.0.1`。 ([Robotics Stack Exchange][5])
* **插件布局自定义**：你可以保存当前插件布局作为界面配置，以后一键加载。
* **模型插入慢**：从 Ignition Fuel 下载模型时可能较慢，建议选择小型模型以快速体验。 ([Gazebo][6])

---

## 6. 与 SDF 仿真交互

* GUI 并不仅是“看界面”，它与 SDF world 文件紧密结合。通过 GUI 插入模型／操作实体，往往生成或修改世界文件中的模型元素。
* 你操作的实体都可以在实体树中看到，并能查看其 pose、链接、关节等属性。借助组件检测器（Component Inspector）可以查看模型在仿真中的实时参数。
* 在 ROS2 + Gazebo 集成场景中，GUI 是观察仿真状态、调试模型／传感器／视觉的关键工具。

---

## 7. 小结

* GUI 是使用 Gazebo Fortress 进行仿真、观察、操作世界的“前端”入口。
* 掌握：工具栏、插件面板、场景视图、运行控制 四大部分。
* 学会插入／选取／移动实体，并熟悉实体树与组件检测器的使用。
* 多用保存界面布局、调整插件、改善交互体验。
* 若遇问题（UI不响应／卡住），可检查版本、环境变量、网络模型下载等。

---

如果你愿意，我可以 **为你摘录官方文档 GUI 部分的 “关键代码／截图／快捷键”** 并以 PDF 或 Markdown 格式整理，方便你离线查阅。你看要不要？

[1]: https://gazebosim.org/docs/latest/gui/?utm_source=chatgpt.com "Understanding the GUI — Gazebo jetty documentation"
[2]: https://gazebosim.org/docs/latest/getstarted/?utm_source=chatgpt.com "Getting Started with Gazebo? — Gazebo jetty documentation"
[3]: https://jeremypedersen.com/posts/2024-08-29-pt16-gazebo-gui/?utm_source=chatgpt.com "Robotics crash course (Part 16 - The Gazebo GUI)"
[4]: https://reference-guide-cb79e5.pages.oit.duke.edu/gazebo-fortress-help/?utm_source=chatgpt.com "Gazebo Fortress Help"
[5]: https://robotics.stackexchange.com/questions/117672/gazebo-fortress-stuck-requesting-list-of-world-names?utm_source=chatgpt.com "Gazebo Fortress stuck \"requesting list of world names\""
[6]: https://gazebosim.org/docs/fortress/fuel_insert/?utm_source=chatgpt.com "Model Insertion from Fuel — Gazebo fortress documentation"
