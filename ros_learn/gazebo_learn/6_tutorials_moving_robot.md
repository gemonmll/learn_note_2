好的，我帮你详细整理一下这篇 Gazebo Fortress 教程 “Moving the robot” 的内容（中文说明 + 关键源码），并指出每段的重点。你可以在构建完机器人模型后，按此教程让机器人 **动起来**。
原文来自官网： “Moving the robot — Gazebo Fortress” ([gazebosim.org][1])

---

## 一、简介

在这篇教程中，作者告诉我们如何让之前构建好的机器人（见 “Building your own robot” 教程）移动。 ([gazebosim.org][1])
我们将用一个插件（plugin）来控制机器人：特别是为差速驱动（differential drive）机器人设计的 DiffDrive 插件。 ([gazebosim.org][1])

---

## 二、什么是插件（What is a plugin）

作者先定义插件：插件是一段编译成共享库（shared library）的代码，可以插入仿真中，用来控制世界、模型、系统等。 ([gazebosim.org][1])
也就是说，在 SDF 文件／世界文件中，我们不仅定义模型的结构（链接、关节、几何、物理属性），还可以通过 `<plugin>` 标签“挂”上功能，让机器人“智能”或“可控”。

---

## 三、DiffDrive 插件（Diff_drive plugin）

### 3.1 插件配置

为差速驱动机器人添加如下代码，在你的模型（例如文章中模型为 `vehicle_blue`）标签内：

````xml
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
``` :contentReference[oaicite:6]{index=6}  
### 3.2 各参数说明  
- `filename`：要载入的插件库文件。  
- `name`：插件的系统命名。  
- `<left_joint>` 和 `<right_joint>`：定义分别连接左右轮子的关节名称（在之前模型中定义为 `left_wheel_joint` 和 `right_wheel_joint`）。  
- `<wheel_separation>`：左右轮子之间的间距。文章中的模型左轮 Y=0.6m、右轮 Y=‐0.6m → 间距为 1.2 m。 :contentReference[oaicite:7]{index=7}  
- `<wheel_radius>`：轮子的半径，在模型中轮子 radius 被定义为 0.4m。  
- `<odom_publish_frequency>`：机器人里程计（odometry）发布频率，这里设置为 1 (Hz) 或 1次／秒。  
- `<topic>`：机器人订阅命令的主题名（topic），此例为 `/cmd_vel`。  

### 3.3 插件作用  
此插件使得机器人模型能够接受速度／角速度命令（通常为线速度 + 角速度），并将其转化为左右轮子的运动，从而在仿真中移动。

---

## 四、主题与消息（Topics and Messages）  
在机器人仿真／控制系统中，**主题(topic)** 是用于消息发布/订阅的通信通道。教程说明：机器人订阅 `/cmd_vel` 主题上发送的消息。 :contentReference[oaicite:8]{index=8}  
消息类型（message type）为 `ignition.msgs.Twist`，包含两个主要部分：  
- `linear`（线速度）  
- `angular`（角速度）  
例如，在一个终端里启动仿真：  
````

ign gazebo building_robot.sdf

```
然后在另一个终端发送命令：  
```

ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"

````:contentReference[oaicite:9]{index=9}
这里 `linear: {x: 0.5}` 表示向前0.5 m/s，`angular: {z: 0.05}` 表示绕 z 轴以 0.05 rad/s 转动（即轻微旋转）。发出命令后，机器人应该在仿真里移动起来。  
提示：别忘了在 GUI 中按 “播放” 按钮让仿真运行。 :contentReference[oaicite:10]{index=10}  

---

## 五、使用键盘控制（Moving the robot using the keyboard）  
除了在终端发送命令之外，还可以用键盘控制机器人。教程介绍了两个插件： KeyPublisher 和 TriggeredPublisher。  
### 5.1 KeyPublisher 插件  
- 该插件读取键盘按键，并在默认主题 `/keyboard/keypress` 上发布消息。 :contentReference[oaicite:13]{index=13}  
- 操作流程：  
  1. 启动仿真：`ign gazebo building_robot.sdf`  
  2. 在 Ignition 界面顶部右上角插件菜单中选择 Key Publisher。  
  3. 在另一个终端运行： `ign topic -e -t /keyboard/keypress` 来查看按键消息。按下不同键，终端会显示数据。 :contentReference[oaicite:14]{index=14}  
### 5.2 TriggeredPublisher 插件  
- 用于把键盘输入（如某个按键编号）映射成 `Twist` 消息并发送到 `/cmd_vel`。  
- 在 `<world>` 标签里添加如下示例代码（Forward 移动）：
```xml
<!-- Moving Forward-->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
``` :contentReference[oaicite:15]{index=15}  
- 这里 `16777235` 是 “↑ Up 箭头键” 的键值。然后按下此键，就会发送 `Twist` 消息使机器人向前。  
- 类似地，为其他箭头键定义对应的插件：  
  - 左 ← : `data`=16777234 → `linear: {x: 0.0}, angular: {z: 0.5}`（原地左转） :contentReference[oaicite:16]{index=16}  
  - 右 → : `data`=16777236 → `linear: {x: 0.0}, angular: {z: ‑0.5}`（原地右转） :contentReference[oaicite:17]{index=17}  
  - 下 ↓ : `data`=16777237 → `linear: {x: ‑0.5}, angular: {z: 0.0}`（后退） :contentReference[oaicite:18]{index=18}  
- 启动后，按箭头键即可控制机器人在仿真中移动。注意需要先按“播放”。  

---

## 六、总结 & 完整示例代码整合  
### 6.1 总结  
- 将差速驱动插件挂到机器人模型中，使其接收 `/cmd_vel` 速度命令。  
- 使用 `ign topic` 工具直接发送 `Twist` 消息使机器人移动。  
- 或者使用键盘（通过 KeyPublisher + TriggeredPublisher 插件）使操作更方便、直观。  
- 此教程为机器人动作控制提供了非常基础但完整的流程。  
### 6.2 示例整合（插入到之前模型文件）  
假设你的 `building_robot.sdf` 中已经定义了模型 `vehicle_blue` 和其左右轮子、关节等。现在在对应位置添加以下内容：  
在 `<model name="vehicle_blue" canonical_link="chassis"> … </model>` 内，靠近模型末尾但在 `</model>` 前，插入：
```xml
<plugin
    filename="libignition-gazebo-diff-drive-system.so"
    name="ignition::gazebo::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
````

然后，在 `<world>` 标签内（与模型同级）插入键盘控制插件，例如：

```xml
<!-- Key control: forward -->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777235</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.5}, angular: {z: 0.0}
    </output>
</plugin>
<!-- Key control: backward -->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777237</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: -0.5}, angular: {z: 0.0}
    </output>
</plugin>
<!-- Key control: turn left -->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777234</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.0}, angular: {z: 0.5}
    </output>
</plugin>
<!-- Key control: turn right -->
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Int32" topic="/keyboard/keypress">
        <match field="data">16777236</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.0}, angular: {z: -0.5}
    </output>
</plugin>
```

保存后，运行仿真：

```
ign gazebo building_robot.sdf
```

然后启动 KeyPublisher 插件（从 GUI 菜单选择），按箭头键来控制机器人移动。

---

如果你愿意，我可以 **翻译完整该教程**（包括所有源码、注释与图片），或者 **演示一个扩展版本**：如何用 ROS 2 接收 `/cmd_vel` 控制机器人，或者如何加入 PID 控制轮子速度。你想要哪个？

[1]: https://gazebosim.org/docs/fortress/moving_robot/ "Moving the robot — Gazebo fortress documentation"
