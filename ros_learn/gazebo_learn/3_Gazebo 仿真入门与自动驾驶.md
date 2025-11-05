

# **从“Hello, World”到自动驾驶：一份面向ROS 2导航的Gazebo仿真综合技术指南**

## **第一部分：仿真“沙盒”—— Gazebo 核心基础**

本部分旨在为入门用户建立一个坚实的基础，首先必须解决一个关乎未来所有工作的关键抉择：选择正确的 Gazebo 版本。这个选择将深刻影响后续的 ROS 集成、教程可用性和功能实现。

### **1.1 核心概念：Gazebo Classic vs. Gazebo (Ignition/Gz) —— 一个关键的十字路口**

Gazebo 仿真生态系统目前处于一个分裂状态，存在两个主要的并行版本：“Gazebo Classic” 和现代的 “Gazebo”（曾用名 Ignition，现简称为 Gz）1。

* **Gazebo Classic (例如 v11)**：这是大多数传统 ROS 1 教程所使用的旧版仿真器 2。它功能成熟，社区资源丰富，但其架构已显陈旧。  
* **Gazebo (Gz/Ignition) (例如 Fortress, Harmonic)**：这是完全重写的现代继任者 1。它被设计为一个模块化的库工具集，提供了更先进的物理引擎接口、更强大的渲染能力和更高性能的传感器模拟 3。

对于刚入门的用户，这个选择至关重要。Gazebo Classic 11 已被宣布将在 2025 年 1 月达到“生命周期终点”（End-of-Life, EOL）5。这意味着任何基于该平台启动的新项目从一开始就面临被淘汰的风险。因此，尽管现存的大部分教程可能基于 Classic，但从长远来看，**新用户应被强烈引导至现代的 Gazebo (Gz) 平台（例如最新的 Harmonic 版本）** 1。

本指南将优先采用这条面向未来的路径：**使用 ROS 2 (Humble/Jazzy) 与 现代 Gazebo (Gz) 相结合**。

这一版本分裂是理解现代 ROS 仿真的关键，它解释了为什么存在两套完全不同的 ROS 集成包（即 gazebo\_ros\_pkgs 和 ros\_gz\_bridge），我们将在第三部分详细探讨这一点。

### **1.2 虚拟世界的架构：.world 文件**

Gazebo 中的一切都发生在一个“世界”（World）中。这个世界通过一个 .world 文件来定义 6。

.world 文件不仅仅是用于放置物体的“场景”文件；它是**仿真物理的“契约”**。该文件使用一种称为\*\*仿真描述格式（Simulation Description Format, SDF）\*\*的 XML 语言编写 7。

SDF 文件定义了仿真的全部要素 7：

* **模型（Models）**：包括机器人、建筑物、桌椅等。  
* **光照（Lights）**：定义场景中的光源。  
* **物理属性（Physics）**：定义物理引擎（如 Bullet 或 DART）、重力、更新速率以及其他核心参数。  
* **环境元素**：可以包括交通信号灯、行人甚至天气条件 7。

这个文件由 Gazebo 的核心进程 gzserver（或在 Classic 中为 gzserver）读取并执行。gzserver 负责所有物理更新循环和传感器数据的生成 7。因此，GUI 界面中无法调整的高级参数（例如修改大气压力、设置复杂的物理关节约束）最终都需要通过直接编辑 .world 文件的 SDF/XML 代码来实现。精通 Gazebo 意味着精通 SDF 格式，而非仅仅是 GUI 的拖拽操作。

### **1.3 高级世界构建：从简单房间到复杂城市**

根据用户的目标（室内导航 vs. 自动驾驶），构建世界的方法截然不同。

#### **路径 1：内置建筑编辑器 (Building Editor)**

对于室内“导航”（例如在公寓中运行 Turtlebot）的目标，Gazebo 提供了一个内置的“建筑编辑器”8。

这是一个高效的工具，允许用户导入一张 2D 的户型图（floor plan），然后像描图一样在该平面图上“绘制”墙壁、门、窗和楼梯 8。这可以快速生成用于 2D SLAM 和导航算法测试的结构化室内环境。

#### **路径 2：高保真模型导入 (Blender/CAD)**

对于“自动驾驶”或需要高保真模型的场景（例如模拟一辆特定的汽车或一个真实的建筑），标准工作流是从专业的 3D 建模软件（如 Blender、SolidWorks 或 FreeCad）导入模型 11。

Gazebo 的 .world 文件（或模型的 .sdf 文件）可以加载这些外部的网格（mesh）文件（例如 .dae, .stl 格式），并将其用作 visual（视觉）和 collision（碰撞）几何体 13。

然而，这个过程隐藏着一个新手的\*\*“导入陷阱”\*\*。一个常见的错误认知是认为可以简单地“文件 \> 导入 \> model.blend”。实际的工作流程要复杂得多，并且充满了非直观的陷阱 15：

1. **单位不匹配**：大多数 CAD 和 3D 建模软件（如 Blender）的默认单位是毫米（mm）或厘米（cm）。而 Gazebo 严格要求使用**米（meters）** 15。如果不进行缩放，导入的模型将比预期大 1000 倍。  
2. **坐标系不匹配**：许多 3D 建模软件（如 Blender）使用“Y 轴向上”（Y-up）的坐标系。而 ROS 和 Gazebo 严格使用\*\*“Z 轴向上”\*\*（Z-up）的坐标系。如 15 所述，这通常需要模型在导出前绕 X 轴旋转 90 度。

如果不执行这两个关键的转换步骤，模型在 Gazebo 中几乎肯定是“错误”的——要么尺寸巨大，要么“躺”在地上。

#### **路径 3：程序化生成城市 (Procedural City Generation)**

对于大规模的“自动驾驶”测试，手动在 Blender 中构建整个城市是不现实的。为此，存在用于程序化生成城市环境的专业工具 16。

Open Robotics 已经开发了 citysim 项目 5 和 gazebo\_terrain\_generator 18 等工具，它们可以根据道路网络描述文件（Road Network Description File）自动生成包含道路、建筑和地形的复杂城市环境。这是自动驾驶仿真的“终极”环境，我们将在第六部分重新审视。

## **第二部分：构建“数字孪生”—— 机器人建模**

定义了“世界”之后，下一步是定义在其中运行的“机器人”。这需要构建一个精确的数学和物理模型，即“数字孪生”。

### **2.1 机器人的蓝图：URDF vs. SDF 的关键二元性**

在 ROS/Gazebo 生态中，机器人模型主要由两种格式定义：

1. **URDF (Unified Robot Description Format)**：这是 **ROS 的标准**格式 19。它基于 XML，主要用于描述机器人的运动学结构（link 连杆和 joint 关节）和视觉外观。ROS 的核心工具（如 robot\_state\_publisher 和 Rviz）都依赖 URDF 来解析和显示机器人状态 19。  
2. **SDF (Simulation Description Format)**：这是 **Gazebo 的标准**格式 7。SDF 是 URDF 的超集；它不仅能定义 URDF 的所有内容，还能定义 Gazebo 所需的额外信息，包括物理属性（摩擦力、阻尼）、传感器、插件、灯光等。

对于入门者来说，最大的困惑在于：ROS 使用 URDF，Gazebo 使用 SDF，那么机器人模型应该用哪种格式？

答案是：**必须使用“混合方法”（Hybrid Approach）**。

* **纯 URDF 的缺陷**：如果只提供一个“纯粹的”URDF（仅包含 link, joint, visual），Gazebo 无法对其进行物理模拟，因为它缺少关键的物理参数。为了让 Gazebo 正确加载 URDF，每个 link 都**必须**包含 \<inertial\>（惯性）和 \<collision\>（碰撞）标签 20。  
* **纯 SDF 的缺陷**：如果只提供一个“纯粹的”SDF，Gazebo 会运行得很好，但 ROS 的工具（如 Rviz 和 robot\_state\_publisher）将无法理解它，导致 ROS 无法获知机器人的状态。

因此，标准的“ROS-Gazebo”工作流程是：

1. 机器人的**主体结构**（运动学和几何）在一个 **URDF** 文件中定义（通常使用 xacro 宏语言以简化编写）。  
2. 所有 **Gazebo 特定的属性**（物理、传感器、插件）都作为 SDF 代码片段，嵌入到 URDF 文件中的特殊 **\<gazebo\>** 标签内。

例如，21 和 22 中的示例清晰地展示了这一点：URDF 文件定义了一个名为 camera\_link 的连杆（确定了传感器的*位置*）；而一个 \<gazebo reference="camera\_link"\>...\</gazebo\> 标签则包含了 SDF 代码（如 \<sensor type="camera"\> 和 \<plugin...\>），用于定义传感器*是*什么以及它如何与 ROS *通信*（通过插件）。

### **2.2 模拟感知：添加传感器（技术细节）**

根据 2.1 的混合方法，为机器人添加一个传感器（如 Lidar）需要两个步骤：在 SDF（即 URDF 中的 \<gazebo\> 标签）中定义 Gazebo 内部的 \<sensor\>，并定义其与 ROS 通信的 \<plugin\> 22。

#### **示例 1：3D GPU LIDAR (自动驾驶的关键)**

对于“自动驾驶”目标，3D Lidar 是必不可少的。

* **CPU vs. GPU Lidar**：21 提到了两种 Lidar 插件：  
  * libgazebo\_ros\_laser.so：用于 type="ray"，使用 CPU 进行射线投射。  
  * libgazebo\_ros\_gpu\_laser.so：用于 type="gpu\_ray"，使用 GPU 加速 4。  
* 对于简单的 2D Lidar，CPU 尚可应付。但对于自动驾驶所需的 32 线或 64 线 3D Lidar，CPU 射线投射将产生巨大的性能瓶颈，导致仿真卡顿。因此，**必须**从一开始就使用 GPU 加速的 Lidar 插件。

下面是一个用于 3D GPU Lidar 的典型 URDF/SDF 插件配置（基于 21）：

XML

\<gazebo reference\="hokuyo\_link"\>  
  \<sensor type\="gpu\_ray" name\="head\_hokuyo\_sensor"\>  
    \<visualize\>false\</visualize\>  
    \<update\_rate\>40\</update\_rate\>  
    \<ray\>  
      \<scan\>  
        \<horizontal\>  
          \<samples\>720\</samples\>  
          \<resolution\>1\</resolution\>  
          \<min\_angle\>\-1.570796\</min\_angle\>  
          \<max\_angle\>1.570796\</max\_angle\>  
        \</horizontal\>  
        \</scan\>  
      \<range\>  
        \<min\>0.10\</min\>  
        \<max\>30.0\</max\>  
        \<resolution\>0.01\</resolution\>  
      \</range\>  
      \<noise\>  
        \<type\>gaussian\</type\>  
        \<mean\>0.0\</mean\>  
        \<stddev\>0.01\</stddev\>  
      \</noise\>  
    \</ray\>  
    \<plugin name\="gazebo\_ros\_head\_hokuyo\_controller" filename\="libgazebo\_ros\_gpu\_laser.so"\>  
      \<topicName\>/rrbot/laser/scan\</topicName\>  
      \<frameName\>hokuyo\_link\</frameName\>  
    \</plugin\>  
  \</sensor\>  
\</gazebo\>

#### **示例 2：深度相机 (RGB-D)**

深度相机（如 Kinect 或 RealSense）对于导航和感知非常有用 23。22 提供了一个用于模拟 RGB-D 相机（libgazebo\_ros\_openni\_kinect.so）的配置。

其关键优势在于**效率**：这一个插件会同时发布**多个** ROS 话题，包括 22：

* sensor\_msgs::Image（彩色图像）  
* sensor\_msgs::Image（深度图像）  
* sensor\_msgs::CameraInfo（相机标定信息）  
* sensor\_msgs::PointCloud2（点云）

这使其成为测试感知算法的理想选择。

#### **示例 3：IMU (惯性测量单元)**

IMU 对于任何需要姿态估计的机器人（包括 SLAM 和导航）都至关重要 4。21 提供了 libgazebo\_ros\_imu.so 插件的完整 XML 示例。该插件允许配置高斯噪声（gaussianNoise）和更新率，这对于测试机器人定位算法（如 EKF 或 AMCL）的鲁棒性至关重要。

### **2.3 模拟驱动：驱动插件（机器人 vs. 汽车）**

用户的查询具有“导航”（室内机器人）和“自动驾驶”（汽车）的双重目标。这两种场景使用完全不同的驱动运动学，因此需要不同的驱动插件。

#### **路径 A：差分驱动 (Differential Drive) \- 用于“导航”**

这是“导航”机器人的标准配置（如 Turtlebot，两个平行的驱动轮）。

* **传统方法**：使用 libgazebo\_ros\_diff\_drive.so 插件 20。  
* **现代 (ROS 2\) 方法**：使用 ros2\_control 框架 24。这是 ROS 2 的标准做法，虽然设置更复杂（需要在 URDF 中添加 ros2\_control 标签，创建控制器 YAML 配置文件，并在 launch 文件中启动 controller\_manager），但它提供了更优秀、更模块化的控制 24。

#### **路径 B：阿克曼转向 (Ackermann Steering) \- 用于“自动驾驶”**

这是“自动驾驶”汽车的标准配置（后轮驱动，前轮转向）。

* **标准插件**：使用 libgazebo\_ros\_ackermann\_drive.so 插件 26。  
* **工作原理**：此插件订阅一个标准的 ROS 话题 geometry\_msgs/Twist。它会智能地将该消息中的 linear.x（线速度）转换为后轮的转速，并将 angular.z（角速度）转换为前轮的精确转向角度，自动处理阿克曼几何 28。  
* 新手的“隐藏转向轮陷阱”：  
  这是一个非常容易出错且违反直觉的陷阱。当用户配置 libgazebo\_ros\_ackermann\_drive.so 插件时，文档和示例 27 会提到一个 steering\_wheel\_joint（方向盘关节）。  
  1. 用户会认为这只是一个视觉上的方向盘，对于物理模拟并非必要，因此尝试删除它。  
  2. 然而，28 的分析指出：“你不应该删除它，因为……整个连杆/关节结构都会受到影响。”  
  3. 27 的用户经验证实了这一点：“我开始时移除了它，但我记得机器人无法正确移动。你可以为它制作一个虚拟连杆。”

结论：该插件在内部逻辑上**依赖**这个 steering\_wheel\_joint 的存在，即使它没有被物理驱动。在模型中必须包含这个（即便是虚拟的）关节，否则插件将无法正常工作。

## **第三部分：连接仿真与 ROS 2 —— 桥接集成**

这是技术上最关键、也是最容易出错的部分。Gazebo (Gz) 和 ROS 2 是两个完全独立的程序。它们如何通信？答案取决于您选择的 Gazebo 版本。

### **3.1 数字神经系统：gazebo\_ros\_pkgs vs. ros\_gz\_bridge**

如 1.1 节所述，集成方法因 Gazebo 版本而异：

* 路径 1 (旧方法): gazebo\_ros\_pkgs (用于 Gazebo Classic)  
  这是一个“元包”（meta package）30，适用于 ROS 1 2 和早期 ROS 2（如 Foxy）与 Gazebo Classic 11 的组合 31。  
  在这种架构中，Gazebo 和 ROS 是紧耦合的。gazebo\_ros\_pkgs 提供了一系列插件（如 libgazebo\_ros\_laser.so），这些插件同时负责模拟传感器并直接将数据发布到 ROS 话题上 32。仿真器本身是“知晓 ROS”的 34。  
* 路径 2 (新方法): ros\_gz\_bridge (用于 Gazebo Gz/Ignition)  
  这是适用于现代 Gazebo (Gz) 和 ROS 2 (Humble及以后) 的架构，也是本指南推荐的路径。  
  在这种架构中，Gazebo 和 ROS 是解耦的。Gazebo Gz 是一个完全独立的仿真器，它对 ROS 一无所知 35。它在自己内部的消息系统（称为 “Gazebo Transport”，以前叫 “Ignition Transport”）上发布数据（例如，在一个名为 /scan 的 Gz 话题上）35。  
  ros\_gz\_bridge 是一个**完全独立的、必须单独运行的 ROS 2 节点** 37。它充当一个“翻译器”：它订阅一个 Gazebo Transport 话题（例如 gz topic: /scan），将其消息格式（例如 gz.msgs.LaserScan）翻译为 ROS 2 消息格式（例如 sensor\_msgs/msg/LaserScan），然后将其发布到一个 ROS 2 话题上（例如 ros2 topic: /scan） 35。

这种新架构更模块化、更健壮，但也增加了一个必须由用户手动配置的额外步骤。

### **3.2 实践深潜：ROS 2 Humble 与 Gazebo Harmonic 的集成**

* **挑战**：ROS 2 Humble（Ubuntu 22.04）官方（通过 packages.ros.org）支持的是 Gazebo *Fortress* 39。然而，Gazebo *Harmonic* 是更新、更推荐的版本 1。  
* **“专家”解决方案**：可以（也推荐）在 Humble 上使用 Harmonic。这需要添加 Gazebo 自己的 apt 仓库 40，然后安装一个特定的桥接包 41：  
  Bash  
  \# 1\. 添加 Gazebo 仓库 (如果尚未添加)  
  sudo sh \-c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable \`lsb\_release \-cs\` main" \> /etc/apt/sources.list.d/gazebo-stable.list'  
  wget http://packages.osrfoundation.org/gazebo.key \-O \- | sudo apt-key add \-

  \# 2\. 安装 Gz Harmonic 和 对应的 ROS 2 Humble 桥接  
  sudo apt-get update  
  sudo apt-get install gz-harmonic  
  sudo apt-get install ros-humble-ros-gzharmonic

* 如何使用桥接（关键的配置步骤）：  
  必须在 launch 文件中运行 ros\_gz\_bridge 的 parameter\_bridge 节点。其参数（即“翻译规则”）具有非常严格且关键的语法 35：  
  \`/GAZEBO\_TOPIC\_NAME@ROS\_MSG\_TYPE\_NAME：  
  * @：双向桥接 (ROS $\\leftrightarrow$ Gz)  
  * \`\`：从 ROS **到** Gz (ROS $\\rightarrow$ Gz)，用于控制命令。

  **示例：**

  1. Lidar (Gz $\\rightarrow$ ROS): 将 Gz 的 /scan 话题桥接到 ROS 2 的 /scan 话题。  
     \`ros2 run ros\_gz\_bridge parameter\_bridge /scan@sensor\_msgs/msg/LaserScan。  
  2. 命令 (ROS $\\rightarrow$ Gz): 将 ROS 2 的 /cmd\_vel 话题桥接到 Gz 的 /model/vehicle\_blue/cmd\_vel 话题。  
     ros2 run ros\_gz\_bridge parameter\_bridge /model/vehicle\_blue/cmd\_vel@geometry\_msgs/msg/Twist\]ignition.msgs.Twist  
     (注意 \] 在 Gz 消息类型之后) 36。

这意味着，用户的 launch 文件必须包含一个**详尽的列表**，列出每一个需要桥接的话题，并正确指定其 ROS 消息类型、Gz 消息类型和**桥接方向**。这是新架构下最常见的错误来源。

### **3.3 验证连接：三步调试法**

当用户在 Rviz 中看不到数据时（例如，Lidar 扫描），在新架构下，问题排查必须遵循一个清晰的流程。

1. **步骤 1：Gazebo 是否在工作？**  
   * *不要*使用 ROS 2 工具。首先使用 Gazebo Transport 工具来检查仿真器是否正在生成数据。  
   * ign topic \-l（列出所有 Gz 话题）  
   * ign topic \-e \-t /scan（实时回显 /scan 话题的数据） 37。  
   * *如果这里没有数据*：问题出在**步骤二**中的机器人 URDF/SDF 模型（传感器或插件配置错误）。  
2. **步骤 2：桥接是否在运行？**  
   * 使用 ROS 2 工具检查桥接节点是否已启动。  
   * ros2 node list  
   * *如果列表中没有 parameter\_bridge 节点*：问题出在**启动文件（launch file）**，它没有正确启动桥接节点。  
3. **步骤 3：ROS 2 是否收到数据？**  
   * 只有当步骤 1 和 2 都成功后，才检查 ROS 2。  
   * ros2 topic list（检查 ROS 2 话题是否存在） 43  
   * ros2 topic echo /scan  
   * *如果这里没有数据*：问题出在**步骤 3.2** 中的**桥接参数**（话题名称、消息类型或方向符号 @ 错误）。

使用 rqt\_graph 43 工具可以直观地看到 parameter\_bridge 节点及其与 ROS 生态其他部分的连接，是调试的利器。

下表总结了推荐的 ROS/Gazebo 配套安装方案，帮助用户避免版本混乱：

**推荐的 ROS / Gazebo 配套集成方案**

| ROS 发行版 | 推荐的 Gazebo 版本 | Gazebo 类型 | Apt 安装的集成包 | 备注 (为何如此) |
| :---- | :---- | :---- | :---- | :---- |
| ROS 1 Noetic | Gazebo 11 | Classic | gazebo\_ros\_pkgs | 传统的 ROS 1 路径，紧耦合 2 |
| ROS 2 Foxy | Gazebo 11 | Classic | ros-foxy-gazebo-ros-pkgs | 早期的 ROS 2 路径，同样紧耦合 31 |
| ROS 2 Humble (官方) | Gazebo Fortress | Gz (Ignition) | ros-humble-ros-gz-bridge | Humble 官方仓库支持的 Gz 版本 39 |
| **ROS 2 Humble (推荐)** | **Gazebo Harmonic** | **Gz (Ignition)** | **ros-humble-ros-gzharmonic** | 需要 OSRF 仓库；功能更新 \[40, 41, 42\] |
| ROS 2 Jazzy | Gazebo Harmonic | Gz (Ignition) | ros-jazzy-ros-gz-bridge | Jazzy 官方仓库支持的 Gz 版本 \[44\] |

## **第四部分：“我在哪里？”—— SLAM 与定位**

本部分解决了用户“导航”目标的第一个核心问题：构建地图（SLAM）和在地图中定位（Localization）。

### **4.1 SLAM 算法对决：Gmapping vs. Hector SLAM**

在 ROS 1 时代，有两个主流的 2D Lidar SLAM 算法：

* **Gmapping (slam\_gmapping)**：基于粒子滤波器的 SLAM 算法 45。它**必须**依赖里程计（Odometry）数据（通常来自车轮编码器）作为运动先验（prior） 46。  
* **Hector SLAM (hector\_mapping)**：基于扫描匹配（Scan-Matching）的 SLAM 算法 48。它**不需要**里程计数据，因此常用于没有轮式里程计的设备（如手持建图设备或无人机）46。

在仿真中该如何选择？  
在现实世界中，这个选择取决于机器人的硬件配置。但在 Gazebo 仿真中，情况则不同：

1. 在 Gazebo 中，驱动插件（无论是差分驱动还是阿克曼）**总是会发布里程计数据**（作为 nav\_msgs/Odometry 话题或 tf 变换）47。  
2. 因此，使用 Hector SLAM 的主要理由（缺乏里程计）在仿真中是不存在的。  
3. Gmapping（及其后继者）采用的融合了里程计和激光扫描的粒子滤波方法，通常比纯扫描匹配的 Hector SLAM 更鲁棒，尤其是在特征稀疏的环境中（如长走廊）或机器人快速运动时 49。

结论：在仿真中，应始终选择一个能够利用仿真器“免费”提供的里程计数据的 SLAM 算法。

### **4.2 现代 2D 建图：在 ROS 2 中实现 slam\_toolbox**

在 ROS 2 中，slam\_toolbox 成为了 Gmapping 的标准继任者 50。

slam\_toolbox 不仅仅是 Gmapping 的简单移植，它提供了几个关键的增强功能 54：

* **生命周期建图 (Lifelong Mapping)**：它支持在已有的地图上继续进行建图和更新，而 Gmapping 只能从头开始 54。  
* **纯定位模式**：slam\_toolbox 可以加载一个已有的地图，并只运行其定位部分，使其成为 AMCL 的一个替代品 52。  
* **离线/在线配置**：支持多种工作模式 54。

### **4.3 最常见的错误点：use\_sim\_time**

这是 ROS 仿真中导致 SLAM 和导航失败的**最常见、最隐蔽**的错误。

* **问题所在**：Gazebo 运行在它自己的“仿真时间”（Sim Time）上。当它发布传感器数据（如 /scan）时，这些消息的时间戳是仿真时间（例如 $t=68.6$ 秒）55。  
* 默认情况下，所有 ROS 2 节点（如 slam\_toolbox）都运行在“系统时间”（Wall Clock，即真实世界的时间）上。  
* **“时间旅行”错误**：当 slam\_toolbox 节点（运行在 $t=1700747075.7$ 的系统时间）收到一个时间戳为 $t=68.6$ 的 /scan 消息时，它会认为这个消息是来自“遥远的过去”，因此无法将其与当前的 tf 数据（也使用系统时间）匹配。  
* 这会导致 tf 查找失败，日志中会充满类似 55 中的错误：“Lookup would require extrapolation into the future... Requested time 68.6... but the latest data is at time 1700747075.7...”。节点会因此丢弃所有传感器数据，导致建图失败。  
* **解决方案**：**必须**告诉**所有**与仿真交互的 ROS 2 节点：“请使用 Gazebo 发布的仿真时间”。这通过在启动**每一个**相关节点时传递 use\_sim\_time:=True 参数来实现 55。

### **4.4 持久化定位：配置 AMCL**

使用 slam\_toolbox 成功构建地图后，应使用 nav2\_map\_server 将地图保存为 .pgm 和 .yaml 文件 57。

在下次启动时，不再运行 SLAM，而是加载这个静态地图并运行**定位**。在 ROS 2 Nav2 堆栈中，标准的定位节点是 **AMCL (Adaptive Monte Carlo Localization)** 58。

AMCL 的工作是 59：

1. 订阅：静态地图、当前的 Lidar 扫描 (/scan)、里程计 (/odom)。  
2. 工作：使用粒子滤波器来估计机器人在地图上的最高概率姿态 ($x, y, \\theta$)。  
3. 发布：计算并发布 map $\\rightarrow$ odom 的 tf 变换。

这个 tf 变换至关重要：它纠正了里程计（odom $\\rightarrow$ base\_link）中不可避免的累积漂移。这使得机器人的 tf 树（$map \\rightarrow odom \\rightarrow base\\\_link$）保持全局一致，这是导航的前提。

## **第五部分：“我如何去那里？”—— ROS 2 导航堆栈 (Nav2)**

本部分深入探讨用户“导航”目标的实现，即 ROS 2 的自主移动核心：Nav2。

### **5.1 Nav2 架构：从 move\_base (ROS 1\) 的范式转变**

* **ROS 1 (旧)**：导航功能由一个名为 move\_base 的\*\*“单体状态机”\*\*（monolithic state machine）控制 58。它功能强大，但高度耦合且难以定制。  
* **ROS 2 (新)**：Nav2 彻底抛弃了 move\_base。如 58 所述，Nav2 “炸开”了 move\_base，将其替换为**一系列独立的、模块化的 ROS 2 动作（Action）服务器** 61：  
  * planner\_server：负责计算从 A 点到 B 点的**全局路径**。  
  * controller\_server：负责（局部地）跟随全局路径，同时实时**避开障碍物**。  
  * behavior\_server：负责在机器人卡住时执行**恢复行为**（如旋转）。  
  * bt\_navigator (行为树导航器)：这是 Nav2 的新“大脑”，负责协调所有其他服务器 60。

行为树 (Behavior Trees, BT) 是 Nav2 的核心哲学 60。  
bt\_navigator 不再使用硬编码的 C++ 状态机，而是加载一个 XML 文件 63，该文件定义了一个“行为树”。这个树描述了机器人的高级决策逻辑，例如：  
“（顺序执行：）计算一个全局路径。如果成功，（循环执行：）跟随该路径。如果失败，（执行恢复：）旋转 360 度，然后重新计算路径。”  
这种方法的巨大优势在于：用户可以通过**编辑 XML 文件**来彻底改变机器人的核心决策逻辑，而无需重新编译任何 C++ 代码 60。

### **5.2 配置核心：深潜 nav2\_params.yaml**

整个 Nav2 堆栈（包含十几个节点）通常由一个（非常大的）YAML 文件来配置，即 nav2\_params.yaml 60。

这对于新手来说可能令人生畏，但其结构背后有一个优雅的 ROS 2 机制，63 揭示了这一点：

1. navigation\_launch.py 62 启动文件会一次性启动所有 Nav2 节点（amcl, bt\_navigator, controller\_server, planner\_server 等）。  
2. 它将这**一个** nav2\_params.yaml 文件传递给**所有**这些节点。  
3. 该 YAML 文件内部是按**节点名称**分区的。每个节点只读取以自己名字命名的顶级参数块。

一个简化的 nav2\_params.yaml 结构如下所示：

YAML

\# AMCL 节点的参数  
amcl:  
  ros\_\_parameters:  
    use\_sim\_time: True  
    alpha1: 0.2  
    base\_frame\_id: "base\_footprint"  
   ...

\# 行为树导航器节点的参数  
bt\_navigator:  
  ros\_\_parameters:  
    use\_sim\_time: True  
    global\_frame: "map"  
    default\_nav\_to\_pose\_bt\_xml: "navigate\_to\_pose\_w\_replanning\_and\_recovery.xml"  
   ...

\# 控制器服务器节点的参数  
controller\_server:  
  ros\_\_parameters:  
    use\_sim\_time: True  
    controller\_frequency: 20.0  
    controller\_plugins: \["FollowPath"\] \# 声明要使用的插件  
    FollowPath: \# 为该插件ID配置参数  
      plugin: "dwb\_core::DWBLocalPlanner" \# 指定插件类型  
     ...

\# 规划器服务器节点的参数  
planner\_server:  
  ros\_\_parameters:  
    use\_sim\_time: True  
    planner\_plugins: \# 声明要使用的插件  
    GridBased: \# 为该插件ID配置参数  
      plugin: "nav2\_navfn\_planner::NavfnPlanner" \# 指定插件类型  
     ...

理解这种“按节点命名的参数命名空间”是配置 Nav2 的关键。

### **5.3 机器人的“反射”：配置局部规划器 (dwb\_controller)**

Nav2 的默认局部规划器（即 controller\_server 的默认插件）是 dwb\_controller 67，它是动态窗口法（Dynamic Window Approach, DWA）的 ROS 2 实现。

dwb\_controller 的调优不是通过调整几个简单的参数，而是通过一个\*\*“评判者”（Critics）\*\*框架来实现的 67：

1. **生成**：StandardTrajectoryGenerator 首先会生成数千条可能的短期轨迹 67。  
2. **评分**：dwb\_controller 会使用一系列“评判者”插件来为每条轨迹打分。  
3. **评判者**：67 列出了一些默认的评判者，例如：  
   * BaseObstacleCritic：惩罚靠近障碍物的轨迹。  
   * GoalAlignCritic：奖励朝向最终目标的轨迹。  
   * PathAlignCritic：奖励贴近全局路径的轨迹。  
   * PreferForwardCritic：惩罚需要倒车的轨迹。  
4. **调优**：用户通过在 nav2\_params.yaml 文件中为这些“评判者”设置不同的\*\*权重（weights）\*\*来“调优”机器人的行为。例如，高 PathAlignCritic 权重会使机器人“死板地”跟随全局路径；而高 GoalAlignCritic 权重则会使其更倾向于“抄近路”。

### **5.4 “代价地图的秘密”：调优的艺术**

Nav2 使用全局和局部代价地图（Costmaps）来表示环境中的障碍物 58。一个关键参数是 inflation\_radius（膨胀半径）。

官方调优指南 68 揭示了一个关于代价地图的**深刻且常见的误解**：

1. **新手的做法**：将膨胀半径设置得很小（例如 10 厘米）。这只在障碍物周围创建了一个很薄的“缓冲区”。  
2. **导致的问题**：全局规划器（如 NavFn 或 Theta\*）看到的是一个巨大的成本为 0 的开放空间和一堵成本为 254 的“硬墙”。它生成的路径会紧贴着缓冲区的边缘，这既不平滑也不安全。  
3. 专家的做法 68：设置一个**非常大**的膨胀半径，大到足以在整个开放空间（例如走廊）中产生一个\*\*“一致的势场”（consistent potential field）\*\*。  
4. **带来的好处**：这会在墙壁（高成本）和走廊中心（低成本）之间产生一个平滑的**成本梯度**。规划器现在会自然地寻找到一条位于**走廊正中间**（成本最低）的路径，这条路径显然更安全、更平滑，也更鲁棒。

仅这一个参数的正确理解和设置，就能极大地改善导航的性能和稳定性。

## **第六部分：终极疆域 —— 自动驾驶仿真**

本部分将探讨用户查询的第二目标——“自动驾驶功能”，这是一个比“导航”更具体、更复杂的领域，通常涉及汽车（阿克曼运动学）和更复杂的感知与决策。

### **6.1 案例研究 1：AutoCarROS2 (ROS 2\) \- 一个非 Nav2 堆栈**

AutoCarROS2 是一个用于模拟自动驾驶汽车的 ROS 2 Foxy 项目 69。

通过分析其包结构 69，我们发现了一个关键信息：

* 它包含 autocar\_description（URDF/SDF）、autocar\_gazebo（World 文件）、autocar\_map（建图）。  
* 它有一个核心包 autocar\_nav（导航堆栈）。  
* 在 autocar\_nav 内部，它实现了一个 **"Stanley controller"**（斯坦利控制器）69。

这揭示了一个重要的架构选择：AutoCarROS2 **没有**使用 Nav2 的 dwb\_controller。

Stanley 控制器（以及 Pure Pursuit，纯追踪）70 是一种**路径跟踪**算法，它被专门设计用于处理**阿克曼运动学（汽车）**。相比之下，DWA/DWB 更适用于差分驱动（机器人）的运动学。

这为用户提供了两条实现“自动驾驶”的路径：

1. **路径 1**：使用完整的 Nav2 堆栈，并将其 controller\_server 的插件替换为 Nav2 提供的 RegulatedPurePursuit（受控纯追踪）控制器 71。  
2. **路径 2**：像 AutoCarROS2 一样，构建一个**自定义堆栈**，完全*替换* Nav2 的 controller\_server，转而使用一个专门为阿克曼车辆定制的控制器（如 Stanley）。

对于严肃的自动驾驶模拟，路径 2 通常能提供更好的性能。

### **6.2 案例研究 2：Project Aslan (ROS 1\) \- 替代的算法**

Project Aslan 是一个开源的、基于 ROS 1 Melodic 的全栈自动驾驶软件 72。

分析这个项目，我们能发现更多自动驾驶领域的专业算法：

* **定位：NDT (Normal Distribution Transform)**：Aslan **没有**使用 AMCL 进行定位。它使用的是 NDT（正态分布变换）72。  
  * 73 解释了其区别：AMCL 是一个粒子滤波器，它匹配 2D 激光扫描和 2D 占据栅格地图。  
  * NDT 是一种**点云到点云**的扫描匹配算法。它获取当前的高清 3D Lidar 点云，并将其与一个预先录制的 3D 高清（HD）点云地图进行匹配，以计算出精确的 6-DOF 姿态。  
  * NDT 是 Autoware 73 等专业自动驾驶框架的核心定位算法。  
* **规划：A**\*：Aslan 使用 A\* 规划器（A\* Planner）进行路径规划 72。

Project Aslan 展示了，在“自动驾驶”领域，存在一整套不同于“室内导航”的专业算法（NDT, A\*），而 Gazebo 正是用于测试这些算法的理想平台。

### **6.3 高级仿真：包含动态交通和行人的协同仿真**

在 Gazebo 中可以放置行人模型 5 和交通信号灯 7，但默认情况下它们是静态的，这并不是一个有效的测试。

为了实现一个包含*真实动态交通流*的测试环境，专业人士会使用\*\*“协同仿真”（Co-Simulation）\*\* 75。

这种终极测试平台的工作流程如下 75：

1. **SUMO (Simulation of Urban Mobility)**：这是一个开源的、微观的**交通流模拟器** 75。SUMO 负责模拟整个城市的交通逻辑——AI 车辆的驾驶行为、行人的随机移动以及交通信号灯的配时。  
2. **Gazebo / ROS**：Gazebo **只**负责模拟**一辆**车——即你的“Ego-Vehicle”（自动驾驶主车）。这辆车拥有完整的高保真物理和传感器（Lidar, Camera, IMU）。  
3. **TraCI (Traffic Control Interface)**：这是 SUMO 用于与外部程序（如 ROS）通信的协议 75。  
4. **桥接**：一个自定义的 ROS 节点会通过 TraCI 订阅 SUMO。它获取所有*其他* AI 车辆和行人的位置，然后在 Gazebo 中实时“生成”或“移动”这些物体的简化模型 75。

**最终结果**：你的高保真、全物理、全传感器的“Ego-Vehicle”（运行在 Gazebo/ROS 中）行驶在一个由 SUMO 逻辑驱动的、充满动态 AI 车辆和行人的城市中。

这是测试自动驾驶堆栈（从感知到规划）的黄金标准。它允许你回答关键问题：“我的感知系统是否能看到 SUMO 生成的闯红灯的汽车？”“我的规划器是否能对 SUMO 控制的行人做出反应？”

## **第七部分：结论与后续步骤**

本指南为从 Gazebo 入门到实现高级导航与自动驾驶功能提供了一个全面的技术路线图。

**核心结论：**

1. **选择决定一切**：入门的第一个、也是最关键的决定是选择**现代 Gazebo (Gz/Ignition)**（如 Harmonic 版本）而非 Gazebo Classic，因为后者即将停止支持 5。这一选择决定了必须使用 **ROS 2** 以及 **ros\_gz\_bridge** 解耦架构 41。  
2. **仿真是“混合”的艺术**：成功的机器人模型（URDF）必须通过 \<gazebo\> 标签嵌入 SDF 代码段，以定义传感器和插件 21。  
3. **桥接是手动的**：ros\_gz\_bridge 要求用户在 launch 文件中**手动**声明每一个需要“翻译”的 ROS-Gz 话题及其方向（@）35。这是新架构的核心配置点。  
4. **use\_sim\_time 不是可选项**：在启动任何与仿真交互的 ROS 节点时，**必须**设置 use\_sim\_time:=True，否则 tf 时间戳错误将导致所有 SLAM 和导航节点失败 55。  
5. **“导航”与“自动驾驶”是不同路径**：  
   * **导航 (Navigation)**：通常指室内机器人。其技术栈是 slam\_toolbox (SLAM) $\\rightarrow$ AMCL (Localization) $\\rightarrow$ Nav2 (Planning/Control)，并使用 dwb\_controller (DWA) 进行运动控制 50。  
   * **自动驾驶 (Autonomous Driving)**：通常指室外车辆。其技术栈更专业，可能包括 NDT (Localization), A\* (Planning) 72, 以及 Stanley 或 Pure Pursuit (Ackermann Control) 69。  
6. **Gazebo 只是沙盒**：Gazebo 提供了高保真的物理和传感器沙盒。真正的“智能”在于运行在 ROS 端的算法（Nav2, slam\_toolbox, 或 Autoware）。  
7. **终极目标是协同仿真**：对于自动驾驶，最终的测试平台是将 Gazebo（用于Ego-Vehicle）与 SUMO（用于AI交通流）通过 ROS/TraCI 协同仿真 75。

对于入门者，建议的路径是循序渐进的：

1. **基础**：在 ROS 2 Humble/Jazzy 上安装 Gazebo Harmonic 和 ros\_gzharmonic 桥接包。  
2. **建模**：构建一个简单的差分驱动机器人 URDF，并为其添加 Lidar 和 IMU 的 \<gazebo\> 插件。  
3. **桥接**：编写一个 launch 文件，启动 Gazebo、robot\_state\_publisher、Rviz 以及用于 Lidar、IMU 和 /cmd\_vel 的 ros\_gz\_bridge 节点。  
4. **SLAM**：使用步骤 3 的启动文件，运行 slam\_toolbox 51，并通过 ros2 topic pub 手动（或使用键盘遥控节点）控制机器人在环境中移动以构建地图。  
5. **导航**：在保存地图后，着手配置完整的 Nav2 堆栈，从理解 nav2\_params.yaml 的结构 63 开始，并特别注意 dwb\_controller 67 和代价地图膨胀 68 的调优。

掌握了 Nav2 之后，用户便可开始探索更高级的领域，例如将控制器更换为适用于阿克曼车辆的 RegulatedPurePursuit，或者转向 Project Aslan 72 和 AutoCarROS2 69 等专用自动驾驶项目，最终实现与 SUMO 的协同仿真。

#### **Works cited**

1. Gazebo \- GitHub, accessed November 4, 2025, [https://github.com/gazebosim](https://github.com/gazebosim)  
2. Tutorial : Installing gazebo\_ros\_pkgs (ROS 1\) \- Gazebo, accessed November 4, 2025, [https://classic.gazebosim.org/tutorials?tut=ros\_installing](https://classic.gazebosim.org/tutorials?tut=ros_installing)  
3. Gazebo, accessed November 4, 2025, [https://gazebosim.org/](https://gazebosim.org/)  
4. gazebosim/gz-sensors: Provides numerous sensor models designed to generate realistic data from simulation environments. \- GitHub, accessed November 4, 2025, [https://github.com/gazebosim/gz-sensors](https://github.com/gazebosim/gz-sensors)  
5. Blog : Vehicle and city simulation \- Gazebo, accessed November 4, 2025, [https://classic.gazebosim.org/blog/car\_sim](https://classic.gazebosim.org/blog/car_sim)  
6. 6.6.4 Gazebo仿真环境搭建· Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程, accessed November 4, 2025, [http://www.autolabor.com.cn/book/ROSTutorials/di-6-zhang-ji-qi-ren-xi-tong-fang-zhen/66-urdfji-cheng-gazebo/664-gazebofang-zhen-huan-jing-da-jian.html](http://www.autolabor.com.cn/book/ROSTutorials/di-6-zhang-ji-qi-ren-xi-tong-fang-zhen/66-urdfji-cheng-gazebo/664-gazebofang-zhen-huan-jing-da-jian.html)  
7. TRAFFIC SIGN DETECTION IN GAZEBO SIMULATION ENVIRONMENT USING YOLOV3 COPYRIGHT © 2019 BY NIKHIL PRABHU \- Jaerock Kwon, Ph.D., accessed November 4, 2025, [https://jrkwon.com/wordpress/wp-content/uploads/2024/04/Final\_thesis\_nikhil.pdf](https://jrkwon.com/wordpress/wp-content/uploads/2024/04/Final_thesis_nikhil.pdf)  
8. Tutorial : Building Editor \- Gazebo Classic, accessed November 4, 2025, [https://classic.gazebosim.org/tutorials?tut=building\_editor](https://classic.gazebosim.org/tutorials?tut=building_editor)  
9. Gazebo: Building Editor \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=Y2LRHl79b3g](https://www.youtube.com/watch?v=Y2LRHl79b3g)  
10. How do I create a Gazebo world | Autonomous Robotics Lab Notebook \- GitBook, accessed November 4, 2025, [https://campus-rover.gitbook.io/lab-notebook/fiiva/create-gazebo.world](https://campus-rover.gitbook.io/lab-notebook/fiiva/create-gazebo.world)  
11. How to Create a Gazebo Model Using CAD and Blender \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=zGWFojrPoSA](https://www.youtube.com/watch?v=zGWFojrPoSA)  
12. How To Insert Custom Models in Gazebo \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=fwoTLfypIMw](https://www.youtube.com/watch?v=fwoTLfypIMw)  
13. Tutorial : Import Meshes \- Gazebo, accessed November 4, 2025, [https://classic.gazebosim.org/tutorials?tut=import\_mesh](https://classic.gazebosim.org/tutorials?tut=import_mesh)  
14. Create And Import Blender Scenes Into Gazebo \- insert, accessed November 4, 2025, [https://insrt.uk/post/ros-gazebo-create-scene-blender/](https://insrt.uk/post/ros-gazebo-create-scene-blender/)  
15. Tutorial : Intermediate: Model Appearance \- Gazebo Classic, accessed November 4, 2025, [https://classic.gazebosim.org/tutorials?tut=guided\_i2](https://classic.gazebosim.org/tutorials?tut=guided_i2)  
16. ROSCon 2017: Vehicle and city simulation with Gazebo and ROS \-- Ian Chen and and Carlos Agu?ero (Open Robotics), accessed November 4, 2025, [https://www.ros.org/news/2018/07/roscon-2017-vehicle-and-city-simulation-with-gazebo-and-ros----ian-chen-and-and-carlos-aguero-open-r.html](https://www.ros.org/news/2018/07/roscon-2017-vehicle-and-city-simulation-with-gazebo-and-ros----ian-chen-and-and-carlos-aguero-open-r.html)  
17. osrf/citysim: Example city simulation for autonomous vehicles in Gazebo Classic. \- GitHub, accessed November 4, 2025, [https://github.com/osrf/citysim](https://github.com/osrf/citysim)  
18. Projects using Gazebo — Gazebo jetty documentation, accessed November 4, 2025, [https://gazebosim.org/docs/latest/projects\_using\_gazebo/](https://gazebosim.org/docs/latest/projects_using_gazebo/)  
19. ROS2与URDF入门教程-使用URDF构建可移动机器人模型- 爱折腾 \- 创客智造, accessed November 4, 2025, [https://www.ncnynl.com/archives/202110/4674.html](https://www.ncnynl.com/archives/202110/4674.html)  
20. Gazebo：三维物理仿真平台 \- 引言- 古月居, accessed November 4, 2025, [https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/3.4\_Gazebo/](https://book.guyuehome.com/ROS2/3.%E5%B8%B8%E7%94%A8%E5%B7%A5%E5%85%B7/3.4_Gazebo/)  
21. Tutorial : Gazebo plugins in ROS, accessed November 4, 2025, [https://classic.gazebosim.org/tutorials?tut=ros\_gzplugins](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)  
22. Tutorial 8: Simulation \- Sensors — 240AR060 \- Introduction to ROS, accessed November 4, 2025, [https://sir.upc.edu/projects/rostutorials/8-gazebo\_sensors\_tutorial/index.html](https://sir.upc.edu/projects/rostutorials/8-gazebo_sensors_tutorial/index.html)  
23. sensors — Gazebo documentation, accessed November 4, 2025, [https://gazebosim.org/libs/sensors/](https://gazebosim.org/libs/sensors/)  
24. ROS2\_Control Wheeled Mobile Robot Gazebo Simulation Diff Drive Controller \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=8N9elizZ1x4](https://www.youtube.com/watch?v=8N9elizZ1x4)  
25. What is the difference between the gazebo differential drive plugin and a differential drive controller spawned by controller\_spawner? \- Robotics Stack Exchange, accessed November 4, 2025, [https://robotics.stackexchange.com/questions/67631/what-is-the-difference-between-the-gazebo-differential-drive-plugin-and-a-differ](https://robotics.stackexchange.com/questions/67631/what-is-the-difference-between-the-gazebo-differential-drive-plugin-and-a-differ)  
26. Class GazeboRosAckermannDrive — gazebo\_plugins 3.5.2 documentation, accessed November 4, 2025, [https://docs.ros.org/en/rolling/p/gazebo\_plugins/generated/classgazebo\_\_plugins\_1\_1GazeboRosAckermannDrive.html](https://docs.ros.org/en/rolling/p/gazebo_plugins/generated/classgazebo__plugins_1_1GazeboRosAckermannDrive.html)  
27. how to use gazebo ackermann plugin ROS2 Humble \- Robotics Stack Exchange, accessed November 4, 2025, [https://robotics.stackexchange.com/questions/105969/how-to-use-gazebo-ackermann-plugin-ros2-humble](https://robotics.stackexchange.com/questions/105969/how-to-use-gazebo-ackermann-plugin-ros2-humble)  
28. How to use ackermann plugin in ROS2 Humble \- Stack Overflow, accessed November 4, 2025, [https://stackoverflow.com/questions/77634282/how-to-use-ackermann-plugin-in-ros2-humble](https://stackoverflow.com/questions/77634282/how-to-use-ackermann-plugin-in-ros2-humble)  
29. Understand Ackermann Kinematics in ROS2 with LIMO Robot | | Robotics Developers Open Class 186 \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=CAyyDYCUoho](https://www.youtube.com/watch?v=CAyyDYCUoho)  
30. gazebo\_ros\_pkgs \- ROS Repository Overview, accessed November 4, 2025, [https://index.ros.org/r/gazebo\_ros\_pkgs/](https://index.ros.org/r/gazebo_ros_pkgs/)  
31. Tutorial : Installing gazebo\_ros\_pkgs (ROS 2\) \- Gazebo, accessed November 4, 2025, [https://classic.gazebosim.org/tutorials?tut=ros2\_installing](https://classic.gazebosim.org/tutorials?tut=ros2_installing)  
32. gazebo\_ros\_pkgs \- ROS Wiki, accessed November 4, 2025, [https://wiki.ros.org/gazebo\_ros\_pkgs](https://wiki.ros.org/gazebo_ros_pkgs)  
33. An overview of the gazebo\_ros\_pkgs interface. The interface contains several packages such as gazebo\_msgs, gazebo\_ros, and gazebo\_plugins \[36\]. \- ResearchGate, accessed November 4, 2025, [https://www.researchgate.net/figure/An-overview-of-the-gazebo-ros-pkgs-interface-The-interface-contains-several-packages\_fig1\_336925430](https://www.researchgate.net/figure/An-overview-of-the-gazebo-ros-pkgs-interface-The-interface-contains-several-packages_fig1_336925430)  
34. ROS2与Gazebo11入门教程-连接至ROS 2 \- 爱折腾-创客智造实验室, accessed November 4, 2025, [https://www.ncnynl.com/archives/202110/4809.html](https://www.ncnynl.com/archives/202110/4809.html)  
35. Use ROS 2 to interact with Gazebo, accessed November 4, 2025, [https://gazebosim.org/docs/latest/ros2\_integration/](https://gazebosim.org/docs/latest/ros2_integration/)  
36. Setting up a robot simulation (Gazebo) \- ROS documentation, accessed November 4, 2025, [https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)  
37. ros\_gz\_bridge: Humble 0.244.20 documentation \- ROS documentation, accessed November 4, 2025, [https://docs.ros.org/en/humble/p/ros\_gz\_bridge/](https://docs.ros.org/en/humble/p/ros_gz_bridge/)  
38. ROS 2 Integration — Gazebo fortress documentation, accessed November 4, 2025, [https://gazebosim.org/docs/fortress/ros2\_integration/](https://gazebosim.org/docs/fortress/ros2_integration/)  
39. ros2tutorials / ros\_gz \- GitLab \- UPC, accessed November 4, 2025, [https://gitioc.upc.edu/ros2tutorials/ros\_gz](https://gitioc.upc.edu/ros2tutorials/ros_gz)  
40. Will Gazebo harmonic ever be recommended for ROS2 Humble? \- General, accessed November 4, 2025, [https://community.gazebosim.org/t/will-gazebo-harmonic-ever-be-recommended-for-ros2-humble/2390](https://community.gazebosim.org/t/will-gazebo-harmonic-ever-be-recommended-for-ros2-humble/2390)  
41. Installing Gazebo with ROS — Gazebo jetty documentation, accessed November 4, 2025, [https://gazebosim.org/docs/latest/ros\_installation/](https://gazebosim.org/docs/latest/ros_installation/)  
42. ROS2 Humble, Gazebo Harmonic, PX4, QGroundControl, Micro XRCE-DDS Agent & Client Installation | by Erdem | Medium, accessed November 4, 2025, [https://medium.com/@erdem.ku.3.14/ros2-humble-gazebo-harmonic-px4-ve-micro-xrce-dds-agent-client-installation-aad32d8f5669](https://medium.com/@erdem.ku.3.14/ros2-humble-gazebo-harmonic-px4-ve-micro-xrce-dds-agent-client-installation-aad32d8f5669)  
43. Understanding topics — ROS 2 Documentation: Foxy documentation, accessed November 4, 2025, [https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)  
44. ROS Gmapping | SLAM 1 | How to map an environment in ROS | ros mapping | ROS Tutorial for Beginners \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=MFqn9i68bfM](https://www.youtube.com/watch?v=MFqn9i68bfM)  
45. slam \- Hector\_Slam (Autonomous Navigation) \- Robotics Stack ..., accessed November 4, 2025, [https://robotics.stackexchange.com/questions/75765/hector-slam-autonomous-navigation](https://robotics.stackexchange.com/questions/75765/hector-slam-autonomous-navigation)  
46. Slam gmapping and hector \- Gazebo Answers archive, accessed November 4, 2025, [http://answers.gazebosim.org/question/15978/](http://answers.gazebosim.org/question/15978/)  
47. ROS mapping | Robot simulation | Mapping | Hector mapping | Github | Gazebo | Riviz \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=6jIMgrqG5eg](https://www.youtube.com/watch?v=6jIMgrqG5eg)  
48. Comparison of SLAM Mapping Algorithms and Possible Navigational Strategies on Simulated and Real World Conditions, accessed November 4, 2025, [https://me336.ancorasir.com/wp-content/uploads/2023/06/Team7-Report.pdf](https://me336.ancorasir.com/wp-content/uploads/2023/06/Team7-Report.pdf)  
49. SLAM | Husarion, accessed November 4, 2025, [https://husarion.com/tutorials/ros2-tutorials/8-slam/](https://husarion.com/tutorials/ros2-tutorials/8-slam/)  
50. ROS2 Nav2 \- Generate a Map with slam\_toolbox \- The Robotics Back-End, accessed November 4, 2025, [https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam\_toolbox/](https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/)  
51. Easy SLAM with ROS using slam\_toolbox \- YouTube, accessed November 4, 2025, [https://www.youtube.com/watch?v=ZaiA3hWaRzE](https://www.youtube.com/watch?v=ZaiA3hWaRzE)  
52. ROS2 SLAM Toolbox Tutorial Mobile Robot Simulation (SLAM Algorithm and Code), accessed November 4, 2025, [https://www.youtube.com/watch?v=0G6LDuslqmA](https://www.youtube.com/watch?v=0G6LDuslqmA)  
53. Comparing different SLAM methods \- Aditya Kamath, accessed November 4, 2025, [https://adityakamath.github.io/2021-09-05-comparing-slam-methods/](https://adityakamath.github.io/2021-09-05-comparing-slam-methods/)  
54. ROS2 Humble- Gazebo & SLAM-Toolbox: TF2 dropped message reason 'the timestamp on the message is earlier than all the data in the transform cache" \- Robotics Stack Exchange, accessed November 4, 2025, [https://robotics.stackexchange.com/questions/115148/ros2-humble-gazebo-slam-toolbox-tf2-dropped-message-reason-the-timestamp-on](https://robotics.stackexchange.com/questions/115148/ros2-humble-gazebo-slam-toolbox-tf2-dropped-message-reason-the-timestamp-on)  
55. bt\_navigator problem following NAV2 tutorial on ROS2 HUMBLE · Issue \#3986 · ros-navigation/navigation2 \- GitHub, accessed November 4, 2025, [https://github.com/ros-planning/navigation2/issues/3986](https://github.com/ros-planning/navigation2/issues/3986)  
56. gurselturkeri/ros2\_diff\_drive\_robot: Differential drive mobile robot simulation in Gazebo with using ROS2 \- GitHub, accessed November 4, 2025, [https://github.com/gurselturkeri/ros2\_diff\_drive\_robot](https://github.com/gurselturkeri/ros2_diff_drive_robot)  
57. Navigation2 Overview, accessed November 4, 2025, [https://roscon.ros.org/2019/talks/roscon2019\_navigation2\_overview\_final.pdf](https://roscon.ros.org/2019/talks/roscon2019_navigation2_overview_final.pdf)  
58. amcl \- ROS Wiki, accessed November 4, 2025, [https://wiki.ros.org/amcl](https://wiki.ros.org/amcl)  
59. ROS 2 Navigation Tuning Guide – Nav2 \- AutomaticAddison.com, accessed November 4, 2025, [https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)  
60. Navigation (ROS 1\) Vs Navigation 2 (ROS 2\) | by Sharad Maheshwari | Medium, accessed November 4, 2025, [https://medium.com/@thehummingbird/navigation-ros-1-vs-navigation-2-ros-2-12398b64cd](https://medium.com/@thehummingbird/navigation-ros-1-vs-navigation-2-ros-2-12398b64cd)  
61. Autonomous robot navigation and Nav2: The first steps. \- Foxglove, accessed November 4, 2025, [https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps](https://foxglove.dev/blog/autonomous-robot-navigation-and-nav2-the-first-steps)  
62. ROS 2 Navigation2 Configuration: Complete Guide to Optimizing Your Rob \- Think Robotics, accessed November 4, 2025, [https://thinkrobotics.com/blogs/learn/ros-2-navigation2-configuration-complete-guide-to-optimizing-your-robot-navigation-stack](https://thinkrobotics.com/blogs/learn/ros-2-navigation2-configuration-complete-guide-to-optimizing-your-robot-navigation-stack)  
63. Writing a New Navigator Plugin — Nav2 1.0.0 documentation, accessed November 4, 2025, [https://docs.nav2.org/plugin\_tutorials/docs/writing\_new\_navigator\_plugin.html](https://docs.nav2.org/plugin_tutorials/docs/writing_new_navigator_plugin.html)  
64. Navigation | Husarion, accessed November 4, 2025, [https://husarion.com/tutorials/ros2-tutorials/9-navigation/](https://husarion.com/tutorials/ros2-tutorials/9-navigation/)  
65. AMCL — Nav2 1.0.0 documentation, accessed November 4, 2025, [https://docs.nav2.org/configuration/packages/configuring-amcl.html](https://docs.nav2.org/configuration/packages/configuring-amcl.html)  
66. DWB Controller — Nav2 1.0.0 documentation, accessed November 4, 2025, [https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)  
67. Tuning Guide — Nav2 1.0.0 documentation, accessed November 4, 2025, [https://docs.nav2.org/tuning/index.html](https://docs.nav2.org/tuning/index.html)  
68. winstxnhdw/AutoCarROS2: A virtual simulation platform for ... \- GitHub, accessed November 4, 2025, [https://github.com/winstxnhdw/AutoCarROS2](https://github.com/winstxnhdw/AutoCarROS2)  
69. Milestone 5: Stanley Controller Path Following Simulation \- EML 4842 \- Read the Docs, accessed November 4, 2025, [https://av1tenth-docs.readthedocs.io/en/latest/assignments/milestones/milestone5.html](https://av1tenth-docs.readthedocs.io/en/latest/assignments/milestones/milestone5.html)  
70. Configuration Guide — Nav2 1.0.0 documentation, accessed November 4, 2025, [https://docs.nav2.org/configuration/index.html](https://docs.nav2.org/configuration/index.html)  
71. project-aslan/Aslan: Open source self-driving software for low speed environments \- GitHub, accessed November 4, 2025, [https://github.com/project-aslan/Aslan](https://github.com/project-aslan/Aslan)  
72. NDT Matching. Here's a quick summary on how normal… | by David Silver | Self-Driving Cars | Medium, accessed November 4, 2025, [https://medium.com/self-driving-cars/ndt-matching-acff8e7e01cb](https://medium.com/self-driving-cars/ndt-matching-acff8e7e01cb)  
73. robustify/gazebo\_traffic\_light \- GitHub, accessed November 4, 2025, [https://github.com/robustify/gazebo\_traffic\_light](https://github.com/robustify/gazebo_traffic_light)  
74. Introduction Objective Simulation Environment Sumo and ... \- Orbit, accessed November 4, 2025, [https://www.orbit-lab.org/raw-attachment/wiki/Other/Summer/2020/OH2020/P05.pdf](https://www.orbit-lab.org/raw-attachment/wiki/Other/Summer/2020/OH2020/P05.pdf)  
75. TraCI \- SUMO Documentation, accessed November 4, 2025, [https://sumo.dlr.de/docs/TraCI.html](https://sumo.dlr.de/docs/TraCI.html)