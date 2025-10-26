

# **ROS 2进阶之路：一份为新手打造的仿真项目实践指南**

## **引言：开启您的 ROS 2 生态系统导览之旅**

对于有志于投身机器人领域的开发者而言，机器人操作系统（ROS）是一个功能强大但体系庞杂的生态系统。初学者往往面临“不知从何学起”的困境。本指南旨在解决这一核心痛点，为 ROS 新手提供一条结构化、循序渐进且以实践为导向的学习路径。我们将摒弃零散的知识点罗列，转而采用一种精心设计的课程化学习方法，通过一系列代码量小、逻辑清晰的开源 GitHub 项目，引导您完成一次从零到一的机器人开发“远征”。

我们的教学方法论遵循机器人构建的自然逻辑：首先，我们将学习如何为机器人绘制“物理蓝图”；其次，为它创建一个可供交互的“虚拟环境”；接着，构建其感知与控制的“神经系统”；最后，赋予它第一个真正的“智能行为”。这一过程将完全在仿真环境中进行，因为仿真为初学者提供了无与伦比的优势：它安全、成本低廉，并且允许开发者快速迭代代码和验证想法，这是在真实硬件上难以实现的 1。本指南将是您的地图和罗盘，指引您在 ROS 2 的世界中稳步前行，最终掌握构建一个完整机器人应用的核心能力。

---

## **第一部分：从蓝图到模型——使用 URDF 定义您的第一个机器人**

在任何机器人项目中，首要任务是创建一个数字化的机器人模型。这不仅是可视化的基础，也是物理仿真的核心。统一机器人描述格式（URDF）正是为此而生的标准。它是一种基于 XML 的文件格式，用于描述机器人的“解剖结构”。

### **核心仓库：ros/urdf\_tutorial**

3

这个官方教程仓库是理解 URDF 的最佳起点。它通过一系列简单示例，系统地介绍了 URDF 的核心概念。

#### **URDF 的解剖学**

一个 URDF 文件由一系列\<link\>（连杆）和\<joint\>（关节）元素构成，它们共同定义了机器人的运动学链。

* \<link\>：机器人的刚性部分  
  连杆是机器人的基本物理单元。在 URDF 中，每个\<link\>都代表一个不会变形的部件。为了让机器人在仿真和可视化工具中正确显示，我们需要为连杆定义两个关键的子元素：  
  * \<visual\>：定义了连杆的外观，包括其几何形状（如长方体、圆柱体、球体或外部三维模型文件）、尺寸和材质（颜色、纹理等）。这部分主要供 RViz 等可视化工具使用。  
  * \<collision\>：定义了连杆的碰撞边界。它的结构与\<visual\>类似，但其几何形状通常更简单，以提高物理引擎的计算效率。这部分是 Gazebo 等物理仿真器进行碰撞检测的依据。  
* \<joint\>：连接与运动的定义  
  关节用于连接两个连杆，并定义它们之间的相对运动方式。关键属性包括：  
  * **类型（type）**：决定了关节的运动自由度，常见的有关节revolute（旋转）、continuous（连续旋转，无限制）、prismatic（滑动）和fixed（固定）。  
  * **父子关系（\<parent\> 和 \<child\>）**：明确定义了哪个连杆是固定的（父），哪个连杆相对于父连杆运动（子），从而构建起整个机器人的运动学树。  
  * **原点（\<origin\>）**：定义了子连杆坐标系相对于父连杆坐标系的位置和姿态，这是精确描述机器人几何结构的关键。

#### **使用 Xacro 提升效率**

随着机器人复杂度的增加，手写 URDF 文件会变得异常冗长和重复。Xacro（XML Macros）应运而生，它为 URDF 引入了宏、变量和数学计算等编程特性，极大地简化了 URDF 的编写和维护 3。通过定义属性和宏，开发者可以创建模块化、可重用的机器人部件描述，显著减少代码量。

#### **为仿真注入“灵魂”：物理属性与 Gazebo 扩展**

一个只有\<visual\>和\<collision\>的 URDF 模型在仿真器中只是一个没有“生命”的空壳。为了让它表现得像一个真实的物理实体，我们必须添加更多信息。

* \<inertial\>：惯性属性  
  这个标签定义了连杆的动态特性，包括质量（mass）和惯性张量（inertia matrix）。这些参数是物理引擎计算力、加速度和扭矩的基础，直接决定了机器人在仿真中的运动行为是否真实 4。  
* \<gazebo\>：仿真专属的“插件”  
  URDF 本身是独立于任何特定仿真器的。为了让模型在 Gazebo 中正确工作，我们需要添加 Gazebo 专用的扩展标签\<gazebo\> 5。这是一个至关重要的设计，它体现了 ROS 架构中一个深刻的原则：关注点分离（Separation of Concerns）。  
  核心的 URDF 文件定义了机器人的通用运动学和视觉描述，这部分信息对于 RViz 可视化、MoveIt\! 运动规划等多个工具都是通用的。而\<gazebo\>标签则封装了所有与 Gazebo 仿真相关的特定实现细节。这种分离确保了核心机器人模型的通用性和可移植性。例如，我们可以在\<gazebo\>标签内为连杆定义更精细的物理参数，如摩擦系数（\<mu1\>, \<mu2\>）和接触刚度（\<kp\>, \<kd\>），这些参数直接影响着仿真物理引擎的行为 4。  
  这种架构也让初学者第一次直面机器人开发中的核心挑战之一——**“仿真到现实的鸿沟”（Sim-to-Real Gap）**。机器人在 Gazebo 中的行为，完全取决于\<gazebo\>标签内定义的物理参数的准确性。如果这些参数（如摩擦力、电机阻尼、传感器噪声模型）与真实世界的物理硬件不匹配，那么一个在仿真中完美运行的控制器，部署到实体机器人上时几乎注定会失败。因此，从学习 URDF 的第一天起就理解这种分离，意味着开始理解仿真的本质：它是一种抽象，其保真度本身就是一个需要精心管理和调试的参数。这为后续更高级的开发工作奠定了坚实的思想基础。

---

## **第二部分：沙盒——掌握 Gazebo 仿真环境**

拥有了机器人模型之后，下一步是为它创建一个可以栖息和互动的虚拟世界。Gazebo 是 ROS 生态中最主流的 3D 物理仿真器，它与 ROS 紧密集成，能够模拟真实世界的光照、物理和传感器数据 1。

### **核心仓库：MOGI-ROS/Week-3-4-Gazebo-basics**

1

这个仓库提供了一个极佳的入门教程，涵盖了从创建 Gazebo 世界到通过 ROS 2 控制机器人的完整流程。

#### **创建一个虚拟世界**

Gazebo 的世界由.sdf（Simulation Description Format）或.world文件定义。这些文件本质上也是 XML 格式，允许用户在其中添加各种元素，包括：

* **基础模型**：如地面、墙壁等。  
* **光照**：定义太阳光或点光源，影响场景的视觉效果。  
* **物理属性**：定义全局重力等。  
* **3D 模型**：可以从 Gazebo 的在线模型库或本地文件中导入复杂的物体，如桌子、椅子等，以构建丰富的测试场景 1。

#### **从 ROS 2 启动和控制 Gazebo**

将机器人模型放入 Gazebo 世界的关键在于 ROS 2 的启动文件（Launch File）。一个典型的.launch.py文件会执行以下关键任务：

1. **启动 Gazebo 服务**：通过 gazebo\_ros 包中的节点启动 Gazebo 的服务器（负责物理计算）和客户端（GUI 界面）。  
2. **生成机器人实体**：使用 spawn\_entity.py 脚本或等效的 ROS 2 节点，将之前创建的 URDF 文件内容加载并生成到正在运行的仿真世界中。  
3. **传递参数**：启动文件还允许我们向 Gazebo 传递参数，例如，使用 \-r 标志可以让仿真在启动后自动开始运行，而不是默认的暂停状态 1。

#### **赋予机器人生命：Gazebo 插件**

默认情况下，一个从 URDF 生成到 Gazebo 中的机器人模型是“死的”，它仅仅是一个具有物理属性的静态模型。要让它能够响应 ROS 的指令，我们需要使用 **Gazebo 插件**。这些插件是动态加载的库，它们充当了仿真世界内部物理模型与外部 ROS 2 网络之间的桥梁。

对于移动机器人而言，最重要的插件莫过于**差分驱动插件**（例如 libdiffdrive\_plugin.so）5。只需在 URDF 文件的\<gazebo\>标签内添加几行配置，这个插件就能为机器人实现一整套核心的底层功能：

* **订阅 /cmd\_vel 话题**：它会自动创建一个 ROS 2 订阅者，监听 geometry\_msgs/msg/Twist 类型的消息。开发者可以通过向这个话题发布消息来控制机器人的线速度和角速度。  
* **发布 /odom 话题**：插件会根据仿真中轮子的转动，计算出机器人的里程计信息（位置和姿态的增量变化），并以 nav\_msgs/msg/Odometry 格式发布出去。  
* **发布 TF 变换**：同时，它还会发布从里程计坐标系（odom）到机器人基座坐标系（base\_link）的 TF 变换，这是机器人定位和导航系统的基础。

#### **驾驶与观察**

配置好插件后，就可以通过 ROS 2 的命令行工具与机器人进行交互了。例如，使用以下命令可以让机器人以 0.5 m/s 的速度前进：

Bash

ros2 topic pub \--once /cmd\_vel geometry\_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

开发者可以在 Gazebo 的 GUI 窗口中直接观察到机器人的运动，同时也可以在 RViz 中通过可视化 /odom 话题和 TF 树来验证里程计数据和坐标变换的正确性 1。至此，我们已经完成了从静态模型到动态可控仿真机器人的关键跨越。

---

## **第三部分：机器人的神经系统——核心 ROS 2 概念实践指南**

在完成了机器人的“物理”构建后，我们现在转向其“软件”层面。一个强大的机器人应用是由多个协同工作的节点构成的，它们之间的通信遵循着 ROS 2 定义的架构模式。本部分将把 ROS 2 最核心的几个概念分解为独立、易于理解的代码示例，为后续构建完整应用打下坚实基础。

一个设计精良的 ROS 2 系统，其节点间的通信并非杂乱无章的数据交换，而是遵循着一套内在的“语法”。这套语法由三个核心支柱构成：**TF2** 定义了“在哪里”（空间上下文），**参数**定义了“如何做”（行为配置），而**自定义接口**则定义了“是什么”（数据契约）。将这三者作为一个整体来学习，能够帮助初学者从一开始就建立起系统设计的思维，而不仅仅是学习零散的 API 调用。当您开始编写一个新的 ROS 节点时，应该首先思考这三个问题：

1. 我的节点需要处理和交换什么样的数据？（接口）  
2. 这些数据涉及到哪些空间坐标系和变换？（TF2）  
3. 节点的哪些行为需要是可配置和可调的？（参数）

这种思维模式将引导您从一个“代码实现者”转变为一个“系统架构师”。

### **3.1 理解空间：使用 TF2 管理坐标系**

在机器人学中，任何一个实体都不是一个简单的点，而是一个由多个部件组成的集合，每个部件都有自己的坐标系（例如，机器人基座、激光雷达、摄像头、机械臂末端）。TF2 (Transform Framework 2\) 就是 ROS 2 中用于管理这些复杂空间关系的系统。它实时地回答着“A 部件相对于 B 部件的位置和姿态是什么？”这个问题，是机器人实现空间感知和定位的基础。

#### **核心示例：Hello Robot stretch\_ros\_tutorials**

6

这个教程提供了一组非常清晰的 Python 示例，用于演示 TF2 的基本用法。（作为参考，其 ROS 1 版本也提供了类似的逻辑 7）。

* **广播器（Broadcaster）**：tf2\_broadcaster.py 脚本展示了如何发布一个坐标变换。其核心步骤是：  
  1. 创建一个 TransformBroadcaster 对象。  
  2. 填充一个 TransformStamped 消息，其中必须包含时间戳、父坐标系ID（header.frame\_id）、子坐标系ID（child\_frame\_id），以及平移（translation）和旋转（rotation，通常为四元数）信息。  
  3. 调用 sendTransform 方法将此变换发布到 /tf 和 /tf\_static 话题。  
* **监听器（Listener）**：tf2\_listener.py 脚本则演示了如何查询坐标变换。核心步骤是：  
  1. 创建一个 tf2\_ros.Buffer 来缓存接收到的变换。  
  2. 创建一个 TransformListener 对象，并将 buffer 传递给它。  
  3. 在需要时，调用 buffer 的 lookup\_transform 方法，传入目标坐标系和源坐标系，即可获得它们之间的最新变换关系。  
* **静态与动态变换**：TF2 区分两种变换。**静态变换**用于描述系统中固定不变的相对关系，例如焊在机器人底盘上的激光雷达。这种变换只需发布一次。**动态变换**则用于描述随时间变化的相对关系，最典型的例子就是由里程计发布的 odom \-\> base\_link 变换，它会随着机器人的移动而不断更新。dottantgal/ROS2-dynamic-TF2-broadcaster 8 仓库提供了一个更高级的动态更新示例。

### **3.2 构建灵活的节点：使用参数进行配置**

硬编码的数值是软件工程的大敌。ROS 2 的参数系统提供了一种强大的机制，允许开发者在不重新编译代码的情况下，动态地配置节点的行为。这对于算法调优、多机器人部署和在不同环境下重用代码至关重要。

#### **核心示例：Foxglove 教程与 ROS 2 官方文档**

9

这些资源详细介绍了在 C++ 和 Python 中使用参数的完整流程。

* **声明参数**：在一个节点中，必须先声明它所接受的参数。这相当于为节点定义了一个公开的配置 API。  
  * C++: this-\>declare\_parameter\<type\>("param\_name", default\_value);  
  * Python: self.declare\_parameter("param\_name", default\_value)  
* **获取参数值**：在节点的代码逻辑中，可以通过 get\_parameter 方法来读取当前配置的参数值。  
  * C++: this-\>get\_parameter("param\_name", variable\_to\_store);  
  * Python: variable\_to\_store \= self.get\_parameter("param\_name").value  
* **设置参数值**：ROS 2 提供了多种灵活的方式来设置参数：  
  1. **命令行**：在启动节点时，通过 \--ros-args \-p \<name\>:=\<value\> 标志直接传入 9。  
  2. **YAML 文件**：可以将所有参数集中写在一个 .yaml 配置文件中，然后在启动时使用 \--params-file 标志加载。这是最常用、最规范的方式 9。  
  3. **运行时动态修改**：使用 ros2 param set \<node\_name\> \<param\_name\> \<value\> 命令，可以在节点运行时实时更改其参数值，非常适合在线调试和调优 10。

### **3.3 定义通用语言：自定义接口 (Msg, Srv, Action)**

ROS 2 提供了大量标准接口类型（如 geometry\_msgs、sensor\_msgs），但在实际项目中，我们常常需要定义自己的数据结构来满足特定需求。自定义接口允许我们为节点间的通信创建一种专有的、清晰的“语言”。

#### **核心示例：ROS 2 官方教程与 example\_interfaces**

12

官方教程详细阐述了创建和使用自定义接口的每一个步骤。

* **接口包的最佳实践**：推荐的做法是创建一个专门用于存放接口定义的包，通常以 \_interfaces 结尾（如 my\_robot\_interfaces）13。这个包不包含任何可执行代码，只负责定义数据结构。在其内部，分别创建 msg、srv 和 action 三个子目录。  
* **定义接口文件**：  
  * **.msg (消息)**：用于话题（Topic）通信，是一种单向的“发布-订阅”模式。例如，可以创建一个 Num.msg 文件，内容为 int64 num，用于传递一个整数 12。  
  * **.srv (服务)**：用于服务（Service）通信，是一种双向的“请求-响应”模式。一个 .srv 文件由 \--- 分隔为两部分，上半部分是请求的数据结构，下半部分是响应的数据结构。例如，AddThreeInts.srv 12。  
  * **.action (动作)**：用于动作（Action）通信，是一种用于长时间运行任务的复杂协议。它包含三个部分，同样由 \--- 分隔：目标（Goal）、结果（Result）和反馈（Feedback）。例如，Fibonacci.action 14。  
* **构建与使用**：定义好接口文件后，需要在包的 CMakeLists.txt 和 package.xml 文件中添加必要的依赖（如 rosidl\_default\_generators）和构建规则（rosidl\_generate\_interfaces）12。使用 colcon build 构建该接口包后，ROS 2 会自动为这些接口生成 C++ 和 Python 的代码。之后，在其他的 ROS 包中，就可以像使用标准接口一样，通过 from my\_robot\_interfaces.msg import Num 或 \#include "my\_robot\_interfaces/msg/num.hpp" 来导入和使用它们了 13。可以通过 ros2 interface show \<interface\_name\> 命令来验证接口是否已成功创建 12。

---

## **第四部分：融会贯通——您的第一个自主机器人项目**

至此，我们已经掌握了构建机器人模型、创建仿真环境以及使用 ROS 2 核心通信机制的全部知识。现在是时候将这些碎片化的技能整合起来，完成一个真正的、端到端的自主机器人项目了。我们将深入剖析一个经典的入门项目：**循墙机器人（Wall Follower）**。这个项目之所以经典，是因为它以最简洁的形式，囊括了移动机器人自主行为的全部核心要素，是检验和巩固所学知识的完美试金石。

### **核心仓库：mit-rss/wall\_follower\_sim**

16

这个由 MIT 提供的仓库是一个教学杰作。它不仅提供了一个功能完整的循墙机器人模拟器，更重要的是，它还包含了一套测试框架，引导学习者思考代码的鲁棒性和可靠性。我们也会参考 ssscassio/ros-wall-follower-2-wheeled-robot 2 仓库中的一些实现思想。

#### **项目解构**

该项目由两部分组成：仿真环境和控制节点。通过分析其启动文件 simulate.launch.xml 和 wall\_follower.launch.xml，我们可以看到，系统启动时会同时加载 Gazebo 世界、生成赛车模型、打开 RViz 可视化界面，并运行核心的 wall\_follower.py 控制节点。

#### **“感知-规划-行动”循环的实践**

wall\_follower.py 节点的逻辑完美地诠释了机器人学中最基础也是最重要的\*\*“感知-规划-行动”（Sense-Plan-Act）\*\*循环。

* 感知（Sense）  
  循环的第一步是感知环境。在代码中，这体现为创建了一个 ROS 2 订阅者，用于接收 /scan 话题上的 sensor\_msgs/msg/LaserScan 消息 16。激光雷达传感器会发布一个包含数百个距离测量值的数组。然而，并非所有数据点都是有用的。一个关键的步骤是数据处理：代码需要从这个庞大的数据数组中，“切分”出与机器人侧面墙壁相关的特定角度范围内的激光点，并从中计算出机器人与墙壁的当前距离。这个看似简单的筛选过程，是所有机器人传感器数据处理任务的缩影。  
* 规划（Plan）  
  在获得当前状态（即与墙的距离）后，机器人需要决策下一步该怎么做。这就是规划阶段。对于循墙机器人，其核心控制算法通常是一个简单的比例控制器（Proportional Controller） 2。其逻辑直观且强大：  
  1. 计算误差：$Error \= Desired\\\_Distance \- Current\\\_Distance$  
  2. 计算控制输出：$Steering\\\_Angle \= Proportional\\\_Gain \\times Error$

这里的 Desired\_Distance（期望与墙保持的距离）和 Proportional\_Gain（比例增益，决定了机器人对误差的反应有多“激进”）是这个算法的两个关键参数。它们正是应用上一部分所学知识的绝佳场景——将它们定义为 ROS 参数，使得我们可以在不修改代码的情况下，通过命令行或 YAML 文件轻松地进行调试和优化。

* 行动（Act）  
  规划出期望的转向角度后，最后一步就是将这个决策转化为对机器人的物理控制。在代码中，这体现为创建了一个 ROS 2 发布者，向 /drive 话题发布 ackermann\_msgs/msg/AckermannDriveStamped 类型的消息 16。这个消息包含了期望的速度和转向角度，仿真环境中的差分驱动插件会接收到这个指令，并驱动模拟的轮子转动。

这个完整的闭环系统——机器人的行动改变了它在世界中的位置，从而改变了它的传感器读数，新的读数又引发了新的规划和行动——正是自主控制的精髓。

完成这个项目，学习者所获得的远不止是编写了一个脚本。他们亲手实现了一个完整的“最小可行机器人”（Minimum Viable Robot）。这个项目迫使他们直面并解决一系列真实世界中的问题：如何从嘈杂的传感器数据中提取有效信息？如何确保激光雷达的坐标系与机器人控制指令的坐标系正确对齐？如何调整控制参数以获得平滑稳定的循墙行为？成功掌握这个项目，意味着学习者已经构建了解决更复杂自主导航任务所需的全套概念工具和实践信心。它不是众多示例中的一个，而是通往自主机器人世界的奠基性实践。

---

## **第五部分：规划您的航线——ROS 2 进阶之旅的下一步**

恭喜您！通过完成循墙机器人项目，您已经掌握了构建一个基本自主机器人的所有核心技能。您已经从一个被动的学习者，成长为一个能够独立解决问题的实践者。现在，您的知识版图已经建立，是时候向更广阔的领域探索了。接下来的学习路径，可以被看作是从**反应式控制**到**审议式控制**的跃迁。循墙机器人是一个无状态的、纯粹的反应式系统，它只根据当前的传感器输入做出瞬时反应。而更高级的机器人系统则是有状态的、审议式的，它们拥有对世界的内部模型（地图），并能够基于长期目标进行规划。

以下是一些关键的进阶主题，以及与之配套的、同样遵循“代码精炼、逻辑清晰”原则的开源仓库，它们将是您下一阶段学习的绝佳资源。

### **从反应式到审议式：进阶主题与仓库推荐**

#### **1\. 地图构建（SLAM）：创建世界模型**

* **概念**：机器人如何在未知环境中，仅通过自身传感器逐步构建出环境的地图，并同时确定自身在该地图中的位置。这就是即时定位与地图构建（SLAM）。其核心数据结构之一是**占据栅格地图（Occupancy Grid）**，它将空间划分为一个个小单元格，并存储每个单元格被障碍物占据的概率 17。  
* **推荐仓库**：  
  * ANYbotics/grid\_map 18：一个功能强大的 C++ 库，专门用于创建和管理多层栅格地图。虽然代码量较大，但其提供的 simple\_demo 是理解栅格地图数据结构的绝佳起点。  
  * YouTube 教程配套代码 19：对于希望用 Python 快速实现一个基础占据栅格地图发布节点的开发者来说，这是一个非常简洁明了的示例。

#### **2\. 定位（Localization）：知晓身在何方**

* **概念**：当机器人拥有一张预先构建好的地图时，如何仅通过传感器数据（如激光雷达）和里程计信息，来实时、精确地确定自己在这张地图上的位置。目前最主流的方法是**蒙特卡洛定位（MCL）**，也称为**粒子滤波（Particle Filter）** 20。  
* **推荐仓库**：  
  * debbynirwan/mcl 21：一个专门为教学目的而创建的 ROS 2 包，清晰地展示了如何在一个仿真机器人上从零开始实现粒子滤波定位算法。  
  * JMU-ROBOTICS-VIVA/particle\_filter 22：另一个优秀的纯 Python 粒子滤波实现，旨在作为 Nav2 标准定位节点的替代品，代码结构清晰，非常适合学习算法细节。

#### **3\. 路径规划（Path Planning）：寻找最优路径**

* **概念**：给定一张地图、一个起点和一个终点，如何计算出一条从起点到终点的、无碰撞的最优路径。经典的算法包括 Dijkstra 算法和 A\* 算法 23。  
* **推荐仓库**：  
  * HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving 25：一个以 Python 为主的项目，从零开始实现了包括 A\* 在内的多种路径规划算法，并结合 OpenCV 进行视觉处理，非常适合学习算法的实际应用。  
  * maker-ATOM/Path-Planning-Algorithms 26：一个专注于路径规划算法可视化的 ROS 2 仓库，提供了多种网格搜索算法（如 BFS、Dijkstra、A\*）的 Python 实现，便于直观理解算法的工作原理。

#### **4\. 任务管理（Task Management）：执行复杂行为**

* **概念**：当机器人需要执行一系列复杂的、有条件的任务时（例如，“先去A点，如果门开了就进去，否则去B点充电”），简单的控制循环难以胜任。**行为树（Behavior Trees）** 作为传统有限状态机（FSM）27 的现代替代方案，提供了一种更模块化、更具反应性的方式来编排复杂的机器人行为。  
* **推荐仓库**：  
  * BehaviorTree.CPP v4 与 ROS 2 集成文档 28：这是学习行为树在 ROS 2 中应用的基础。官方文档提供了将 ROS 2 的 Action 和 Service 封装为行为树节点的标准方法。  
  * polymathrobotics/ros2\_behavior\_tree\_example 29：一个简洁的 ROS 2 行为树示例，通过一个“Ping-Pong”的例子，清晰地展示了如何构建和运行一个包含自定义节点的行为树。  
  * Adlink-ROS/BT\_ros2 30：一个更贴近实际应用的示例，展示了如何使用行为树来控制机器人在 Gazebo 中执行导航任务，甚至集成了中断处理和视觉检测等高级功能。

#### **5\. 集成式学习平台**

* 推荐仓库：henki-robotics/robotics\_essentials\_ros2 31  
  这个仓库本身就是一门完整的 ROS 2 机器人学入门课程。它以一个名为 Andino 的机器人的仿真为核心，系统性地涵盖了从 ROS 2 基础、SLAM 地图构建到 Nav2 自主导航等一系列高级主题。当您完成了本指南前四个部分的学习后，这个仓库将是您进行系统性拔高训练的理想选择。

### **总结与学习路径图**

为了给您的 ROS 2 学习之旅提供一个清晰的参考地图，下表总结了本指南所规划的完整学习路径。您可以将此表作为一个检查清单，在完成每个阶段的学习后，回顾所掌握的核心概念和达成的学习目标。

| 学习阶段 | 推荐核心仓库 | 覆盖的核心 ROS 概念 | 主要学习目标 |
| :---- | :---- | :---- | :---- |
| 1\. 机器人建模 | ros/urdf\_tutorial 3 | URDF, Xacro, Links, Joints, Gazebo 标签 | 定义机器人的物理结构和仿真属性。 |
| 2\. 仿真环境 | MOGI-ROS/Week-3-4-Gazebo-basics 1 | Gazebo Worlds, 启动文件, 控制插件 | 将机器人模型集成到动态物理仿真中并实现基本控制。 |
| 3\. 核心概念 | 多个教程仓库 (例如 stretch\_ros\_tutorials 6) | Nodes, Topics, TF2, Parameters, 自定义接口 | 掌握 ROS 2 的基础软件构建模块和通信模式。 |
| 4\. 首个项目 | mit-rss/wall\_follower\_sim 16 | Subscribers, Publishers, 闭环控制, 测试 | 从零开始构建一个完整的、自主的“感知-规划-行动”应用。 |
| 5\. 进阶探索 | henki-robotics/robotics\_essentials\_ros2 31 | SLAM, 导航 (Nav2), 路径规划, 行为树 | 从简单的反应式控制过渡到复杂的、有状态的审议式系统。 |

机器人学是一个广阔而迷人的领域，而 ROS 2 是探索这个领域的强大工具。希望本指南能为您提供一条清晰、坚实的起点，并激发您持续学习和创造的热情。祝您在机器人开发的道路上一帆风顺！

#### **Works cited**

1. MOGI-ROS/Week-3-4-Gazebo-basics: Introduction to URDF and Gazebo Harmonic with ROS2 Jazzy \- GitHub, accessed October 19, 2025, [https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics](https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics)  
2. ssscassio/ros-wall-follower-2-wheeled-robot \- GitHub, accessed October 19, 2025, [https://github.com/ssscassio/ros-wall-follower-2-wheeled-robot](https://github.com/ssscassio/ros-wall-follower-2-wheeled-robot)  
3. ros/urdf\_tutorial \- GitHub, accessed October 19, 2025, [https://github.com/ros/urdf\_tutorial](https://github.com/ros/urdf_tutorial)  
4. Tutorial: Using a URDF in Gazebo, accessed October 19, 2025, [https://classic.gazebosim.org/tutorials/?tut=ros\_urdf](https://classic.gazebosim.org/tutorials/?tut=ros_urdf)  
5. Tutorial : Gazebo plugins in ROS, accessed October 19, 2025, [https://classic.gazebosim.org/tutorials?tut=ros\_gzplugins](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)  
6. Tf2 Broadcaster and Listener \- Stretch Documentation, accessed October 19, 2025, [https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example\_10/](https://docs.hello-robot.com/0.2/stretch-tutorials/ros2/example_10/)  
7. Tf2 Broadcaster and Listener \- Stretch Documentation, accessed October 19, 2025, [https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/example\_10/](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/example_10/)  
8. dottantgal/ROS2-dynamic-TF2-broadcaster \- GitHub, accessed October 19, 2025, [https://github.com/dottantgal/ROS2-dynamic-TF2-broadcaster](https://github.com/dottantgal/ROS2-dynamic-TF2-broadcaster)  
9. How to Use ROS 2 Parameters \- Foxglove, accessed October 19, 2025, [https://foxglove.dev/blog/how-to-use-ros2-parameters](https://foxglove.dev/blog/how-to-use-ros2-parameters)  
10. Understanding parameters — ROS 2 Documentation: Humble documentation, accessed October 19, 2025, [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)  
11. ROS2 Part 8 \- ROS2 Parameters in node classes, in Python and C++ \- RoboticsUnveiled, accessed October 19, 2025, [https://www.roboticsunveiled.com/ros2-parameters-python-and-cpp/](https://www.roboticsunveiled.com/ros2-parameters-python-and-cpp/)  
12. Creating custom msg and srv files — ROS 2 Documentation, accessed October 19, 2025, [https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)  
13. ROS2 Create Custom Message (Msg/Srv) \- The Robotics Back-End, accessed October 19, 2025, [https://roboticsbackend.com/ros2-create-custom-message/](https://roboticsbackend.com/ros2-create-custom-message/)  
14. Creating an action — ROS 2 Documentation: Foxy documentation, accessed October 19, 2025, [https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Creating-an-Action.html)  
15. ros2/example\_interfaces: Msg, Srv, etc. ROS interfaces used in examples \- GitHub, accessed October 19, 2025, [https://github.com/ros2/example\_interfaces](https://github.com/ros2/example_interfaces)  
16. mit-rss/wall\_follower\_sim: Drive forwards while maintaining a constant distance to a wall on either the left or right on a simulated racecar. \- GitHub, accessed October 19, 2025, [https://github.com/mit-rss/wall\_follower\_sim](https://github.com/mit-rss/wall_follower_sim)  
17. Lab 8: Building Occupancy Grids with TurtleBot \- GitHub, accessed October 19, 2025, [https://pages.github.berkeley.edu/EECS-106/fa21-site/assets/labs/Lab\_8\_\_Occupancy\_Grids.pdf](https://pages.github.berkeley.edu/EECS-106/fa21-site/assets/labs/Lab_8__Occupancy_Grids.pdf)  
18. ANYbotics/grid\_map: Universal grid map library for mobile robotic mapping \- GitHub, accessed October 19, 2025, [https://github.com/ANYbotics/grid\_map](https://github.com/ANYbotics/grid_map)  
19. ROS2 Occupancy Grid Node for Nav2 \- YouTube, accessed October 19, 2025, [https://www.youtube.com/watch?v=suqhnzIyq7w](https://www.youtube.com/watch?v=suqhnzIyq7w)  
20. amcl \- ROS Wiki, accessed October 19, 2025, [https://wiki.ros.org/amcl](https://wiki.ros.org/amcl)  
21. debbynirwan/mcl: This ROS2 package aims to demonstrate how the Particle Filter or Monte Carlo Localization is implemented in a real robot in a simulation world. \- GitHub, accessed October 19, 2025, [https://github.com/debbynirwan/mcl](https://github.com/debbynirwan/mcl)  
22. JMU-ROBOTICS-VIVA/particle\_filter: ROS2 Particle Filter \- GitHub, accessed October 19, 2025, [https://github.com/JMU-ROBOTICS-VIVA/particle\_filter](https://github.com/JMU-ROBOTICS-VIVA/particle_filter)  
23. Robot Navigation and Path Planning \- Rishi Khajuriwala, accessed October 19, 2025, [https://rkhajuriwala.github.io/assets/documents/Robot-Navigation-and-Path-planning.pdf](https://rkhajuriwala.github.io/assets/documents/Robot-Navigation-and-Path-planning.pdf)  
24. NavFn Planner — Nav2 1.0.0 documentation, accessed October 19, 2025, [https://docs.nav2.org/configuration/packages/configuring-navfn.html](https://docs.nav2.org/configuration/packages/configuring-navfn.html)  
25. HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving: Developing a maze solving robot in ROS2 that leverages information from a drone or Satellite's camera using OpenCV algorithms to find its path to the goal and solve the maze. \- GitHub, accessed October 19, 2025, [https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving](https://github.com/HaiderAbasi/ROS2-Path-Planning-and-Maze-Solving)  
26. ROS2 project to visualise path planning algorithms used in Robotics. \- GitHub, accessed October 19, 2025, [https://github.com/maker-ATOM/Path-Planning-Algorithms](https://github.com/maker-ATOM/Path-Planning-Algorithms)  
27. This code is an example for how to implement a simple State Machine using just standard Python (no library) \- GitHub, accessed October 19, 2025, [https://github.com/walkerdustin/simple-Python-Statemachine](https://github.com/walkerdustin/simple-Python-Statemachine)  
28. Integration with ROS2 | BehaviorTree.CPP, accessed October 19, 2025, [https://www.behaviortree.dev/docs/ros2\_integration/](https://www.behaviortree.dev/docs/ros2_integration/)  
29. polymathrobotics/ros2\_behavior\_tree\_example: Toy example for running a simple behavior tree with ROS2 \- GitHub, accessed October 19, 2025, [https://github.com/polymathrobotics/ros2\_behavior\_tree\_example](https://github.com/polymathrobotics/ros2_behavior_tree_example)  
30. Adlink-ROS/BT\_ros2: Behavior Tree that runs on ROS2 \- GitHub, accessed October 19, 2025, [https://github.com/Adlink-ROS/BT\_ros2](https://github.com/Adlink-ROS/BT_ros2)  
31. henki-robotics/robotics\_essentials\_ros2: Learn the basics of robotics through hands-on experience using ROS 2 and Gazebo simulation. \- GitHub, accessed October 19, 2025, [https://github.com/henki-robotics/robotics\_essentials\_ros2](https://github.com/henki-robotics/robotics_essentials_ros2)