

# **PX4开发综合学习路径：从基础概念到高级eVTOL飞控仿真**

## **第1节：解构PX4生态系统：基础概览**

在编写任何代码之前，必须首先建立一个坚实的概念框架。理解PX4的架构哲学至关重要，因为它决定了开发的方方面面。对于具备C++、RTOS和ROS背景的开发者而言，掌握这些核心概念将极大地加速学习进程。

### **1.1 分层架构：中间件与飞行栈**

PX4的软件由两个主要层次构成：**中间件（Middleware）和飞行栈（Flight Stack）** 1。这种分层设计是PX4能够轻松适配包括eVTOL在内的多种硬件和载具类型的关键所在 3。

* **中间件层**：这是一个通用的机器人技术层，其核心职责是硬件抽象和通信。它包含了用于各类传感器和外设的设备驱动程序，并提供了操作系统之上的通信机制。  
* **飞行栈层**：这一层包含了实现无人机自主飞行的特定算法，如制导、导航和控制（Guidance, Navigation, and Control, GNC）算法。

这种明确的职责分离使得PX4具有高度的可移植性和模块化特性。开发者可以专注于飞行栈中的特定算法，而不必担心底层硬件的复杂性。

### **1.2 响应式系统：使用uORB进行异步消息传递**

PX4并非一个单一、顺序执行的程序，而是一个**响应式系统（Reactive System）** 1。其所有功能都被划分为独立的、可互换的模块，这些模块之间通过异步消息传递进行通信。这种架构选择不仅仅是一个技术细节，它是PX4灵活性和鲁棒性的核心驱动力。

这种通信机制由\*\*uORB（Micro Object Request Broker）\*\*实现，它是一个轻量级的发布/订阅（Publish/Subscribe）消息总线 1。对于嵌入式开发者来说，这是最需要掌握的核心概念。该发布/订阅模型允许所有操作完全并行化，并以线程安全的方式进行，使整个系统由事件驱动。这种设计直接促成了PX4的核心开发循环：开发者可以编写一个新模块（例如，一个新的传感器驱动或自定义算法），该模块订阅现有的uORB主题并发布其处理结果。系统的其余部分无需知道这个新模块的存在，它们只关心uORB总线上的数据。因此，该架构极大地促进了迭代式开发和研究，这也是PX4成为领先无人机研究平台的原因.3 对于有志于eVTOL开发的工程师而言，这意味着可以专注于解决特定问题（如过渡逻辑），而无需重写整个飞行栈。

### **1.3 操作系统抽象：NuttX与POSIX兼容性**

PX4的运行依赖于一个实时操作系统（RTOS）。它主要运行在**NuttX**上，这是一个轻量级、高度可配置且符合POSIX（Portable Operating System Interface）标准的RTOS。同时，PX4也可以在Linux或其他类POSIX系统上运行 1。

开发者的RTOS背景在此处尤为重要。POSIX API为任务创建、调度和进程间通信提供了一个标准化的、熟悉的环境。PX4的中间件，特别是uORB，正是构建在这些坚实的RTOS服务之上。

### **1.4 外部通信：MAVLink协议**

\*\*MAVLink（Micro Air Vehicle Link）\*\*是PX4飞控与外部世界（如地面控制站或机载计算机）通信的主要协议 2。它是一种为资源受限系统设计的轻量级、仅头文件、二进制序列化协议。其数据包结构包含系统ID、组件ID和消息ID等字段，为后续章节中与ROS的集成提供了必要的背景知识 9。

## **第2节：开发者工坊：环境搭建与配置**

一个正确配置的开发环境是后续所有工作的基础，能够避免无数小时的故障排查。本节将提供在Ubuntu Linux上建立一个完整、功能齐全的开发环境的详尽步骤。

### **2.1 安装PX4开发工具链**

在Ubuntu 20.04或22.04上搭建工具链的最直接方法是使用官方提供的便利脚本 10。这个过程本身就是对PX4生态系统依赖关系的一次学习。

该脚本（ubuntu.sh）会自动安装所有关键依赖项，包括git、cmake、python3、用于构建NuttX目标的gcc-arm-none-eabi交叉编译器以及Gazebo等仿真工具 10。

### **2.2 克隆PX4-Autopilot代码仓库**

PX4的核心软件仓库是GitHub上的PX4/PX4-Autopilot 7。克隆该仓库时，必须使用--recursive标志，以确保所有必要的子模块（如NuttX RTOS和MAVLink库）都被正确下载 12。

Bash

git clone https://github.com/PX4/PX4-Autopilot.git \--recursive

\--recursive标志并非可选项，而是强制要求。这是PX4分层模块化设计的直接体现，因为核心项目依赖于其他独立的项目（NuttX、MAVLink等）。如果忘记此标志，构建将因缺少基本组件而失败。这个构建失败并非随机错误，而是项目模块化结构的必然结果。因此，安装过程本身就在教导开发者：PX4不是一个单一的整体代码库，而是多个专业软件项目的集成。

### **2.3 安装与配置QGroundControl (GCS)**

QGroundControl是用于载具设置、状态监控和任务规划的主要地面控制站软件 13。在Ubuntu上安装它，需要的步骤比仅仅下载一个AppImage文件要多 13。

**关键的预安装步骤包括：**

1. **授予串口访问权限**：将用户添加到dialout组，并重新登录以使权限生效。  
   Bash  
   sudo usermod \-aG dialout "$(id \-un)"

2. **禁用冲突的ModemManager**：通用操作系统中的ModemManager会占用串口，与机器人硬件通信产生冲突。  
   Bash  
   sudo systemctl mask \--now ModemManager.service

3. **安装GStreamer**：为支持视频流功能安装相关库。

### **2.4 为仿真（SITL）构建源代码**

首次编译PX4固件是验证整个工具链是否正确安装的最后一步。目标是构建一个用于软件在环（Software-in-the-Loop, SITL）仿真的版本。

Bash

make px4\_sitl\_default gazebo

该命令将为Gazebo仿真环境编译PX4固件 16。

## **第3节：首次飞行：掌握软件在环（SITL）仿真**

仿真是现代飞行控制系统开发的基石。它提供了一个安全、快速且经济高效的环境，用于测试从基本稳定性到复杂自主任务的所有内容。对于eVTOL的开发，这一环节尤为关键。

### **3.1 SITL与Gazebo简介**

SITL的概念是将完整的PX4飞行栈代码编译并在桌面计算机上运行，然后连接到一个基于物理引擎的模拟器（如Gazebo）中的虚拟载具 17。Gazebo是一个功能强大的3D模拟环境，特别适用于测试计算机视觉和避障算法，这些都是高级eVTOL的关键技术 17。

### **3.2 启动并控制标准多旋翼**

首次仿真将从一个标准的四旋翼无人机开始。执行以下命令可以一站式地完成构建、启动PX4 SITL以及打开带有x500四旋翼模型的Gazebo模拟器 19。

Bash

make px4\_sitl gz\_x500

启动后，QGroundControl应能自动连接到模拟的载具，允许通过游戏手柄或屏幕控件进行手动控制，并可通过MAVLink控制台执行如解锁和起飞等基本命令。

### **3.3 仿真eVTOL：标准VTOL与垂尾VTOL模型**

现在转向您最感兴趣的领域。PX4内置了对多种VTOL构型的支持。下表提供了一个快速参考，用于启动最相关的eVTOL仿真目标，将分散在文档中的信息整合到一个易于使用的资源中 19。

**表：常见的VTOL SITL仿真make命令**

| 载具类型 | make 命令 | 描述 |
| :---- | :---- | :---- |
| 标准VTOL（倾转旋翼/复合翼） | make px4\_sitl gz\_standard\_vtol | 一种在多旋翼和固定翼模式间转换的通用VTOL模型，是研究过渡逻辑的理想选择。 |
| 四旋翼垂尾（Tailsitter） | make px4\_sitl gz\_quadtailsitter | 一种垂直起飞，然后向前倾斜进行水平飞行的垂尾模型，使用差分推力进行控制。 |

此外，还存在更高级的仿真方案，例如使用X-Plane进行高保真空气动力学仿真 22，或使用MATLAB/Simulink进行硬件在环（HITL）和复杂环境可视化 23。

### **3.4 高级仿真技术**

SITL的make命令是一个高级抽象。要解锁高级、可定制和可扩展的仿真，关键在于理解其底层的启动脚本和环境变量 19。这使得开发者从仅仅*运行*仿真，转变为*设计*仿真实验。

* **无头模式（Headless Mode）**：通过添加HEADLESS=1前缀，可以在不启动Gazebo图形界面的情况下运行仿真，这对于自动化测试非常有用 18。  
  Bash  
  HEADLESS=1 make px4\_sitl gz\_standard\_vtol

* **分离启动**：可以分别启动PX4和Gazebo，从而在调试时只需重启轻量的PX4进程，大大缩短了调试周期 17。  
* **环境变量**：使用PX4\_GZ\_WORLD和PX4\_GZ\_MODEL\_POSE等环境变量，可以在不修改源代码的情况下自定义仿真世界和载具的初始位姿 19。这种能力对于多载具仿真或将PX4集成到CI/CD流水线中至关重要。

## **第4节：深入自驾仪核心：飞行控制算法剖析**

本节是学习计划的理论核心。我们将剖析使PX4驱动的载具能够飞行的基本算法，将抽象的控制理论与飞行栈中的具体实现联系起来。

### **4.1 状态估计：扩展卡尔曼滤波器（EKF2）**

状态估计器的作用是融合来自多个传感器的带有噪声、不完整且有时间延迟的数据，从而得出一个单一、连贯且可靠的载具状态（姿态、位置、速度）估计 24。

**EKF2**是PX4中默认且推荐的状态估计器 24。它融合了多种传感器数据：

* **核心数据源**：IMU（用于状态预测）、磁力计（用于偏航角）、气压计/GPS/测距仪（用于高度）。  
* **辅助数据源**：GPS（用于位置/速度）、光流、视觉惯性里程计（VIO）、空速管 25。

EKF2\_AID\_MASK参数是启用或禁用特定传感器融合源的总开关，对于为不同环境（如GPS拒止环境）配置系统至关重要。

### **4.2 串级控制架构**

PX4采用分层的PID控制架构，这是其稳定性的基石 26。这个串级控制器从外环到内环依次为：

1. **位置控制器（P环）**：接收位置设定点和当前位置估计，输出期望的速度。关键参数：MPC\_XY\_P、MPC\_Z\_P 28。  
2. **速度控制器（PID环）**：接收速度设定点和当前速度估计，输出期望的加速度，该加速度随后被转换为期望的姿态和推力。关键参数：MPC\_XY\_VEL\_P\_ACC等 28。  
3. **姿态控制器（P环）**：接收姿态设定点（以四元数形式）和当前姿态估计，输出期望的角速率。关键参数：MC\_ROLL\_P、MC\_PITCH\_P 28。  
4. **角速率控制器（PID环）**：最内层、最快的控制环。接收角速率设定点和陀螺仪的当前角速率测量值，输出电机指令（通过混控器）。关键参数：MC\_ROLLRATE\_P/I/D等 26。

### **4.3 在仿真中进行PID调参**

PID调参过程不仅是为了提升性能，它更是一个基础的系统诊断工具。每个控制环的行为都直接反映了底层硬件和传感器数据的健康状况。

调参必须遵循**从内到外**（角速率 \-\> 姿态 \-\> 速度 \-\> 位置）的原则 27。这是因为一个稳定的外环永远无法建立在一个不稳定的内环之上。使用QGroundControl中的实时绘图工具，可以通过遥控器摇杆产生阶跃响应，并观察Setpoint与Response曲线来调整P、I、D增益，以获得快速且无超调的响应 27。

例如，如果角速率环（MC\_ROLLRATE\_P/D）即使在低增益下也无法调定，出现剧烈振荡，这通常表明问题不在于调参本身，而在于控制器的输入。D项会放大噪声 27，因此无法控制的角速率振荡直接指向高频噪声进入系统，其来源很可能是电机/螺旋桨的物理振动传递到了IMU。反之，如果内环（角速率和姿态）调校完美，但位置环（MPC\_XY\_P）出现振荡或漂移，那么问题很可能出在EKF2的位置估计质量上，这可能指向GPS信号差、磁力计干扰或影响加速度计的高振动。因此，分层调参过程是系统性地验证从物理机架（角速率环）到外部传感器（位置环）整个系统的有效方法。

## **第5节：从用户到贡献者：PX4开发实战指南**

本节将理论付诸实践，利用您的C++技能为PX4创建自定义软件。

### **5.1 创建自定义应用：“Hello Sky”教程**

我们将详细解读经典的第一个应用程序教程“Hello Sky” 29。这个例子是整个PX4开发模式的缩影，涵盖了以下核心步骤：

1. **创建模块目录和CMakeLists.txt文件**：定义模块并将其添加到构建系统中。  
2. **编写主函数**：实现px4\_simple\_app\_main入口函数。  
3. **订阅uORB主题**：使用orb\_subscribe()订阅一个现有主题（如sensor\_combined）。  
4. **等待新数据**：使用poll()系统调用高效地等待新数据到达。  
5. **拷贝数据**：使用orb\_copy()将新数据复制到本地结构体中。  
6. **广播新主题**：使用orb\_advertise()声明一个新的uORB主题。  
7. **发布数据**：使用orb\_publish()将处理后的数据发布到新主题上。

### **5.2 深入理解uORB**

在“Hello Sky”的基础上，我们将正式讲解如何定义一个新的.msg文件，将其添加到构建系统，并使用自动生成的C++头文件 5。uORB消息系统作为一个强大的抽象层，强制实现了数据生产者（驱动、估计器）和数据消费者（控制器、日志记录器）之间的清晰分离。

这种强制解耦使得系统非常健壮和可扩展。例如，开发者可以更换一个物理IMU传感器及其驱动。只要新的驱动发布相同的sensor\_combined uORB消息，飞行栈的其余部分——EKF2、控制器等——将无需任何修改即可正常工作。这对于硬件集成和测试具有深远影响。

此外，还将介绍在MAVLink控制台中调试uORB的实用命令：

* uorb top：查看所有主题的发布频率、订阅者数量等统计信息。  
* listener \<topic\_name\>：实时监听并打印指定主题的消息内容 5。

### **5.3 驱动开发入门**

本节将概述PX4的驱动开发框架 30。其核心原则是：驱动程序的主要职责是通过I2C、SPI等总线与硬件交互，并**发布一个标准化的uORB消息** 31。

同时，将解释DeviceID的概念及其重要性。这是一个唯一的标识符，用于将校准和配置参数与特定的物理传感器关联起来 31。

## **第6节：释放自主性：使用ROS进行机外控制**

本节将直接利用您已有的ROS专业知识，展示如何从机载计算机上对PX4飞控进行指令控制，这是通往高级自主性的大门。

### **6.1 桥梁：MAVROS与MAVLink协议**

**MAVROS**是官方的ROS软件包，它充当一个桥梁，在MAVLink消息和ROS主题/服务之间进行转换 16。我们将介绍如何安装并启动MAVROS，并将其连接到SITL实例。

### **6.2 机外控制状态机**

我们将详细解读标准的MAVROS机外控制示例代码（offb\_node.cpp） 33。这个过程背后体现了PX4核心的安全哲学：飞控必须在交出控制权之前，验证机外计算机的健康状况和就绪状态。

PX4的commander模块为安全起见，要求遵循一个不可协商的序列：

1. **等待飞控连接**：节点必须首先确认正在接收来自飞控的心跳包 (while(\!current\_state.connected))。  
2. **流式传输设定点**：在请求进入机外模式**之前**，节点必须开始向/mavros/setpoint\_position/local等主题发布设定点。这是一个至关重要的安全特性。  
3. **请求机外模式**：使用/mavros/set\_mode ROS服务请求切换到“OFFBOARD”模式。  
4. **解锁载具**：一旦进入机外模式，使用/mavros/cmd/arming ROS服务来解锁载具。

这个看似僵化的程序并非技术上的怪癖，而是一个经过深思熟虑的、稳健的安全协议。通过要求首先接收到稳定的设定点数据流，PX4 commander可以验证通信链路是活跃的，并且机外节点是健康和响应的。

此外，PX4内部存在一个500毫秒的超时机制 33。如果设定点的数据流中断超过这个时间，commander会假定机外链接丢失或节点崩溃，并安全地回退到上一个模式（如位置保持），这是一个防止失控的“死人开关”。

### **6.3 ROS2与未来**

我们将简要讨论使用ROS2的现代方法，它通过px4\_ros\_com软件包和Fast RTPS/DDS桥接器与PX4通信 12。虽然MAVROS（ROS1）仍然非常流行，但了解这种新架构对于未来的项目很重要。

## **第7节：提升专业技能：定制化与未来方向**

本节为从标准开发转向更高级、更专业的领域提供了一份路线图，尤其与eVTOL研究相关。

### **7.1 创建新的飞行模式**

我们将总结在PX4中添加新飞行模式的复杂过程，并将其与编写高级应用程序区分开来 37。这是一个高级主题，涉及到修改核心飞行栈：

1. 在commander\_state.msg中定义新的状态。  
2. 在commander.cpp和state\_machine\_helper.cpp中添加逻辑以处理到新模式的转换。  
3. 在vehicle\_status.msg中定义新的导航状态。  
4. 在vehicle\_control\_mode.msg中设置适当的控制标志。  
5. 实现实际的飞行行为，通常是在位置控制器中作为一个新的“飞行任务（Flight Task）” 39。

“应用程序”和“飞行模式”之间的区别揭示了PX4开发的两个主要层次。“应用开发者”将已有的PX4架构作为一个平台来使用，而“飞行模式开发者”则是在修改架构本身。理解这种区别对于开发者正确地确定工作范围和理解其代码变更的影响至关重要。

### **7.2 eVTOL控制的特殊考量**

最后，我们将简要提及在eVTOL开发中会遇到的独特挑战：

* **过渡逻辑**：在垂直（多旋翼）和水平（固定翼）飞行之间平稳过渡所需的复杂软件和控制逻辑，这是一个主要的研发领域。  
* **控制分配（混控）**：VTOL的混控文件更为复杂，因为它们必须管理不同的执行器组（如垂直升力电机与前向推力电机和控制舵面），并在过渡期间进行融合。  
* **传感器使用**：空速管对于稳定的固定翼飞行以及触发飞行过渡至关重要 4。

## **结论**

对于具备C++、RTOS和ROS基础的开发者而言，入门PX4并深入到eVTOL仿真与算法开发领域是一条清晰且可行的路径。本报告提出的学习计划，从理解其独特的响应式和分层架构开始，到建立一个稳固的开发与仿真环境，再到剖析其核心的状态估计算法和串级PID控制器，旨在为您打下坚实的基础。

实践是关键。通过“Hello Sky”教程和驱动开发入门，您可以将理论知识转化为实际的编码能力。利用您已有的ROS经验，通过MAVROS进行机外控制开发，将是您通往高级自主性的快车道。最终，无论是为eVTOL开发复杂的过渡逻辑，还是创建全新的飞行模式，您都将具备解决这些高级挑战所需的知识体系。

成功的关键在于系统性地学习，并始终将理论与仿真实践相结合。PX4是一个庞大而活跃的开源项目，其深度和广度既是挑战也是机遇。遵循本报告规划的路径，您将能够高效地导航这个生态系统，并最终为下一代空中交通工具（如eVTOL）的发展做出贡献。

## **附录：精选资源与代码仓库**

本附录提供了一个快速参考指南，列出了在学习过程中最重要的软件仓库和文档。

**表：关键软件与代码仓库**

| 组件 | 代码仓库 | URL | 描述 |
| :---- | :---- | :---- | :---- |
| **PX4自驾仪** | PX4/PX4-Autopilot | [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | 核心飞行栈与中间件源代码 7。 |
| **QGroundControl** | mavlink/qgroundcontrol | [https://github.com/mavlink/qgroundcontrol](https://github.com/mavlink/qgroundcontrol) | 官方地面控制站软件 15。 |
| **MAVROS (ROS1)** | mavlink/mavros | [https://github.com/mavlink/mavros](https://github.com/mavlink/mavros) | 用于机外控制的官方MAVLink到ROS1桥接器 34。 |
| **PX4 ROS2桥接器** | PX4/px4\_ros\_com | [https://github.com/PX4/px4\_ros\_com](https://github.com/PX4/px4_ros_com) | 通过DDS桥接器连接PX4与ROS2的现代接口 36。 |
| **PX4消息定义** | PX4/px4\_msgs | [https://github.com/PX4/px4\_msgs](https://github.com/PX4/px4_msgs) | 与内部uORB消息对应的ROS/ROS2消息定义 36。 |
| **SITL Gazebo** | PX4/PX4-SITL\_gazebo-classic | ([https://github.com/PX4/PX4-SITL\_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic)) | 用于Gazebo模拟器的插件、模型和世界文件 36。 |

**表：核心学习资源**

| 资源 | URL | 描述 |
| :---- | :---- | :---- |
| PX4开发者指南 | [https://docs.px4.io/main/en/development/development.html](https://docs.px4.io/main/en/development/development.html) | 软件开发者修改PX4的主要资源。 |
| PX4用户指南 | [https://docs.px4.io/main/en/](https://docs.px4.io/main/en/) | 理解载具设置、配置和飞行模式的基础 40。 |
| PX4官方网站 | [https://px4.io/](https://px4.io/) | 高层概览、新闻和社区链接 3。 |
| MAVLink协议指南 | [https://mavlink.io/en/](https://mavlink.io/en/) | MAVLink协议的官方文档。 |
| ROS Wiki (MAVROS) | [http://wiki.ros.org/mavros](http://wiki.ros.org/mavros) | MAVROS软件包的文档，包括主题和服务信息 35。 |

#### **Works cited**

1. PX4 Architectural Overview | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/concept/architecture](https://docs.px4.io/main/en/concept/architecture)  
2. Architectural Overview · PX4 Development Guide \- shnuzxd, accessed October 26, 2025, [https://shnuzxd.gitbooks.io/px4-development-guide/en/concept/architecture.html](https://shnuzxd.gitbooks.io/px4-development-guide/en/concept/architecture.html)  
3. PX4 Autopilot: Open Source Autopilot for Drones, accessed October 26, 2025, [https://px4.io/](https://px4.io/)  
4. Basic Concepts | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/getting\_started/px4\_basic\_concepts](https://docs.px4.io/main/en/getting_started/px4_basic_concepts)  
5. uORB Messaging | PX4 User Guide (v1.12), accessed October 26, 2025, [https://docs.px4.io/v1.12/en/middleware/uorb.html](https://docs.px4.io/v1.12/en/middleware/uorb.html)  
6. uORB Messaging | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/middleware/uorb](https://docs.px4.io/main/en/middleware/uorb)  
7. PX4 Autopilot Software \- GitHub, accessed October 26, 2025, [https://github.com/PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)  
8. Journal Paper \- Micro Air Vehicle Link (MAVLink) in a Nutshell: A Survey \- CISTER, accessed October 26, 2025, [https://cister-labs.pt/docs/micro\_air\_vehicle\_link\_(mavlink)\_in\_a\_nutshell\_\_a\_survey/1551/view.pdf](https://cister-labs.pt/docs/micro_air_vehicle_link_\(mavlink\)_in_a_nutshell__a_survey/1551/view.pdf)  
9. MAVLink \- Wikipedia, accessed October 26, 2025, [https://en.wikipedia.org/wiki/MAVLink](https://en.wikipedia.org/wiki/MAVLink)  
10. Set up PX4 Tool Chain on Ubuntu 20.04 and 22.04 \- MATLAB ..., accessed October 26, 2025, [https://www.mathworks.com/help/uav/px4/ug/setting-px4-toolchain-ubuntu.html](https://www.mathworks.com/help/uav/px4/ug/setting-px4-toolchain-ubuntu.html)  
11. Linux · PX4 Developer Guide \- jalpanchal1, accessed October 26, 2025, [https://jalpanchal1.gitbooks.io/px4-developer-guide/en/setup/dev\_env\_linux.html](https://jalpanchal1.gitbooks.io/px4-developer-guide/en/setup/dev_env_linux.html)  
12. How to setup PX4 SITL with ROS2 and XRCE-DDS Gazebo simulation on Ubuntu 22, accessed October 26, 2025, [https://kuat-telegenov.notion.site/How-to-setup-PX4-SITL-with-ROS2-and-XRCE-DDS-Gazebo-simulation-on-Ubuntu-22-e963004b701a4fb2a133245d96c4a247](https://kuat-telegenov.notion.site/How-to-setup-PX4-SITL-with-ROS2-and-XRCE-DDS-Gazebo-simulation-on-Ubuntu-22-e963004b701a4fb2a133245d96c4a247)  
13. Download and Install | QGC Guide, accessed October 26, 2025, [https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting\_started/download\_and\_install.html](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)  
14. x-tools-author/siyiqgroundcontrol \- GitHub, accessed October 26, 2025, [https://github.com/x-tools-author/siyiqgroundcontrol](https://github.com/x-tools-author/siyiqgroundcontrol)  
15. mavlink/qgroundcontrol: Cross-platform ground control station for drones (Android, iOS, Mac OS, Linux, Windows) \- GitHub, accessed October 26, 2025, [https://github.com/mavlink/qgroundcontrol](https://github.com/mavlink/qgroundcontrol)  
16. The PX4 Drone Control Stack \- Nick Rotella, accessed October 26, 2025, [https://nrotella.github.io/journal/px4-drone-control-stack.html](https://nrotella.github.io/journal/px4-drone-control-stack.html)  
17. Gazebo Simulation · PX4 Developer Guide \- jalpanchal1, accessed October 26, 2025, [https://jalpanchal1.gitbooks.io/px4-developer-guide/en/simulation/gazebo.html](https://jalpanchal1.gitbooks.io/px4-developer-guide/en/simulation/gazebo.html)  
18. Gazebo Simulation | PX4 User Guide (v1.12), accessed October 26, 2025, [https://docs.px4.io/v1.12/en/simulation/gazebo.html](https://docs.px4.io/v1.12/en/simulation/gazebo.html)  
19. Gazebo Simulation | PX4 User Guide (v1.14), accessed October 26, 2025, [https://docs.px4.io/v1.14/en/sim\_gazebo\_gz/](https://docs.px4.io/v1.14/en/sim_gazebo_gz/)  
20. Gazebo Simulation \- PX4 User Guide \- GitBook, accessed October 26, 2025, [https://px4.gitbook.io/px4-user-guide/development/simulation/sim\_gazebo\_gz](https://px4.gitbook.io/px4-user-guide/development/simulation/sim_gazebo_gz)  
21. Gazebo Vehicles | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/sim\_gazebo\_gz/vehicles](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles)  
22. PX4 Autonomously Controls Alia-250 eVTOL in X-Plane SITL | PX4-XPlane v1.1.0, accessed October 26, 2025, [https://www.youtube.com/watch?v=mjNmpx4Zpww](https://www.youtube.com/watch?v=mjNmpx4Zpww)  
23. Visualize PX4 Hardware-in-the-Loop (HITL) Simulation with VTOL UAV Over Urban Environment \- MATLAB & Simulink \- MathWorks, accessed October 26, 2025, [https://www.mathworks.com/help/uav/ug/visualize-PX4-HITL-with-VTOL-UAV-in-urban-environment.html](https://www.mathworks.com/help/uav/ug/visualize-PX4-HITL-with-VTOL-UAV-in-urban-environment.html)  
24. Switching State Estimators | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/advanced/switching\_state\_estimators](https://docs.px4.io/main/en/advanced/switching_state_estimators)  
25. ecl EKF · px4-devguide \- bkueng, accessed October 26, 2025, [https://bkueng.gitbooks.io/px4-devguide/en/tutorials/tuning\_the\_ecl\_ekf.html](https://bkueng.gitbooks.io/px4-devguide/en/tutorials/tuning_the_ecl_ekf.html)  
26. Multicopter PID Tuning Guide · px4-user-guide \- bkueng, accessed October 26, 2025, [https://bkueng.gitbooks.io/px4-user-guide/en/advanced\_config/pid\_tuning\_guide\_multicopter.html](https://bkueng.gitbooks.io/px4-user-guide/en/advanced_config/pid_tuning_guide_multicopter.html)  
27. Multicopter PID Tuning Guide (Manual/Basic) | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/config\_mc/pid\_tuning\_guide\_multicopter\_basic](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter_basic)  
28. Controller Diagrams | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/flight\_stack/controller\_diagrams](https://docs.px4.io/main/en/flight_stack/controller_diagrams)  
29. First Application Tutorial (Hello Sky) | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/modules/hello\_sky](https://docs.px4.io/main/en/modules/hello_sky)  
30. Tutorials · PX4 Developer Guide \- jalpanchal1, accessed October 26, 2025, [https://jalpanchal1.gitbooks.io/px4-developer-guide/content/en/tutorials/tutorials.html](https://jalpanchal1.gitbooks.io/px4-developer-guide/content/en/tutorials/tutorials.html)  
31. Driver Development | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/middleware/drivers](https://docs.px4.io/main/en/middleware/drivers)  
32. mavros 2.9.0 documentation \- ROS documentation, accessed October 26, 2025, [https://docs.ros.org/en/iron/p/mavros/](https://docs.ros.org/en/iron/p/mavros/)  
33. MAVROS Offboard Example · PX4 Developer Guide \- jalpanchal1, accessed October 26, 2025, [https://jalpanchal1.gitbooks.io/px4-developer-guide/en/ros/mavros\_offboard.html](https://jalpanchal1.gitbooks.io/px4-developer-guide/en/ros/mavros_offboard.html)  
34. mavros \- ROS Repository Overview, accessed October 26, 2025, [https://index.ros.org/r/mavros/](https://index.ros.org/r/mavros/)  
35. mavros \- ROS Wiki, accessed October 26, 2025, [https://wiki.ros.org/mavros](https://wiki.ros.org/mavros)  
36. PX4 repositories \- GitHub, accessed October 26, 2025, [https://github.com/orgs/PX4/repositories](https://github.com/orgs/PX4/repositories)  
37. Flight Mode Configuration | PX4 Guide (main), accessed October 26, 2025, [https://docs.px4.io/main/en/config/flight\_mode](https://docs.px4.io/main/en/config/flight_mode)  
38. PX4 Research Log \[10\] – Adding a new flight mode (Preparation ..., accessed October 26, 2025, [https://uav-lab.org/2016/11/07/px4-research-log-10-adding-a-new-flight-mode-preparation/](https://uav-lab.org/2016/11/07/px4-research-log-10-adding-a-new-flight-mode-preparation/)  
39. SITL new flight mode \- PX4 Discussion Forum, accessed October 26, 2025, [https://discuss.px4.io/t/sitl-new-flight-mode/9485](https://discuss.px4.io/t/sitl-new-flight-mode/9485)  
40. PX4 Autopilot User Guide (v1.11.0), accessed October 26, 2025, [https://docs.px4.io/v1.11/en/](https://docs.px4.io/v1.11/en/)



好的，没问题。基于您的背景和学习目标，我为您量身定制了一份详尽的PX4入门学习计划。该计划从理论基础开始，逐步深入到仿真实践和代码开发，并为每个阶段提供了具体的学习资料链接。

### **PX4 eVTOL入门学习计划**

本计划分为六个阶段，旨在系统性地引导您从理解PX4的核心架构到最终能够进行eVTOL飞控算法的仿真与开发。

-----

#### **第一阶段：基础入门与环境搭建 (建议时间：1周)**

此阶段的目标是建立对PX4生态系统的宏观理解，并成功搭建功能完备的开发与仿真环境。

  * **1.1 理解PX4架构**

      * **学习目标**：掌握PX4的分层、响应式架构，理解中间件（Middleware）、飞行栈（Flight Stack）以及核心的uORB异步消息机制 [1, 2]。这对您理解后续代码的组织方式至关重要。
      * **学习资料**：
          * [PX4 Architectural Overview](https://docs.px4.io/main/en/concept/architecture.html) [1]

  * **1.2 在Ubuntu上搭建开发工具链**

      * **学习目标**：使用官方脚本在Ubuntu 20.04/22.04上安装所有必要的编译工具、交叉编译器和仿真器依赖 [3, 4]。
      * **学习资料**：
          * ([https://docs.px4.io/v1.14/en/dev\_setup/dev\_env\_linux\_ubuntu.html](https://docs.px4.io/v1.14/en/dev_setup/dev_env_linux_ubuntu.html)) [4]

  * **1.3 克隆核心代码仓库**

      * **学习目标**：从GitHub克隆PX4的核心代码库，并理解`--recursive`参数的重要性。
      * **软件仓库**：
          * **PX4 Autopilot**: `git clone https://github.com/PX4/PX4-Autopilot.git --recursive` [5]
          * **QGroundControl**: [https://github.com/mavlink/qgroundcontrol](https://github.com/mavlink/qgroundcontrol) [6]
          * **MAVROS (for ROS1)**: [https://github.com/mavlink/mavros](https://github.com/mavlink/mavros) [7]

  * **1.4 安装与配置QGroundControl地面站**

      * **学习目标**：安装QGroundControl并完成必要的系统配置，如串口权限设置和禁用ModemManager，以确保其能与仿真或硬件正常通信 [8]。
      * **学习资料**：
          * [QGroundControl Installation Guide](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) [8]

-----

#### **第二阶段：软件在环(SITL)仿真初体验 (建议时间：1周)**

此阶段将理论付诸实践，通过仿真环境安全、快速地熟悉PX4的基本操作和飞行控制。

  * **2.1 首次编译与运行SITL**

      * **学习目标**：编译并启动一个标准的四旋翼SITL仿真实例，验证开发环境是否配置正确，并学习如何通过QGroundControl连接到仿真无人机 [9]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/sim\_gazebo\_classic/](https://docs.px4.io/main/en/sim_gazebo_classic/)) [9]

  * **2.2 掌握SITL仿真命令与Gazebo环境**

      * **学习目标**：学习不同的`make`目标，理解如何启动无头(headless)仿真，以及如何通过环境变量自定义仿真世界和无人机初始位置 [10, 11]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/sim\_gazebo\_gz/](https://docs.px4.io/main/en/sim_gazebo_gz/)) [11]

  * **2.3 仿真eVTOL模型**

      * **学习目标**：启动PX4内置的标准VTOL（复合翼/倾转旋翼）和垂尾(Tailsitter)模型，为后续的eVTOL算法研究做准备 [12]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/sim\_gazebo\_gz/vehicles.html](https://docs.px4.io/main/en/sim_gazebo_gz/vehicles.html)) [12]
          * **标准VTOL仿真命令**: `make px4_sitl gz_standard_vtol` [13]
          * **垂尾VTOL仿真命令**: `make px4_sitl gz_quadtailsitter` [12]

-----

#### **第三阶段：核心概念与算法理论 (建议时间：2周)**

深入PX4的软件核心，理解其关键模块和控制算法的原理。

  * **3.1 深入uORB消息机制**

      * **学习目标**：彻底理解uORB的发布/订阅模型。学习如何定义一个新的`.msg`文件，以及如何在代码中发布和订阅主题 [14, 15]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/middleware/uorb.html](https://docs.px4.io/main/en/middleware/uorb.html)) [15]
          * ([https://docs.px4.io/v1.14/en/msg\_docs/](https://docs.px4.io/v1.14/en/msg_docs/)) [16]

  * **3.2 理解MAVLink通信协议**

      * **学习目标**：了解MAVLink协议的结构和作用，它是PX4与地面站、ROS节点等外部系统通信的桥梁 [17, 18]。
      * **学习资料**：
          * (https://mavlink.io/en/)
          * [PX4 MAVLink Integration Guide](https://docs.px4.io/main/en/mavlink/index.html) [19]

  * **3.3 剖析状态估计 (EKF2)**

      * **学习目标**：理解扩展卡尔曼滤波器(EKF2)如何融合多传感器数据（IMU、GPS、磁力计等）来提供准确的姿态、位置和速度估计 [20, 21]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/advanced\_config/ecl\_ekf.html](https://www.google.com/search?q=https://docs.px4.io/main/en/advanced_config/ecl_ekf.html))
          * ([https://docs.px4.io/main/en/advanced/switching\_state\_estimators.html](https://docs.px4.io/main/en/advanced/switching_state_estimators.html)) [20]

  * **3.4 剖析串级PID控制架构**

      * **学习目标**：掌握PX4从外环到内环的位置、速度、姿态、角速率控制器架构。理解PID调参的基本原则（从内到外） [22, 23]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/flight\_stack/controller\_diagrams.html](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html)) [23]
          * ([https://docs.px4.io/main/en/config\_mc/pid\_tuning\_guide\_multicopter\_basic.html](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter_basic.html)) [22]

-----

#### **第四阶段：C++开发实践 (建议时间：2周)**

利用您的C++背景，开始编写和修改PX4代码。

  * **4.1 编写第一个PX4应用 ("Hello Sky")**

      * **学习目标**：跟随官方教程，完成一个从订阅传感器数据到发布自定义uORB主题的完整应用。这是掌握PX4开发模式的最佳实践 [24]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/modules/hello\_sky.html](https://docs.px4.io/main/en/modules/hello_sky.html)) [24]

  * **4.2 驱动开发入门**

      * **学习目标**：了解PX4驱动程序的基本框架，核心原则是将硬件数据读取并发布为标准的uORB消息 [25]。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/middleware/drivers.html](https://docs.px4.io/main/en/middleware/drivers.html)) [25]

-----

#### **第五阶段：ROS集成与机外控制 (建议时间：2周)**

发挥您的ROS优势，实现高级自主控制。

  * **5.1 MAVROS安装与配置**

      * **学习目标**：安装并配置MAVROS，使其能够连接到PX4 SITL实例，并桥接MAVLink消息与ROS主题/服务 [26]。
      * **学习资料**：
          * ([https://docs.px4.io/v1.13/zh/ros/mavros\_installation.html](https://docs.px4.io/v1.13/zh/ros/mavros_installation.html)) [26]

  * **5.2 理解并实践机外控制 (Offboard Control)**

      * **学习目标**：深入分析官方的`offb_node.cpp`示例，理解进入Offboard模式所需的安全握手协议（先流式发送设定点，再请求模式切换），并成功在仿真中复现 [27, 28]。
      * **学习资料**：
          * ([https://docs.px4.io/v1.12/en/ros/mavros\_offboard.html](https://docs.px4.io/v1.12/en/ros/mavros_offboard.html)) [28]

  * **5.3 (选学) 探索ROS2接口**

      * **学习目标**：了解通过uXRCE-DDS桥接器实现PX4与ROS2通信的现代架构。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/ros2/user\_guide.html](https://docs.px4.io/main/en/ros2/user_guide.html)) [29]

-----

#### **第六阶段：进阶方向与eVTOL专题 (持续学习)**

在掌握基础后，您可以开始探索更高级和专业的领域。

  * **6.1 创建新的飞行模式**

      * **学习目标**：了解添加新飞行模式所涉及的步骤，这通常比编写独立应用更复杂，需要修改`commander`等核心模块 [30]。
      * **学习资料**：
          * ([https://uav-lab.org/2016/11/07/px4-research-log-10-adding-a-new-flight-mode-preparation/](https://uav-lab.org/2016/11/07/px4-research-log-10-adding-a-new-flight-mode-preparation/)) [30]
          * ([https://discuss.px4.io/t/sitl-new-flight-mode/9485](https://discuss.px4.io/t/sitl-new-flight-mode/9485)) [31]

  * **6.2 eVTOL过渡逻辑与混控**

      * **学习目标**：这是eVTOL开发的核心挑战。此阶段需要您深入阅读`VTOL Attitude Control`模块的源代码，并结合仿真进行实验，理解从多旋翼到固定翼模式转换的控制逻辑。
      * **学习资料**：
          * ([https://docs.px4.io/main/en/frames\_vtol/](https://docs.px4.io/main/en/frames_vtol/))
          * 在`PX4-Autopilot/src/modules/vtol_att_control/`目录下深入研究代码。

祝您学习顺利！PX4社区非常活跃，遇到问题时，官方的([https://discuss.px4.io/)和(https://discord.com/invite/dronecode)都是寻求帮助的好地方](https://discuss.px4.io/)和([https://discord.com/invite/dronecode)都是寻求帮助的好地方](https://discord.com/invite/dronecode)都是寻求帮助的好地方))。
