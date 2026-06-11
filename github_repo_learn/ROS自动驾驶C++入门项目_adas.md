

# **面向C++开发者的ROS自动驾驶开源项目深度解析与进阶指南**

## **第一章：开源自动驾驶的技术版图：C++开发者的第一份入门指南**

对于期望进入自动驾驶领域的C++开发者而言，首要任务是建立一个清晰的技术版图。这个领域由多个关键技术支柱构成，包括机器人操作系统（ROS）、模块化的自动驾驶软件栈以及高保真度的仿真环境。理解这些组件如何协同工作，以及C++在其中扮演的核心角色，是成功驾驭后续复杂项目的先决条件。

### **1.1 ROS生态系统：为您的项目选择合适的基石 (ROS 1 vs. ROS 2\)**

机器人操作系统（Robot Operating System, ROS）并非传统意义上的操作系统，而是一个为机器人软件开发提供服务的开源元操作系统（meta-operating system）1。它提供了硬件抽象、低层设备控制、常用功能实现、进程间消息传递以及包管理等功能 1。对于自动驾驶系统而言，ROS是连接感知、规划、控制等各个软件模块的神经网络。

#### **核心概念：ROS世界的通用语言**

在深入探讨版本差异之前，必须掌握ROS的基本词汇。整个系统由一系列独立运行的程序——**节点 (Nodes)** 构成。节点之间通过**话题 (Topics)** 进行异步通信，发布或订阅标准化的**消息 (Messages)**。例如，一个激光雷达驱动节点会向 /lidar\_scan 话题发布包含点云数据的消息，而感知算法节点则会订阅该话题以获取数据。此外，系统还支持同步的请求-响应模式，称为**服务 (Services)**，以及用于执行长时任务的**动作库 (Actionlib)** 1。

#### **ROS 1 (Noetic Ninjemys): 成熟的遗留平台**

ROS 1，特别是其最后一个长期支持版本Noetic（适配Ubuntu 20.04），拥有一个极其庞大和成熟的社区及软件包生态系统 2。它使用catkin作为构建系统，其C++客户端库为roscpp。许多经典的教程、学术项目和遗留工业系统仍在使用ROS 1。然而，其底层设计在多机器人通信、实时性保证和网络安全方面存在先天不足，这些恰恰是现代自动驾驶系统所要求的关键特性。

#### **ROS 2 (Humble Hawksbill): 现代工业标准**

ROS 2是对ROS 1的彻底重新设计，旨在满足工业界和学术界对高性能、高可靠性机器人系统的需求。其旗舰版本Humble（适配Ubuntu 22.04）已成为Autoware等领先开源项目的标准平台 4。

对于C++开发者而言，ROS 2的优势尤为突出：

* **现代化的构建与开发**：ROS 2采用colcon作为构建系统，并全面拥抱现代C++标准（C++17及以上）。其C++客户端库rclcpp大量使用智能指针（std::shared\_ptr, std::unique\_ptr）、标准库容器和lambda表达式，引导开发者编写更安全、更高效、更具表现力的代码。  
* **实时性能**：ROS 2基于DDS（Data Distribution Service）标准，提供了对实时（real-time）通信的支持，允许对服务质量（QoS）进行精细控制，这对于延迟敏感的控制回路至关重要。  
* **组件化架构**：ROS 2引入了“组件(Components)”的概念，允许将多个节点编译进同一个进程中，通过进程内通信避免了数据序列化和反序列化的开销，极大地提升了通信效率。  
* **跨平台与安全性**：ROS 2原生支持Linux、Windows和macOS，并内置了更完善的安全机制 6。

#### **对C++学习的实际影响**

从roscpp到rclcpp的转变，不仅仅是API的更新，更是编程范式的进化。ROS 2鼓励开发者采用现代C++的最佳实践，这使其成为一个远比ROS 1更适合提升C++水平的平台。对于新人而言，尽管部分教程可能仍基于ROS 1，但长远来看，直接从ROS 2入手是更明智的投资。

### **1.2 自动驾驶软件栈剖析：C++大放异彩的核心领域**

一个完整的自动驾驶软件栈通常被划分为三大核心模块：感知（Perception）、规划（Planning）和控制（Control）8。C++凭借其无与伦比的性能、对内存的精细控制和底层硬件交互能力，在这些计算密集型模块中占据了主导地位。

#### **感知：车辆的“眼睛”与“耳朵”**

感知模块负责解释来自各种传感器（如激光雷达LiDAR、摄像头、雷达、IMU）的数据，以构建对周围环境的理解 8。这包括识别车道线、检测和跟踪其他车辆与行人、理解交通信号灯等任务。

* **C++的角色**：感知算法需要实时处理海量数据。例如，一个64线激光雷达每秒可以产生数百万个数据点。C++的高性能对于在毫秒级时间内完成点云滤波、分割、聚类等操作至关重要 8。像PCL（Point Cloud Library）和OpenCV这类库，其核心就是用C++编写的，以实现极致的计算效率。在这一领域，开发者将深入实践多线程编程以并行处理数据、高效的内存管理以避免不必要的拷贝，以及SIMD（单指令多数据流）等底层优化技术。enginBozkurt/LidarObstacleDetection项目便是一个专注于LiDAR障碍物检测的优秀C++实践案例.8

#### **规划：车辆的“大脑”**

规划模块接收来自感知模块的环境信息和来自定位模块的车辆位置，然后计算出一条安全、舒适且高效的行驶路径 5。这通常分为几个层次：任务规划（决定总体路线）、行为规划（处理交叉口、变道等场景）和运动规划（生成平滑的局部轨迹）。

* **C++的角色**：路径搜索算法（如A\*、RRT\*）和轨迹优化算法本质上是计算密集型的。它们涉及复杂的图搜索、计算几何和数值优化。C++的强类型系统、面向对象特性和高性能使其成为实现这些复杂算法的理想选择。开发者将在这里运用到高级数据结构（图、树、优先队列）、算法设计与分析以及线性代数等知识。

#### **控制：车辆的“手”与“脚”**

控制模块负责将规划模块生成的期望轨迹转化为具体的车辆执行器指令，即方向盘转角、油门和刹车 9。

* **C++的角色**：车辆控制要求极低的延迟和高度的确定性。控制算法，如经典的PID（比例-积分-微分）控制器，必须在严格的时间循环内可靠地执行。C++能够生成高度优化的机器码，且没有垃圾回收等不确定性延迟，因此是实现控制器的不二之选。wolfgang-stefani/PID-Controller这类项目提供了一个极佳的、自包含的C++ PID控制器实现，是学习控制理论与实践的绝佳范例 9。

一个值得注意的现象是，许多大型项目中C++与Python常常并存 2。这并非偶然，而是一种深思熟虑的工程决策。Python通常扮演“胶水语言”的角色，用于系统启动、参数配置、快速原型验证和与机器学习框架的接口 2。而C++则作为“性能引擎”，负责实现所有对性能和实时性有严苛要求的核心算法模块 10。对于学习者而言，这意味着无需试图用C++构建整个系统。更有效的策略是专注于理解和修改单个C++节点——那些性能关键的组件——并将它们集成到一个可能由Python和ROS启动文件编排的更大系统中。

### **1.3 虚拟试验场：为何仿真器不可或缺**

对于没有权限接触价值数百万美元的自动驾驶测试车的初学者来说，高保真仿真器是唯一可行的开发和测试环境 4。它们能够模拟真实世界的物理规律、传感器特性和交通场景，为开发者提供一个闭环的“编码-测试-调试”流程。

行业内主流的仿真器包括CARLA、AWSIM等 4。CARLA尤其受欢迎，因为它提供了逼真的视觉效果和传感器数据，并有成熟的ROS接口（ROS Bridge）11。这些虚拟环境是学习者应用和磨练其C++与ROS技能的沙盒，也是通往真实世界应用的必要阶梯。

## **第二章：结构化学习平台：您的引导式入门路径**

在直接挑战工业级项目的复杂性之前，从专为教学设计的平台入手是构建坚实基础的明智之举。这些平台提供了一条结构化的、循序渐进的学习路径，非常适合初学者建立核心技能和信心。

### **2.1 深度解析：F1TENTH自动驾驶赛车课程**

F1TENTH被广泛认为是自动驾驶赛车领域首屈一指的教育和研究平台。它巧妙地将1:10比例的实体赛车、高仿真环境和一套完整的大学级别课程资料结合在一起，为学习者提供了一个既易于上手又充满挑战的实践环境 13。

#### **基于实验的阶梯式学习结构**

F1TENTH的核心优势在于其精心设计的、基于实验（Lab）的学习结构。这一系列实验构成了一条完美的学习曲线，引导学习者从零开始，逐步掌握自动驾驶的核心技术 15。

* **实验一：ROS入门 (Introduction to ROS)**：这是机器人学的“Hello, World\!”。学习者将学习如何创建ROS工作空间、功能包，并编写最基本的发布者（Publisher）和订阅者（Subscriber）节点。这个实验旨在让学习者熟悉ROS的基本框架和命令行工具 15。  
* **实验二：安全节点/自动紧急制动 (Safety Node / AEB)**：这是一个理想的第一个C++项目。学习者需要实现一个ROS节点，该节点订阅LiDAR的LaserScan消息和车辆的里程计（Odometry）信息，计算碰撞时间（Time-to-Collision, TTC），并在TTC低于某个阈值时发布一条AckermannDriveStamped消息来紧急制动车辆。这个任务清晰、目标明确，能够有效地巩固ROS通信的核心概念 16。  
* **实验三：墙壁跟随 (Wall Following)**：这个实验将学习者引入了控制理论的世界。任务是使用PID控制器，让赛车与赛道的一侧墙壁保持恒定距离。学习者需要在一个C++节点中实现PID算法，理解比例（P）、积分（I）、微分（D）三个参数的作用，并通过在仿真中反复调试（tuning）来获得最佳的控制效果。这是一个将理论与实践紧密结合的经典机器人学问题 16。  
* **实验四：间隙跟随 (Gap Following)**：这是反应式避障算法的基础。学习者需要编写一个节点来处理LiDAR数据，找出视野中最宽阔、最安全的“间隙”（gap），并引导车辆朝该间隙的中心行驶。这个实验标志着从纯粹的控制转向了由感知驱动的控制，是实现更高级别自主能力的关键一步 13。  
* **高级实验 (纯追踪、运动规划等)**：完成基础实验后，学习者可以挑战更高级的课题，如实验六的纯追踪（Pure Pursuit）算法，它引入了更先进的路径跟踪方法；以及实验七的运动规划（Motion Planning），其中可能涉及RRT等基于采样的搜索算法 13。

#### **C++适用性分析**

F1TENTH课程明确支持并鼓励使用C++进行开发。每个实验都提供了清晰的指导、待完成的代码框架（skeleton code）和明确的交付成果要求 15。这种将C++编程直接应用于解决具体机器人问题的模式，对技能提升的价值无可估量。在GitHub上可以找到大量学生和课程提供的C++解决方案，这些都是极佳的学习和参考资料 16。

F1TENTH的教学模式揭示了一个重要的学习原理。它没有一开始就用一个完整的自动驾驶软件栈的复杂性来压倒初学者，而是提出了一系列离散且定义明确的“问题”，例如“不要撞墙”、“沿着墙走”、“穿过最大的缝隙”。这种“问题优先”的模式对于技能习得极为高效。学习者不是在抽象地学习PID控制器，而是在实现一个PidController.cpp类，调试$K\_p$, $K\_i$, $K\_d$参数，并在仿真中立即看到物理结果。C++代码有了清晰而直接的目标，这在编码和结果之间建立了一个紧密的反馈循环，是高效学习的关键。

### **2.2 其他可供选择的教育资源**

* **Self-Driving and ROS 2 \- Learn by Doing\!**：这个在GitHub上的课程仓库是另一个绝佳的起点 5。它基于现代的ROS 2，涵盖了卡尔曼滤波器、里程计、控制等关键概念。最重要的是，该课程的所有实验代码都同时提供了Python和C++两种实现。这使得学习者可以方便地对比两种语言的实现差异，并专注于C++的学习 5。  
* **Udacity自动驾驶工程师纳米学位项目**：Udacity的课程在业界享有盛誉，其相关的开源项目代码也散落在GitHub上 8。虽然这些项目并非一个连贯的整体，但其中单个的模块，如交通标志分类器或LiDAR障碍物检测，都为学习者提供了范围明确、目标清晰的C++编程挑战，非常适合作为专项练习 8。

## **第三章：工业级平台：攀登学习曲线的进阶之路**

在通过结构化学习平台打下坚实基础后，学习者便可以向工业研究和实际部署中使用的全功能软件栈发起挑战。这些平台是通往专业领域的“下一步”，它们复杂、全面，蕴藏着海量的学习机会。

### **3.1 解构Autoware：世界级开源系统的导览**

Autoware是全球领先的“一体化”开源自动驾驶软件项目，由Autoware基金会托管，旨在为从学术研究到商业部署的各种应用提供一个统一的软件平台 4。需要明确的是，Autoware经历了从基于ROS 1的Autoware.AI到基于ROS 2的现代架构（最初称为Autoware.Auto，现已统一为Autoware主项目，其核心软件包在autoware.universe等仓库中）的演进 6。对于新学习者，强烈建议直接从基于ROS 2的最新版本入手。

#### **导航Autoware架构 (autoware.universe)**

Autoware的规模可能会让初学者望而生畏。其代码库被拆分在多个仓库中，其中autoware\_core和autoware\_universe是核心 10。autoware.universe仓库包含了绝大多数高级功能模块，并且其代码主体绝大部分由C++构成（占比超过90%），这使其成为一个学习高级C++编程和自动驾驶算法的宝库 10。

#### **为初学者设计的战略性入门路径**

试图一次性理解或构建整个Autoware系统是不现实的。正确的入门方式是遵循官方文档推荐的、由简到繁的路径，利用其强大的仿真功能。

* **第一步：安装与仿真环境设置**：学习者应首先遵循官方文档，完成从源码的安装过程。Autoware提供了详尽的教程，指导用户如何设置仿真环境，这是无需硬件即可开始学习的关键 12。  
* **第二步：规划仿真 (planning\_simulator)**：这是官方为新用户推荐的第一个接触点 12。规划仿真器使用简化的、虚拟的传感器数据，从而允许学习者绕过复杂的感知和定位模块，直接聚焦于**规划**和**控制**两大核心。用户可以在可视化工具Rviz中设置一个目标点，然后观察车辆如何自主规划路径并行驶过去。这提供了一个对系统行为的宏观理解。  
* **第三步：Rosbag回放仿真 (rosbag-replay)**：在熟悉了规划仿真后，下一步是使用rosbag文件进行回放仿真。Rosbag记录了真实世界中车辆传感器和内部状态的所有数据。通过回放，学习者可以在一个可控、可重复的环境中测试**感知**和**定位**模块的性能，从而开始接触真实传感器数据带来的复杂性 12。

#### **寻找适合入门的C++功能包**

当熟悉了仿真操作流程后，便可以开始深入代码。建议从autoware.universe中功能相对独立、与F1TENTH中所学概念相对应的C++包开始。

* **控制包**：像mpc\_lateral\_controller（模型预测控制）或更简单的PID控制器包是很好的起点。它们的输入（一条期望轨迹）和输出（车辆控制指令）非常明确，易于理解 24。  
* **规划包**：可以从一些辅助性的规划工具包或简单的避障规划器开始，例如obstacle\_avoidance\_planner。  
* **感知包**：对于感知，可以从一个简单的点云滤波节点（如体素滤波）或一个聚类算法包（如euclidean\_cluster）入手，这些是进入3D数据处理世界的良好开端 25。

#### **社区健康度与支持**

Autoware拥有一个极其活跃和健康的开源社区。其在GitHub上拥有超过1万个星标和3千多个复刻，Issues、Pull Requests和Discussions板块都非常活跃 22。这意味着当学习者遇到问题时，有很大的概率可以通过搜索历史记录或直接提问来获得帮助，这是一个宝贵的资源。

### **3.2 CARLA仿真器与ROS Bridge：不可或缺的虚拟试验场**

CARLA是一款基于虚幻引擎（Unreal Engine）开发的开源仿真器，以其能够生成照片级逼真度的摄像头图像和高精度的LiDAR点云而闻名 4。它为自动驾驶算法的开发和测试提供了一个近乎真实的虚拟世界。

#### **ros-bridge：连接C++代码与虚拟世界的桥梁**

ros-bridge是使用CARLA进行ROS开发的核心组件。它作为一个独立的程序与CARLA服务器并行运行，其主要功能是双向翻译：将CARLA仿真世界中的状态（如车辆位置、传感器数据）转换成标准的ROS消息发布出来，同时订阅特定的ROS话题，将接收到的消息（如控制指令）转换成CARLA可以理解的命令，从而控制仿真中的车辆 28。

* **订阅传感器数据**：开发者可以编写一个C++ ROS节点，订阅ros-bridge发布的标准ROS话题，例如/carla/ego\_vehicle/lidar（类型为sensor\_msgs/PointCloud2）或/carla/ego\_vehicle/rgb\_front/image\_color（类型为sensor\_msgs/Image）。这为练习编写感知算法提供了源源不断的、高质量的原始数据。  
* **发布控制指令**：同样，开发者可以编写一个C++节点，在执行完自己的算法逻辑后，向/carla/ego\_vehicle/vehicle\_control\_cmd话题（类型为carla\_msgs/CarlaEgoVehicleControl）发布指令，从而驱动仿真车辆移动 31。这就构成了一个完整的“感知-规划-控制”闭环。

#### **通往Autoware的垫脚石**

CARLA可以被视为从F1TENTH到Autoware的完美中间步骤。在F1TENTH中掌握了单个算法节点的开发后，学习者可以利用CARLA来构建和测试更复杂的、依赖多传感器输入的C++节点，然后再尝试将这些节点集成到完整的Autowtackare软件栈中。值得一提的是，Autoware的官方文档中也包含了如何与CARLA进行联合仿真的教程，这进一步证明了其在生态系统中的重要地位 12。

从F1TENTH到CARLA，再到Autoware的旅程，实际上是一个精心设计的、抽象层次不断提升的学习过程。F1TENTH的抽象层次很低，代码几乎是直接控制硬件或简单仿真；CARLA引入了传感器和环境的抽象层；而Autoware则在此之上构建了多层架构抽象（如任务规划、行为规划、运动规划）。这个“抽象梯度”是成为一名合格自动驾驶软件工程师的隐藏课程。在F1tENTH阶段，目标是实现**算法**；在CARLA阶段，目标是管理从真实传感器到算法的**数据流**；在Autoware阶段，目标是理解允许多个算法协同工作的**架构**。认识到这一进程，是避免挫败感、在每个阶段最大化学习效果的关键。

## **第四章：战略路线图与项目综合分析**

本章将前述所有分析整合成一个清晰、可执行的行动计划。它将提供最终的项目建议、一个用于评估项目的比较框架，以及一条结构化的学习路径，旨在引导C++开发者从ROS新手成长为能够为工业级项目做出贡献的参与者。

### **4.1 推荐学习轨迹：从“Hello, ROS”到Autoware贡献者**

这条路径被设计为三个循序渐进的阶段，每个阶段都有明确的学习目标和项目建议。

#### **第一阶段：掌握基础 (预计1-2个月)**

此阶段的目标是建立对ROS 2和基本机器人算法的扎实理解。

1. **环境搭建**：在Ubuntu 22.04上安装ROS 2 Humble，并配置好C++开发环境（如VS Code）1。  
2. **ROS 2官方教程**：系统性地完成ROS 2官方的C++教程，特别是关于编写发布者/订阅者、服务/客户端、动作/客户端以及启动文件的部分。这是掌握rclcpp API的必要步骤。  
3. **F1TENTH课程实践**：在F1TENTH的仿真环境中，用C++完成实验1至实验4。重点是亲手实现并调试自动紧急制动（AEB）、PID墙壁跟随和间隙跟随算法。这个过程将理论知识转化为实际的编程经验 15。

#### **第二阶段：在高保真环境中构建 (预计2-3个月)**

此阶段的目标是学习处理更真实的传感器数据，并构建更复杂的独立应用。

1. **CARLA环境设置**：安装CARLA仿真器及其ROS 2 Bridge，并成功运行示例 28。  
2. **项目构想一：算法迁移**：将在F1TENTH中实现的“间隙跟随”算法移植到CARLA中。这将迫使学习者从处理LaserScan消息转向处理更复杂的PointCloud2消息，并适应CARLA中更真实的车辆动力学模型。  
3. **项目构想二：视觉车道保持**：编写一个C++节点，订阅CARLA的摄像头图像，使用OpenCV进行基础的图像处理（如颜色阈值分割或边缘检测）来识别车道线，并发布转向指令以使车辆保持在车道内。  
4. **项目构想三：简单巡航控制**：编写一个C++节点，订阅车辆的里程计信息（速度），并发布油门/刹车指令以维持一个恒定的速度。

#### **第三阶段：融入工业级系统 (持续进行)**

此阶段的目标是学习理解和参与大型、复杂的软件系统。

1. **Autoware安装与探索**：遵循官方文档，从源码安装Autoware 12。  
2. **熟悉系统行为**：完整地运行planning\_simulator和rosbag-replay教程，从宏观上理解Autoware是如何工作的 12。  
3. **贡献目标一 (代码阅读与分析)**：在autoware.universe中选择一个相对独立的C++包（例如vehicle\_pid\_controller）。通读其所有C++源码，添加自己的理解注释，并尝试画出该模块的输入、输出和内部逻辑框图。  
4. **贡献目标二 (参数调整与实验)**：尝试修改所选包中的一个参数（例如PID增益），并在planning\_simulator中观察其对车辆行为的影响。这是理解代码作用的最直接方式。  
5. **贡献目标三 (代码贡献)**：在Autoware的GitHub仓库中寻找标记为“good first issue”或“help wanted”的议题 26。尝试解决一个简单的问题，这可能是一个小bug的修复、文档的完善或是一个微小功能的添加。完成一次成功的Pull Request将是学习过程中的一个重要里程碑。

### **4.2 评估C++代码库的框架**

为了培养独立学习和评估项目的能力，建议开发者在接触新项目时，使用以下清单进行考察：

* **构建系统 (CMakeLists.txt)**：检查项目是否使用了现代CMake实践，如target\_include\_directories和target\_link\_libraries。一个清晰、模块化的CMake文件通常预示着一个结构良好的项目。  
* **现代C++语言特性**：代码是否使用了C++17/20的特性？注意智能指针（std::unique\_ptr, std::shared\_ptr）的使用以替代原始指针、STL容器的广泛应用、const正确性以及基于范围的for循环等。  
* **代码质量与风格**：项目中是否存在.clang-format文件 22？这表明团队对代码风格一致性的重视。代码注释是否充分？逻辑是否被合理地划分到具有明确职责的类和函数中？  
* **文档**：项目是否有详细的README文件？是否有像Autoware那样的外部文档网站，解释其架构和使用方法？

### **4.3 项目对比分析与最终建议**

为了直观地总结各个项目的特点，下表提供了一个综合性的对比分析。

**表1：推荐自动驾驶项目对比分析**

| 项目 | 主要用例 | ROS 版本 | 新手友好度 | C++学习潜力 | 文档与社区支持 |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **F1TENTH** | 教育课程与竞赛平台 | ROS 1 & ROS 2 | **高**：结构化的实验引导，从零开始构建核心技能。 | **高**：提供具体的、目标明确的算法实现任务（PID, TTC, Gap Following），非常适合实践C++在控制和基础感知中的应用。 | **良好**：提供完整的课程讲义、实验指导和活跃的社区。 |
| **CARLA ROS Bridge** | 高保真仿真器接口 | ROS 1 & ROS 2 | **中**：需要一定的ROS基础来配置和使用，但一旦运行起来，接口清晰。 | **高**：是练习处理真实传感器数据（点云、图像）的理想平台。开发者可以专注于编写独立的C++感知或控制节点。 | **良好**：官方文档详尽，社区活跃，是许多研究的基础平台。 |
| **Autoware** | 全功能自动驾驶软件栈 | ROS 2 | **低**：系统极其庞大复杂，不适合直接作为入门项目。 | **非常高**：包含工业级的、高度优化的C++代码，涵盖感知、规划、控制的各个方面。是深入学习高级C++实践和复杂系统架构的终极目标。 | **优秀**：拥有非常详尽的官方文档、活跃的开发者社区和定期的工作组会议。 |

#### **结论与建议**

对于寻求进入自动驾驶领域并提升C++水平的开发者，最有效的路径并非选择一个“最好”的项目，而是根据自身所处的阶段，战略性地利用不同项目的优势。

最终的建议是采纳**第四章4.1节中提出的三阶段学习轨迹**：

1. **从F1TENTH开始**，利用其结构化的课程建立对ROS、C++和核心自动驾驶算法的坚实基础。  
2. **迁移到CARLA**，利用其高保真环境，练习处理复杂的传感器数据，并构建更强大的独立C++应用。  
3. **最终以Autoware为目标**，将其作为一个学习和贡献的平台。通过阅读其高质量的C++代码、理解其复杂的系统架构，并尝试做出自己的贡献，将技能提升到工业级水平。

遵循这条路径，开发者将能够系统性地、由浅入深地掌握自动驾驶软件开发所需的C++技能，最终实现从新手到领域贡献者的转变。

#### **Works cited**

1. cybergeekgyan/ROS-for-Beginners-: ROS for Beginners Basics, Motion, and OpenCV \- GitHub, accessed October 19, 2025, [https://github.com/cybergeekgyan/ROS-for-Beginners-](https://github.com/cybergeekgyan/ROS-for-Beginners-)  
2. UT-ADL/autoware\_mini: Autoware Mini is a minimalistic Python-based autonomy software., accessed October 19, 2025, [https://github.com/UT-ADL/autoware\_mini](https://github.com/UT-ADL/autoware_mini)  
3. Self-driving Car Based on Learning from Vision Demonstration \- Hackster.io, accessed October 19, 2025, [https://www.hackster.io/gch981213/self-driving-car-based-on-learning-from-vision-demonstration-1ef30d](https://www.hackster.io/gch981213/self-driving-car-based-on-learning-from-vision-demonstration-1ef30d)  
4. ros · GitHub Topics, accessed October 19, 2025, [https://github.com/topics/ros](https://github.com/topics/ros)  
5. AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control \- GitHub, accessed October 19, 2025, [https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control](https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control)  
6. usdot-fhwa-stol/autoware.auto \- GitHub, accessed October 19, 2025, [https://github.com/usdot-fhwa-stol/autoware.auto](https://github.com/usdot-fhwa-stol/autoware.auto)  
7. autocore-ai/AutowareAuto \- GitHub, accessed October 19, 2025, [https://github.com/autocore-ai/AutowareAuto](https://github.com/autocore-ai/AutowareAuto)  
8. Self-driving car github repositories and projects | Aionlinecourse, accessed October 19, 2025, [https://www.aionlinecourse.com/blog/self-driving-car-github-repositories-and-projects](https://www.aionlinecourse.com/blog/self-driving-car-github-repositories-and-projects)  
9. autonomous-driving · GitHub Topics, accessed October 19, 2025, [https://github.com/topics/autonomous-driving?l=c%2B%2B\&o=asc\&s=forks](https://github.com/topics/autonomous-driving?l=c%2B%2B&o=asc&s=forks)  
10. autowarefoundation/autoware\_universe \- GitHub, accessed October 19, 2025, [https://github.com/autowarefoundation/autoware\_universe](https://github.com/autowarefoundation/autoware_universe)  
11. autonomous-driving · GitHub Topics, accessed October 19, 2025, [https://github.com/topics/autonomous-driving](https://github.com/topics/autonomous-driving)  
12. Tutorials \- Autoware Documentation, accessed October 19, 2025, [https://autowarefoundation.github.io/autoware-documentation/main/tutorials/](https://autowarefoundation.github.io/autoware-documentation/main/tutorials/)  
13. Crash Course on Autonomous Racing \- GitHub, accessed October 19, 2025, [https://github.com/FT-Autonomous/Autonomous\_Crash\_Course](https://github.com/FT-Autonomous/Autonomous_Crash_Course)  
14. F1TENTH Ressources, accessed October 19, 2025, [https://iros2024-race.f1tenth.org/f1tenth\_ressources.html](https://iros2024-race.f1tenth.org/f1tenth_ressources.html)  
15. Lab 1 \- Introduction to ROS — F1TENTH \- Learn latest documentation, accessed October 19, 2025, [https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/lab1.html](https://f1tenth-coursekit.readthedocs.io/en/stable/assignments/labs/lab1.html)  
16. CPP-F1TENTH-SDP/Labs: F1TENTH Labs & Simulation Install \- GitHub, accessed October 19, 2025, [https://github.com/CPP-F1TENTH-SDP/Labs](https://github.com/CPP-F1TENTH-SDP/Labs)  
17. Lab 1 \- Introduction to ROS2 — RoboRacer \- Learn latest documentation, accessed October 19, 2025, [https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab1.html](https://f1tenth-coursekit.readthedocs.io/en/latest/assignments/labs/lab1.html)  
18. Hamza-cpp/f1tenth\_lab2\_safety\_pkg: The implementation of the Lab 2 \- Automatic Emergency Braking (AEB) from the F1TENTH course \- GitHub, accessed October 19, 2025, [https://github.com/Hamza-cpp/f1tenth\_lab2\_safety\_pkg](https://github.com/Hamza-cpp/f1tenth_lab2_safety_pkg)  
19. f1tenth/f1tenth\_lab3\_template \- GitHub, accessed October 19, 2025, [https://github.com/f1tenth/f1tenth\_lab3\_template](https://github.com/f1tenth/f1tenth_lab3_template)  
20. Hamza-cpp/f1tenth\_lab4\_follow\_the\_gap \- GitHub, accessed October 19, 2025, [https://github.com/Hamza-cpp/f1tenth\_lab4\_follow\_the\_gap](https://github.com/Hamza-cpp/f1tenth_lab4_follow_the_gap)  
21. jafriztrillo/UVMF1TENTH: Lab assignments for F1tenth course \- GitHub, accessed October 19, 2025, [https://github.com/jafriztrillo/UVMF1TENTH](https://github.com/jafriztrillo/UVMF1TENTH)  
22. autowarefoundation/autoware: Autoware \- the world's ... \- GitHub, accessed October 19, 2025, [https://github.com/autowarefoundation/autoware](https://github.com/autowarefoundation/autoware)  
23. The Autoware Foundation \- GitHub, accessed October 19, 2025, [https://github.com/autowarefoundation](https://github.com/autowarefoundation)  
24. \`mpc\_lateral\_controller\` does not compile · Issue \#3182 · autowarefoundation/autoware\_universe \- GitHub, accessed October 19, 2025, [https://github.com/autowarefoundation/autoware\_universe/issues/3182](https://github.com/autowarefoundation/autoware_universe/issues/3182)  
25. Releases · autowarefoundation/autoware\_universe \- GitHub, accessed October 19, 2025, [https://github.com/autowarefoundation/autoware\_universe/releases](https://github.com/autowarefoundation/autoware_universe/releases)  
26. Issues · autowarefoundation/autoware \- GitHub, accessed October 19, 2025, [https://github.com/autowarefoundation/autoware/issues](https://github.com/autowarefoundation/autoware/issues)  
27. carla-simulator/carla: Open-source simulator for autonomous driving research. \- GitHub, accessed October 19, 2025, [https://github.com/carla-simulator/carla](https://github.com/carla-simulator/carla)  
28. ROS bridge installation \- CARLA Simulator, accessed October 19, 2025, [https://carla.readthedocs.io/en/0.9.10/ros\_installation/](https://carla.readthedocs.io/en/0.9.10/ros_installation/)  
29. carla-simulator/ros-bridge \- GitHub, accessed October 19, 2025, [https://github.com/carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)  
30. ROS bridge installation \- CARLA Simulator \- Read the Docs, accessed October 19, 2025, [https://carla.readthedocs.io/en/0.9.9/ros\_installation/](https://carla.readthedocs.io/en/0.9.9/ros_installation/)  
31. The ROS bridge package \- CARLA Simulator, accessed October 19, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/run\_ros/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/)  
32. ROS-CARLA Integration \> How to use ROS Bridge package \- Semi-Automatic Labelling for Atlascar using Adaptive Perception, accessed October 19, 2025, [https://silvamfpedro.github.io/thesis-blog/manual.html](https://silvamfpedro.github.io/thesis-blog/manual.html)  
33. MPC-Berkeley/carla-ros-bridge: ROS bridge for CARLA Simulator. Goes with fork of Carla simulator. \- GitHub, accessed October 19, 2025, [https://github.com/MPC-Berkeley/carla-ros-bridge](https://github.com/MPC-Berkeley/carla-ros-bridge)