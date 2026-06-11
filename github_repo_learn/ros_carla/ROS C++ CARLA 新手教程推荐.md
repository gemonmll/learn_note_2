

# **掌握ROS、C++与CARLA：一份面向自动驾驶系统开发的综合指南**

## **第一部分：ROS与C++基础技能**

本部分旨在为机器人软件开发奠定坚实的知识基础。我们将严谨地构建一个专业的开发环境，然后深入剖析ROS中两种主要的通信范式，并使用C++作为实现语言。

### **第1章：构建您的机器人开发环境**

**目标**：创建一个稳定、专业且高效的开发环境，以消除初学者常遇到的配置难题。

**内容详情**：

* **操作系统与ROS安装**：本章提供在物理机或虚拟机上安装Ubuntu（例如，针对ROS Noetic的20.04版本）的详细说明，并讨论两种方式的利弊 1。官方ROS Wiki教程将作为安装过程的基准参考 3。  
* **Catkin工作空间管理**：分步介绍如何创建和编译Catkin工作空间（mkdir \-p \~/catkin\_ws/src, catkin\_make），并解释src、build和devel目录的用途 4。 sourcing setup.bash文件（source devel/setup.bash）的重要性将被重点强调，因为它负责修改ROS\_PACKAGE\_PATH等环境变量，从而让ROS系统能够找到您的功能包 1。  
* **核心开发工具**：推荐使用集成开发环境（IDE），并强烈建议选择Visual Studio Code。其轻量级特性、集成的终端以及对C++、CMake和ROS的优秀扩展支持，使其与Eclipse等重型工具相比更具优势 1。  
* **Git版本控制**：简要介绍为何版本控制对于严肃项目是不可或缺的，并指导如何克隆本指南中讨论的各个代码仓库。

### **第2章：ROS通信核心：C++中的发布者/订阅者模型**

**目标**：通过代码层面的深入分析，透彻理解构成ROS骨干的异步、多对多通信模式。

**内容详情**：

* **概念概览**：介绍ROS节点（Nodes）、话题（Topics）和消息（Messages）的基本概念。我们将使用经典的“talker”（发布者）与“listener”（订阅者）比喻来进行说明 6。  
* **发布者（talker.cpp）代码剖析**：  
  * \#include "ros/ros.h" 与 \#include "std\_msgs/String.h"：解释核心ROS头文件和消息类型头文件的作用 6。  
  * ros::init(argc, argv, "talker")：阐述初始化节点的必要性 7。  
  * ros::NodeHandle n：解释NodeHandle是与ROS通信的主要接口 7。  
  * ros::Publisher chatter\_pub \= n.advertise\<std\_msgs::String\>("chatter", 1000)：详细解析advertise函数的调用——指定消息类型、话题名称（"chatter"）和队列大小。队列大小作为缓冲区的意义将被重点说明 6。  
  * ros::Rate loop\_rate(10)：如何控制发布频率 7。  
  * while (ros::ok())：主循环以及ros::ok()返回false的几种情况（如接收到Ctrl-C信号、ros::shutdown()被调用等）6。  
  * std\_msgs::String msg;... chatter\_pub.publish(msg);：创建、填充并发布消息对象 9。  
  * ros::spinOnce()：提及它在处理回调函数中的作用，尽管它对订阅者更为关键 9。  
* **订阅者（listener.cpp）代码剖析**：  
  * void chatterCallback(const std\_msgs::String::ConstPtr& msg)：解释回调函数的签名，以及使用ConstPtr（指向常量消息的共享指针）来避免数据复制，并通过msg-\>data访问消息内容 7。  
  * ros::Subscriber sub \= n.subscribe("chatter", 1000, chatterCallback)：解析subscribe函数的调用，它将一个话题与一个回调函数关联起来 6。  
  * ros::spin()：阐明ros::spin()的关键作用，它是一个阻塞式调用，会进入一个循环，在消息到达时处理回调函数 6。  
* **编译节点**：详细解释需要在CMakeLists.txt中添加的指令，包括add\_executable、add\_dependencies和target\_link\_libraries 8。

初学者在实践中常常会发现，订阅者会丢失发布者发出的最初几条消息。这是一个典型的“竞态条件”问题：发布者在订阅者完全连接到ROS Master并建立起通信链路之前，就已经开始广播消息了。这种通信的非即时性要求开发者在设计节点时必须考虑到连接建立所需的时间。一个简单而有效的解决方案是在发布循环开始前加入一段等待代码：while(chatter\_pub.getNumSubscribers() \== 0\) loop\_rate.sleep(); 9。这行代码会强制发布者暂停，直到至少有一个订阅者成功连接。这个看似微小的改动，能将一个不可靠的节点转变为一个可靠的节点，是构建稳健机器人系统的基本技巧。

此外，通过分析wenjiu2001/Beginner-Tutorials-ROS 4、VBot2410/beginner\_tutorials 5 和 sanhuezapablo/beginner\_tutorials 11 等面向初学者的GitHub仓库，可以发现它们并非创造全新的示例，而是将ROS官方Wiki教程 7 的内容打包成了一个预配置好、可直接克隆运行的格式。这种模式揭示了一种高效的学习路径：官方Wiki提供了知识的“源代码”，而这些GitHub项目则提供了学习的“二进制包”——即开即用的功能包，降低了初学者从零开始创建文件、目录和配置CMakeLists.txt的门槛。

### **第3章：同步交互：C++中的服务/客户端模型**

**目标**：解释请求-响应通信模型，这对于需要确保获得结果后才能继续执行的任务至关重要。

**内容详情**：

* **概念概览**：介绍服务（Services）及其.srv文件、服务调用的阻塞特性，以及何时应选择服务而非话题（例如，触发特定动作、请求配置数据等）3。  
* **定义服务（.srv）**：如何创建一个自定义的.srv文件（如AddTwoInts.srv），其中请求和响应部分由---分隔 14。  
* **服务节点（add\_two\_ints\_server.cpp）**：  
  * \#include "beginner\_tutorials/AddTwoInts.h"：包含由.srv文件自动生成的头文件 14。  
  * bool add(beginner\_tutorials::AddTwoInts::Request \&req, beginner\_tutorials::AddTwoInts::Response \&res)：服务回调函数的签名，它接收请求和响应对象的引用，并返回一个布尔值表示是否成功 14。  
  * res.sum \= req.a \+ req.b;：填充响应对象 14。  
  * ros::ServiceServer service \= n.advertiseService("add\_two\_ints", add)：发布服务，将服务名称与回调函数关联 14。  
* **客户端节点（add\_two\_ints\_client.cpp）**：  
  * ros::ServiceClient client \= n.serviceClient\<beginner\_tutorials::AddTwoInts\>("add\_two\_ints")：为特定服务创建一个客户端对象 14。  
  * beginner\_tutorials::AddTwoInts srv; srv.request.a \=...;：实例化服务类并填充请求内容 14。  
  * if (client.call(srv))：对服务进行阻塞式调用。代码将在此处暂停，直到服务返回结果 14。  
* **编译系统修改**：解释为处理.srv文件编译，需要在package.xml（添加message\_generation依赖）和CMakeLists.txt（使用find\_package、add\_service\_files、generate\_messages）中进行必要的修改 11。

### **第4章：精选入门代码仓库评测**

**目标**：对最佳的入门级代码仓库进行比较分析，以便用户根据自己的学习风格选择最合适的项目。

**内容详情**：

* cybergeekgyan/ROS-for-Beginners- 1：一个全面的、课程式的代码仓库，涵盖了从ROS基础到使用OpenCV进行感知和Arduino集成的整个生态系统。适合希望获得广泛、结构化介绍的学习者。  
* BV-Pradeep/RIA-ROS-Basics-with-CPP-in-5-days 13：一个由Robot Ignite Academy出品的、重点突出的课程代码仓库。其每日学习的结构非常适合喜欢有指导、有节奏的学习者。该课程明确覆盖了话题、服务和动作（Actions）。  
* wenjiu2001/Beginner-Tutorials-ROS 4：一个极简主义的代码仓库，提供了直接明了的示例。其主要特点是拥有一个独立的cpp-devel分支，使其成为一个纯净的C++入门点。  
* VBot2410/beginner\_tutorials 5：ROS教程的经典实现，值得注意的是，它包含了一个通过服务来修改发布者字符串的示例，展示了两种通信模式之间的简单交互。

**表1：入门级ROS/C++代码仓库对比分析**

| 代码仓库 | 涵盖的关键概念 | 结构与风格 | 核心优势 | 适用人群 |
| :---- | :---- | :---- | :---- | :---- |
| cybergeekgyan/ROS-for-Beginners- | 发布/订阅, 服务/客户端, 动作, OpenCV, Arduino | 综合性课程 | 覆盖ROS生态系统，内容广泛 | 希望全面了解ROS及其与外部工具集成的学习者 |
| BV-Pradeep/RIA-ROS-Basics-with-CPP-in-5-days | 发布/订阅, 服务/客户端, 动作, 调试工具 | 结构化的5天课程 | 节奏清晰，目标明确 | 喜欢按部就班、有计划进行一周学习的学习者 |
| wenjiu2001/Beginner-Tutorials-ROS | 发布/订阅 | 极简示例 | 干净的C++分支，无干扰 | 希望快速测试核心概念、代码简洁的学习者 |
| VBot2410/beginner\_tutorials | 发布/订阅, 服务/客户端, 启动文件 | 经典教程打包 | 包含服务与话题交互的示例 | 希望在实践中理解不同通信模式如何协同工作的学习者 |

## **第二部分：连接ROS与CARLA仿真器**

本部分将从抽象的ROS概念过渡到一个具体的高保真仿真环境。我们将探讨如何让CARLA与ROS进行通信，从而让用户能将新学的C++技能应用于自动驾驶车辆。

### **第5章：CARLA与ROS Bridge简介**

**目标**：成功安装并运行CARLA仿真器，并与ROS建立双向通信链接。

**内容详情**：

* **CARLA概览**：介绍CARLA作为一个用于自动驾驶研究的开源仿真器，并强调其传感器模拟和环境控制等特性 16。同时会提及系统要求 17。  
* **ROS Bridge安装**：分步指导如何将ros-bridge仓库（carla-simulator/ros-bridge）克隆到Catkin工作空间，并使用rosdep安装其依赖项 18。设置PYTHONPATH以包含CARLA .egg文件的关键步骤将被详细解释 19。  
* **启动仿真**：详细说明双终端启动流程：首先启动CARLA服务器（./CarlaUE4.sh）19，然后启动ROS Bridge（roslaunch carla\_ros\_bridge carla\_ros\_bridge.launch）22。  
* **配置（settings.yaml）**：解释关键配置参数，特别是synchronous\_mode 22。

synchronous\_mode（同步模式）参数不仅是一个配置选项，它是实现可复现、具备科学有效性的仿真结果的基石。在默认的异步模式下，CARLA服务器会以其最快速度运行，而ROS Bridge则在可能的情况下抓取数据。这可能导致非确定性行为、传感器数据帧丢失以及无法复现的仿真结果。通过设置synchronous\_mode: true，CARLA服务器的“tick”（仿真步进）将由客户端（即ROS Bridge）控制。Bridge会等待所有预期的传感器数据都接收完毕后，才允许仿真进入下一步 22。这确保了对于给定的控制输入，仿真输出将始终保持一致。对于任何从兴趣探索转向严谨算法开发与测试的研究者来说，这是一个不容忽视的核心概念。

### **第6章：指令与感知：车辆控制与传感器集成**

**目标**：编写简单的C++ ROS节点，以订阅来自CARLA的传感器数据，并发布控制指令来移动车辆。

**内容详情**：

* **生成主控车辆（Ego Vehicle）**：使用如carla\_ros\_bridge\_with\_example\_ego\_vehicle.launch等提供的启动文件，自动生成一辆车及其相关的ROS话题 22。  
* **订阅传感器数据**：编写一个C++“订阅者”节点，订阅如/carla/ego\_vehicle/lidar或/carla/ego\_vehicle/rgb\_front/image\_raw等话题，以接收传感器数据。  
* **发布控制指令**：编写一个C++“发布者”节点，向/carla/ego\_vehicle/vehicle\_control\_cmd（使用carla\_msgs/CarlaEgoVehicleControl消息）或/carla/ego\_vehicle/twist（使用geometry\_msgs/Twist消息）发布指令。这两种控制方式的区别将被解释 20。

**表2：CARLA ROS Bridge核心话题速查表**

| 话题名称 | 消息类型 | 方向 | 用途 |
| :---- | :---- | :---- | :---- |
| /carla/ego\_vehicle/rgb\_front/image\_raw | sensor\_msgs/Image | CARLA \-\> ROS | 提供前置摄像头的原始RGB图像 |
| /carla/ego\_vehicle/lidar | sensor\_msgs/PointCloud2 | CARLA \-\> ROS | 提供激光雷达的点云数据 |
| /carla/ego\_vehicle/imu | sensor\_msgs/Imu | CARLA \-\> ROS | 提供惯性测量单元（IMU）数据 |
| /carla/ego\_vehicle/gnss | sensor\_msgs/NavSatFix | CARLA \-\> ROS | 提供GNSS（全球导航卫星系统）数据 |
| /carla/ego\_vehicle/odometry | nav\_msgs/Odometry | CARLA \-\> ROS | 提供车辆的里程计信息（位置和速度） |
| /carla/ego\_vehicle/vehicle\_control\_cmd | carla\_msgs/CarlaEgoVehicleControl | ROS \-\> CARLA | 发送油门、转向和刹车指令 |
| /carla/ego\_vehicle/twist | geometry\_msgs/Twist | ROS \-\> CARLA | 发送线速度和角速度指令 |

### **第7章：高性能接口：原生CARLA C++客户端**

**目标**：介绍一种绕过ROS、性能更高的CARLA接口方式，适用于对延迟有严苛要求的应用。

**内容详情**：

* **基本原理**：讨论ROS Bridge（易于与ROS生态集成）与原生C++客户端（延迟更低、控制更直接）之间的权衡。  
* **官方示例（Examples/CppClient）**：剖析CARLA自带的官方C++客户端示例 23。关键步骤将被重点讲解：连接服务器、加载世界、获取蓝图库、生成actor以及施加控制 23。  
* **社区实现**：展示如mjxu96/carla-client-cpp 26 等简化的、独立的C++客户端仓库。这些项目旨在用最少的源文件和依赖生成客户端，使初学者更容易理解和编译。

## **第三部分：实践应用：从控制理论到自动驾驶**

这是本指南的压轴部分，我们将综合运用之前学到的所有技能，构建功能性的自动驾驶特性。我们将分析现有的（通常基于Python的）项目，提取其核心控制逻辑，并为使用C++实现该逻辑提供清晰的路线图。

### **第8章：项目深度实践：实现车道保持辅助系统**

**目标**：通过在C++中实现并集成基础车辆控制算法，构建一个完整的车道保持系统。

**内容详情**：

* **概念框架**：以通俗易懂的方式解释驾驶中的两个主要控制问题：纵向控制（保持速度）和横向控制（保持在车道内）。  
* **纵向控制（PID）**：  
  * **理论**：解释PID控制器中的比例（Proportional）、积分（Integral）和微分（Derivative）项 27。  
  * **C++实现**：提供一个PID控制器的C++类结构。我们将参考几个在GitHub上提供简洁、独立PID实现的C++仓库（如 27）作为优秀的学习范例。来自hfoffani/PID-controller的手动调参建议（“先将所有参数设为0，然后调P，再调D，最后调I”）将作为一个实用的方法论被重点介绍 27。  
* **横向控制（Pure Pursuit & Stanley）**：  
  * **理论**：基于对Lane-Keeping-Assist-on-CARLA项目的清晰逻辑分析 29，提供对这两种算法的视觉化和数学化解释。Pure Pursuit的“前瞻点”和Stanley同时利用朝向误差与横向误差的特点将被进行对比。  
  * **C++实现**：参考专门实现这些控制器的C++仓库，例如用于Pure Pursuit的larics/pure\_pursuit 32，以及ME5413\_Planning\_Project 33 中明确包含Stanley控制器的C++模板代码。  
* **系统集成**：指导如何构建最终的ROS功能包。这将涉及一个核心的C++节点，它订阅车辆状态（位姿、速度）和目标路径点，计算PID和Stanley/Pure Pursuit的输出，并发布最终的CarlaEgoVehicleControl消息。

许多高阶的、以算法为中心的CARLA项目（如Lane-Keeping-Assist-on-CARLA 29）为了便于快速原型验证，通常使用Python编写。然而，其底层的控制算法（如PID、Stanley等）是与语言无关的数学概念。这揭示了一种强大而实用的学习方法：首先，找到一个目标领域（如车道保持）中可运行的高阶Python项目；其次，分析其Python代码以理解核心逻辑 29；再次，寻找实现了这些特定算法的、专注的C++代码仓库（如 27）；最后，以这些C++仓库为参考，在一个新的C++ ROS节点中重新实现控制逻辑。这种“解构再重构”的方法弥合了语言间的差距，并教授了模块化、可测试的C++开发思想。

**表3：横向控制算法决策矩阵**

| 算法 | 核心原理 | 优点 | 缺点 | 关键调参参数 |
| :---- | :---- | :---- | :---- | :---- |
| Pure Pursuit (纯追踪) | 追逐路径上的一个“前瞻点” | 几何上直观，路径平滑 | 容易“切弯”，低速时可能不稳定 | 前瞻距离 ($L\_d$) |
| Stanley | 最小化朝向误差和横向误差 | 路径跟踪更精确 | 可能过于激进，对噪声敏感 | 比例增益 ($k$) |

### **第9章：项目深度实践：构建自动泊车系统架构**

**目标**：分析一个更复杂的自动驾驶系统，并理解机器人领域中一种常见的架构模式：将高性能的C++组件与灵活的Python组件分离。

**内容详情**：

* **架构分析**：以pparmesh/Autonomous-Parking-AV仓库 34 为案例进行研究。报告将重点介绍其三部分架构：一个C++运动规划器、一个Python车辆控制器和CARLA仿真器 34。  
* **C++运动规划器**：C++组件的作用是进行“高效计算”，实现一个“基于格的规划器”（lattice based planner）34。该组件负责搜索最优路径的重度计算任务。  
* **Python控制器**：Python组件利用“Carla Python-API”来实现轨迹跟踪（LQR和PID控制器）34。  
* **ROS作为粘合剂**：ROS在C++规划器和Python控制器之间实现“每个仿真步”的通信，其关键作用将被强调 34。C++节点可能会在一个话题上发布规划好的路径，而Python节点则订阅该话题。  
* **初学者路线图**：为用户提供一个简化的项目计划：1) 手动定义一条泊车轨迹（一个简单的航点列表）；2) 使用第8章中的控制器实现一个C++路径跟踪节点；3) 使用ROS将控制指令发送给CARLA。这为后续实现更复杂的规划器提供了垫脚石。

## **第四部分：前进之路：提升您的机器人专业技能**

最后一部分为持续学习提供了路线图，介绍了ROS中更高级的复杂概念，并提供了资源以确保用户的技能不断成长。

### **第10章：ROS导航栈简介**

**目标**：弥合在已知的仿真世界（CARLA）中控制车辆与在未知的真实世界环境中进行自主导航之间的差距。

**内容详情**：

* **超越CARLA**：解释虽然CARLA在控制和感知方面非常出色，但真正的自主性还需要在非结构化环境中进行建图、定位和规划。  
* **高层概览**：介绍ROS导航栈（ROS1中的move\_base，ROS2中的Nav2）的概念 35。  
* **核心组件**：对以下核心概念进行简要的、概念性的解释：  
  * **SLAM（同步定位与建图）**：在构建地图的同时跟踪机器人在地图中的位置（例如，gmapping）35。  
  * **定位**：在预先存在的地图上确定机器人的位置（例如，AMCL \- 自适应蒙特卡洛定位）35。  
  * **路径规划**：全局规划器（在整个地图上寻找路径）和局部规划器（避开即时障碍物）的角色 35。  
* **推荐仓库**：向用户推荐结构化的学习资源，如BV-Pradeep/RIA-ROS-Navigation-in-5-days 35，作为他们ROS学习之旅理想的下一步。

### **第11章：最佳实践与进阶资源**

**目标**：为用户装备长期成功所需的专业习惯和资源。

**内容详情**：

* **代码质量**：强调代码风格、注释以及使用cpplint等工具的重要性 5。  
* **调试**：提醒用户掌握rqt\_console、rqt\_plot、rqt\_graph和rviz等核心ROS调试工具 13。  
* **社区参与**：鼓励参与ROS Answers和Robotics Stack Exchange等论坛 37。  
* **精选进阶主题**：提供一个GitHub主题列表（例如，ros-navigation 38、stanley-controller 39、pure-pursuit-controller 40），以发现更高级和专业化的代码仓库。

### **结论**

本报告为初学者提供了一条从零开始，逐步掌握ROS、C++和CARLA仿真，并最终应用于自动驾驶系统开发的清晰路径。报告的核心建议可以归纳为以下几点：

1. **打好基础**：从一个稳定、配置完善的开发环境开始，并透彻理解ROS中最核心的两种通信模式——发布/订阅和 服务/客户端。利用GitHub上打包好的官方教程仓库可以显著降低入门门槛。  
2. **理论与实践结合**：将抽象的ROS编程技能与CARLA高保真仿真器相结合。掌握ros-bridge的配置，特别是同步模式，是进行可靠、可复现研究的关键。  
3. **借鉴与重构**：采用“解构Python，重构C++”的学习策略。通过分析现有Python项目中成熟的控制算法逻辑，并参考专注的C++算法库，学习者可以高效地用C++构建出高性能、模块化的ROS节点。  
4. **循序渐进**：从实现如车道保持等基础功能开始，逐步过渡到自动泊车等更复杂的系统。理解将计算密集型任务（如路径规划）置于C++中，而将与API交互的任务（如车辆控制）置于Python中的混合架构模式。  
5. **放眼未来**：在掌握了基于已知环境的控制后，应将学习目标转向ROS导航栈，掌握SLAM、定位和动态路径规划等技术，这是迈向真正自主机器人的必经之路。

通过遵循本指南的结构化路径和建议，初学者可以系统地建立起在机器人和自动驾驶领域进行软件开发所需的关键能力。

#### **Works cited**

1. cybergeekgyan/ROS-for-Beginners-: ROS for Beginners Basics, Motion, and OpenCV \- GitHub, accessed October 24, 2025, [https://github.com/cybergeekgyan/ROS-for-Beginners-](https://github.com/cybergeekgyan/ROS-for-Beginners-)  
2. Beginners Guide on setting up ROS Project using C++ and version control using Git, accessed October 24, 2025, [https://medium.com/@surajsapkal07/beginners-guide-on-setting-up-ros-project-using-c-and-version-control-using-git-545018cb0dc6](https://medium.com/@surajsapkal07/beginners-guide-on-setting-up-ros-project-using-c-and-version-control-using-git-545018cb0dc6)  
3. ROS/Tutorials, accessed October 24, 2025, [https://wiki.ros.org/ROS/Tutorials](https://wiki.ros.org/ROS/Tutorials)  
4. wenjiu2001/Beginner-Tutorials-ROS: Craft straightforward illustrative examples for initiating ROS program development. \- GitHub, accessed October 24, 2025, [https://github.com/wenjiu2001/Beginner-Tutorials-ROS](https://github.com/wenjiu2001/Beginner-Tutorials-ROS)  
5. VBot2410/beginner\_tutorials: ROS Beginner Tutorials \- GitHub, accessed October 24, 2025, [https://github.com/VBot2410/beginner\_tutorials](https://github.com/VBot2410/beginner_tutorials)  
6. Writing a Simple Publisher and Subscriber (C++) (plain cmake) \- ROS Wiki, accessed October 24, 2025, [https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29%28plain%20cmake%29](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29%28plain%20cmake%29)  
7. ROS/Tutorials/WritingPublisherSubscriber(c++) \- ROS Wiki, accessed October 24, 2025, [https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c++)](https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber\(c++\))  
8. Code for a simple publisher and subscriber (C++ and Python), accessed October 24, 2025, [https://people.eng.unimelb.edu.au/pbeuchat/asclinic/software/ros\_code\_pub\_and\_sub\_simple.html](https://people.eng.unimelb.edu.au/pbeuchat/asclinic/software/ros_code_pub_and_sub_simple.html)  
9. \[ROS Q\&A\] 176 \- Publisher/Subscriber tutorial \- avoid losing some ..., accessed October 24, 2025, [https://www.theconstruct.ai/ros-qa-176-publisher-subscriber-tutorial-loosing-some-messages-cpp/](https://www.theconstruct.ai/ros-qa-176-publisher-subscriber-tutorial-loosing-some-messages-cpp/)  
10. ROS tutorial \#07 Publisher & Subscriber P1 \- YouTube, accessed October 24, 2025, [https://www.youtube.com/watch?v=FjbBGnKElLY](https://www.youtube.com/watch?v=FjbBGnKElLY)  
11. sanhuezapablo/beginner\_tutorials: ROS Beginner Tutorials \- Introduction to Publisher and Subscriber, w/ the use of ROS services \- GitHub, accessed October 24, 2025, [https://github.com/sanhuezapablo/beginner\_tutorials](https://github.com/sanhuezapablo/beginner_tutorials)  
12. roscpp\_tutorials \- ROS Wiki, accessed October 24, 2025, [https://wiki.ros.org/roscpp\_tutorials](https://wiki.ros.org/roscpp_tutorials)  
13. BV-Pradeep/RIA-ROS-Basics-with-CPP-in-5-days \- GitHub, accessed October 24, 2025, [https://github.com/BV-Pradeep/RIA-ROS-Basics-with-CPP-in-5-days](https://github.com/BV-Pradeep/RIA-ROS-Basics-with-CPP-in-5-days)  
14. ROS/Tutorials/WritingServiceClient(c++) \- ROS Wiki, accessed October 24, 2025, [https://wiki.ros.org/ROS/Tutorials/WritingServiceClient(c++)](https://wiki.ros.org/ROS/Tutorials/WritingServiceClient\(c++\))  
15. Writing a simple service and client (C++) — ROS 2 documentation documentation, accessed October 24, 2025, [https://ros.ncnynl.com/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html](https://ros.ncnynl.com/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)  
16. Amin-Tgz/awesome-CARLA: CARLA resources such as tutorial, blog, code and etc https://github.com/carla-simulator/carla \- GitHub, accessed October 24, 2025, [https://github.com/Amin-Tgz/awesome-CARLA](https://github.com/Amin-Tgz/awesome-CARLA)  
17. carla-simulator/carla: Open-source simulator for autonomous driving research. \- GitHub, accessed October 24, 2025, [https://github.com/carla-simulator/carla](https://github.com/carla-simulator/carla)  
18. carla-simulator/ros-bridge \- GitHub, accessed October 24, 2025, [https://github.com/carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)  
19. Install ROS Bridge for ROS 2 \- CARLA Simulator \- CARLA documentation, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros\_installation\_ros2/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/)  
20. MPC-Berkeley/carla-ros-bridge: ROS bridge for CARLA Simulator. Goes with fork of Carla simulator. \- GitHub, accessed October 24, 2025, [https://github.com/MPC-Berkeley/carla-ros-bridge](https://github.com/MPC-Berkeley/carla-ros-bridge)  
21. lardemua/ros\_bridge: ROS Bridge Driver for Carla Simulator package : https://github.com/carla-simulator/carla \- GitHub, accessed October 24, 2025, [https://github.com/lardemua/ros\_bridge](https://github.com/lardemua/ros_bridge)  
22. The ROS bridge package \- CARLA Simulator, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/run\_ros/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/)  
23. C++ client example \- CARLA Simulator \- CARLA documentation, accessed October 24, 2025, [https://carla.readthedocs.io/en/latest/adv\_cpp\_client/](https://carla.readthedocs.io/en/latest/adv_cpp_client/)  
24. C++ reference \- CARLA Simulator \- Read the Docs, accessed October 24, 2025, [https://carla.readthedocs.io/en/latest/ref\_cpp/](https://carla.readthedocs.io/en/latest/ref_cpp/)  
25. Running Carla in C++ API · carla-simulator carla · Discussion \#6455 \- GitHub, accessed October 24, 2025, [https://github.com/carla-simulator/carla/discussions/6455](https://github.com/carla-simulator/carla/discussions/6455)  
26. mjxu96/carla-client-cpp: Carla C++ Client Example \- GitHub, accessed October 24, 2025, [https://github.com/mjxu96/carla-client-cpp](https://github.com/mjxu96/carla-client-cpp)  
27. hfoffani/PID-controller: A C++ implementation of a PID ... \- GitHub, accessed October 24, 2025, [https://github.com/hfoffani/PID-controller](https://github.com/hfoffani/PID-controller)  
28. Implementation of a simple PID controller in C++ \- GitHub, accessed October 24, 2025, [https://github.com/LukasLeonardKoening/PID-Controller](https://github.com/LukasLeonardKoening/PID-Controller)  
29. paulyehtw/Lane-Keeping-Assist-on-CARLA: Implementing ... \- GitHub, accessed October 24, 2025, [https://github.com/paulyehtw/Lane-Keeping-Assist-on-CARLA](https://github.com/paulyehtw/Lane-Keeping-Assist-on-CARLA)  
30. PatrickBaus/PID-CPP: An efficient PID controller implemented in C++, optimized for the ARM Cortex M4 platform \- GitHub, accessed October 24, 2025, [https://github.com/PatrickBaus/PID-CPP](https://github.com/PatrickBaus/PID-CPP)  
31. lbr-stack/pid: PID controller implementation in C++. \- GitHub, accessed October 24, 2025, [https://github.com/lbr-stack/pid](https://github.com/lbr-stack/pid)  
32. larics/pure\_pursuit: A ROS implementation of the pure ... \- GitHub, accessed October 24, 2025, [https://github.com/larics/pure\_pursuit](https://github.com/larics/pure_pursuit)  
33. NUS-Advanced-Robotics-Centre ... \- GitHub, accessed October 24, 2025, [https://github.com/NUS-Advanced-Robotics-Centre/ME5413\_Planning\_Project](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Planning_Project)  
34. pparmesh/Autonomous-Parking-AV \- GitHub, accessed October 24, 2025, [https://github.com/pparmesh/Autonomous-Parking-AV](https://github.com/pparmesh/Autonomous-Parking-AV)  
35. ROS-Navigation-in-5-days-Robot-Ignite-Academy \- GitHub, accessed October 24, 2025, [https://github.com/BV-Pradeep/RIA-ROS-Navigation-in-5-days](https://github.com/BV-Pradeep/RIA-ROS-Navigation-in-5-days)  
36. ROS Planning \- GitHub, accessed October 24, 2025, [https://github.com/ros-planning](https://github.com/ros-planning)  
37. ROS beginner tutorials:Writing Simple Service and Client (C++) \- need explanation, accessed October 24, 2025, [https://robotics.stackexchange.com/questions/50467/ros-beginner-tutorialswriting-simple-service-and-client-c-need-explanatio](https://robotics.stackexchange.com/questions/50467/ros-beginner-tutorialswriting-simple-service-and-client-c-need-explanatio)  
38. ros-navigation · GitHub Topics, accessed October 24, 2025, [https://github.com/topics/ros-navigation](https://github.com/topics/ros-navigation)  
39. stanley-controller · GitHub Topics, accessed October 24, 2025, [https://github.com/topics/stanley-controller?o=desc\&s=forks](https://github.com/topics/stanley-controller?o=desc&s=forks)  
40. pure-pursuit-controller · GitHub Topics, accessed October 24, 2025, [https://github.com/topics/pure-pursuit-controller](https://github.com/topics/pure-pursuit-controller)