

# **超越基础：一份面向学习者的TurtleBot3 ROS 2仿真资源库深度指南**

TurtleBot3 (TB3) 已成为机器人操作系统 (ROS) 学习生态系统中的黄金标准 1。其开源特性、相对简单的硬件配置（配备了激光雷达和可选摄像头）以及由ROBOTIS官方提供的强大支持，使其成为验证和学习机器人算法的理想平台。

然而，对于任何一个有追求的学习者来说，官方文档和基础教程只是一个起点。官方电子手册 (e-Manual) 明确指出，虽然“假节点”(fake node) 仿真适合测试基本运动，但任何涉及传感器的严肃开发，如SLAM（即时定位与地图构建）或自主导航，都必须在Gazebo等3D机器人模拟器中进行 2。

本报告旨在满足那些希望超越 ros2 launch turtlebot3\_gazebo turtlebot3\_world.launch.py 2 这一基础命令的学习者的需求。我们将深入探索ROS 2生态系统中由社区贡献的、丰富的、且往往是专业化的TurtleBot3仿真项目。本指南将这些资源库按主题领域进行划分，旨在为学习者构建一个从基础应用到前沿研究的渐进式学习路径。

## **基础：官方资源与课程驱动的生态系统**

在探索“其他”资源库之前，必须先掌握构建在其之上的基础平台。

### **官方的ROBOTIS基石**

任何TurtleBot3的学习之旅都始于ROBOTIS官方维护的两个核心代码仓库：

1. **ROBOTIS-GIT/turtlebot3** 1：包含TB3的核心ROS 2软件包、turtlebot3\_msgs消息定义以及基础节点。  
2. **ROBOTIS-GIT/turtlebot3\_simulations** 3：提供所有必需的Gazebo仿真环境、机器人模型(URDF/SDF)和启动文件。

从源代码构建这些包（如32中所示）并熟悉官方电子手册 2 是学习后续所有项目的前提。

### **结构化学习：课程驱动的代码仓库**

在官方文档之外，一个充满活力的在线课程生态系统为学习者提供了结构化的指导。分析多个社区项目后，一个显著的趋势浮出水面：**许多独立的学习者项目都源自同一个核心课程**。

多个GitHub仓库 4 明确引用了Muhammad Luqman在Udemy上开设的课程：“ROS2 Autonomous Driving and SLAM using NAV2 with TurtleBot3”。这表明该课程已成为ROS 2自主导航领域一个事实上的社区学习中心，为众多学习者提供了共同的技术基础。

这种现象的价值在于其“经社区验证”的特性。例如，Autonomous-Maze-Solving-Turtlebot3-Simulation 5 项目，其作者在完成了上述课程后，特意将项目从ament\_python重构为ament\_cmake。这不仅仅是简单的“复制粘贴”课程代码，而是一种更深层次的参与、修改和扩展，证明了该课程材料具有激发深度学习和实践的能力。

对于寻求指导性学习路径的学习者来说，这些与课程绑定的代码仓库（如33、5、4）是超越官方电子手册的最佳第一步。

## **掌握自主导航：SLAM与自主探索**

这是TB3仿真中最核心、最常见的应用领域：使机器人在未知环境中实现自主建图、定位和导航。

### **核心技术栈**

该领域的大多数项目都依赖于ROS 2导航生态系统的几个关键组件：

* **SLAM**: slam\_toolbox 5 和 Google Cartographer 7 是ROS 2中的主流选择。一些项目可能仍会使用gmapping 9 作为对比。  
* **Navigation**: Nav2 (即Navigation2) 5 是ROS 2中无可争议的标准导航栈。

### **深度剖析 1：DaniGarciaLopez/ros2\_explorer (自主探索算法对比)**

这个代码仓库 7 是一个展示“算法权衡”的绝佳案例。它基于**ROS 2 Humble**、**Cartographer**和**Nav2**构建 7。

该项目的真正价值在于，它不仅仅是*使用*Nav2，而是开发了*指挥*Nav2的高层探索逻辑。它实现了两种截然不同的自定义探索算法，为学习者提供了直观的比较：

1. **"Wanderer" (漫游者) 算法**: 这是一种简单的、反应式的策略。当检测到障碍物时，它会进行“随机转向” 7。这种方法对于小型地图很方便，但在大型地图中效率低下且耗时。  
2. **"Discoverer" (发现者) 算法**: 这是一种更复杂的、基于规划的策略。它通过“对占据栅格图进行卷积”来“优先处理地图中特定的未知热点区域” 7。这种方法在大型地图中效率更高，但代价是更高的计算成本。

对于学习者而言，这个仓库是一个宝藏。他们可以在同一个仿真环境中，通过阅读和运行代码，直观地比较两种算法的性能差异。这是一个关于计算机科学中经典的“时空权衡”（或在此处的“效率与计算成本权衡”）的完美实践课程。

### **深度剖析 2：AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2 (经典自主循环)**

这个项目 6 在**ROS 2 Humble/Foxy**上进行了测试，使用**SLAM Toolbox**和**Nav2** 6，提供了一个极为清晰的“经典自主探索”实现。

该项目完美地展示了**基于边界 (frontier-based) 的自主探索循环** 6：

1. Explorer Node (探索节点) 订阅 /map 主题以获取地图。  
2. 它检测“边界”——即位于已知自由空间和未知区域之间的单元格。  
3. 系统动态选择**最近的**未探索边界作为目标。  
4. 它向Nav2的NavigateToPose动作服务器发送一个导航目标，以启动自主导航。

这是一个在ROS 2中实现异步、基于目标的自主系统的优雅范例。通过比较这个项目 6 和ros2\_explorer 7，学习者可以对比两种核心的探索范式：“最近边界搜索” 6 与“热点区域优先” 7。

### **其他相关项目**

其他许多项目也围绕这一主题，提供了不同的实现细节，例如ROS2-FrontierBaseExplorationForAutonomousRobot 11（另一个边界探索实现）以及ros\_autonomous\_slam 12 和 Autonomous\_Exploration 13（均展示了自主建图和路径规划的完整流程）。

## **新前沿：用于导航的深度强化学习 (DRL)**

这是一个更高级的、面向研究的领域：训练一个智能体（Agent）通过试错来“学习”导航策略（例如避障），而不是由工程师显式地编写规则。

### **深度剖析 3：tomasvr/turtlebot3\_drlnav (DRL框架与环境)**

这是一个极为全面和强大的DRL导航**框架**，而不仅仅是一个项目 14。

技术配置: 它是一个基于ROS 2 Foxy和PyTorch的框架 14。  
核心价值: 其灵活性是无与伦比的。它允许在同一环境中实现、训练和评估多种主流的DRL算法，包括DQN、DDPG和TD3 14。这使学习者能够直接对比不同算法的性能。  
关键的学习点：Docker作为ML研究的必要条件  
该项目的README 14 强烈推荐使用Docker进行安装。查看其手动安装说明 14 就能明白原因：这是一个极其冗长、复杂且脆弱的过程，需要精确版本的Ubuntu (20.04)、ROS 2 (Foxy)、PyTorch、CUDA以及Nvidia Container Toolkit 14。这种环境被称为“依赖地狱”(dependency hell)。  
对于DRL研究而言，结果的复现性严格依赖于这个由ML库、GPU驱动和ROS组成的精确技术栈。因此，由Dockerfile 14 启用的Docker安装，不仅仅是一种便利，它是确保研究可复现性的*科学必要手段*。该仓库给学习者上了关于“机器人DevOps”的宝贵一课：管理环境与算法本身同样重要。

### **深度剖析 4：TannerGilbert/Turtlebot3-DRL-Navigation (生态系统的演进阵痛)**

这个代码仓库 16 专注于**TD3** (Twin Delayed Deep Deterministic Policy Gradient) 算法。然而，更有价值的信息来自作者在ROS论坛上的讨论帖子 17。

关键的学习点：Gazebo Classic vs. 现代Gazebo (Ignition/Harmonic)  
在帖子 17 中，作者（TannerGilbert）讨论了他尝试将项目迁移到“更新的Gazebo版本，包括Fortress和Harmonic”时遇到的困难。  
DRL训练对模拟器有一个独特且严苛的要求：**每小时需要进行数千次的快速、可靠的环境重置**。作者发现，在新的Gazebo中，用于DRL训练的关键功能（例如重置机器人位置）要么缺失，要么工作不正常。例如，重置环境的指令 (msg.reset.all \= True) 会*彻底删除*TurtleBot模型，导致训练中断；而用于重置位置的/world/default/set\_pose服务也无法正常桥接。

这个案例提供了一个深刻的见解：DRL仓库 16 之所以“坚守”在看似“过时”的Gazebo Classic，并不是因为开发者懒惰，而是因为*更新的*模拟器在发布时，尚未完全支持这种高强度、特定于DRL的用例。这教育学习者在选择工具时，要认识到“最新”并不总是等同于“最合适”。

### **DRL领域的其他探索**

该领域也在不断发展，包括用于多智能体强化学习 (MARL) 的项目 19 和专注于特定行为（如碰撞规避 20）的算法。

## **感知在行动：计算机视觉 (CV) 应用**

本节将焦点从基于LiDAR的导航转向基于摄像头的感知和控制。

### **深度剖析 5：gabrielnhn/ros2-line-follower (视觉伺服的"Hello, World\!")**

这个项目 21 是机器人学中经典的“感知到行动”控制循环的一个近乎完美的最小化示例。

**技术配置**: 它是一个ROS 2包，使用了**OpenCV** (cv2, cv\_bridge)、一个自定义的Gazebo世界 (new\_track.launch.py) 和一个**P控制器** 21。

关键的学习点：经典的视觉伺服循环  
根据分析 21，该项目的逻辑流程清晰地展示了视觉控制的基础：

1. **感知 (Perception)**: 使用cv\_bridge从摄像头读取图像。  
2. **处理 (Processing)**: 使用OpenCV算法（如颜色阈值、轮廓检测）找到图像中的黑线，并计算“误差”信号（例如，线的中心点与图像中心的水平像素差）。  
3. **控制 (Control)**: 将这个像素误差输入一个简单的P控制器（比例控制器）。  
4. **行动 (Action)**: P控制器的输出是一个angular.z（角速度）值，发布到/cmd\_vel主题，从而控制机器人转向以保持在黑线上。

这是视觉伺服 (visual servoing) 的“Hello, World\!”。它将抽象的控制理论（P控制器）与一个具体的、可见的任务（循线）直接联系起来。作为对比，学习者还可以参考34和35，它们展示了使用全功能**PID控制器**（比例-积分-微分）的循线实现。

### **深度剖析 6：MOGI-ROS/Week-1-8-Cognitive-robotics (从经典CV到ML的演进)**

这是一个在教学法上极为出色的现代课程仓库 22。它采用了最前沿的技术栈：**ROS 2 Jazzy** 和 **Gazebo Harmonic** 22。

关键的学习点：教学法上的演进  
该课程的巧妙之处在于它展示了一个解决方案的演进过程 22：

1. **第一步 (经典CV)**: 课程首先要求学生使用**OpenCV**实现一个循线器。  
2. **第二步 (识别失败)**: 接着，它引导学生分析这种经典方法的*失败之处*：它对光照变化极其敏感、鲁棒性差、只能理解“颜色”而不能理解“线的形状”。  
3. **第三步 (转向ML)**: 这些被识别出来的“失败”，成为了转向使用**卷积神经网络 (CNN)** 方案的*强大动机*。

该仓库随后提供了录制训练数据、构建和训练CNN的工具，以实现一个更鲁棒的循线器。这种教学方式的价值在于，学习者不仅学到了“正确的”ML解决方案，更重要的是，他们理解了*为什么*经典方法会失败，以及ML*具体解决了*哪些问题。

### **CV应用的多样性**

计算机视觉的应用远不止于循线。其他项目展示了更高级的CV任务，包括：

* **墙壁跟随**: 36（一个ROS 1项目）展示了使用PID控制器，但其输入是*激光雷达*数据。37则提到了一个反应式的墙壁跟随行为。  
* **行人跟随**: 24项目展示了一个更高级的CV任务：使用一个预训练的**FaceNet**深度学习模型来实现行人检测和跟随。

这展示了一个清晰的能力进阶：从简单的颜色检测 21，到为特定任务定制的ML模型 22，再到使用大型预训练模型进行复杂的目标识别 24。

## **扩展行动：多机器人仿真框架**

本节探讨一个更具挑战性的领域：同时仿真和协调多个机器人。

### **核心挑战：命名空间 (Namespaces) 与 TF树**

在深入研究算法之前，多机器人系统首先是一个**基础设施和架构问题**。

ROS论坛上的一个帖子 25 完美地概括了这一挑战。发帖人描述了他如何“花了三天时间”才搞明白如何生成一个TB3，而当他试图添加*第二个*机器人时，他不得不手动编辑SDF和模型文件，以硬编码方式添加一个新的命名空间 (\<namespace\>/tb3\_second\</namespace\>)。他的问题是：“有没有更简单的方法？”

这个问题，连同另一篇关于如何启动30个机器人的指南 26，揭示了多机器人仿真的真正难点：管理命名空间、确保TF（变换）树的隔离，以及避免模拟器过载。

### **深度剖析 7：arshadlab/tb3\_multi\_robot (多机器人基础设施的黄金标准)**

这个代码仓库 27 就是25中那位用户所提问题的*完美答案*。它是一个可扩展的ROS 2框架，用于在Gazebo中仿真带有**Nav2**支持的多个TB3。

**技术配置与现代性**: 这个仓库极为现代化且维护良好。其master分支支持**ROS 2 Jazzy/Gazebo Harmonic**，同时提供了humble和foxy分支 27。这使其成为一个极其宝贵的学习资源。

关键的学习点：多机器人TF的“秘密武器”  
该项目 27 解决的多机器人TF问题，是新手和中级用户最容易忽视、也是最致命的陷阱。

* **问题所在**: ROS 2的tf2\_ros库（用于坐标变换）倾向于硬编码*绝对*的主题名称，即/tf和/tf\_static 27。  
* **灾难性后果**: 在多机器人配置中，如果robot1、robot2和robot3都向同一个全局/tf主题发布它们的坐标变换，结果将是一个包含所有机器人、相互冲突、完全无法使用的“TF树大杂烩”。  
* **优雅的解决方案**: arshadlab仓库采用的解决方案是架构上的点睛之笔。它通过ROS 2的remap机制，将绝对路径的/tf重映射为*相对路径*的tf 27。当这个重映射在带命名空间的启动文件中执行时，每个机器人的TF数据就会被自动隔离在各自的命名空间下（例如，/robot1/tf、/robot2/tf、/robot3/tf）。

这个看似微小的架构决策，是实现清晰、隔离、可扩展的多机器人Nav2导航的*唯一途径*。

### **多机器人应用**

一旦有了像arshadlab这样坚实的基础设施，学习者就可以开始构建真正的多机器人应用，例如：

* **协作SLAM (C-SLAM)**: 31描述了一个项目，其中两个TB3各自构建地图，然后使用multirobot\_map\_merger包将地图融合。  
* **多机器人探索**: 38和39中列出了大量关于多机器人探索和协调的研究主题。  
* **其他框架**: 40、41和42也提供了其他实现多机器人仿真的方法。

## **现代自主性：行为树与开发工作流**

最后一个主题领域关注的不是*具体任务*，而是用于构建复杂、鲁棒的机器人行为和可复现环境的*现代方法论*。

### **深度剖析 8：sea-bass/turtlebot3\_behavior\_demos (用行为树(BT)设计任务)**

这个前沿的代码仓库 28 演示了如何使用**ROS 2 Jazzy**、Nav2和**行为树 (Behavior Trees, BTs)** 来设计自主行为。

关键的学习点：超越Nav2，构建“任务逻辑”  
Nav2本身在内部就大量使用了行为树来编排其导航逻辑（例如“规划路径”、“跟随路径”、“如果失败则恢复”）。而sea-bass这个项目 28 回答了下一个问题：“我有了能从A点到B点的Nav2，现在我如何用它来构建一个复杂的任务？”  
答案就是使用行为树作为更高一层的抽象。该项目演示了一个自定义任务：“在已知位置之间导航，目的是找到一个特定颜色（红、绿或蓝）的方块” 28。

这种方法的强大之处在于它允许以模块化、可组合的方式构建复杂的、非线性的任务逻辑。例如，一个BT可以这样设计：  
（序列：\[前往A点\] \-\> (回退：\[搜索红色方块\] \-\> \-\> \[搜索红色方块\]))  
这比传统的、脆弱的“有限状态机”(FSM) 更为强大和可扩展。该仓库的另一个巨大价值在于，它同时提供了**C++** (BehaviorTree.CPP) 和**Python** (py\_trees) 两种实现方式 28，这对于学习者来说是无价的。

### **现代工作流：Docker的普遍化**

在分析了所有这些先进的代码仓库后，一个统一的趋势变得不可忽视：**Docker已经从一个“高级工具”演变为一个“基础工具”**。

这种演变体现在多个层面：

1. **DRL领域的“必要性”**: 在turtlebot3\_drlnav 14 中，由于ML环境的极端复杂性，Docker是确保可复现性的*必要手段*。  
2. **现代自主性领域的“标准实践”**: 在turtlebot3\_behavior\_demos 28 中，Docker Compose被视为*标准工作流程*的一部分，用于“自动化构建”。  
3. **基础仿真领域的“便利性”**: 像ros2-turtlebot3-gazebo-docker 29 和ros2-turtlebot3-sim 30 这样的仓库，其*全部目的*就是为基础教程提供一个干净、隔离、开箱即用的TB3 \+ Nav2 \+ Rviz环境。

对于新一代的ROS 2学习者来说，结论是明确的：学习Docker不再是可选的。管理ROS环境的痛苦（如32中冗长的手动安装指南所示）现在已经普遍通过docker compose up来解决。Docker已成为现代ROS 2开发流程中不可或缺的核心部分。

## **综合与推荐的学习路径**

基于以上分析，我们为寻求在TB3仿真中进阶的学习者勾勒出一条推荐的学习路径：

1. **第1步：夯实基础 (必需)**  
   * **目标**: 熟练掌握TB3的基础操作和Nav2导航栈。  
   * **资源**:  
     * 官方 ROBOTIS-GIT/turtlebot3\_simulations 3 和 电子手册 2。  
     * 一个结构化的课程，例如与5、4、4相关的Udemy课程，以获得对Nav2的全面理解。  
2. **第2步：核心应用 (中级)**  
   * **目标**: 从*使用*Nav2转向*指挥*Nav2，实现完整的自主探索。  
   * **资源 (二选一)**:  
     * **A (算法对比)**: DaniGarciaLopez/ros2\_explorer 7。重点是复现并对比 "Wanderer" 和 "Discoverer" 算法的性能。  
     * **B (经典蓝图)**: AniArka/Autonomous-Explorer-and-Mapper 6。重点是理解并实现一个清晰的、基于边界的探索循环。  
3. **第3步：专业化 (高级)**  
   * **目标**: 根据个人兴趣，深入一个专业领域。  
   * **路径A：人工智能 / 机器学习**  
     1. gabrielnhn/ros2-line-follower 21：理解经典的“感知-控制”视觉伺服循环。  
     2. MOGI-ROS/Week-1-8-Cognitive-robotics 22：在最新的ROS 2 Jazzy上，体验从经典CV到CNN的演进，并理解ML的必要性。  
     3. tomasvr/turtlebot3\_drlnav 14：使用其Docker环境，深入探索DQN、DDPG和TD3等DRL算法。  
   * **路径B：鲁棒系统 / 自主性架构**  
     1. sea-bass/turtlebot3\_behavior\_demos 28：集中精力掌握行为树 (Behavior Trees)。这是构建复杂、可靠、产品级任务逻辑的途径。  
   * **路径C：集群 / 系统工程**  
     1. arshadlab/tb3\_multi\_robot 27：使用此仓库作为你的*基础设施*，学习并解决多机器人命名空间和TF的核心挑战。  
     2. 在此基础上，实现一个应用，例如Collaborative\_SLAM 31（地图融合）。

### **最终结论**

对这些代码仓库的分析揭示了一个清晰的行业趋势：虽然ROS 2 Humble仍然是当前的长期支持版 (LTS)，但最前沿的社区项目（如28、23、22、27）正在迅速向**ROS 2 Jazzy**和**Gazebo Harmonic**迁移。通过学习这些现代化的仓库，学习者不仅能掌握机器人技术本身，更能掌握最新的ROS 2工具、工作流和最佳实践。

#### **Works cited**

1. ROS packages for Turtlebot3 \- GitHub, accessed November 2, 2025, [https://github.com/ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)  
2. TurtleBot3 Simulation \- ROBOTIS e-Manual, accessed November 2, 2025, [https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)  
3. ROBOTIS-GIT/turtlebot3\_simulations: Simulations for TurtleBot3 \- GitHub, accessed November 2, 2025, [https://github.com/ROBOTIS-GIT/turtlebot3\_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)  
4. noshluk2/ROS2-Autonomous-Driving-and-Navigation ... \- GitHub, accessed November 2, 2025, [https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3](https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3)  
5. Preetamk97/Autonomous-Maze-Solving-Turtlebot3-Simulation \- GitHub, accessed November 2, 2025, [https://github.com/Preetamk97/Autonomous-Maze-Solving-Turtlebot3-Simulation](https://github.com/Preetamk97/Autonomous-Maze-Solving-Turtlebot3-Simulation)  
6. AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2: An ... \- GitHub, accessed November 2, 2025, [https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2](https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2)  
7. DaniGarciaLopez/ros2\_explorer: In this repo we use ... \- GitHub, accessed November 2, 2025, [https://github.com/DaniGarciaLopez/ros2\_explorer](https://github.com/DaniGarciaLopez/ros2_explorer)  
8. gadirajus13/Autonomous-Turtlebot: Autonomous Turtlebot3 Waffle PI capable of autonomous SLAM using Nav2 and Cartographer \- GitHub, accessed November 2, 2025, [https://github.com/gadirajus13/Autonomous-Turtlebot](https://github.com/gadirajus13/Autonomous-Turtlebot)  
9. winwinashwin/frontier\_exploration: ROS package for frontier exploration based on wavefront frontier detection \- GitHub, accessed November 2, 2025, [https://github.com/winwinashwin/frontier\_exploration](https://github.com/winwinashwin/frontier_exploration)  
10. ROS 2 Navigation \- Part 3 (creating Nav2 Simple demo with TurtleBot3) \- YouTube, accessed November 2, 2025, [https://m.youtube.com/watch?v=QP-cxh8qUJQ](https://m.youtube.com/watch?v=QP-cxh8qUJQ)  
11. abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot: Our autonomous ground vehicle uses Frontier Based exploration to navigate and map unknown environments. Equipped with sensors, it can avoid obstacles and make real-time decisions. It has potential applications in search and rescue, agriculture, and logistics, and represents an important step forward in autonomous ground vehicle development \- GitHub, accessed November 2, 2025, [https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot](https://github.com/abdulkadrtr/ROS2-FrontierBaseExplorationForAutonomousRobot)  
12. fazildgr8/ros\_autonomous\_slam: ROS package which uses the Navigation Stack to autonomously explore an unknown environment with help of GMAPPING and constructs a map of the explored environment. Finally, a path planning algorithm from the Navigation stack is used in the newly generated map to reach the goal. The Gazebo simulator is used for the simulation of \- GitHub, accessed November 2, 2025, [https://github.com/fazildgr8/ros\_autonomous\_slam](https://github.com/fazildgr8/ros_autonomous_slam)  
13. sohamsarpotdar2001/Autonomous\_Exploration: A Project on exploring autonomously and mapping an unknown environment using Turtlebot. \- GitHub, accessed November 2, 2025, [https://github.com/sohamsarpotdar2001/Autonomous\_Exploration](https://github.com/sohamsarpotdar2001/Autonomous_Exploration)  
14. tomasvr/turtlebot3\_drlnav: A ROS2-based framework for ... \- GitHub, accessed November 2, 2025, [https://github.com/tomasvr/turtlebot3\_drlnav](https://github.com/tomasvr/turtlebot3_drlnav)  
15. ROS2 Deep Reinforcement Learning Robot Navigation (TurtleBot3) : r/ROS \- Reddit, accessed November 2, 2025, [https://www.reddit.com/r/ROS/comments/137a3i0/ros2\_deep\_reinforcement\_learning\_robot\_navigation/](https://www.reddit.com/r/ROS/comments/137a3i0/ros2_deep_reinforcement_learning_robot_navigation/)  
16. TannerGilbert/Turtlebot3-DRL-Navigation: Deep ... \- GitHub, accessed November 2, 2025, [https://github.com/TannerGilbert/Turtlebot3-DRL-Navigation](https://github.com/TannerGilbert/Turtlebot3-DRL-Navigation)  
17. Turtlebot 3 Deep Reinforcement Learning Navigation \- Projects \- Open Robotics Discourse, accessed November 2, 2025, [https://discourse.openrobotics.org/t/turtlebot-3-deep-reinforcement-learning-navigation/42405](https://discourse.openrobotics.org/t/turtlebot-3-deep-reinforcement-learning-navigation/42405)  
18. amjadmajid/ROS2-DRL-Turtlebot3-like-LIDAR-Robot \- GitHub, accessed November 2, 2025, [https://github.com/amjadmajid/ROS2-DRL-Turtlebot3-like-LIDAR-Robot](https://github.com/amjadmajid/ROS2-DRL-Turtlebot3-like-LIDAR-Robot)  
19. Multi-Agent Reinforcement Learning for TurtleBot3 Using ROS2 Humble and Gazebo. \- GitHub, accessed November 2, 2025, [https://github.com/elte-collective-intelligence/student-turtlebot3s](https://github.com/elte-collective-intelligence/student-turtlebot3s)  
20. hanlinniu/turtlebot3\_ddpg\_collision\_avoidance: Mapless Collision Avoidance of Turtlebot3 Mobile Robot Using DDPG and Prioritized Experience Replay \- GitHub, accessed November 2, 2025, [https://github.com/hanlinniu/turtlebot3\_ddpg\_collision\_avoidance](https://github.com/hanlinniu/turtlebot3_ddpg_collision_avoidance)  
21. gabrielnhn/ros2-line-follower: A ROS2 package designed ... \- GitHub, accessed November 2, 2025, [https://github.com/gabrielnhn/ros2-line-follower](https://github.com/gabrielnhn/ros2-line-follower)  
22. MOGI-ROS/Week-1-8-Cognitive-robotics: Machine learning ... \- GitHub, accessed November 2, 2025, [https://github.com/MOGI-ROS/Week-1-8-Cognitive-robotics](https://github.com/MOGI-ROS/Week-1-8-Cognitive-robotics)  
23. \[TB3\] New TurtleBot3 Examples Are Here\! \- TurtleBot \- Open Robotics Discourse, accessed November 2, 2025, [https://discourse.openrobotics.org/t/tb3-new-turtlebot3-examples-are-here/43239](https://discourse.openrobotics.org/t/tb3-new-turtlebot3-examples-are-here/43239)  
24. carlosabadia/turtlebot\_person\_follower: Robot person follower using TurtleBot and ROS, accessed November 2, 2025, [https://github.com/carlosabadia/turtlebot\_person\_follower/](https://github.com/carlosabadia/turtlebot_person_follower/)  
25. Working with namespace correctly \- Multiple Turtlebot3 robots inside the same simulation in Gazebo : r/ROS \- Reddit, accessed November 2, 2025, [https://www.reddit.com/r/ROS/comments/vpntyo/working\_with\_namespace\_correctly\_multiple/](https://www.reddit.com/r/ROS/comments/vpntyo/working_with_namespace_correctly_multiple/)  
26. Creating and Managing a Multi-Robot Simulation in Gazebo with ROS 2 Foxy: A Guide for Scalable TurtleBot3 Deployment | by Arshad Mehmood | Medium, accessed November 2, 2025, [https://medium.com/@arshad.mehmood/efficient-deployment-and-operation-of-multiple-turtlebot3-robots-in-gazebos-f72f6a364620](https://medium.com/@arshad.mehmood/efficient-deployment-and-operation-of-multiple-turtlebot3-robots-in-gazebos-f72f6a364620)  
27. arshadlab/tb3\_multi\_robot \- GitHub, accessed November 2, 2025, [https://github.com/arshadlab/tb3\_multi\_robot](https://github.com/arshadlab/tb3_multi_robot)  
28. sea-bass/turtlebot3\_behavior\_demos: Example repository ... \- GitHub, accessed November 2, 2025, [https://github.com/sea-bass/turtlebot3\_behavior\_demos](https://github.com/sea-bass/turtlebot3_behavior_demos)  
29. brean/ros2-turtlebot3-gazebo-docker: ROS 2 docker image using the Gazebo simulation, accessed November 2, 2025, [https://github.com/brean/ros2-turtlebot3-gazebo-docker](https://github.com/brean/ros2-turtlebot3-gazebo-docker)  
30. apresland/ros2-turtlebot3-sim \- GitHub, accessed November 2, 2025, [https://github.com/apresland/ros2-turtlebot3-sim](https://github.com/apresland/ros2-turtlebot3-sim)  
31. GutlapalliNikhil/Collaborative\_SLAM: Multirobot SLAM \- GitHub, accessed November 2, 2025, [https://github.com/GutlapalliNikhil/Collaborative\_SLAM](https://github.com/GutlapalliNikhil/Collaborative_SLAM)  
32. twming/ros2\_turtlebot3: turtlebot3 and other robots setup on ROS2 \- GitHub, accessed November 2, 2025, [https://github.com/twming/ros2\_turtlebot3](https://github.com/twming/ros2_turtlebot3)  
33. AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control \- GitHub, accessed November 2, 2025, [https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control](https://github.com/AntoBrandi/Self-Driving-and-ROS-2-Learn-by-Doing-Odometry-Control)  
34. ROS 2 | TurtleBot3 Line Following \[Tutorial\] \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=T9KhnS3QIjs](https://www.youtube.com/watch?v=T9KhnS3QIjs)  
35. ROS 2 | TurtleBot3 Line Following with Sim2Real Transfer \[Tutorial\] \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=rYgJdqtO7TI](https://www.youtube.com/watch?v=rYgJdqtO7TI)  
36. JayshKhan/TurtleBot3-Wall-Following \- GitHub, accessed November 2, 2025, [https://github.com/JayshKhan/TurtleBot3-Wall-Following](https://github.com/JayshKhan/TurtleBot3-Wall-Following)  
37. turtlebot3-burger · GitHub Topics, accessed November 2, 2025, [https://github.com/topics/turtlebot3-burger](https://github.com/topics/turtlebot3-burger)  
38. multi-robot · GitHub Topics, accessed November 2, 2025, [https://github.com/topics/multi-robot](https://github.com/topics/multi-robot)  
39. frontier-exploration · GitHub Topics, accessed November 2, 2025, [https://github.com/topics/frontier-exploration](https://github.com/topics/frontier-exploration)  
40. Eric-nguyen1402/turtlebot3\_multi\_robot: This project is to show how to deploy multiple Turtlebot3 in the Gazebo simulation environment with ROS2 humble.. \- GitHub, accessed November 2, 2025, [https://github.com/Eric-nguyen1402/turtlebot3\_multi\_robot](https://github.com/Eric-nguyen1402/turtlebot3_multi_robot)  
41. Taeyoung96/Multi-turtlebot3-Gazebo-ROS2 \- GitHub, accessed November 2, 2025, [https://github.com/Taeyoung96/Multi-turtlebot3-Gazebo-ROS2](https://github.com/Taeyoung96/Multi-turtlebot3-Gazebo-ROS2)  
42. francocipollone/multi\_turtlebot\_sim: Multi turtlebot3 simulation \- GitHub, accessed November 2, 2025, [https://github.com/francocipollone/multi\_turtlebot\_sim](https://github.com/francocipollone/multi_turtlebot_sim)