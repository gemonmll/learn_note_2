

# **TurtleBot入门权威指南：从平台选择到代码实践 (2025版)**

## **第一部分：执行摘要：规划您的TurtleBot学习路径**

TurtleBot作为机器人操作系统（ROS）的事实标准教育平台，为全球无数开发者、研究人员和学生提供了进入机器人世界的切入点 1。您对“入门资料”的探寻，特别是对“配套资料”和“代码仓库”的明确需求，表明您寻求的是一条实践性强、资源丰富的学习路径。

本报告旨在提供一份全面、深入的技术分析，以满足这一需求。然而，在2025年的今天，一个简单的“TurtleBot入门”请求引出了一个必须首先解决的关键战略决策：您是选择成熟、模块化的 **TurtleBot 3 (TB3)** 生态系统，还是选择现代、ROS 2原生的 **TurtleBot 4 (TB4)** 平台？

这个选择并非无足轻重。它不仅决定了您的初始硬件体验，更关键的是，它将您锁定在特定的软件堆栈、仿真工具链和社区生态中。TB3拥有庞大的历史项目积淀，而TB4则是为现代ROS 2发行版（如Humble, Jazzy）和新一代仿真工具而全新设计的 3。

因此，本报告将详细剖析**两个**生态系统。我们将系统性地编目和评估它们各自的核心文档、关键GitHub代码仓库、核心技能教程（仿真、SLAM、导航）以及高级项目实例。最终目标是为您提供足够的技术深度和战略洞察，使您能够根据自己的学习目标做出明智的平台选择，并提供一个清晰的、可执行的学习蓝图。

## **第二部分：根本性决策：平台选择 (TurtleBot 3 vs. TurtleBot 4\)**

在深入研究具体的教程和代码之前，必须首先对两个主要的TurtleBot平台进行详细的比较。

### **TurtleBot 3 (ROBOTIS)：成熟的模块化平台**

TurtleBot 3是由ROBOTIS推出的一款模块化、紧凑且可定制的新一代移动机器人 1。其设计哲学侧重于DIY（自己动手）和教育价值，是教授机器人导航和运动学的优秀工具 6。它主要有两个型号：“Burger”和“Waffle” 8，其中“Waffle”型号配备了性能更强的Raspberry Pi和Pi相机 3。

### **TurtleBot 4 (Clearpath / iRobot)：现代的集成平台**

TurtleBot 4被定位为“下一代”开源机器人平台 2。它构建于iRobot® Create® 3教育机器人的坚固基础上 9。TB4的核心理念是“快速入门” 9：它**完全组装**交付，并**预装和配置了ROS 2** 9。这使其成为一个开箱即用的开发和学习平台。TB4也提供两种型号：“Standard”和“Lite” 2。

### **2025年新手的战略比较**

对于2025年的入门者来说，选择TB3还是TB4会带来截然不同的学习体验。以下是关键差异的深入分析：

1. **硬件与组装 (学习的摩擦力)**  
   * **TB3**: 学习过程从硬件组装开始。用户反馈表明，组装TB3至少需要2.5小时，这虽然具有教育意义，但对于急于开始软件开发的用户来说是一个显著的门槛 6。  
   * **TB4**: 学习过程从软件配置开始。TB4完全组装 9，允许用户立即通电，连接网络，并开始运行ROS节点。这代表了两种不同的教学理念：TB3的“通过组装学习”与TB4的“通过编码学习”。对于专注于软件和算法的用户，TB4提供了更低的学习摩擦力。  
2. **软件 (ROS 1 遗留 vs. ROS 2 原生)**  
   * **TB3**: 拥有深厚的ROS 1根基，但已全面移植到ROS 2。其代码仓库同时支持ROS 1 Noetic和ROS 2 Humble 11。  
   * **TB4**: 是**ROS 2原生**平台 3，也是第一款正式支持ROS 2的TurtleBot 4。它专为现代ROS 2发行版设计，如Humble和最新的Jazzy 2。  
3. **仿真器 (关键的技术分水岭)**  
   * **TB3**: 绝大多数教程和官方仿真包（如turtlebot3\_gazebo）都基于**Gazebo Classic** (11.x版本) 17。然而，正如社区所指出的，Gazebo Classic已进入生命周期末期(EOL) 20。  
   * **TB4**: **完全且专门**使用现代的**Gazebo (原名Ignition, 现为Gz)** 9。例如，其仿真仓库明确指出使用“Harmonic Gazebo for ROS 2 Jazzy” 21。  
   * *分析*：这是对新手而言最重要的技术决策点。在2025年投入时间学习Gazebo Classic，意味着在学习一个即将被淘汰的工具。选择TB4则迫使学习者从一开始就掌握现代的、未来导向的仿真堆栈(Gz)，这是一个更具价值的长期技能投资。  
4. **传感器与能力 (AI能力的开箱即用)**  
   * **TB3**: Waffle版配备了标准的Raspberry Pi相机 3。  
   * **TB4**: 标配了**OAK-D立体AI摄像头** (Standard版为OAK-D Pro, Lite版为OAK-D) 2。该摄像头具有板载Myriad X VPU，能够直接在边缘端运行复杂的计算机视觉和AI模型 23。  
   * *分析*：这是一个变革性的差异。TB4开箱即具备强大的空间AI能力，使其能够立即运行TB3标准配置无法实现的高级应用，例如FollowBot（跟随人）示例 24。  
5. **支持与未来保障**  
   * **TB3**: 拥有庞大的GitHub社区和历史悠久的代码库 25。  
   * **TB4**: 拥有Clearpath Robotics和Open Robotics的**积极、官方的承诺**。Clearpath已公开确认将继续生产和支持TB4硬件至2026年中，并确保对ROS 2 Jazzy等新软件版本的支持 4。

为了直观总结，下表提供了两个平台的关键指标对比：

**表 1：平台核心对比：TurtleBot 3 vs. TurtleBot 4 (2025年视角)**

| 特性 | TurtleBot 3 (ROBOTIS) | TurtleBot 4 (Clearpath / iRobot) |
| :---- | :---- | :---- |
| **核心底盘** | ROBOTIS 自研底盘 | iRobot® Create® 3 9 |
| **CPU/计算** | Raspberry Pi (Waffle版) | Raspberry Pi 4B (4GB) \[2, 23\] |
| **核心相机** | Raspberry Pi 相机 (Waffle版) 3 | OAK-D 立体AI相机 (Pro 或 Lite) \[2, 3\] |
| **核心LiDAR** | 360° 2D LiDAR (LDS-01) | 360° 2D LiDAR (RPLIDAR A1) \[2, 23\] |
| **ROS 支持** | ROS 1 (Noetic) / ROS 2 (Humble) \[11\] | ROS 2 原生 (Humble, Jazzy) \[2, 14\] |
| **仿真器** | **Gazebo Classic** (已 EOL) \[17, 20\] | **Gazebo (Gz / Ignition)** (现代) 21 |
| **组装** | **需要用户自行组装** (约 2.5+ 小时) 6 | **完全组装** 9 |
| **GitHub 组织** | ROBOTIS-GIT 25 | turtlebot 27 |
| **2025年状态** | 成熟，资料丰富，但技术栈显陈旧 | 现代，积极支持，ROS 2原生首选 |

## **第三部分：TurtleBot 3 生态系统：全面的资源指南**

如果您选择TB3路径（例如，出于预算考虑或对硬件组装的特定兴趣），以下是您需要的核心资料和代码仓库。

### **核心文档：ROBOTIS e-Manual**

所有TB3学习的起点是官方的 **ROBOTIS e-Manual** 1。这份详尽的指南涵盖了从硬件组装到软件安装、ROS基础操作和应用示例的所有内容。它还链接到了相关的ROS Wiki页面，这是另一个重要的信息来源 5。

### **核心代码仓库 (GitHub): ROBOTIS-GIT**

您请求的“代码仓库”主要集中在GitHub上的 ROBOTIS-GIT 组织内 25。这些是构建和运行TB3项目所需的基础。

**表 2：关键TurtleBot 3 GitHub代码仓库 (ROBOTIS-GIT)**

| 代码仓库 | GitHub (缩写) | 用途 |
| :---- | :---- | :---- |
| **turtlebot3** | ROBOTIS-GIT/turtlebot3 | 核心ROS 1/ROS 2软件包。包含机器人的描述文件(URDF)、bringup启动文件和基本控制节点 \[28\]。 |
| **turtlebot3\_simulations** | ROBOTIS-GIT/turtlebot3\_simulations | **关键仿真包**。包含turtlebot3\_fake\_node（用于模拟）和turtlebot3\_gazebo（Gazebo Classic仿真环境）\[11\]。该仓库有humble分支，确认了ROS 2 Humble的支持 \[11\]。 |
| **turtlebot3\_msgs** | ROBOTIS-GIT/turtlebot3\_msgs | 定义TB3使用的自定义ROS消息类型 \[11, 29\]。 |
| **turtlebot3\_autorace** | ROBOTIS-GIT/turtlebot3\_autorace | 一个用于自主驾驶的高级项目（见第六部分）30。 |
| **DynamixelSDK** | ROBOTIS-GIT/DynamixelSDK | 用于控制ROBOTIS DYNAMIXEL伺服电机（用于TB3的轮子）的软件开发工具包 \[11\]。 |
| **OpenCR** | ROBOTIS-GIT/OpenCR | TB3主控制板(OpenCR)的固件和硬件文件 \[11\]。 |

### **关键入门教程 (仿真与应用)**

#### **1\. ROS 2 设置与仿真**

* **安装**: 官方e-Manual提供了ROS 2的安装指南。虽然早期有针对Crystal版（Ubuntu 18.04）的指南 12，但现在已有更现代的ROS 2 Humble安装指南 13。  
* **启动仿真 (Gazebo Classic)**: 安装完成后，您可以在ROS 2中启动Gazebo Classic仿真环境。e-Manual提供了多个世界的启动命令 17，例如：  
  Bash  
  export TURTLEBOT3\_MODEL=waffle  
  ros2 launch turtlebot3\_gazebo turtlebot3\_world.launch.py

* **无安装选项**: 对于希望快速体验的用户，The Construct等平台提供了基于Web的仿真环境，号称5分钟内即可运行TB3仿真，无需本地安装ROS或Gazebo 31。

#### **2\. SLAM (即时定位与地图构建)**

* **多算法支持**: TB3是学习和比较不同SLAM算法的绝佳平台。官方教程和视频涵盖了**Gmapping**和**Cartographer** 32。  
* **教程实例**:  
  * 有详细的社区项目展示了如何使用Gmapping 19。  
  * 视频教程 33 展示了使用Cartographer在ROS 2中进行SLAM的过程。  
  * 一个使用ROS 2 Humble的完整Sim2Real（从仿真到真实）教程 33 则使用了**SLAM Toolbox**。

#### **3\. 导航 (Nav2 Stack)**

* **从SLAM到导航**: 官方视频教程涵盖了“Navigation” 32。  
* **核心流程 (Nav2)**: Navigation2 (Nav2) 是ROS 2中的标准导航堆栈。一个针对**物理TB3**的详尽教程 34 概括了在真实世界中运行Nav2的标准流程：  
  1. 启动TB3机器人节点。  
  2. 启动Nav2堆栈（加载先前SLAM生成的地图）。  
  3. 启动RVIZ（可视化工具）。  
  4. 在RVIZ中，使用“2D Pose Estimate”工具告知机器人它在地图上的初始位置。  
  5. 发送一个“Goal Pose”（目标姿态），机器人将自主导航。  
* **Sim2Real 视频**: 有完整的视频教程 33 展示了整个流程：使用SLAM (Cartographer) 构建地图 \-\> 保存地图 \-\> 加载地图并使用Nav2进行自主导航。

### **视频学习路径**

* **ROBOTIS 官方**: 官方YouTube播放列表 11 是最好的起点，涵盖了组装、SLAM、导航等。  
* **第三方**: Mecharithm的“ROS 101”系列 18 和其他针对初学者的教程 36 提供了额外的视角。

## **第四部分：TurtleBot 4 生态系统：现代ROS 2原生路径**

这是我们为2025年入门者**强烈推荐的路径**。TB4生态系统从一开始就为现代ROS 2工具链设计。

### **核心文档：TurtleBot 4 User Manual**

TB4的主要资源是其**官方用户手册** 2。这份由Clearpath Robotics维护的文档（托管于GitHub Pages）极其详尽，是您的一站式商店 2。

其内容结构清晰，包括 2：

* **Setup**: 基本设置、网络（这是关键步骤）。  
* **Software**: 软件概览、ROS 2包详情、**Simulation**（仿真）2。  
* **Mechanical / Electrical**: 硬件细节，包括如何添加有效载荷 38。  
* **Tutorials**: 一套完整的入门教程（驾驶、创建节点、建图、导航）。  
* **Changelogs**: 针对不同ROS 2发行版（Galactic, Humble, **Jazzy**）的更新日志 2。这证明了该平台正受到积极的、持续的软件支持。

### **核心代码仓库 (GitHub): turtlebot**

TB4的所有官方代码都托管在GitHub上的 turtlebot 组织下 27。

**表 3：关键TurtleBot 4 GitHub代码仓库 (turtlebot / Clearpath)**

| 代码仓库 | GitHub (缩写) | 用途 |
| :---- | :---- | :---- |
| **turtlebot4** | turtlebot/turtlebot4 | 核心通用软件包。包含机器人描述(URDF)、消息定义和基本bringup启动文件 \[40\]。 |
| **turtlebot4\_simulator** | turtlebot/turtlebot4\_simulator | **关键仿真包**。专为现代Gazebo (Gz) 设计 21。README明确指出，它使用 **"Harmonic Gazebo for ROS 2 Jazzy"** 21。 |
| **turtlebot4\_examples** | turtlebot/turtlebot4\_examples | 高级应用示例，例如FollowBot（见第六部分）24。 |
| **turtlebot4\_robot** | turtlebot/turtlebot4\_robot | 运行在TB4物理机器人上的特定软件包 \[27, 41\]。 |
| **turtlebot4-user-manual** | turtlebot/turtlebot4-user-manual | 官方用户手册的源文件 27。 |

### **关键入门教程 (仿真与应用)**

#### **1\. ROS 2 设置与仿真**

* **开箱即用**: TB4预装了ROS 2 9。用户手册 2 和入门视频 42 将指导您完成PC的ROS 2（Humble或Jazzy）安装和网络配置。  
* **启动仿真 (Gazebo Gz)**: 启动TB4的仿真使用了专为Gz设计的新命令 22。请注意命令中的\_gz\_（而不是TB3的\_gazebo\_）：  
  Bash  
  \# 针对ROS 2 Jazzy 和 Gz Harmonic  
  ros2 launch turtlebot4\_gz\_bringup turtlebot4\_gz.launch.py nav2:=true slam:=false localization:=true rviz:=true

  这个仿真包已作为二进制文件发布，易于安装 15。

#### **2\. SLAM (即时定位与地图构建)**

* **标准化工具**: TB4生态系统标准化地使用了 **SLAM Toolbox** 43。  
* **官方教程**: 官方手册中的“Generating a map” 2 和配套的视频 44 提供了清晰的演练。  
* **核心流程 (SLAM Toolbox)** 44：  
  1. （在仿真或真实机器人上）启动SLAM Toolbox。  
  2. 启动RVIZ进行可视化。  
  3. 使用手柄或键盘遥控机器人（teleop）探索整个环境，RVIZ中将实时显示地图构建过程 44。  
  4. 对地图满意后，运行特定命令保存地图。这将生成一个.yaml和一个.pgm文件，供导航使用 44。

#### **3\. 导航 (Nav2 Stack)**

* **官方教程**: 手册提供了“Navigation” 45 和“TurtleBot 4 Navigator” 2 两个教程。  
* **核心流程 (Nav2)** 44：  
  1. 启动Nav2堆栈，**并指定您刚刚保存的地图文件** 44。  
  2. 启动RVIZ 44。  
  3. 在RVIZ中，使用“**2D Pose Estimate**”工具点击地图，以设置机器人的初始位置和朝向 44。  
  4. 点击“**Nav2 Goal**”按钮，在地图上设置一个目标点。Nav2将立即规划路径并驱动机器人自主移动 44。  
* **高级导航**: TB4还提供了更高级的导航脚本，如nav\_through\_poses（按顺序通过一系列航点）和follow\_waypoints 22。

### **视频学习路径**

* **Clearpath 官方**: Clearpath Robotics的YouTube频道是TB4的权威视频来源。  
* **必看视频**:  
  * **"Turtlebot 4 | Unboxing & Getting Started"** 42: **这是您的第一步**。它涵盖了开箱、网络连接、控制器配对和基本遥控。  
  * **"TurtleBot 4 | Mapping & Navigation"** 44: **这是您的第二步**。它完整演练了上一节中描述的SLAM和Nav2流程。  
  * **"How to Upgrade Your TurtleBot 4 to ROS 2 Jazzy"** 16: 这个视频证明了平台的未来保障性，展示了清晰的软件升级路径。

## **第五部分：掌握核心能力：从仿真到自主导航**

分析两个平台后，我们可以提炼出入门者需要掌握的核心技能，并比较两个平台在培养这些技能上的差异。

### **技能 1：仿真 (Sim2Real的基础)**

* 这是入门最关键的第一步，也是两个平台最大的技术分歧点。  
* **TB3 路径**: 学习者将主要使用 **Gazebo Classic** 17。如前所述，这是一个已进入生命周期末期(EOL)的工具 20。您在此投入的时间可能在未来几年内迅速贬值。  
* **TB4 路径**: 学习者将使用 **Gazebo (Ignition / Gz)** 9。这是ROS 2生态系统当前和未来的标准仿真工具。  
* *结论*: 选择TB4不仅是选择一个机器人，更是选择学习**现代**仿真技术栈，这是一项更具价值的投资。

### **技能 2：SLAM (建图)**

* **TB3 路径**: 提供了一个“算法实验室”。教程涵盖了 **Gmapping** 19, **Cartographer** 32, 乃至 **SLAM Toolbox** 33。这提供了极大的灵活性，但也可能让初学者感到困惑。  
* **TB4 路径**: 标准化地采用了 **SLAM Toolbox** 43。这种标准化的方法为初学者提供了一条清晰、集成的路径，减少了“选择困难”。  
* *结论*: TB3适合希望深入比较不同经典SLAM算法的用户。TB4适合希望快速掌握一个现代、集成SLAM方案的用户。

### **技能 3：导航 (自主)**

* 这是最终的入门目标，也是两个平台**趋同**的地方。  
* **共同的收敛点**: 无论选择哪个平台，在ROS 2上实现自主导航的**核心都是 Nav2 堆栈** 33。  
* **可转移的技能**: 两个平台的Nav2教程所描述的用户流程几乎完全相同 34：(1) 加载地图, (2) 启动Nav2, (3) 在RVIZ中设置“2D Pose Estimate”, (4) 在RVIZ中发送“Nav2 Goal”。  
* *结论*: 这是最关键的启示：无论您从哪个平台开始，您学到的**核心自主导航逻辑 (Nav2) 都是100%可转移的**。平台选择影响的是仿真的*环境*和感知的*输入*（传感器），但自主的*大脑*（Nav2）是通用的。

## **第六部分：超越基础：高级项目与社区资源**

掌握了基础知识后，您将寻求更高级的挑战。两个平台都提供了丰富的“配套资料”和“代码仓库”，但侧重点不同。

### **高级项目代码库（以代码为中心）**

**表 4：精选高级项目代码仓库**

| 项目名称 | 平台 | 核心技能/概念 | 代码仓库 (GitHub) |
| :---- | :---- | :---- | :---- |
| **turtlebot3\_mapper** | TB3 | ROS 2 Humble, **Frontier Exploration**, 障碍物检测 (OpenCV), 自定义建图节点 46 | autonomous-robots/turtlebot3\_mapper |
| **turtlebot3\_autorace** | TB3 | 自主驾驶, 机器视觉 (车道检测), PID控制 30 | ROBOTIS-GIT/turtlebot3\_autorace |
| **turtlebot3\_drlnav** | TB3 | 深度强化学习 (DRL), 导航 \[47\] | tomasvr/turtlebot3\_drlnav |
| **FollowBot** | TB4 | **OAK-D AI相机**, 人体检测, 视觉伺服, 现代CV 24 | turtlebot/turtlebot4\_examples |
| **ROSCon 2025 Docker Workshop** | TB4 | **Docker**, **ROS 2 Jazzy**, **Gz Harmonic**, 现代开发工作流 48 | kscottz/turtlebot4\_docker |
| **TurtleBot4Lessons** | TB4 | 结构化课程, 幻灯片, 课堂项目 \[49\] | turtlebot/TurtleBot4Lessons |

* **TB3 (经典算法的深度)**: 如表4所示，TB3拥有大量深入研究经典机器人算法的项目 19。项目如turtlebot3\_mapper 46 是一个出色的ROS 2 Humble示例，教您如何编写自己的**边界探索（frontier exploration）和障碍物计数**算法。turtlebot3\_autorace 30 则深入研究自主驾驶。  
* **TB4 (现代AI与工作流的广度)**: TB4的项目较新，但专注于最前沿的领域。FollowBot 24 直接利用了OAK-D摄像头的AI能力。而turtlebot4\_docker 48 项目本身就是一个关于**如何使用Docker、ROS 2 Jazzy和Gz**进行现代机器人开发的ROSCon 2025研讨会。  
* *结论*: 您的选择取决于您的兴趣。TB3在“经典机器人算法”方面拥有**更深的历史积淀**。TB4在“现代AI应用和现代软件实践”方面拥有**更高质量的官方资源**。

### **社区生态系统 (获取帮助)**

* **官方渠道**: 对于Bug报告，GitHub Issues是两个平台的主要渠道 26。  
* **问答论坛 (知识库的迁移)**:  
  * **ROS Answers**: 这是历史悠久的问答网站。它包含了海量的 turtlebot 和 turtlebot3 标签下的问题和答案 55。然而，该网站现已**存档** 58。  
  * **Robotics StackExchange**: 这是ROS Answers的**现代继任者**。turtlebot4 标签在这里被创建 59，关于TB4的问题（如Humble兼容性）也在这里被提出 60。  
  * **ROS Discourse**: 这是用于高级讨论的官方论坛。TB3 61 和TB4 61 都在此被积极讨论。帖子“TB4用户们都在哪里？” 63 及其后续讨论表明，这是**TB4社区增长的中心**。  
* *结论*: 社区支持正在发生迁移。TB3的知识库**广泛但陈旧且分散**。TB4的知识库**较新但集中在活跃的、现代的平台**（StackExchange, Discourse）上。对于新手来说，在TB4上更容易找到与现代ROS 2版本相关的、未过时的帮助。

## **第七部分：专家建议与战略学习路径**

基于上述全面分析，以下是针对2025年入门者的明确建议和行动路线图。

### **明确建议：选择 TurtleBot 4**

对于在2025年寻求“入门资料、配套资料和代码仓库”的新手，**TurtleBot 4 是明确的、无保留的推荐选择**。

**理由如下：**

1. **未来保障的软件栈**: TB4是ROS 2原生的，并已获得对最新ROS 2发行版（如Humble和Jazzy）的官方支持 2。  
2. **现代的仿真工具**: 您将学习和使用的是现代Gazebo (Gz) 21，而不是已淘汰的Gazebo Classic 20。这项技能更具长期价值。  
3. **更低的入门摩擦力**: TB4开箱即用（预组装、预装ROS 2）9，让您可以跳过耗时的硬件组装 6，立即专注于您所关心的软件和代码。  
4. **卓越的传感器能力**: 标配的OAK-D AI摄像头 3 使您能够立即探索TB3标准配置无法实现的现代AI和计算机视觉项目 24。  
5. **活跃和集中的支持**: 您将获得来自Clearpath和Open Robotics的持续官方支持 4，以及一个集中在现代、活跃论坛（Discourse, StackExchange）上的、不断增长的社区 59。

何时考虑 TurtleBot 3？  
TB3仍然是一个可行的平台，但仅限于以下特定情况：(a) 您的预算极其有限，并且可以获得二手TB3；(b) 您对硬件组装过程本身的兴趣大于对软件开发的兴趣 6；(c) 您需要维护或使用一个专门为TB3/ROS 1编写的旧项目。

### **推荐的战略学习路径 (TB4行动清单)**

以下是您使用TurtleBot 4的建议入门步骤：

1. **决策**: 选择TurtleBot 4 (Lite或Standard) 2。  
2. **核心文档**: 访问并收藏 **TurtleBot 4 User Manual** 2。严格遵循“Basic Setup”和“Networking”指南完成初始设置。  
3. **首次运行**: 观看并跟随 **"Unboxing & Getting Started"** 视频 42 完成开箱、网络连接、控制器配对和首次遥控驾驶。  
4. **获取代码**: 在您的PC上克隆(clone)核心的代码仓库，尤其是 turtlebot/turtlebot4 和 turtlebot/turtlebot4\_simulator 21。  
5. **仿真入门**: 按照手册教程，在您的PC上启动Gazebo (Gz) 仿真环境 22。熟悉在仿真世界中控制机器人。  
6. **核心技能 (SLAM)**: 完整执行 **"Generating a map"** 教程 2。在仿真或现实环境中，使用SLAM Toolbox构建并保存您的第一张地图 44。  
7. **核心技能 (导航)**: 完整执行 **"Navigation"** 教程 2。使用Nav2，加载您保存的地图，在RVIZ中设置初始姿态，并发送您的第一个自主导航目标点 44。  
8. **探索示例**: 深入研究 turtlebot/turtlebot4\_examples 代码仓库 24。尝试运行 **FollowBot** 示例 24，体验OAK-D摄像头的强大AI能力。  
9. **提升工作流**: (高级) 学习 turtlebot4\_docker 项目 48，了解如何使用Docker、ROS 2 Jazzy和Gz构建可移植、可复现的现代机器人开发环境。  
10. **寻求帮助**: 当您遇到问题时，优先在 **Robotics StackExchange** 60 搜索和提问，并在 **ROS Discourse** 63 上参与讨论。

#### **Works cited**

1. TurtleBot 3 \- DYNAMIXEL SYSTEM \- Robotis U.S., accessed November 2, 2025, [https://www.robotis.us/turtlebot-3/](https://www.robotis.us/turtlebot-3/)  
2. User Manual · Turtlebot4 User Manual \- GitHub Pages, accessed November 2, 2025, [https://turtlebot.github.io/turtlebot4-user-manual/](https://turtlebot.github.io/turtlebot4-user-manual/)  
3. TurtleBot4 vs. TurtleBot3 : What are the differences and improvements? \- Génération Robots, accessed November 2, 2025, [https://www.generationrobots.com/blog/en/turtlebot4-vs-turtlebot3-what-are-the-differences-and-improvements/](https://www.generationrobots.com/blog/en/turtlebot4-vs-turtlebot3-what-are-the-differences-and-improvements/)  
4. Clearpath Robotics reaffirms support for TurtleBot 4, accessed November 2, 2025, [https://www.therobotreport.com/clearpath-robotics-reaffirms-support-for-turtlebot-4/](https://www.therobotreport.com/clearpath-robotics-reaffirms-support-for-turtlebot-4/)  
5. turtlebot3 \- ROS Wiki, accessed November 2, 2025, [https://wiki.ros.org/turtlebot3](https://wiki.ros.org/turtlebot3)  
6. TurtleBot4 VS TurtleBot3（一） \- Blog \- Technical Support \- Jingtian robot, accessed November 2, 2025, [http://www.jingtianrobots.com/en/index.php?id=931](http://www.jingtianrobots.com/en/index.php?id=931)  
7. Which ROS Robot to Buy for Education? \- ROS Developers Podcast \- The Construct, accessed November 2, 2025, [https://www.theconstruct.ai/which-ros-robot-to-buy-for-education/](https://www.theconstruct.ai/which-ros-robot-to-buy-for-education/)  
8. \[ROS Courses\] Mastering with ROS: Turtlebot 3 Robot | Robot Ignite Academy, accessed November 2, 2025, [https://www.theconstruct.ai/robotigniteacademy\_learnros/ros-courses-library/mastering-with-ros-turtlebot3/](https://www.theconstruct.ai/robotigniteacademy_learnros/ros-courses-library/mastering-with-ros-turtlebot3/)  
9. TurtleBot 4 \- Clearpath Robotics, accessed November 2, 2025, [https://clearpathrobotics.com/turtlebot-4/](https://clearpathrobotics.com/turtlebot-4/)  
10. Clearpath Robotics Launches TurtleBot 4, accessed November 2, 2025, [https://clearpathrobotics.com/blog/2022/05/clearpath-robotics-launches-turtlebot-4/](https://clearpathrobotics.com/blog/2022/05/clearpath-robotics-launches-turtlebot-4/)  
11. ROBOTIS-GIT/turtlebot3\_simulations: Simulations for ... \- GitHub, accessed November 2, 2025, [https://github.com/ROBOTIS-GIT/turtlebot3\_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)  
12. Install TurtleBot3 ROS2 Packages, accessed November 2, 2025, [https://liusanchuan.github.io/docs/en/platform/turtlebot3/ros2/](https://liusanchuan.github.io/docs/en/platform/turtlebot3/ros2/)  
13. twming/ros2\_turtlebot3: turtlebot3 and other robots setup on ROS2 \- GitHub, accessed November 2, 2025, [https://github.com/twming/ros2\_turtlebot3](https://github.com/twming/ros2_turtlebot3)  
14. TurtleBot 4 Setup · User Manual \- GitHub Pages, accessed November 2, 2025, [https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4\_setup.html](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_setup.html)  
15. ROS 2 Humble Now Available on TurtleBot 4 \- Open Robotics, accessed November 2, 2025, [https://www.openrobotics.org/blog/2023/6/5/ros-2-humble-hawksbill-now-available-on-turtlebot-4](https://www.openrobotics.org/blog/2023/6/5/ros-2-humble-hawksbill-now-available-on-turtlebot-4)  
16. How to Set Up Your TurtleBot 4 with ROS 2 Jazzy \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=TPXzNKgafJg](https://www.youtube.com/watch?v=TPXzNKgafJg)  
17. TurtleBot3 Simulation \- ROBOTIS e-Manual, accessed November 2, 2025, [https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)  
18. Turtlebot3 Gazebo Tutorial | ROS 101 | ROS Tutorials for Beginners | Lesson 6 \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=n5NaLaJHYko](https://www.youtube.com/watch?v=n5NaLaJHYko)  
19. This is a small documentation to use TurtleBot3 Simulation in ROS (Melodic) and Gazebo (9) along with slam mapping. \- GitHub, accessed November 2, 2025, [https://github.com/jayprajapati009/TurtleBot3-Simulation-with-slam-mapping](https://github.com/jayprajapati009/TurtleBot3-Simulation-with-slam-mapping)  
20. Is there a structured course/tutorial to get going with ROS, turtlesim and stuff? \- Reddit, accessed November 2, 2025, [https://www.reddit.com/r/ROS/comments/1fybi6r/is\_there\_a\_structured\_coursetutorial\_to\_get\_going/](https://www.reddit.com/r/ROS/comments/1fybi6r/is_there_a_structured_coursetutorial_to_get_going/)  
21. turtlebot/turtlebot4\_simulator: TurtleBot 4 Simulator packages \- GitHub, accessed November 2, 2025, [https://github.com/turtlebot/turtlebot4\_simulator](https://github.com/turtlebot/turtlebot4_simulator)  
22. TurtleBot 4 Navigator · User Manual \- GitHub Pages, accessed November 2, 2025, [https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4\_navigator.html](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)  
23. User Manual Turtlebot 4 \- RobotShop, accessed November 2, 2025, [https://cdn.robotshop.com/media/c/cle/rb-cle-03/pdf/user\_manual\_turtlebot\_4.pdf](https://cdn.robotshop.com/media/c/cle/rb-cle-03/pdf/user_manual_turtlebot_4.pdf)  
24. turtlebot/turtlebot4\_examples \- GitHub, accessed November 2, 2025, [https://github.com/turtlebot/turtlebot4\_examples](https://github.com/turtlebot/turtlebot4_examples)  
25. ROBOTIS \- GitHub, accessed November 2, 2025, [https://github.com/robotis-git](https://github.com/robotis-git)  
26. Clearpath Robotics Reaffirms Commitment to TurtleBot 4 Support, accessed November 2, 2025, [https://clearpathrobotics.com/blog/2025/01/clearpath-robotics-reaffirms-commitment-to-turtlebot-4-support/](https://clearpathrobotics.com/blog/2025/01/clearpath-robotics-reaffirms-commitment-to-turtlebot-4-support/)  
27. turtlebot \- GitHub, accessed November 2, 2025, [https://github.com/turtlebot](https://github.com/turtlebot)  
28. ROBOTIS-GIT/turtlebot3\_autorace: Autonomous Driving with TurtleBot3 \- GitHub, accessed November 2, 2025, [https://github.com/ROBOTIS-GIT/turtlebot3\_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)  
29. TurtleBot 3 \- The Construct, accessed November 2, 2025, [https://www.theconstruct.ai/turtlebot3/](https://www.theconstruct.ai/turtlebot3/)  
30. Videos \- TurtleBot3, accessed November 2, 2025, [https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/](https://emanual.robotis.com/docs/en/platform/turtlebot3/videos/)  
31. ROS 2 | TurtleBot3 SLAM and Autonomous Navigation with Sim2Real Transfer \[Tutorial\], accessed November 2, 2025, [https://www.youtube.com/watch?v=-4Ewuhfgbx8](https://www.youtube.com/watch?v=-4Ewuhfgbx8)  
32. Navigating with a Physical Turtlebot 3 — Nav2 1.0.0 documentation, accessed November 2, 2025, [https://docs.nav2.org/tutorials/docs/navigation2\_on\_real\_turtlebot3.html](https://docs.nav2.org/tutorials/docs/navigation2_on_real_turtlebot3.html)  
33. Learning ROS Through Simulation with TurtleBot3 and Gazebo \[Part 1\] \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=cOMtzShsGug](https://www.youtube.com/watch?v=cOMtzShsGug)  
34. Getting Started with TurtleBot3 in ROS: A Humble Beginner's Guide \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=e-7SlNDh8A8](https://www.youtube.com/watch?v=e-7SlNDh8A8)  
35. TurtleBot 4 Documentation \- Clearpath Robotics, accessed November 2, 2025, [https://clearpathrobotics.com/turtlebot-4-documentation/](https://clearpathrobotics.com/turtlebot-4-documentation/)  
36. TurtleBot 4 · User Manual \- GitHub Pages, accessed November 2, 2025, [https://turtlebot.github.io/turtlebot4-user-manual/mechanical/turtlebot4.html](https://turtlebot.github.io/turtlebot4-user-manual/mechanical/turtlebot4.html)  
37. TurtleBot 4 Humble · User Manual \- GitHub Pages, accessed November 2, 2025, [https://turtlebot.github.io/turtlebot4-user-manual/changelogs/humble.html](https://turtlebot.github.io/turtlebot4-user-manual/changelogs/humble.html)  
38. Turtlebot4 common packages. \- GitHub, accessed November 2, 2025, [https://github.com/turtlebot/turtlebot4](https://github.com/turtlebot/turtlebot4)  
39. TurtleBot 4 | Unboxing & Getting Started \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=QN01AXjoLdQ](https://www.youtube.com/watch?v=QN01AXjoLdQ)  
40. Navigating while Mapping (SLAM) — Nav2 1.0.0 documentation, accessed November 2, 2025, [https://docs.nav2.org/tutorials/docs/navigation2\_with\_slam.html](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)  
41. TurtleBot 4 | Mapping & Navigation with ROS 2 Navigation Stack \- YouTube, accessed November 2, 2025, [https://www.youtube.com/watch?v=T3if0aPj0Eo](https://www.youtube.com/watch?v=T3if0aPj0Eo)  
42. Navigation · User Manual \- GitHub Pages, accessed November 2, 2025, [https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html)  
43. autonomous-robots/turtlebot3\_mapper: Set of ROS2 nodes ... \- GitHub, accessed November 2, 2025, [https://github.com/autonomous-robots/turtlebot3\_mapper](https://github.com/autonomous-robots/turtlebot3_mapper)  
44. kscottz/turtlebot4\_docker: A Docker container for the ... \- GitHub, accessed November 2, 2025, [https://github.com/kscottz/turtlebot4\_docker](https://github.com/kscottz/turtlebot4_docker)  
45. SLAM tasks on the TurtleBot3 burger. \- GitHub, accessed November 2, 2025, [https://github.com/ctsaitsao/turtlebot3-slam](https://github.com/ctsaitsao/turtlebot3-slam)  
46. This repo consists of the code and ROS files developed to autonomously navigate in an unknown terrain WITHOUT a pre-saved map using the TurtleBot3 sources using SLAM technique \- GitHub, accessed November 2, 2025, [https://github.com/Srishivanth/Autonomous-NavigatIon-of-Turtlebot3-using-ROS](https://github.com/Srishivanth/Autonomous-NavigatIon-of-Turtlebot3-using-ROS)  
47. noshluk2/ROS2-Autonomous-Driving-and-Navigation ... \- GitHub, accessed November 2, 2025, [https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3](https://github.com/noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3)  
48. YaelBenShalom/Turtlebot3-Navigation-with-SLAM \- GitHub, accessed November 2, 2025, [https://github.com/YaelBenShalom/Turtlebot3-Navigation-with-SLAM](https://github.com/YaelBenShalom/Turtlebot3-Navigation-with-SLAM)  
49. For troubleshooting of installation/ROS errors please post on the TurtleBot3 GitHub \[LINKED\] \- Technical Support \- ROBOTIS Forum, accessed November 2, 2025, [https://forum.robotis.com/t/for-troubleshooting-of-installation-ros-errors-please-post-on-the-turtlebot3-github-linked/74](https://forum.robotis.com/t/for-troubleshooting-of-installation-ros-errors-please-post-on-the-turtlebot3-github-linked/74)  
50. FAQ \- TurtleBot, accessed November 2, 2025, [https://www.turtlebot.com/qna/](https://www.turtlebot.com/qna/)  
51. Turtlebot 3 Questions \- Robotics Stack Exchange, accessed November 2, 2025, [https://robotics.stackexchange.com/questions/83479/turtlebot-3-questions](https://robotics.stackexchange.com/questions/83479/turtlebot-3-questions)  
52. Turtlebot3 \- Start on ROS1 or ROS2? \- ROS Answers archive, accessed November 2, 2025, [https://answers.ros.org/question/363876/](https://answers.ros.org/question/363876/)  
53. ROS Answers archive, accessed November 2, 2025, [https://answers.ros.org/questions/](https://answers.ros.org/questions/)  
54. Newest 'tags' Questions \- Robotics Meta Stack Exchange, accessed November 2, 2025, [https://robotics.meta.stackexchange.com/questions/tagged/tags](https://robotics.meta.stackexchange.com/questions/tagged/tags)  
55. Turtlebot4 using ROS2 Humble \- Robotics Stack Exchange, accessed November 2, 2025, [https://robotics.stackexchange.com/questions/102081/turtlebot4-using-ros2-humble](https://robotics.stackexchange.com/questions/102081/turtlebot4-using-ros2-humble)  
56. Big Deal: A TurtleBot4 Went To The Kitchen \- Open Robotics Discourse, accessed November 2, 2025, [https://discourse.openrobotics.org/t/big-deal-a-turtlebot4-went-to-the-kitchen/42283](https://discourse.openrobotics.org/t/big-deal-a-turtlebot4-went-to-the-kitchen/42283)  
57. Latest TurtleBot topics \- Open Robotics Discourse, accessed November 2, 2025, [https://discourse.openrobotics.org/c/turtlebot/12](https://discourse.openrobotics.org/c/turtlebot/12)  
58. \[TB4\] Where are the TB4 owners hanging out? \- TurtleBot \- Open Robotics Discourse, accessed November 2, 2025, [https://discourse.openrobotics.org/t/tb4-where-are-the-tb4-owners-hanging-out/28274](https://discourse.openrobotics.org/t/tb4-where-are-the-tb4-owners-hanging-out/28274)