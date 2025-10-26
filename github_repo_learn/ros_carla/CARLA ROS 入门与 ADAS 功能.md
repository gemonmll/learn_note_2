

# **自动驾驶系统开发研究手册：从环境搭建到ADAS实现的CARLA与ROS集成**

---

## **第一部分：构建仿真生态系统**

本部分为整个开发流程奠定关键基础。主要重点是理解各个组件，并成功构建一个稳定、兼容的软件栈，这通常是初学者最容易失败的环节。

### **第1节：CARLA-ROS集成的架构基础**

在深入研究技术细节之前，必须首先理解构成该强大仿真框架的三个核心组件：CARLA仿真器、机器人操作系统（ROS）以及连接两者的桥梁。它们各自扮演着独特的角色，共同构成了一个用于开发、测试和验证自动驾驶算法的完整生态系统。

#### **1.1 CARLA：高保真数字孪生**

CARLA (Car Learning to Act) 远不止一个“游戏引擎”；它是一个为自动驾驶研究而生的开源科学工具 1。CARLA的核心价值在于其能够生成物理上可信的传感器数据，为自动驾驶系统的感知算法提供接近真实的输入。它支持灵活的传感器套件配置，包括激光雷达（LIDAR）、雷达（Radar）、多类型摄像头（RGB、深度、语义分割）、IMU和GNSS等 3。

CARLA采用客户端-服务器（Client-Server）架构。服务器负责运行仿真世界、渲染图形、计算物理动态以及管理所有静态和动态参与者（如车辆、行人）。客户端则通过其强大的Python API与服务器通信，允许研究人员以编程方式控制仿真环境的方方面面，包括生成交通流、改变天气条件、生成和销毁参与者，以及从传感器中检索数据 5。这种架构将繁重的仿真计算与用户的控制逻辑分离开来，提供了极大的灵活性和可扩展性。

#### **1.2 ROS：机器人领域的标准中间件**

机器人操作系统（ROS）是一个为机器人软件开发提供服务的框架，它已成为机器人学和自动驾驶领域的行业标准。ROS并非传统意义上的操作系统，而是一个中间件，提供了一系列库和工具来帮助软件开发者创建模块化、可复用的机器人应用程序 7。

ROS的核心概念包括：

* **节点 (Nodes):** 执行特定任务的可执行程序。在自动驾驶中，一个节点可能负责激光雷达数据处理，另一个负责路径规划，还有一个负责车辆控制。  
* **话题 (Topics):** 节点之间通过发布（publish）和订阅（subscribe）消息进行通信的命名总线。例如，激光雷达节点会向/lidar\_points话题发布点云数据，而感知节点则订阅该话题以接收数据。  
* **消息 (Messages):** 在话题上传输的数据结构。ROS为常用数据类型（如图像、点云、IMU数据）定义了标准消息格式。  
* **服务 (Services):** 一种请求/响应式的通信模式，用于节点间的同步交互。

ROS的模块化特性使得构建复杂的自动驾驶软件栈变得更加容易管理。每个功能（感知、规划、控制）都可以封装在一个或多个独立的节点中，通过定义好的话题接口进行通信，实现了高度的解耦和代码复用。

#### **1.3 carla-ros-bridge：中央神经系统**

carla-ros-bridge是连接CARLA仿真器和ROS生态系统的核心组件。它本质上是一个ROS节点，其内部运行一个CARLA客户端，从而在两个独立的系统之间建立起双向通信的桥梁 3。

其工作机制可以概括为：

1. **从CARLA到ROS的翻译：** 桥接器连接到正在运行的CARLA服务器，并订阅仿真世界的状态更新。它将CARLA中的信息，如传感器数据（摄像头图像、激光雷达点云）、参与者状态（车辆位置、速度）、交通信号灯状态等，转换成标准的ROS消息格式，并发布到相应的ROS话题上 4。  
2. **从ROS到CARLA的翻译：** 同时，桥接器订阅特定的ROS话题，以接收来自其他ROS节点的指令。例如，当一个控制节点向/carla/ego\_vehicle/vehicle\_control\_cmd话题发布车辆控制消息（包含油门、刹车、转向值）时，桥接器会接收该消息，并将其转换为CARLA Python API可以理解的命令，发送给CARLA服务器以控制主车（ego vehicle）的运动 3。

通过这种方式，carla-ros-bridge有效地将CARLA仿真器“伪装”成一个真实的、带有ROS接口的机器人硬件。这使得为真实机器人开发的ROS软件栈可以无缝地在CARLA中进行测试和验证，极大地加速了开发和迭代过程。

### **第2节：环境安装与配置权威指南**

成功搭建一个稳定且版本兼容的开发环境是利用CARLA-ROS进行研究的第一步，也是最关键的一步。由于软件栈涉及多个快速迭代的开源项目，版本不匹配是导致安装失败的最常见原因。本节将提供一个明确、可复现的工作流程，帮助您避免常见陷阱。

#### **2.1 系统先决条件与硬件建议**

* **硬件要求：** 自动驾驶仿真是计算密集型任务，尤其是在图形渲染方面。强烈建议使用配备强大NVIDIA GPU（例如，RTX系列，显存不低于6 GB）的计算机，以确保仿真流畅运行 1。  
* **软件要求：** 本指南将以**Ubuntu 22.04 LTS**作为标准操作系统。此外，还需要特定版本的Python（通常是3.8或3.10，取决于CARLA和ROS的版本）、git版本控制工具以及相关的编译工具链 12。

#### **2.2 核心组件分步安装**

**1\. 安装ROS 2 Humble Hawksbill (适用于Ubuntu 22.04)**

ROS 2 Humble是与Ubuntu 22.04 LTS匹配的长期支持版本。请严格按照以下步骤操作 12：

Bash

\# 1\. 设置区域设置，确保支持UTF-8  
sudo apt update && sudo apt install locales  
sudo locale-gen en\_US en\_US.UTF-8  
sudo update-locale LC\_ALL=en\_US.UTF-8 LANG=en\_US.UTF-8  
export LANG=en\_US.UTF-8

\# 2\. 添加ROS 2的APT仓库  
sudo apt install software-properties-common  
sudo add-apt-repository universe  
sudo apt update && sudo apt install curl \-y  
sudo curl \-sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \-o /usr/share/keyrings/ros-archive-keyring.gpg  
echo "deb \[arch=$(dpkg \--print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg\] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU\_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list \> /dev/null

\# 3\. 安装ROS 2桌面完整版  
sudo apt update  
sudo apt install ros-humble-desktop-full

\# 4\. 配置环境  
echo "source /opt/ros/humble/setup.bash" \>\> \~/.bashrc  
source \~/.bashrc

**2\. 安装CARLA仿真器**

建议使用CARLA的预编译版本，以避免从源码编译的复杂过程。

Bash

\# 1\. 从CARLA的GitHub发布页面下载与您的系统匹配的版本（例如，0.9.14或0.9.15）  
\# 假设下载了 CARLA\_0.9.14.tar.gz  
mkdir \~/carla\_simulator  
tar \-xvzf CARLA\_0.9.14.tar.gz \-C \~/carla\_simulator

\# 2\. 安装CARLA Python API  
\# CARLA的Python API以.egg或.whl文件的形式提供，必须与您的Python版本匹配。  
\# 对于Ubuntu 22.04 (Python 3.10)，您可能需要从社区仓库下载适配的文件。  
\# 例如，从 gezp/carla\_ros 的发布页面下载 carla-0.9.14-py3.10-linux-x86\_64.egg  
\# 将下载的文件放入 \~/carla\_simulator/PythonAPI/carla/dist/ 目录  
pip3 install \--user \~/carla\_simulator/PythonAPI/carla/dist/carla-0.9.14-cp310-cp310-linux\_x86\_64.whl \# 如果是.whl文件  
\# 或者配置PYTHONPATH  
echo "export CARLA\_ROOT=\~/carla\_simulator" \>\> \~/.bashrc  
echo "export PYTHONPATH=$PYTHONPATH:$CARLA\_ROOT/PythonAPI/carla/dist/carla-0.9.14-py3.10-linux-x86\_64.egg" \>\> \~/.bashrc  
source \~/.bashrc

#### **2.3 构建carla-ros-bridge：驾驭Fork生态系统**

这是整个安装过程中最需要注意的环节。官方的carla-simulator/ros-bridge仓库（11）更新通常滞后于最新的ROS发行版。直接使用官方主分支在ROS 2 Humble上编译几乎一定会失败。成功的关键在于选择一个由社区维护、明确支持您所用CARLA和ROS版本的Fork仓库。

这个问题的根源在于软件栈的快速演进。当一个新的Ubuntu LTS版本发布时（如22.04），ROS社区会推出新的LTS版本（如Humble）。然而，CARLA和ros-bridge的官方维护者可能需要时间来跟进这些更新，导致在官方支持发布之前存在一个“兼容性真空期”。在此期间，社区开发者会创建分支或Fork，自行修复兼容性问题，以满足前沿研究的需求。因此，选择正确的Fork不是一个“捷径”，而是当前在现代系统上进行开发的必要步骤。

**构建步骤：**

Bash

\# 1\. 创建一个colcon工作空间  
mkdir \-p \~/carla\_ws/src  
cd \~/carla\_ws

\# 2\. 根据您的CARLA版本，克隆正确的ros-bridge Fork  
\# 示例：对于CARLA 0.9.15 和 ROS 2 Humble，使用ttgamage的Fork   
git clone \--recurse-submodules https://github.com/ttgamage/carla-ros-bridge.git src/ros-bridge  
\# 示例：对于CARLA 0.9.14 和 ROS 2 Humble，使用gezp的Fork   
\# git clone \--recurse-submodules https://github.com/gezp/carla\_ros.git \-b humble-carla-0.9.14 src/ros-bridge

\# 3\. 安装依赖项  
source /opt/ros/humble/setup.bash  
sudo apt-get update  
rosdep update  
rosdep install \--from-paths src \--ignore-src \-r \-y

\# 4\. 编译工作空间  
colcon build \--symlink-install

#### **表 2.1: 版本兼容性矩阵**

为了从根本上解决版本匹配问题，下表整合了来自不同来源的信息，为常见的系统配置提供了明确的指导路径。这张表格是确保安装成功的关键参考。

| Ubuntu 版本 | ROS 发行版 | CARLA 版本 | 推荐的 ros-bridge Fork | 关键说明 |
| :---- | :---- | :---- | :---- | :---- |
| 22.04 LTS | ROS 2 Humble | 0.9.15 | https://github.com/ttgamage/carla-ros-bridge 13 | 需要Python 3.10的客户端库。 |
| 22.04 LTS | ROS 2 Humble | 0.9.14 | https://github.com/gezp/carla\_ros 14 | Fork的发布页面提供了预编译的Python 3.10 .whl 文件。 |
| 20.04 LTS | ROS 2 Foxy | 0.9.13 | https://github.com/carla-simulator/ros-bridge (官方) 11 | 官方支持的组合，相对稳定。 |
| 18.04 LTS | ROS Melodic | 0.9.9 | https://github.com/carla-simulator/ros-bridge (官方, 检出 0.9.8 标签) 15 | 基于ROS 1的经典配置。 |

---

## **第二部分：掌握CARLA-ROS接口**

成功搭建环境后，下一步是深入理解并熟练使用carla-ros-bridge。本部分将解构桥接器的功能，并通过一个动手实践的教程，帮助您建立对仿真环境的直观认识和操控信心。

### **第3节：carla-ros-bridge深度解析**

carla-ros-bridge不仅仅是一个单一的程序，它是一个包含多个ROS包的集合，每个包都提供了特定的功能，共同构成了CARLA与ROS之间的完整接口。

#### **3.1 核心包与节点分析**

ros-bridge工作空间内通常包含以下几个关键的ROS包 9：

* carla\_ros\_bridge: 这是最核心的包，包含了启动桥接器本身的主节点。它负责处理CARLA与ROS之间的所有基本数据转换和通信。  
* carla\_spawn\_objects: 提供了一个通用的接口，允许用户通过ROS服务（service）或启动文件（launch file）在CARLA世界中生成车辆、行人、传感器等参与者（actors）。  
* carla\_manual\_control: 提供了一个基于ROS的可视化手动控制工具，类似于CARLA自带的manual\_control.py脚本。它会订阅传感器数据（如摄像头图像）并显示在一个PyGame窗口中，同时将用户的键盘输入转换为车辆控制指令发布出去。  
* carla\_ackermann\_control: 一个控制器节点，用于将标准的AckermannDrive消息（包含速度和转向角）转换为CARLA原生的CarlaEgoVehicleControl消息（包含油门、刹车、转向值）。这对于集成许多现有的自动驾驶软件栈非常有用。  
* carla\_ad\_demo: 一个完整的演示包，它提供了一键启动整个CARLA-ROS环境所需的所有配置文件和启动文件，包括生成一辆配备完整传感器的自动驾驶主车。这是学习和理解如何组织一个完整项目的绝佳起点。

#### **3.2 仿真控制：同步模式的关键性**

在CARLA-ROS仿真中，存在两种运行模式：异步模式和同步模式。虽然异步模式设置简单，但对于任何严肃的科学研究或算法验证，**同步模式都是必不可少的**。

其背后的因果关系如下：科学实验要求结果是可复现的。在一个由CARLA服务器和多个ROS节点组成的复杂分布式系统中，网络延迟和计算时间的波动是不可避免的。在异步模式下，CARLA服务器以其固定的时间步长（tick）独立运行，而ROS桥接器则尽其所能地接收和发布数据。这会导致一个严重的问题：数据不同步。例如，一个算法节点可能在同一时刻收到了来自第N个仿真时刻的摄像头图像，但却收到了来自第N+1个时刻的激光雷达点云。基于这种时间上错位的数据进行融合和决策，将导致对世界状态的错误感知，其结果是随机且不可复现的。

同步模式（通过在启动桥接器时设置synchronous\_mode:=True）解决了这个问题。在该模式下，CARLA服务器的“时钟”被暂停，它不会自动前进。相反，它会等待客户端（即carla-ros-bridge）发送一个“tick”指令后，才将仿真推进一个时间步。桥接器节点则遵循一个严格的逻辑：只有在当前仿真时刻的所有传感器数据都已成功接收并发布到ROS话题之后，它才会向服务器发送下一个“tick”指令 17。

这种机制虽然可能会降低仿真的实时运行速度，但它确保了在任何一个ROS时间戳下，所有传感器数据都严格对应于同一个CARLA仿真时刻，从而保证了数据的一致性和实验的可复现性。

**关键启动参数** 18：

* host 和 port: 连接CARLA服务器的网络地址和端口。  
* timeout: 连接服务器的超时时间。  
* fixed\_delta\_seconds: 仿真时间步长（秒）。为了保证物理仿真的稳定性，该值必须小于0.1。  
* passive: 在多客户端同步模式下使用。当设置为True时，桥接器不会主动“tick”世界，而是被动地等待其他客户端来推进仿真。

#### **表 3.1: 关键ROS话题与消息**

下表汇总了与仿真车辆和世界交互所需的最常用ROS话题，为开发者提供了一份快速参考的“API文档”。这填补了官方文档中的一个空白，整合了从各个代码库和示例中收集的实用信息。

| 功能 | ROS 话题模式 | 消息类型 | 描述 |
| :---- | :---- | :---- | :---- |
| RGB摄像头 | /carla/\<role\_name\>/camera/rgb/\<sensor\_name\>/image\_color | sensor\_msgs/Image | 来自RGB摄像头的原始图像数据。 19 |
| 激光雷达 (LiDAR) | /carla/\<role\_name\>/lidar/\<sensor\_name\>/point\_cloud | sensor\_msgs/PointCloud2 | 来自激光雷达传感器的3D点云数据。 20 |
| 惯性测量单元 (IMU) | /carla/\<role\_name\>/imu/\<sensor\_name\>/imu | sensor\_msgs/Imu | 惯性测量数据，包括角速度和线性加速度。 20 |
| 全球导航卫星系统 (GNSS) | /carla/\<role\_name\>/gnss/\<sensor\_name\>/fix | sensor\_msgs/NavSatFix | 提供经度、纬度和海拔信息。 19 |
| 车辆状态 (里程计) | /carla/\<role\_name\>/odometry | nav\_msgs/Odometry | 车辆的位姿（位置和姿态）和速度信息。 20 |
| 车辆原生控制 | /carla/\<role\_name\>/vehicle\_control\_cmd | carla\_msgs/CarlaEgoVehicleControl | 直接的油门(0-1)、转向(-1-1)、刹车(0-1)指令。 17 |
| 阿克曼控制 | /carla/\<role\_name\>/ackermann\_cmd | ackermann\_msgs/AckermannDrive | 阿克曼式控制指令（速度和转向角）。 17 |
| 目标检测 (真值) | /carla/objects | derived\_object\_msgs/ObjectArray | 世界中其他参与者（车辆、行人）的真值边界框和信息。 19 |
| 碰撞检测 | /carla/\<role\_name\>/collision | carla\_msgs/CarlaCollisionEvent | 当主车发生碰撞时发布事件消息。 19 |
| 车道侵入检测 | /carla/\<role\_name\>/lane\_invasion | carla\_msgs/CarlaLaneInvasionEvent | 当主车压到车道线时发布事件消息。 19 |

*注：\<role\_name\>通常是ego\_vehicle，\<sensor\_name\>是在生成传感器时定义的名称。*

### **第4节：您的第一次交互式仿真**

本节将通过一个完整的动手教程，指导您启动环境、可视化数据并手动控制车辆，从而将前面介绍的理论知识付诸实践。

#### **4.1 启动环境**

请打开三个独立的终端，并按顺序执行以下步骤：

**终端 1: 启动CARLA服务器**

Bash

\# 导航到CARLA安装目录  
cd \~/carla\_simulator

\# 启动仿真器。-quality-level=Low 可以降低对GPU的要求  
./CarlaUE4.sh \-quality-level=Low

您应该会看到一个CARLA仿真窗口弹出，显示一个默认的城镇地图。

**终端 2: 启动carla-ros-bridge**

Bash

\# 导航到您的colcon工作空间  
cd \~/carla\_ws

\# source工作空间的环境  
source install/setup.bash

\# 启动桥接器，并指定城镇地图和同步模式  
ros2 launch carla\_ros\_bridge carla\_ros\_bridge.launch.py town:=Town04 synchronous\_mode:=true

这个命令会启动核心的桥接节点，它将连接到CARLA服务器并开始发布世界信息。

**终端 3: 生成主车 (Ego Vehicle)**

Bash

\# 导航到您的colcon工作空间  
cd \~/carla\_ws

\# source工作空间的环境  
source install/setup.bash

\# 使用预定义的配置文件生成一辆带有传感器的车辆  
ros2 launch carla\_spawn\_objects carla\_example\_ego\_vehicle.launch.py

执行此命令后，您将在CARLA窗口中看到一辆新的主车被生成，并配备了一套默认的传感器（通常包括摄像头、激光雷达和IMU）21。

#### **4.2 使用RViz进行可视化**

RViz是ROS中功能强大的3D可视化工具，可以帮助您直观地理解仿真环境中发生的一切。

**终端 4: 启动RViz**

Bash

\# 启动RViz2  
rviz2

在RViz窗口中，您可以添加不同类型的显示插件来订阅和可视化ROS话题：

1. **添加TF：** 点击左下角的“Add”按钮，选择“TF”。这将显示坐标系转换树，让您可以看到车辆和各个传感器之间的相对位置关系 17。确保将“Global Options”中的“Fixed Frame”设置为map。  
2. **添加激光雷达点云：** 点击“Add”，选择“PointCloud2”。在“Topic”字段中，选择/carla/ego\_vehicle/lidar/lidar1/point\_cloud。您将在RViz中看到车辆周围的3D点云。  
3. **添加摄像头图像：** 点击“Add”，选择“Image”。在“Topic”字段中，选择/carla/ego\_vehicle/camera/rgb/front/image\_color。您将看到一个显示车辆前视摄像头画面的窗口。  
4. **添加目标边界框：** 点击“Add”，选择“MarkerArray”。在“Topic”字段中，选择/carla/objects。如果场景中有其他车辆或行人，您将看到代表它们的3D边界框。

#### **4.3 通过ROS进行手动控制**

现在，让我们通过ROS来控制刚刚生成的主车。

**方法一：使用carla\_manual\_control包**

这个包提供了一个方便的用户界面。

Bash

\# 在一个新的终端中执行  
cd \~/carla\_ws  
source install/setup.bash  
ros2 launch carla\_manual\_control carla\_manual\_control.launch.py

这将启动一个PyGame窗口，显示前视摄像头图像。您可以使用W, A, S, D键来控制车辆的油门、刹车和转向 9。

**方法二：直接从命令行发布控制指令**

为了更深入地理解其底层机制，我们可以直接使用ROS 2的命令行工具向控制话题发布消息。

Bash

\# 在一个新的终端中，发布一个使车辆缓慢直行的指令  
\# 让车辆以50%的油门直行  
ros2 topic pub \--once /carla/ego\_vehicle/vehicle\_control\_cmd carla\_msgs/msg/CarlaEgoVehicleControl "{throttle: 0.5, steer: 0.0, brake: 0.0}"

\# 发布一个使车辆右转的指令  
\# 让车辆以30%的油门，20%的幅度向右转向  
ros2 topic pub \--once /carla/ego\_vehicle/vehicle\_control\_cmd carla\_msgs/msg/CarlaEgoVehicleControl "{throttle: 0.3, steer: 0.2, brake: 0.0}"

通过执行这些命令，您可以看到车辆在CARLA仿真器中做出相应的动作。这清晰地展示了ROS节点如何通过发布消息来控制仿真中的参与者，这是构建任何自动驾驶功能的基础 5。

---

## **第三部分：ADAS开发入门**

本部分是报告的核心，旨在满足用户对实现简单ADAS功能的请求。它将理论与实践相结合，展示如何在ROS中构建与CARLA环境交互的感知和控制节点。

### **第5节：实现横向控制：车道保持辅助（LKA）**

车道保持辅助（Lane Keeping Assist, LKA）是一项基本的ADAS功能，其目标是自动控制车辆的转向，使其保持在当前车道内行驶。一个完整的LKA系统通常由感知模块和控制模块组成。

#### **5.1 理论基础**

**1\. 感知：基于计算机视觉的车道线检测**

感知模块的任务是从车载摄像头获取的图像中检测出车道线的位置。一个经典的、不依赖深度学习的计算机视觉流程如下 23：

* **灰度转换 (Grayscale Conversion):** 将三通道的RGB图像转换为单通道的灰度图像，以减少计算量 23。  
* **高斯模糊 (Gaussian Blur):** 对图像进行平滑处理，以去除噪声，避免在后续步骤中产生错误的边缘检测结果 23。  
* **Canny边缘检测 (Canny Edge Detection):** 这是一种多阶段的边缘检测算法，能够有效地识别图像中强度变化剧烈的区域（即边缘）26。  
* **感兴趣区域掩码 (Region of Interest Masking):** 摄像头图像中包含了天空、路边建筑等与车道线无关的区域。通过定义一个多边形（通常是梯形）来框选出路面区域，可以屏蔽掉无关信息，提高后续处理的准确性和效率 23。  
* **霍夫变换 (Hough Transform):** 经过上述处理后，图像中剩下的是代表车道线的边缘像素点。霍夫变换是一种特征提取技术，可以将图像空间中的直线（或其他形状）映射到参数空间中的点。通过在参数空间中寻找峰值，可以有效地检测出图像中的直线段，从而识别出车道线 24。

**2\. 控制：几何路径跟踪控制器**

控制模块根据感知模块提供的车道线信息，计算出使车辆沿车道中心行驶所需的转向指令。两种经典的几何控制器是：

* **纯跟踪控制器 (Pure Pursuit):** 该算法的核心思想是“追逐”前方路径上的一个“预瞄点”（look-ahead point）。控制器计算出一个圆弧，该圆弧的曲率可以使车辆从当前位置正好经过这个预瞄点。预瞄点与车辆之间的距离（预瞄距离）是一个关键的调节参数：较小的预瞄距离会使车辆更激进地回归路径，但可能导致超调和振荡；较大的预瞄距离则会使行驶轨迹更平滑，但在弯道处的路径跟踪误差可能更大 28。  
* **Stanley控制器:** 这种控制器同时考虑了两个误差：横向误差（crosstrack error），即车辆前轴中心到最近路径点的垂直距离；以及航向误差（heading error），即车辆的航向与路径切线方向之间的夹角。Stanley控制律通过一个综合公式，将这两个误差结合起来计算出所需的转向角，通常在较高速度下表现得比纯跟踪更稳定 31。

#### **5.2 ROS中的实践实现**

在ROS中，一个完整的LKA系统被实现为一个由多个节点组成的流水线（pipeline）。我们可以通过分析像Erendrgnl/Carla-Ros-Lane-Keeping-System 34这样的开源项目或paulyehtw/Lane-Keeping-Assist-on-CARLA 35中的概念来理解其架构。

一个典型的LKA系统ROS架构如下：

1. **感知节点 (/perception\_node):**  
   * **订阅:** 订阅来自carla-ros-bridge的摄像头图像话题，例如/carla/ego\_vehicle/camera/rgb/front/image\_color。  
   * **处理:** 在回调函数中，该节点执行上一节描述的OpenCV计算机视觉流程。它将ROS的sensor\_msgs/Image消息转换为OpenCV图像格式，进行车道线检测。  
   * **发布:** 处理的结果不是简单的图像，而是对车辆控制有用的信息，例如计算出的车道中心线的多项式系数，或者一个目标路径点。这些信息被发布到一个自定义的话题上，例如/lka/target\_path。  
2. **控制节点 (/control\_node):**  
   * **订阅:** 订阅感知节点发布的/lka/target\_path话题，以及车辆的当前状态（如位置和速度），通常来自/carla/ego\_vehicle/odometry话题。  
   * **处理:** 该节点实现了Stanley或Pure Pursuit控制算法。它根据订阅到的目标路径和车辆当前状态，计算出为了跟踪该路径所需要的转向角。  
   * **发布:** 最后，控制节点将计算出的控制指令（包含转向角和预设的油门/速度值）打包成一个carla\_msgs/CarlaEgoVehicleControl或ackermann\_msgs/AckermannDrive消息，并发布到carla-ros-bridge监听的相应控制话题上（例如/carla/ego\_vehicle/vehicle\_control\_cmd），从而闭合整个控制环路，使车辆在CARLA中实现自动转向。

**代码片段示例 (Python \- 感知节点):**

Python

import rclpy  
from rclpy.node import Node  
from sensor\_msgs.msg import Image  
from cv\_bridge import CvBridge  
import cv2  
import numpy as np

class LaneDetectionNode(Node):  
    def \_\_init\_\_(self):  
        super().\_\_init\_\_('lane\_detection\_node')  
        self.subscription \= self.create\_subscription(  
            Image,  
            '/carla/ego\_vehicle/camera/rgb/front/image\_color',  
            self.image\_callback,  
            10)  
        self.bridge \= CvBridge()

    def image\_callback(self, msg):  
        cv\_image \= self.bridge.imgmsg\_to\_cv2(msg, "bgr8")  
          
        \# 1\. 灰度转换和高斯模糊  
        gray \= cv2.cvtColor(cv\_image, cv2.COLOR\_BGR2GRAY)  
        blur \= cv2.GaussianBlur(gray, (5, 5), 0)  
          
        \# 2\. Canny边缘检测  
        edges \= cv2.Canny(blur, 50, 150)  
          
        \# 3\. 感兴趣区域掩码  
        height, width \= edges.shape  
        mask \= np.zeros\_like(edges)  
        polygon \= np.array(\[\[(0, height), (width, height), (width // 2, height // 2)\]\], np.int32)  
        cv2.fillPoly(mask, polygon, 255)  
        masked\_edges \= cv2.bitwise\_and(edges, mask)  
          
        \# 4\. 霍夫变换检测直线  
        lines \= cv2.HoughLinesP(masked\_edges, 2, np.pi / 180, 100, np.array(), minLineLength=40, maxLineGap=5)  
          
        \#... 后续处理：平均/拟合车道线，计算目标路径...  
        \#... 发布目标路径到 /lka/target\_path...

### **第6节：实现纵向控制：ACC与AEB**

纵向控制主要关注车辆前后方向的运动，包括速度控制和碰撞避免。自适应巡航控制（ACC）和自动紧急制动（AEB）是两项关键的纵向控制功能。

#### **6.1 理论基础**

**1\. 自适应巡航控制 (Adaptive Cruise Control, ACC):**

ACC是传统巡航控制的升级版，它不仅能保持设定的速度，还能在检测到前方有较慢车辆时自动减速，与前车保持一个安全距离。当道路通畅时，它会自动加速回到设定速度。实现ACC的核心是**PID控制器** 36。

PID（Proportional-Integral-Derivative，比例-积分-微分）控制器是一种经典的反馈控制算法。它通过计算当前值与期望值之间的“误差”，并结合比例、积分、微分三个部分来生成控制输出，以最小化这个误差 37。

* **比例(P)项:** 控制作用与当前误差成正比。误差越大，控制输出越强。它提供最基本的控制响应 38。  
* **积分(I)项:** 累积过去的误差。如果系统存在持续的稳态误差（例如，由于风阻或坡度导致车辆速度始终略低于设定值），积分项会不断增大，直到误差被消除 38。  
* **微分(D)项:** 响应误差的变化率。它能预测误差的未来趋势，并提前做出反应，以防止超调和振荡，使系统响应更平稳 38。

在ACC中，误差可以是“期望跟车距离”与“实际跟车距离”之差，PID控制器的输出则对应于车辆的油门或刹车指令。

**2\. 自动紧急制动 (Automatic Emergency Braking, AEB):**

AEB系统在检测到即将发生碰撞且驾驶员没有采取行动时，会自动施加制动以避免或减轻碰撞。AEB决策的核心指标是**碰撞时间 (Time-to-Collision, TTC)** 39。

TTC的计算非常直观：

$$TTC \= \\frac{\\text{相对距离}}{\\text{相对速度}}$$

其中，相对距离是本车与前方障碍物之间的距离，相对速度是两者的速度差。TTC的物理意义是，如果两者都保持当前速度不变，将在多长时间后发生碰撞。AEB系统会设定一个或多个TTC阈值。当计算出的TTC低于某个阈值时（例如，2.0秒），系统会触发制动 40。

#### **6.2 ROS中的实践实现**

ACC和AEB的实现同样遵循ROS的模块化思想。虽然简单的AEB可以仅依赖激光雷达，但稳健的系统通常需要融合多种传感器的数据。我们将分析使用激光雷达或雷达获取前方车辆距离和相对速度的项目 42。

一个典型的AEB/ACC系统ROS架构如下：

1. **感知节点 (/perception\_node):**  
   * **订阅:** 订阅激光雷达点云话题 (/carla/ego\_vehicle/lidar/front/point\_cloud) 和/或本车的里程计话题 (/carla/ego\_vehicle/odometry)。  
   * **处理:**  
     * 处理点云数据，以识别正前方的障碍物（通常是寻找路径上最近的点簇），并计算出**相对距离**。  
     * 通过订阅本车的里程计信息获取**本车速度**。  
     * 为了计算**相对速度**，可以通过对前方障碍物的位置进行时间上的差分来估算其速度，或者更简单地，从/carla/objects真值话题中直接获取前方车辆的速度信息。  
   * **发布:** 将计算出的关键信息（如相对距离、相对速度）发布到一个自定义话题，例如/lead\_vehicle\_status。  
2. **AEB节点 (/aeb\_node):**  
   * **订阅:** 订阅/lead\_vehicle\_status话题。  
   * **处理:** 在回调函数中，根据接收到的相对距离和相对速度计算TTC。  
   * **决策与发布:** 如果TTC小于预设的安全阈值（例如，1.5秒），该节点会立即发布一个最大刹车指令（brake: 1.0）到/carla/ego\_vehicle/vehicle\_control\_cmd话题，以执行紧急制动。  
3. **ACC节点 (/acc\_node):**  
   * **订阅:** 同样订阅/lead\_vehicle\_status话题。  
   * **处理:** 该节点实现一个PID控制器。  
     * **误差计算:** error \= actual\_distance \- (safe\_time\_gap \* ego\_velocity \+ min\_distance)。这里的期望距离是一个动态值，通常由一个安全时间间隙（例如，2秒）加上一个最小静止距离组成。  
     * **PID计算:** 将此误差输入PID控制器，计算出所需的加速度或减速度。  
   * **发布:** 将PID控制器的输出转换为油门和刹车指令，并发布到/carla/ego\_vehicle/vehicle\_control\_cmd话题。如果输出为正，则转换为油门值；如果为负，则转换为刹车值。

通过分析如ethanmclark1/carla\_aebs 44（尽管它使用了强化学习，但其感知和控制分离的思想是相通的）和ZhuoyunZhong/Personalizing-ADAS-in-CARLA 45中的ACC概念，可以为构建此类系统提供参考。

---

## **第四部分：进阶资源与未来方向**

本报告为您提供了从零开始搭建CARLA-ROS环境并实现基础ADAS功能的完整指南。为了支持您在自动驾驶领域的持续学习和探索，本部分将提供一份精选的开源项目列表，并总结关键的最佳实践。

### **第7节：知名CARLA-ROS开源项目概览**

探索和学习现有的开源项目是提升技能的最有效方法之一。以下项目覆盖了从完整的自动驾驶软件栈到特定算法实现等不同层面。

#### **7.1 完整的自动驾驶软件栈**

这些项目旨在实现一个端到端的自动驾驶系统，通常包含感知、规划和控制的完整模块，是理解系统集成的绝佳案例。

* **soumya997/carla-e2e-av-stack** 8: 这个项目提供了一个即插即用的感知和控制栈，兼容CARLA 0.9.11以及ROS1 Noetic和ROS2 Humble。其功能包括加载地图、使用RViz设置目标点、基于A-LOAM的激光雷达里程计、PID路径跟踪以及IMU和LiDAR的EKF传感器融合。它为ROS1和ROS2提供了独立的实现，结构清晰。  
* **mohamedameen93/An-Autonomous-Vehicle-System-For-Carla** 7: 该项目为Udacity自动驾驶课程而构建，其ROS节点实现了自动驾驶的核心功能，包括交通灯检测与分类、车辆控制和路径点跟随。它清晰地展示了如何将感知（交通灯状态）、规划（路径点更新）和控制（DBW，Drive-by-Wire）三个子系统在ROS框架下协同工作。

#### **7.2 专门的ADAS与算法实现**

这些项目专注于解决自动驾驶中的特定问题，适合深入研究某一具体技术。

* **vignif/carla-parking** 46: 一个实现自动泊车功能的项目。它采用基于规则的开环控制方法，在预设的场景中生成两辆停好的车，并控制主车在它们之间完成泊车动作。这是一个学习低速精确操控的好例子。  
* **ZhuoyunZhong/Personalizing-ADAS-in-CARLA** 45: 这个项目专注于“个性化”ADAS，特别是自适应巡航控制（ACC）和车道变换。它通过学习驾驶员的行为数据（如期望速度、安全车头时距），使用高斯混合模型（GMM）来调整ADAS的参数，使其更符合特定驾驶员的风格。尽管它不直接使用ROS，但其算法思想对于开发更人性化的ADAS系统具有重要参考价值。  
* **ethanmclark1/carla\_aebs** 44: 一个基于强化学习的自动紧急制动系统。它创新地集成了两个神经网络：一个用于车道跟随的CNN和一个用于决策的Dueling DQN，结合视觉输入、速度和距离信息来执行制动，展示了将现代AI技术应用于传统ADAS问题的前沿方法。

#### **7.3 善用官方示例**

carla-ros-bridge本身就提供了一些非常有价值的示例包，它们是构建自定义系统的理想起点。

* **carla\_ad\_agent** 和 **carla\_ad\_demo** 9: 这两个包共同构成了一个简单的自动驾驶代理。carla\_ad\_agent实现了一个基本的行为代理，能够跟随路径、避免碰撞并遵守交通规则。carla\_ad\_demo则提供了启动这个代理所需的所有配置。通过研究和修改这两个包，您可以快速搭建起自己的自动驾驶原型，而无需从零开始编写所有基础逻辑。

### **第8节：结论：从新手到实践者**

本手册引领您完成了从环境搭建到基础ADAS开发的完整旅程。整个工作流程可以总结为四个核心步骤：**搭建 (Setup) \-\> 交互 (Interface) \-\> 实现 (Implement) \-\> 测试 (Test)**。

**最佳实践总结:**

* **版本控制至关重要:** 在开始任何项目之前，请使用版本兼容性矩阵（表2.1）确保您的CARLA、ROS和ros-bridge版本相互匹配。这是避免挫败感和节省调试时间的最重要一步。  
* **坚持使用同步模式:** 对于任何需要可复现结果的开发和测试工作，请始终启用同步模式。这是保证数据一致性和算法可靠性的基石。  
* **将RViz作为您的调试利器:** 充分利用RViz来可视化传感器数据、车辆位姿、规划路径和内部状态。直观地看到系统正在“想”什么，是调试复杂算法的最高效方式。  
* **从模块化开始:** 遵循ROS的设计哲学，将您的系统分解为独立的、功能单一的节点。这不仅使代码更易于管理和测试，也提高了其可复用性。

未来方向:  
随着您对基础知识的掌握，可以向更高级的主题迈进：

* **CARLA Scenario Runner:** 这是CARLA官方提供的一个工具，用于执行基于OpenSCENARIO标准的交通场景测试。通过carla\_ros\_scenario\_runner包，您可以将这些标准化的测试场景集成到ROS工作流程中，对您的自动驾驶算法进行系统化、标准化的验证 9。  
* **多智能体与协同仿真:** CARLA支持与其他仿真器进行协同仿真，最著名的是与交通流仿真器SUMO的集成 4。这使得您可以在宏观层面模拟复杂的城市交通流（由SUMO负责），同时在微观层面精确模拟自动驾驶车辆及其传感器的行为（由CARLA负责），为更高级的V2X（Vehicle-to-Everything）通信和群体智能算法研究提供了可能。

自动驾驶是一个充满挑战和机遇的跨学科领域。通过将CARLA的高保真仿真能力与ROS强大的软件生态系统相结合，您已经拥有了一个世界一流的开发和研究平台。希望本手册能成为您探索这个激动人心领域征程中的坚实起点。

#### **Works cited**

1. carla-simulator/carla: Open-source simulator for autonomous driving research. \- GitHub, accessed October 24, 2025, [https://github.com/carla-simulator/carla](https://github.com/carla-simulator/carla)  
2. CARLA Simulator, accessed October 24, 2025, [https://carla.org/](https://carla.org/)  
3. ROS Bridge \- CARLA Simulator, accessed October 24, 2025, [https://carla.readthedocs.io/en/latest/ros\_documentation/](https://carla.readthedocs.io/en/latest/ros_documentation/)  
4. 3rd Party Integrations \- CARLA Simulator, accessed October 24, 2025, [https://carla.readthedocs.io/en/latest/3rd\_party\_integrations/](https://carla.readthedocs.io/en/latest/3rd_party_integrations/)  
5. ROS-CARLA Integration \> How to use ROS Bridge package \- Semi-Automatic Labelling for Atlascar using Adaptive Perception, accessed October 24, 2025, [https://silvamfpedro.github.io/thesis-blog/manual.html](https://silvamfpedro.github.io/thesis-blog/manual.html)  
6. UrbanPistek/carla\_ros\_simulation: A simulation stack using Docker to run CARLA with ROS nodes and additional tooling such as Foxglove. \- GitHub, accessed October 24, 2025, [https://github.com/UrbanPistek/carla\_ros\_simulation](https://github.com/UrbanPistek/carla_ros_simulation)  
7. mohamedameen93/An-Autonomous-Vehicle-System-For-Carla: In this project, we built ROS nodes to implement the core functionality of the autonomous vehicle system, including traffic light detection and classification, vehicle control control, and waypoint following. \- GitHub, accessed October 24, 2025, [https://github.com/mohamedameen93/An-Autonomous-Vehicle-System-For-Carla](https://github.com/mohamedameen93/An-Autonomous-Vehicle-System-For-Carla)  
8. soumya997/carla-e2e-av-stack: This repository contains the ... \- GitHub, accessed October 24, 2025, [https://github.com/soumya997/carla-e2e-av-stack](https://github.com/soumya997/carla-e2e-av-stack)  
9. ROS Bridge Documentation \- CARLA Simulator \- Read the Docs, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/)  
10. Simulation Environment for Validation of Automated Lane-Keeping System, accessed October 24, 2025, [https://dspace.cvut.cz/bitstream/handle/10467/112330/Vlasak\_Sojka\_Hanzalek\_\_Simulation\_Environment\_for\_Validation\_of\_Automated\_LaneKeeping\_System\_\_%282023%29\_AAM\_369040.pdf](https://dspace.cvut.cz/bitstream/handle/10467/112330/Vlasak_Sojka_Hanzalek__Simulation_Environment_for_Validation_of_Automated_LaneKeeping_System__%282023%29_AAM_369040.pdf)  
11. carla-simulator/ros-bridge \- GitHub, accessed October 24, 2025, [https://github.com/carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)  
12. ROS2 and Carla Setup Guide for Ubuntu 22.04 \- LearnOpenCV, accessed October 24, 2025, [https://learnopencv.com/ros2-and-carla-setup-guide/](https://learnopencv.com/ros2-and-carla-setup-guide/)  
13. ttgamage/carla-ros-bridge: ROS Humble bridge for CARLA v0.9.15 \- GitHub, accessed October 24, 2025, [https://github.com/ttgamage/carla-ros-bridge](https://github.com/ttgamage/carla-ros-bridge)  
14. gezp/carla\_ros: ROS bridge for CARLA Simulator \- GitHub, accessed October 24, 2025, [https://github.com/gezp/carla\_ros](https://github.com/gezp/carla_ros)  
15. Install Carla-ROS Bridge in Ubuntu 18.04 | by Francisco Maria | Medium, accessed October 24, 2025, [https://francismaria.medium.com/install-carla-ros-bridge-in-ubuntu-18-04-94fe972ec529](https://francismaria.medium.com/install-carla-ros-bridge-in-ubuntu-18-04-94fe972ec529)  
16. Ros Carla Simulator | PDF | Lidar | Computing \- Scribd, accessed October 24, 2025, [https://www.scribd.com/document/907728042/Ros-Carla-Simulator](https://www.scribd.com/document/907728042/Ros-Carla-Simulator)  
17. MPC-Berkeley/carla-ros-bridge: ROS bridge for CARLA Simulator. Goes with fork of Carla simulator. \- GitHub, accessed October 24, 2025, [https://github.com/MPC-Berkeley/carla-ros-bridge](https://github.com/MPC-Berkeley/carla-ros-bridge)  
18. The ROS bridge package \- CARLA Simulator, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/run\_ros/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/)  
19. Launchfiles reference \- CARLA Simulator \- Read the Docs, accessed October 24, 2025, [https://carla.readthedocs.io/en/0.9.8/ros\_launchs/](https://carla.readthedocs.io/en/0.9.8/ros_launchs/)  
20. CARLA sensor reference \- CARLA Simulator \- CARLA documentation, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros\_sensors/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_sensors/)  
21. lardemua/ros\_bridge: ROS Bridge Driver for Carla Simulator package : https://github.com/carla-simulator/carla \- GitHub, accessed October 24, 2025, [https://github.com/lardemua/ros\_bridge](https://github.com/lardemua/ros_bridge)  
22. Set Up and Connect to CARLA Simulator \- MATLAB & Simulink, accessed October 24, 2025, [https://www.mathworks.com/help/ros/ug/set-up-and-connect-to-carla-simulator.html](https://www.mathworks.com/help/ros/ug/set-up-and-connect-to-carla-simulator.html)  
23. OpenCV | Real Time Road Lane Detection \- GeeksforGeeks, accessed October 24, 2025, [https://www.geeksforgeeks.org/machine-learning/opencv-real-time-road-lane-detection/](https://www.geeksforgeeks.org/machine-learning/opencv-real-time-road-lane-detection/)  
24. Tutorial: Build a lane detector \- Towards Data Science, accessed October 24, 2025, [https://towardsdatascience.com/tutorial-build-a-lane-detector-679fd8953132/](https://towardsdatascience.com/tutorial-build-a-lane-detector-679fd8953132/)  
25. Hands-On Tutorial on Real-Time Lane Detection using OpenCV (Self-Driving Car Project\!), accessed October 24, 2025, [https://www.analyticsvidhya.com/blog/2020/05/tutorial-real-time-lane-detection-opencv/](https://www.analyticsvidhya.com/blog/2020/05/tutorial-real-time-lane-detection-opencv/)  
26. OpenCV Line Detection | by Amit Yadav \- Medium, accessed October 24, 2025, [https://medium.com/@amit25173/opencv-line-detection-ccf6ee026c5c](https://medium.com/@amit25173/opencv-line-detection-ccf6ee026c5c)  
27. OpenCV Python Tutorial For Beginners 31 \- Road Lane Line Detection with OpenCV (Part 1), accessed October 24, 2025, [https://www.youtube.com/watch?v=yvfI4p6Wyvk](https://www.youtube.com/watch?v=yvfI4p6Wyvk)  
28. Pure Pursuit Controller \- MATLAB & Simulink \- MathWorks, accessed October 24, 2025, [https://www.mathworks.com/help/nav/ug/pure-pursuit-controller.html](https://www.mathworks.com/help/nav/ug/pure-pursuit-controller.html)  
29. Implementation of the Pure Pursuit Path Tracking Algorithm \- CMU Robotics Institute, accessed October 24, 2025, [https://www.ri.cmu.edu/pub\_files/pub3/coulter\_r\_craig\_1992\_1/coulter\_r\_craig\_1992\_1.pdf](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)  
30. Pure Pursuit \- F1TENTH L10 \- YouTube, accessed October 24, 2025, [https://www.youtube.com/watch?v=x9s8J4ucgO0](https://www.youtube.com/watch?v=x9s8J4ucgO0)  
31. Lateral Controller Stanley \- Control steering angle of vehicle for path following by using Stanley method \- Simulink \- MathWorks, accessed October 24, 2025, [https://www.mathworks.com/help/driving/ref/lateralcontrollerstanley.html](https://www.mathworks.com/help/driving/ref/lateralcontrollerstanley.html)  
32. A Stanley Controller Design for Enhancing Vehicle Lane Keeping and Departure Performance Using Active Rear Wheel Steering | Request PDF \- ResearchGate, accessed October 24, 2025, [https://www.researchgate.net/publication/372842885\_A\_Stanley\_Controller\_Design\_for\_Enhancing\_Vehicle\_Lane\_Keeping\_and\_Departure\_Performance\_Using\_Active\_Rear\_Wheel\_Steering](https://www.researchgate.net/publication/372842885_A_Stanley_Controller_Design_for_Enhancing_Vehicle_Lane_Keeping_and_Departure_Performance_Using_Active_Rear_Wheel_Steering)  
33. Geometric explanation of Stanley controller. | Download Scientific Diagram \- ResearchGate, accessed October 24, 2025, [https://www.researchgate.net/figure/Geometric-explanation-of-Stanley-controller\_fig5\_351468458](https://www.researchgate.net/figure/Geometric-explanation-of-Stanley-controller_fig5_351468458)  
34. Erendrgnl/Carla-Ros-Lane-Keeping-System \- GitHub, accessed October 24, 2025, [https://github.com/Erendrgnl/Carla-Ros-Lane-Keeping-System](https://github.com/Erendrgnl/Carla-Ros-Lane-Keeping-System)  
35. paulyehtw/Lane-Keeping-Assist-on-CARLA: Implementing ... \- GitHub, accessed October 24, 2025, [https://github.com/paulyehtw/Lane-Keeping-Assist-on-CARLA](https://github.com/paulyehtw/Lane-Keeping-Assist-on-CARLA)  
36. Adaptive Cruise Control Method for Automobiles Based on Dynamic Spacing Strategy & PID, accessed October 24, 2025, [https://www.ewadirect.com/proceedings/ace/article/view/21693](https://www.ewadirect.com/proceedings/ace/article/view/21693)  
37. Proportional–integral–derivative controller \- Wikipedia, accessed October 24, 2025, [https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative\_controller](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)  
38. From Cruise Control to Complex Industrial Systems – A Deep Dive into PID Control, accessed October 24, 2025, [https://electronicdrives.com/home/pid-control/](https://electronicdrives.com/home/pid-control/)  
39. US20170210360A1 \- Estimated time-to-collision (ttc) calculation apparatus and estimated ttc calculation method \- Google Patents, accessed October 24, 2025, [https://patents.google.com/patent/US20170210360A1/en](https://patents.google.com/patent/US20170210360A1/en)  
40. Assessing The Safety Benefit of Automatic Collision Avoidance Systems (During Emergency Braking Situations) \- NHTSA, accessed October 24, 2025, [https://www.nhtsa.gov/sites/nhtsa.gov/files/18esv-000381.pdf](https://www.nhtsa.gov/sites/nhtsa.gov/files/18esv-000381.pdf)  
41. (PDF) Calculating the Brake-Application Time of AEB System by Considering Maximum Deceleration Rate during a Primary Accident in Penang's Urban Road \- ResearchGate, accessed October 24, 2025, [https://www.researchgate.net/publication/343650815\_Calculating\_the\_Brake-Application\_Time\_of\_AEB\_System\_by\_Considering\_Maximum\_Deceleration\_Rate\_during\_a\_Primary\_Accident\_in\_Penang's\_Urban\_Road](https://www.researchgate.net/publication/343650815_Calculating_the_Brake-Application_Time_of_AEB_System_by_Considering_Maximum_Deceleration_Rate_during_a_Primary_Accident_in_Penang's_Urban_Road)  
42. Digital Twin for CAV — Part 2 (CARLA, ROS2 and Simulink Implementation 🏗️) \- Medium, accessed October 24, 2025, [https://medium.com/networkers-fiit-stu/digital-twin-for-automated-vehicles-using-carla-simulator-ros2-and-matlab-simulink-e87df45c6a6a](https://medium.com/networkers-fiit-stu/digital-twin-for-automated-vehicles-using-carla-simulator-ros2-and-matlab-simulink-e87df45c6a6a)  
43. Evaluation and Optimization of Adaptive Cruise Control in Autonomous Vehicles using the CARLA Simulator: A Study on Performance \- arXiv, accessed October 24, 2025, [https://arxiv.org/pdf/2405.01504](https://arxiv.org/pdf/2405.01504)  
44. ethanmclark1/carla\_aebs: Reinforcement learning based ... \- GitHub, accessed October 24, 2025, [https://github.com/ethanmclark1/carla\_aebs](https://github.com/ethanmclark1/carla_aebs)  
45. ZhuoyunZhong/Personalizing-ADAS-in-CARLA ... \- GitHub, accessed October 24, 2025, [https://github.com/ZhuoyunZhong/Personalizing-ADAS-in-CARLA](https://github.com/ZhuoyunZhong/Personalizing-ADAS-in-CARLA)  
46. vignif/carla-parking: carla simulator for autonomous driving research \- GitHub, accessed October 24, 2025, [https://github.com/vignif/carla-parking](https://github.com/vignif/carla-parking)