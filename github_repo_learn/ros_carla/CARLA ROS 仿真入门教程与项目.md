

# **面向新手的详尽指南：精通CARLA与ROS自动驾驶仿真**

## **Part 1: 基础环境搭建**

在自动驾驶的算法开发与验证领域，一个稳定、可靠且配置正确的仿真环境是所有工作的基石。CARLA与ROS的结合提供了一个功能强大的平台，但也因其组件众多、版本依赖复杂，使得环境搭建成为初学者面临的第一个，也往往是最大的挑战。本部分旨在提供一条清晰、稳妥的“黄金路径”，引导用户从零开始，构建一个功能完备的开发环境，并主动规避那些最常见的配置陷阱。

### **1.1 系统要求与版本选择的“黄金路径”**

CARLA与ROS的集成生态系统是由多个独立开发的软件组件构成的，包括操作系统、ROS发行版、CARLA模拟器本身、CARLA Python API以及连接两者的carla-ros-bridge。这些组件之间存在着严格的版本依赖关系，任何一个环节的版本错配都可能导致整个系统无法正常工作。对于初学者而言，最常见的失败原因就是随意下载各个组件的“最新版本”，从而陷入无尽的依赖冲突和编译错误中。

为了确保成功，必须遵循一条经过验证的兼容路径。这种版本间的脆弱依赖性并非一个微不足道的细节，而是搭建环境时必须首要解决的核心问题。理解这一点，即从一开始就选择一个兼容的软件组合，是后续所有工作顺利进行的前提。以下表格提供了一个清晰的版本兼容性矩阵，为用户提供一个“单一事实来源”，避免在多个官方文档之间交叉引用和猜测。

**Table 1: CARLA与ROS版本兼容性矩阵**

| 操作系统 (Ubuntu) | ROS1 发行版 | 推荐CARLA版本 | 对应 carla-ros-bridge 版本 |
| :---- | :---- | :---- | :---- |
| Ubuntu 18.04 (Bionic Beaver) | ROS Melodic | 0.9.10, 0.9.11 | 0.9.10, 0.9.11 |
| Ubuntu 20.04 (Focal Fossa) | ROS Noetic | 0.9.11, 0.9.13, 0.9.15 | 0.9.11, 0.9.13, 0.9.15 |

**选择建议：**

* 对于新项目，强烈建议使用 **Ubuntu 20.04 \+ ROS Noetic** 的组合，因为这是ROS1的最后一个长期支持版本，社区支持和软件包也最为完善 1。  
* 在选择CARLA版本时，应尽量使其与carla-ros-bridge的版本号相匹配，这能最大程度地保证API的兼容性 1。例如，若计划使用CARLA 0.9.13，则应选择同样适配0.9.13版本的carla-ros-bridge。

### **1.2 分步安装指南**

遵循上一节选定的版本组合，本节将提供详细的命令行安装步骤。

#### **1\. 安装ROS (Robot Operating System)**

ROS是机器人软件开发的框架。根据选择的Ubuntu版本，安装对应的ROS发行版。

* **对于Ubuntu 20.04 (ROS Noetic):**  
  1. **设置软件源:**  
     Bash  
     sudo sh \-c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb\_release \-sc) main" \> /etc/apt/sources.list.d/ros-latest.list'

  2. **设置密钥:**  
     Bash  
     sudo apt install curl  
     curl \-s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add \-

  3. **安装ROS:** 推荐安装包含桌面工具（如Rviz）的完整版。  
     Bash  
     sudo apt update  
     sudo apt install ros-noetic-desktop-full

  4. **环境设置:** 将ROS环境设置脚本添加到.bashrc中，以便在每个新终端中自动加载。  
     Bash  
     echo "source /opt/ros/noetic/setup.bash" \>\> \~/.bashrc  
     source \~/.bashrc

  5. **安装构建依赖:**  
     Bash  
     sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential  
     sudo rosdep init  
     rosdep update

     2  
* **对于Ubuntu 18.04 (ROS Melodic):** 安装步骤与Noetic类似，只需将命令中的noetic替换为melodic即可。

#### **2\. 安装CARLA模拟器**

CARLA的安装方式主要有Debian包安装和直接下载压缩包。考虑到官方Debian服务器可能存在不稳定的情况，并且为了更好地控制版本，**强烈推荐使用下载官方发布压缩包的方式** 2。

1. **安装依赖:** CARLA依赖一些图形和系统库。  
   Bash  
   sudo apt-get \-y install libomp5

2. **下载CARLA:** 从CARLA的GitHub发布页面 ([https://github.com/carla-simulator/carla/releases](https://github.com/carla-simulator/carla/releases)) 下载与您选择的版本对应的.tar.gz文件，例如CARLA\_0.9.15.tar.gz。文件较大（约16GB），请耐心等待.2  
   Bash  
   wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA\_0.9.15.tar.gz

3. **解压文件:** 将下载的文件解压到您选择的目录，例如/opt/carla-simulator。  
   Bash  
   sudo mkdir \-p /opt/carla-simulator  
   sudo tar \-xzvf CARLA\_0.9.15.tar.gz \-C /opt/carla-simulator/

   2

#### **3\. 安装 carla-ros-bridge**

carla-ros-bridge是连接CARLA和ROS的桥梁，它负责将CARLA中的仿真数据（如传感器读数、车辆状态）转换为ROS话题，并将ROS中的控制指令转换为CARLA中的车辆控制命令 3。安装方式有两种：Debian包和源码编译。

* 方法 A: 使用Debian包（仅推荐Ubuntu 18.04）  
  这是最简单的方法，但仅在Ubuntu 18.04上有较好的支持 1。  
  1. **设置CARLA的Debian源:**  
     Bash  
     sudo apt-key adv \--keyserver keyserver.ubuntu.com \--recv-keys 1AF1527DE64CB8D9  
     sudo add-apt-repository "deb \[arch=amd64\] http://dist.carla.org/carla $(lsb\_release \-sc) main"

  2. **安装 carla-ros-bridge:**  
     Bash  
     sudo apt update  
     \# 安装最新版  
     sudo apt install carla-ros-bridge  
     \# 或安装特定版本以匹配CARLA版本  
     \# apt-cache madison carla-ros-bridge \# 查看可用版本  
     \# sudo apt-get install carla-ros-bridge=0.9.10-1 \# 安装指定版本

     1  
* 方法 B: 从源码编译（推荐，适用于所有系统）  
  此方法更具通用性，是Ubuntu 20.04用户的必需选择，也让开发者能更灵活地修改桥接代码。  
  1. **创建Catkin工作空间:** 这是ROS项目的标准工作目录。  
     Bash  
     mkdir \-p \~/carla-ros-bridge/catkin\_ws/src  
     cd \~/carla-ros-bridge/catkin\_ws/src

  2. **克隆仓库:** 克隆ros-bridge仓库及其所有子模块。  
     Bash  
     git clone \--recurse-submodules https://github.com/carla-simulator/ros-bridge.git

  3. **安装依赖:** 使用rosdep工具自动安装所有必需的ROS包依赖。  
     Bash  
     cd \~/carla-ros-bridge/catkin\_ws  
     rosdep update  
     rosdep install \--from-paths src \--ignore-src \-r

  4. **编译工作空间:**  
     Bash  
     \# 推荐使用catkin build  
     catkin build  
     \# 或者使用传统的catkin\_make  
     \# catkin\_make

     1

### **1.3 关键环境配置**

安装完成后，还有两个至关重要的配置步骤，它们是保证系统正常通信的关键，也是初学者最容易忽略的地方。理解这些配置背后的原理，能帮助用户在遇到问题时快速定位。

整个仿真系统可以看作一个通信链条：**CARLA模拟器 (世界) \<-\> CARLA Python API (接口) \<-\> carla-ros-bridge (客户端) \<-\> ROS网络 (话题)**。配置的目的就是确保这个链条的每一环都能正确连接。

1. 设置 PYTHONPATH 以连接CARLA Python API  
   carla-ros-bridge本身是一个Python程序，它需要通过CARLA提供的Python API来与模拟器通信。为了让Python解释器能找到这个API库，必须将其路径添加到PYTHONPATH环境变量中。如果此步骤缺失，启动ros-bridge时会立即报错 "ImportError: no module named carla" 5。  
   * **定位.egg文件:** CARLA的Python API被打包成一个.egg文件，通常位于CARLA安装目录下的PythonAPI/dist/中。文件名包含了CARLA版本和Python版本信息，例如carla-0.9.15-py3.7-linux-x86\_64.egg。您需要根据您系统中的Python版本选择正确的文件。  
   * **配置环境变量:**  
     Bash  
     \# 将\<path-to-carla\>替换为您的CARLA安装根目录  
     \# 将\<carla\_version\_and\_arch\>替换为实际的.egg文件名  
     export PYTHONPATH=$PYTHONPATH:\<path-to-carla\>/PythonAPI/dist/carla-\<carla\_version\_and\_arch\>.egg

     例如，如果CARLA安装在/opt/carla-simulator，版本为0.9.15，Python为3.7:  
     Bash  
     export PYTHONPATH=$PYTHONPATH:/opt/carla-simulator/PythonAPI/dist/carla-0.9.15-py3.7-linux-x86\_64.egg

   * **验证配置:** 运行以下命令，如果成功输出"Success"，则表示配置正确。  
     Bash  
     python3 \-c 'import carla;print("Success")'

     3  
   * **永久生效:** 为了避免每次打开新终端都要重新设置，建议将export命令添加到\~/.bashrc文件的末尾，然后执行source \~/.bashrc 6。  
2. Sourcing ROS工作空间以激活ROS环境  
   为了在终端中使用ROS命令（如roslaunch）并让ROS系统找到carla-ros-bridge的功能包，必须"source"相应的工作空间设置文件。  
   * **Sourcing ROS主环境:** 这一步在安装ROS时已经添加到.bashrc中 (source /opt/ros/\<distro\>/setup.bash)。  
   * **Sourcing carla-ros-bridge工作空间:** 这一步是告诉ROS您当前要使用的工作空间。  
     * **对于Debian包安装:**  
       Bash  
       source /opt/carla-ros-bridge/\<melodic或noetic\>/setup.bash

     * **对于源码编译安装:**  
       Bash  
       source \~/carla-ros-bridge/catkin\_ws/devel/setup.bash

1与PYTHONPATH一样，建议将此命令也添加到\~/.bashrc中，以便自动加载。注意： 如果您同时处理多个ROS工作空间，最好在需要时手动source，以避免环境冲突 3。

### **1.4 Docker备选方案：一条捷径**

对于那些在本地安装过程中遇到困难，或者希望快速启动并运行仿真的用户，Docker提供了一个极佳的替代方案。Docker容器技术可以将整个复杂的环境（包括操作系统、依赖库、CARLA和ROS）打包成一个独立的、可移植的镜像，从而完全绕过本地安装的繁琐过程。

* **使用官方CARLA镜像:** CARLA官方提供了包含模拟器的Docker镜像。  
  Bash  
  \# 拉取最新镜像  
  docker pull carlasim/carla:latest  
  \# 运行容器，需要NVIDIA显卡和nvidia-docker2支持  
  docker run \\  
      \-p 2000-2002:2000-2002 \\  
      \--runtime=nvidia \--gpus all \\  
      \-it carlasim/carla:latest \\  
     ./CarlaUE4.sh

  8  
* **使用社区提供的集成镜像:** 社区中有许多项目提供了已经集成好carla-ros-bridge的Dockerfile或现成镜像 9。这些项目通常提供了一键启动脚本，可以同时启动CARLA服务器和ROS桥接，极大地简化了流程。一些在线研讨会也提供了完整的Docker化仿真栈教程 10。

选择Docker方案的优势是环境一致性和快速部署，缺点是可能需要一些Docker基础知识，并且对硬件（特别是NVIDIA GPU）的配置要求较高。对于初学者，如果本地安装遇到难以解决的问题，转向Docker是一个非常值得尝试的 pragmatic 选择。

## **Part 2: 首次仿真：从启动到控制**

成功搭建环境后，下一步就是将这个静态的软件栈激活，运行一个动态的、可交互的仿真。本部分将引导用户完成从启动模拟器到通过ROS控制车辆、接收传感器数据的完整流程，这是进行任何算法仿真的基础。

### **2.1 启动仿真程序栈**

CARLA和ROS Bridge的运行遵循经典的客户端-服务器（Client-Server）架构。CARLA模拟器本身是服务器，负责物理世界的模拟、渲染和状态更新。carla-ros-bridge则扮演客户端的角色，连接到服务器，请求数据并发送指令。因此，**必须先启动CARLA服务器，再启动ROS Bridge客户端**。

1. 启动CARLA服务器:  
   打开一个终端，进入CARLA的安装目录，运行启动脚本。  
   Bash  
   \# 假设CARLA安装在/opt/carla-simulator  
   cd /opt/carla-simulator

./CarlaUE4.sh  
\`\`\`

1

执行后，一个显示CARLA仿真世界的窗口将会出现。此时，服务器正在监听默认端口2000，等待客户端连接 11。

2. 启动ROS Bridge:  
   打开另一个新的终端（确保该终端已经正确source了ROS和carla-ros-bridge工作空间的环境变量），运行ROS Bridge的启动文件（launch file）。  
   Bash  
   \# 选项1: 仅启动ROS Bridge  
   roslaunch carla\_ros\_bridge carla\_ros\_bridge.launch

   \# 选项2: 启动ROS Bridge并自动生成一辆可控的Ego Vehicle (主车)  
   roslaunch carla\_ros\_bridge carla\_ros\_bridge\_with\_example\_ego\_vehicle.launch

   1

   启动后，您将在终端看到连接成功的日志信息。如果使用了选项2，仿真世界中会出现一辆新的汽车。

#### **同步模式的重要性**

对于算法开发和验证，一个至关重要的概念是**同步模式（Synchronous Mode）**。

* **异步模式（默认）:** 模拟器以尽可能快的速度运行，不受客户端影响。传感器数据在生成时立即发布。这会导致仿真步长不固定，数据可能丢失或乱序，使得两次运行的结果无法复现。  
* **同步模式:** 模拟器会暂停，等待一个客户端（如此处的ROS Bridge）发送“tick”指令后，才前进一个固定的时间步长。这确保了每次仿真的结果都是**可复现的** 3。对于调试控制算法或训练机器学习模型，这是必不可少的。

要启用同步模式，需要在启动ros-bridge时传递参数。同时，将use\_sim\_time参数设为true，这会告诉ROS网络中的所有节点使用由CARLA发布的仿真时间（通过/clock话题），而不是系统的真实时间，从而实现整个ROS系统与仿真时间的同步 12。

Bash

\# 启动时启用同步模式，并设置固定时间步长为0.05秒  
roslaunch carla\_ros\_bridge carla\_ros\_bridge.launch synchronous\_mode:=true fixed\_delta\_seconds:=0.05 use\_sim\_time:=true

对于任何严肃的算法仿真任务，都应始终启用同步模式。

### **2.2 与仿真交互：Rviz与核心ROS话题**

启动仿真后，我们需要工具来观察和理解仿真世界中正在发生什么。在ROS生态中，Rviz是首选的可视化工具，而rostopic命令则是检查数据流的瑞士军刀。

1. 使用Rviz进行可视化:  
   Rviz可以订阅ROS话题并以3D形式将其可视化，例如点云、摄像头图像、车辆模型等。carla-ros-bridge提供了一个集成了Rviz配置的启动文件。  
   Bash  
   \# 在一个新终端中启动  
   roslaunch carla\_ros\_bridge carla\_ros\_bridge\_with\_rviz.launch

   3

   启动后，Rviz窗口会打开，并显示来自CARLA的地图和传感器数据。  
2. 使用rostopic进行数据探查:  
   rostopic系列命令可以帮助我们查看当前活跃的话题、消息类型以及消息内容。  
   * rostopic list: 列出所有当前发布的ROS话题 13。  
   * rostopic info \<topic\_name\>: 显示特定话题的发布者、订阅者和消息类型。  
   * rostopic echo \<topic\_name\>: 在终端实时打印特定话题的消息内容，非常适合快速检查数据 13。

#### **核心ROS话题**

carla-ros-bridge为可控的主车（Ego Vehicle）创建了一系列标准化的ROS话题。理解这些话题是编写控制和感知节点的基础。

**Table 2: carla-ros-bridge关键ROS话题与消息类型**

| 功能分类 | 话题名称 (Topic Name) | 消息类型 (Message Type) | 数据流向 | 描述 |
| :---- | :---- | :---- | :---- | :---- |
| **车辆控制** | /carla/\<role\_name\>/vehicle\_control\_cmd | carla\_msgs/CarlaEgoVehicleControl | **发布**到此话题 | 发送油门、刹车、转向指令来控制车辆 14。 |
| **车辆状态** | /carla/\<role\_name\>/vehicle\_status | carla\_msgs/CarlaEgoVehicleStatus | **订阅**此话题 | 获取车辆的实时速度、加速度、控制状态等 14。 |
| **车辆信息** | /carla/\<role\_name\>/vehicle\_info | carla\_msgs/CarlaEgoVehicleInfo | **订阅**此话题 | 获取车辆的静态信息，如车轮、最大转向角等 14。 |
| **RGB相机** | /carla/\<role\_name\>/camera/rgb/front/image\_color | sensor\_msgs/Image | **订阅**此话题 | 获取前置RGB相机的原始图像数据 14。 |
| **激光雷达** | /carla/\<role\_name\>/lidar/front/point\_cloud | sensor\_msgs/PointCloud2 | **订阅**此话题 | 获取前置激光雷达的点云数据。 |
| **GNSS/GPS** | /carla/\<role\_name\>/gnss/front/fix | sensor\_msgs/NavSatFix | **订阅**此话题 | 获取车辆的地理定位信息 15。 |
| **里程计** | /carla/\<role\_name\>/odometry | nav\_msgs/Odometry | **订阅**此话题 | 获取车辆的位置、姿态和速度的估计值 15。 |
| **仿真时间** | /clock | rosgraph\_msgs/Clock | **订阅**此话题 | 获取由CARLA同步的仿真时间 12。 |

*注：\<role\_name\>是生成主车时指定的角色名，默认为ego\_vehicle。*

这个表格为开发者提供了一份清晰的API地图，将抽象的“控制”和“感知”概念与具体的ROS接口联系起来。

### **2.3 通过ROS控制Ego Vehicle**

许多CARLA原生教程使用其Python API（例如vehicle.apply\_control()）来直接控制车辆 11。然而，当使用ROS集成时，工作流程发生了变化。carla-ros-bridge将这种直接的API调用抽象为了一个ROS话题。因此，**在ROS环境中，正确的控制方式是向指定的ROS话题发布控制消息**，而不是直接调用CARLA的Python API。这种方式实现了控制逻辑与仿真核心的解耦，是ROS架构的核心思想。

以下是一个简单的Python脚本，使用rospy库来创建一个ROS节点，该节点会持续发布控制指令，使车辆直线前进。

Python

\#\!/usr/bin/env python

import rospy  
from carla\_msgs.msg import CarlaEgoVehicleControl

def vehicle\_controller():  
    \# 1\. 初始化ROS节点  
    rospy.init\_node('vehicle\_controller\_node', anonymous=True)  
      
    \# 2\. 创建一个发布者，发布到/carla/ego\_vehicle/vehicle\_control\_cmd话题  
    \#    消息类型为CarlaEgoVehicleControl  
    \#    \<role\_name\> 默认为 ego\_vehicle  
    control\_publisher \= rospy.Publisher('/carla/ego\_vehicle/vehicle\_control\_cmd', CarlaEgoVehicleControl, queue\_size=10)  
      
    \# 3\. 设置发布频率为10Hz  
    rate \= rospy.Rate(10)  
      
    while not rospy.is\_shutdown():  
        \# 4\. 创建一个CarlaEgoVehicleControl消息实例  
        control\_cmd \= CarlaEgoVehicleControl()  
          
        \# 5\. 填充消息字段  
        control\_cmd.throttle \= 0.5  \# 50%的油门  
        control\_cmd.steer \= 0.0     \# 转向角为0，即直行  
        control\_cmd.brake \= 0.0     \# 不刹车  
        control\_cmd.hand\_brake \= False  
        control\_cmd.reverse \= False  
          
        \# 6\. 发布消息  
        rospy.loginfo("Publishing vehicle control command")  
        control\_publisher.publish(control\_cmd)  
          
        \# 7\. 按照设定的频率休眠  
        rate.sleep()

if \_\_name\_\_ \== '\_\_main\_\_':  
    try:  
        vehicle\_controller()  
    except rospy.ROSInterruptException:  
        pass

要运行此脚本：

1. 将代码保存为simple\_controller.py。  
2. 赋予执行权限：chmod \+x simple\_controller.py。  
3. 确保CARLA和ros-bridge（带有ego\_vehicle）正在运行。  
4. 在新的终端中运行脚本：./simple\_controller.py。  
   您将看到仿真世界中的主车开始向前行驶。

### **2.4 在ROS中理解传感器数据**

自动驾驶算法的输入来自于各种传感器。在CARLA中，传感器被定义为一种特殊的“Actor”，可以通过蓝图（Blueprint）进行配置（如分辨率、视场角等），然后附加到车辆上 17。当使用ros-bridge时，这个过程被简化了。桥接器会自动检测附加在主车上的传感器，并为它们创建相应的ROS话题来发布数据 12。

以下Python脚本演示了如何创建一个ROS节点来订阅前置摄像头的图像话题，并使用cv\_bridge库将ROS图像消息转换为OpenCV图像格式，最后通过cv2库显示出来。

Python

\#\!/usr/bin/env python

import rospy  
from sensor\_msgs.msg import Image  
from cv\_bridge import CvBridge  
import cv2

class ImageSubscriber:  
    def \_\_init\_\_(self):  
        \# 1\. 初始化ROS节点  
        rospy.init\_node('image\_subscriber\_node', anonymous=True)  
          
        \# 2\. 创建一个CvBridge实例，用于ROS Image和OpenCV图像之间的转换  
        self.bridge \= CvBridge()  
          
        \# 3\. 创建一个订阅者，订阅/carla/ego\_vehicle/camera/rgb/front/image\_color话题  
        \#    当接收到消息时，调用callback函数  
        self.image\_sub \= rospy.Subscriber('/carla/ego\_vehicle/camera/rgb/front/image\_color', Image, self.callback)  
          
        rospy.loginfo("Image subscriber node started, waiting for images...")

    def callback(self, data):  
        try:  
            \# 4\. 使用cv\_bridge将ROS Image消息转换为OpenCV图像 (BGR8格式)  
            cv\_image \= self.bridge.imgmsg\_to\_cv2(data, "bgr8")  
        except Exception as e:  
            rospy.logerr(e)  
            return  
              
        \# 5\. 使用OpenCV显示图像  
        cv2.imshow("CARLA Camera View", cv\_image)  
        cv2.waitKey(1) \# 必须有waitKey来刷新窗口

    def run(self):  
        \# 保持节点运行，直到被关闭  
        rospy.spin()

if \_\_name\_\_ \== '\_\_main\_\_':  
    try:  
        image\_viewer \= ImageSubscriber()  
        image\_viewer.run()  
    except rospy.ROSInterruptException:  
        cv2.destroyAllWindows()

要运行此脚本：

1. 确保您的系统安装了cv\_bridge和OpenCV的Python绑定 (pip install opencv-python)。  
2. 将代码保存为image\_viewer.py并赋予执行权限。  
3. 确保CARLA和ros-bridge正在运行，并且主车上附加了一个角色名为rgb\_front的RGB摄像头（使用carla\_ros\_bridge\_with\_example\_ego\_vehicle.launch会自动配置）。  
4. 运行脚本：./image\_viewer.py。  
   一个名为"CARLA Camera View"的窗口将会弹出，实时显示车辆前方的视角。

至此，用户已经掌握了自动驾驶算法开发的基本闭环：通过ROS发送控制指令（输出），并通过ROS接收传感器数据（输入）。这是构建更复杂算法，如路径规划、目标检测和决策制定的基础。

## **Part 3: 为新手甄选的仿真项目仓库**

掌握了基础操作后，通过学习和实践现有的开源项目是提升技能的最佳途径。然而，网络上大量的CARLA项目对于初学者来说可能过于复杂或版本陈旧。本部分旨在为用户筛选出真正适合入门的、结构清晰的项目，并指导用户如何在此基础上构建自己的第一个简单算法。

### **3.1 甄选标准**

在浩如烟海的开源项目中，一个适合初学者的项目应具备以下特点：

* **目标单一明确:** 项目专注于解决一个基本问题，如停车、循迹等，而不是一个完整的自动驾驶堆栈。  
* **文档清晰:** 拥有一个详尽的README.md文件，清晰地说明了项目的目标、安装步骤和运行命令。  
* **依赖项少:** 除了CARLA和ROS本身，不引入复杂的机器学习框架（如TensorFlow/PyTorch）或其他大型库。  
* **代码简洁:** 代码结构简单，易于阅读和理解，侧重于展示核心的ROS通信和控制逻辑。

一个普遍的现象是，许多在awesome-CARLA等资源列表中列出的项目，要么是为非常旧的CARLA版本（如0.8.x）编写的，要么是复杂的端到端深度学习模型，这对于只想学习CARLA-ROS基础集成的初学者来说，门槛过高 9。因此，本指南将聚焦于质量而非数量，通过深度解析一两个“黄金范例”，教会用户识别和应用这些项目中的核心模式。

### **3.2 项目深度解析1：基于规则的carla-parking泊车项目**

vignif/carla-parking ([https://github.com/vignif/carla-parking](https://github.com/vignif/carla-parking)) 是一个近乎完美的入门项目，它完全符合上述所有标准 18。

1. 项目目标:  
   该项目的目标非常纯粹：实现一个基于预定几何路径的“开环”自动泊车功能。所谓“开环”，意味着车辆控制完全依赖于一个预先计算好的指令序列，而不依赖于任何实时传感器反馈。这使得项目能够完全隔离“控制”这一核心环节，让初学者可以专注于理解如何通过发布ROS消息来精确控制车辆的运动轨迹 18。  
2. 环境设置与运行:  
   其README文件提供了清晰的指令，用户只需按部就班即可运行。  
   * **安装:**  
     Bash  
     \# 进入你的catkin工作空间src目录  
     cd \~/\<catkin\_ws\>/src  
     \# 克隆仓库  
     git clone https://github.com/vignif/carla\_parking.git  
     \# 返回工作空间根目录并编译  
     cd..  
     catkin\_make

   * **运行环境:**  
     Bash  
     \# 终端1: 启动CARLA服务器

   ./CarlaUE4.sh\# 终端2: 启动ROS Bridgeroslaunch carla\_ros\_bridge carla\_ros\_bridge.launch\`\`\`

   * **运行项目:**  
     Bash  
     \# 终端3: source工作空间并运行脚本  
     source devel/setup.bash  
     rosrun carla\_park park.py

18执行后，仿真世界中将生成一辆主车和两辆障碍车，然后主车会自动执行一套精准的倒车入库动作。

3. 代码分析:  
   项目的核心在于park.py脚本。通过阅读代码可以发现，其逻辑本质是一个状态机，它按照顺序执行一系列动作（前进、左转、后退、右转等）。每个动作都通过向/carla/ego\_vehicle/vehicle\_control\_cmd话题发布一个CarlaEgoVehicleControl消息来实现，并通过rospy.sleep()来控制每个动作的持续时间。  
   学习价值:  
   这个项目向初学者展示了：  
   * 如何在一个ROS节点中构建一个简单的行为序列。  
   * 如何通过精确控制油门、刹车和转向角度，实现复杂的车辆机动。  
   * 一个完整的、自包含的CARLA-ROS项目的基本结构。

### **3.3 构建你自己的简单项目：PID车道保持**

在分析完一个现有项目后，最好的学习方式就是动手创建一个属于自己的项目。本节将指导用户构建一个比“直线行驶”更复杂，但比完整自动驾驶系统简单得多的功能：**基于PID控制器的车道保持**。这个项目将整合第二部分学到的感知和控制知识，并引入一个经典的控制算法。一些在线教程也探讨了类似的概念，但这里将专注于为ROS1初学者提供一个简化的实现思路 19。

1. 项目概念:  
   目标是让车辆利用前置摄像头的数据，始终行驶在车道中央。我们将通过以下步骤实现：  
   * **感知:** 从摄像头图像中识别车道线，并计算出车辆中心线与车道中心线的横向偏移误差（centerline\_error）。  
   * **控制:** 将这个误差输入到一个PID控制器中。PID控制器会根据当前误差（P）、累计误差（I）和误差变化率（D）计算出一个合适的转向指令，以减小这个误差。  
   * **执行:** 将PID控制器输出的转向值发布到车辆控制话题，实现闭环控制。  
2. 节点设计:  
   我们可以将这个系统分解为两个ROS节点，这符合ROS的模块化设计思想。  
   * **perception\_node.py (感知节点):**  
     * **订阅:** 订阅/carla/ego\_vehicle/camera/rgb/front/image\_color话题。  
     * **处理:** 在回调函数中，对接收到的图像进行简单的计算机视觉处理。  
       * **简化处理:** 为了避免复杂的深度学习，可以采用传统的图像处理技术，如：  
         1. 颜色阈值分割：提取图像中的黄色或白色车道线。  
         2. 边缘检测：找到车道线的轮廓。  
         3. 霍夫变换或轮廓拟合：识别出直线或曲线的车道线。  
         4. 计算中心线：找到左右车道线的中心位置。  
       * **计算误差:** 将图像中心的x坐标与计算出的车道中心线的x坐标进行比较，得到横向偏移误差（以像素为单位）。  
     * **发布:** 创建一个新的话题，例如/lane\_follower/centerline\_error，将计算出的误差发布出去。  
   * **control\_node.py (控制节点):**  
     * **订阅:** 订阅/lane\_follower/centerline\_error话题。  
     * **PID实现:** 在节点中实现一个简单的PID控制器。  
       Python  
       \# 伪代码  
       class PIDController:  
           def \_\_init\_\_(self, Kp, Ki, Kd):  
               self.Kp, self.Ki, self.Kd \= Kp, Ki, Kd  
               self.integral \= 0  
               self.last\_error \= 0

           def compute(self, error, dt):  
               self.integral \+= error \* dt  
               derivative \= (error \- self.last\_error) / dt  
               self.last\_error \= error  
               return self.Kp \* error \+ self.Ki \* self.integral \+ self.Kd \* derivative

     * **处理:** 在回调函数中，将接收到的误差输入PID控制器，计算出转向值steer\_value。  
     * **发布:** 创建一个CarlaEgoVehicleControl消息，将steer\_value赋给steer字段，并设置一个恒定的油门值，然后发布到/carla/ego\_vehicle/vehicle\_control\_cmd。  
3. 价值与拓展:  
   完成这个项目将使用户从一个被动的代码“运行者”转变为一个主动的算法“开发者”。它不仅巩固了ROS的通信机制，还引入了控制理论中的一个基石算法。在此基础上，用户可以进一步拓展，例如：  
   * 将PID控制器用于纵向速度控制。  
   * 使用更高级的图像处理或深度学习方法来提高车道线检测的鲁棒性。  
   * 结合路径规划，让车辆不仅能保持车道，还能跟随预定轨迹。

### **3.4 拓展探索：awesome-CARLA资源中心**

当用户对基础知识和简单项目感到游刃有余时，就应该将目光投向更广阔的社区资源。awesome-CARLA ([https://github.com/Amin-Tgz/awesome-CARLA](https://github.com/Amin-Tgz/awesome-CARLA)) 是一个由社区维护的、全面的CARLA相关资源列表，它收录了大量的开源项目、教程、论文和数据集 9。

这个仓库是用户进阶学习的“下一步”。在这里可以找到：

* **不同算法的实现:** 包括强化学习、模仿学习、多智能体协同等 9。  
* **更复杂的传感器应用:** 如激光雷达SLAM、3D目标检测等。  
* **完整的自动驾驶方案:** 例如将CARLA与Autoware或Apollo等工业级软件栈集成的项目。

使用建议:  
在探索awesome-CARLA时，务必牢记第一部分学到的教训：时刻关注项目的版本兼容性。列表中的许多项目都明确标注了其兼容的CARLA版本。在尝试任何新项目之前，请仔细检查其要求，以避免不必要的环境配置麻烦。将这个仓库视为一个灵感来源和技术参考，而不是一个即插即用的项目集合。

## **结论**

本指南为初学者提供了一条从环境搭建到自主开发算法的完整学习路径。核心要点可以总结如下：

1. **环境是成功的关键:** 自动驾驶仿真环境的复杂性要求用户必须严格遵循版本兼容性原则。选择一个经过验证的“黄金组合”（如Ubuntu 20.04 \+ ROS Noetic \+ 匹配的CARLA与ROS Bridge版本）是避免挫败感、顺利起步的先决条件。  
2. **理解通信架构:** 掌握CARLA作为服务器、ROS Bridge作为客户端的架构，以及两者之间通过Python API和ROS话题进行通信的机制，是解决配置和运行时问题的基础。特别是对PYTHONPATH和ROS工作空间source的理解，能帮助用户从“复制粘贴命令”提升到“理解并调试系统”。  
3. **掌握核心ROS接口:** 仿真中的感知和控制最终都归结为对特定ROS话题的订阅和发布。熟练使用Table 2中列出的核心话题，并掌握rospy库来编写节点，是进行任何算法开发的基本功。  
4. **从简单项目开始，逐步构建:** 与其一开始就挑战复杂的端到端模型，不如从carla-parking这样目标单一、代码简洁的项目入手，深入理解其控制逻辑。然后，通过亲手构建一个如PID车道保持这样的项目，将感知和控制结合起来，完成从学习者到开发者的转变。

CARLA与ROS的结合为自动驾驶研究和开发提供了一个近乎无限的虚拟试验场。虽然初期的学习曲线可能有些陡峭，但通过系统性的学习、严谨的环境配置和循序渐进的实践，任何开发者都可以掌握这个强大的工具链，并在虚拟世界中安全、高效地测试和验证自己的算法，最终为现实世界的自动驾驶技术贡献力量。

#### **Works cited**

1. Install ROS Bridge for ROS 1 \- CARLA Simulator, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros\_installation\_ros1/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/)  
2. Installation and use \- Behavior Metrics \- JdeRobot, accessed October 24, 2025, [https://jderobot.github.io/BehaviorMetrics/install/](https://jderobot.github.io/BehaviorMetrics/install/)  
3. ROS bridge installation \- CARLA Simulator \- Read the Docs, accessed October 24, 2025, [https://carla.readthedocs.io/en/0.9.10/ros\_installation/](https://carla.readthedocs.io/en/0.9.10/ros_installation/)  
4. carla-simulator/ros-bridge \- GitHub, accessed October 24, 2025, [https://github.com/carla-simulator/ros-bridge](https://github.com/carla-simulator/ros-bridge)  
5. ROS bridge installation \- CARLA Simulator \- Read the Docs, accessed October 24, 2025, [https://carla.readthedocs.io/en/0.9.8/ros\_installation/](https://carla.readthedocs.io/en/0.9.8/ros_installation/)  
6. Install ROS Bridge for ROS 2 \- CARLA Simulator \- CARLA documentation, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros\_installation\_ros2/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/)  
7. AUTONOMOUS VEHICLE SIMULATION USING OPEN SOURCE SOFTWARE CARLA \- Faculty Web Pages \- Kennesaw State University, accessed October 24, 2025, [https://facultyweb.kennesaw.edu/kmcfall/2019ECTC01.pdf](https://facultyweb.kennesaw.edu/kmcfall/2019ECTC01.pdf)  
8. UrbanPistek/carla\_ros\_simulation: A simulation stack using Docker to run CARLA with ROS nodes and additional tooling such as Foxglove. \- GitHub, accessed October 24, 2025, [https://github.com/UrbanPistek/carla\_ros\_simulation](https://github.com/UrbanPistek/carla_ros_simulation)  
9. Amin-Tgz/awesome-CARLA: CARLA resources such as ... \- GitHub, accessed October 24, 2025, [https://github.com/Amin-Tgz/awesome-CARLA](https://github.com/Amin-Tgz/awesome-CARLA)  
10. Docker, CARLA, ROS, Foxglove Workshop \- ME599 UWAFT \- YouTube, accessed October 24, 2025, [https://www.youtube.com/watch?v=a7EO-nI4rdI](https://www.youtube.com/watch?v=a7EO-nI4rdI)  
11. First steps \- CARLA Simulator, accessed October 24, 2025, [https://carla.readthedocs.io/en/latest/tuto\_first\_steps/](https://carla.readthedocs.io/en/latest/tuto_first_steps/)  
12. The ROS bridge package \- CARLA Simulator, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/run\_ros/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/run_ros/)  
13. Dataspeed Quick Guide To Running Carla Simulator | PDF \- Scribd, accessed October 24, 2025, [https://www.scribd.com/document/513477360/Dataspeed-Quick-Guide-to-Running-Carla-Simulator](https://www.scribd.com/document/513477360/Dataspeed-Quick-Guide-to-Running-Carla-Simulator)  
14. ROS-CARLA Integration \> How to use ROS Bridge package, accessed October 24, 2025, [https://silvamfpedro.github.io/thesis-blog/manual.html](https://silvamfpedro.github.io/thesis-blog/manual.html)  
15. CARLA Manual Control \- CARLA Simulator \- Read the Docs, accessed October 24, 2025, [https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla\_manual\_control/](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_manual_control/)  
16. Controlling the Car and getting Camera Sensor Data \- Self-driving cars with Carla and Python p.2 \- YouTube, accessed October 24, 2025, [https://www.youtube.com/watch?v=2hM44nr7Wms](https://www.youtube.com/watch?v=2hM44nr7Wms)  
17. Sensors and data \- CARLA Simulator \- CARLA documentation, accessed October 24, 2025, [https://carla.readthedocs.io/en/latest/core\_sensors/](https://carla.readthedocs.io/en/latest/core_sensors/)  
18. vignif/carla-parking: carla simulator for autonomous driving ... \- GitHub, accessed October 24, 2025, [https://github.com/vignif/carla-parking](https://github.com/vignif/carla-parking)  
19. Building Autonomous Vehicle in Carla- PID Controller & ROS 2 \- LearnOpenCV, accessed October 24, 2025, [https://learnopencv.com/pid-controller-ros-2-carla/](https://learnopencv.com/pid-controller-ros-2-carla/)  
20. carla · GitHub Topics, accessed October 24, 2025, [https://github.com/topics/carla](https://github.com/topics/carla)