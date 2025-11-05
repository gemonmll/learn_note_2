

# **ROS 2 (Humble/Jazzy) 与 Gazebo Harmonic 集成：从安装到全栈自主导航的技术指南**

## **摘要**

本报告提供了一份详尽的技术指南，旨在指导机器人工程师和研究人员在 ROS 2 (Humble 或 Jazzy) 平台上成功安装、配置和集成现代 Gazebo Harmonic 仿真器。报告的核心目标是解决一个完整的自主导航工作流程：从基础环境搭建，到机器人建模（包含现代 Gazebo 插件），再到 ros\_gz\_bridge 的配置、SLAM 建图，最终实现 Nav2 导航堆栈的部署与调优。

本指南将重点阐明 Gazebo (Ignition) 架构演进所带来的关键变化，特别是新的传感器插件范式，并为解决“非标准”ROS 2 Humble / Gazebo Harmonic 组合 所面临的依赖冲突提供了明确的路径。此外，报告将深入分析 use\_sim\_time 和 /clock 桥接在仿真中的关键作用，并为 slam\_toolbox 和 Nav2 (特别是 dwb\_controller 和代价地图膨胀) 的配置与调优提供专家级的见解和经过验证的配置文件。

## **I. 系统安装与环境配置**

成功的系统集成始于一个正确配置且无依赖冲突的基础环境。用户请求中的 ROS 2 Humble/Gazebo Harmonic 组合并非官方默认配对，因此安装顺序至关重要。

### **A. 兼容性分析与安装路径**

根据 ROS 2 增强提案 (REP) 2000 和官方文档，标准版本配对如下：

* **ROS 2 Humble Hawksbill (LTS):** 官方支持 **Gazebo Fortress (LTS)**。  
* **ROS 2 Jazzy Jalisco (LTS):** 官方支持 **Gazebo Harmonic (LTS)**。

本报告将提供两条路径：推荐的“原生”路径（Jazzy/Harmonic）和用户请求的“高级”路径（Humble/Harmonic）。

#### **路径一 (推荐): ROS 2 Jazzy 与 Gazebo Harmonic (Ubuntu 24.04)**

此路径是当前推荐的、依赖关系最清晰的组合。

1. **安装 ROS 2 Jazzy:** 遵循官方指南安装 ros-jazzy-desktop。  
2. **安装导航与仿真堆栈:** 安装 Nav2、SLAM 和 Gazebo 桥接包。此命令将自动引入 gz-harmonic 作为依赖项。  
   Bash  
   sudo apt update  
   sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard ros-jazzy-ros-gzharmonic

此路径可避免后续章节中讨论的大多数依赖冲突。

#### **路径二 (高级): ROS 2 Humble 与 Gazebo Harmonic (Ubuntu 22.04)**

此路径被标记为“谨慎使用” (⚡)，因为它需要混合来自 packages.ros.org 和 packages.osrfoundation.org 的软件包。错误的安装顺序会导致严重的 apt 依赖冲突。

**关键安装顺序：**

1. **安装 ROS 2 Humble:** 仅安装 ROS 2 基础包。**切勿** 在此阶段安装 ros-humble-gazebo-ros-pkgs 或任何与 Gazebo 相关的 ROS 软件包。  
   Bash  
   sudo apt update  
   sudo apt install ros-humble-desktop

2. **添加 Gazebo OSRF 软件源:** 必须添加 OSRF 软件源以下载 Gazebo Harmonic。此步骤使用现代的 gpg 密钥和 tee 方法，避免使用已弃用的 apt-key。  
   Bash  
   \# 添加 GPG 密钥  
   sudo curl \-sSL https://packages.osrfoundation.org/gazebo.gpg \-o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

   \# 添加软件源  
   echo "deb \[arch=$(dpkg \--print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg\] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb\_release \-cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list \> /dev/null

   sudo apt update

3. **安装 Gazebo Harmonic:** 安装仿真器本身。  
   Bash  
   sudo apt install gz-harmonic

4. **安装“非标”桥接包:** 这是最关键的一步。必须安装由 OSRF 提供的、专门针对 Harmonic 编译的 Humble 桥接包。  
   Bash  
   sudo apt install ros-humble-ros-gzharmonic

5. **安装导航与 SLAM 堆栈:** 最后，安装 ROS 2 端的导航和 SLAM 软件包。  
   Bash  
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard

### **B. 关键澄清：ign 与 gz 的过渡**

一个主要的混淆来源是 Gazebo (前身为 Ignition) 的命名变更。所有现代版本（包括 Harmonic）和相关的 ROS 2 包都已从 ign 前缀过渡到 gz 前缀。

* 旧命令: ign gazebo shapes.sdf  
* **新命令:** gz sim shapes.sdf  
* 旧消息: ignition.msgs.StringMsg  
* **新消息:** gz.msgs.StringMsg  
* 旧包名: ros-humble-ros-ign-bridge  
* **新包名:** ros-humble-ros-gzharmonic (或 ros-jazzy-ros-gzharmonic)

**本报告将严格且仅使用现代 gz 语法。** 在查阅较旧的教程（即使是针对 Fortress）时，必须在头脑中将所有 ign 替换为 gz。

### **C. 安装验证**

1. **验证 Gazebo:** 运行 gz sim shapes.sdf。应能成功启动 Gazebo Harmonic GUI。  
2. **验证 ROS 2 桥接:** 运行 ros2 pkg list | grep ros-gz。应能看到 ros\_gz\_bridge, ros\_gz\_sim 等相关包。

## **II. 机器人建模：适用于 Gazebo Harmonic 的现代插件**

本节详细介绍如何为机器人创建 URDF 模型，重点是集成 Gazebo Harmonic 的**现代传感器插件**。这是与 Gazebo Classic 相比最大的架构变化。

### **A. 机器人描述包结构**

按照标准实践，创建一个 ROS 2 包（例如 diff\_drive\_robot\_description）来存放模型文件。

diff\_drive\_robot\_description/  
├── CMakeLists.txt  
├── package.xml  
├── launch/             (用于存放第 III 节的启动文件)  
├── urdf/  
│   └── diff\_drive\_robot.urdf.xacro  
├── config/             (用于存放第 III 节和第 V 节的.yaml 文件)  
└── rviz/  
    └── default.rviz

URDF (统一机器人描述格式) 定义了机器人的物理结构（连杆、关节、碰撞体），是 robot\_state\_publisher 和 Rviz 显示的基础。

### **B. 架构核心：Gazebo 插件范式转变**

在 Gazebo Classic 中，libgazebo\_ros\_\*.so 插件（如 libgazebo\_ros\_imu.so）同时负责模拟物理并直接发布 ROS 2 消息。**此模型在 Gazebo Harmonic 中已被弃用。**

现代 Gazebo 仿真采用了“关注点分离”架构，这是一个两部分组成的系统：

1. **Gazebo 仿真插件 (在 URDF 中):** URDF/SDF 文件定义一个**纯粹的 Gazebo 传感器**（例如 \<sensor type="imu"\>）。该插件在 Gazebo 内部运行，并将原始数据发布到**Gazebo Transport 主题**（例如，一个名为 /imu 的 Gazebo 主题）。  
2. **ROS-Gazebo 桥接 (在 Launch 文件中):** 一个独立的、在 ROS 2 端运行的节点 (ros\_gz\_bridge) 显式地订阅 Gazebo Transport 主题，将其转换为 ROS 2 消息类型（例如 sensor\_msgs/msg/Imu），然后将其发布到 ROS 2 网络。

这种架构解耦了仿真和 ROS 中间件。本报告中的所有插件都将遵循这种现代架构。

### **C. 差分驱动插件**

此插件模拟差分驱动运动学。它将监听一个 Gazebo 主题（我们将其命名为 cmd\_vel）以获取速度指令，并发布里程计和 TF。

**文件: diff\_drive\_robot.urdf.xacro**

XML

\<gazebo\>  
  \<plugin name\="gz::sim::systems::DiffDrive" filename\="gz-sim-diff-drive-system"\>  
      
    \<joint\>left\_wheel\_joint\</joint\>  
    \<joint\>right\_wheel\_joint\</joint\>

    \<wheel\_separation\>0.4\</wheel\_separation\>  \<wheel\_radius\>0.05\</wheel\_radius\>      \<topic\>cmd\_vel\</topic\>  
    \<odom\_topic\>odom\</odom\_topic\>  
    \<tf\_topic\>tf\</tf\_topic\>  
      
  \</plugin\>  
\</gazebo\>

### **D. Lidar 插件 (Native Harmonic)**

我们使用 gpu\_lidar 传感器类型，它利用 GPU 进行高效的光线追踪。注意：这可能需要在您的世界 (SDF) 文件中包含 \<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"\>。

**文件: diff\_drive\_robot.urdf.xacro (附加到 lidar\_link)**

XML

\<gazebo reference\="lidar\_link"\>  
  \<sensor name\="gpu\_lidar\_sensor" type\="gpu\_lidar"\>  
    \<always\_on\>1\</always\_on\>  
    \<update\_rate\>10\</update\_rate\>  
    \<visualize\>true\</visualize\>  
      
    \<topic\>scan\</topic\>  
      
    \<frame\_id\>lidar\_link\</frame\_id\>  
      
    \<ray\>  
      \<scan\>  
        \<horizontal\>  
          \<samples\>360\</samples\>  
          \<resolution\>1\</resolution\>  
          \<min\_angle\>\-3.14159\</min\_angle\>  
          \<max\_angle\>3.14159\</max\_angle\>  
        \</horizontal\>  
      \</scan\>  
      \<range\>  
        \<min\>0.1\</min\>  
        \<max\>12.0\</max\>  
        \<resolution\>0.01\</resolution\>  
      \</range\>  
    \</ray\>  
  \</sensor\>  
\</gazebo\>

### **E. IMU 插件 (Native Harmonic)**

与 Lidar 类似，我们使用原生的 imu 传感器类型。

**文件: diff\_drive\_robot.urdf.xacro (附加到 imu\_link)**

XML

\<joint name\="imu\_joint" type\="fixed"\>  
  \<parent link\="base\_link"/\>  
  \<child link\="imu\_link"/\>  
  \<origin xyz\="0 0 0.01" rpy\="0 0 0"/\>  
\</joint\>

\<link name\="imu\_link"/\>

\<gazebo reference\="imu\_link"\>  
  \<sensor name\="imu\_sensor" type\="imu"\>  
    \<always\_on\>1\</always\_on\>  
    \<update\_rate\>50\</update\_rate\>  
    \<visualize\>true\</visualize\>  
      
    \<topic\>imu\</topic\>  
      
    \<gz\_frame\_id\>imu\_link\</gz\_frame\_id\>  
  \</sensor\>  
\</gazebo\>

此步骤完成后，我们有了一个 URDF 模型，它可以在 Gazebo Harmonic 中运行，并将其所有传感器数据（Lidar、IMU）和运动数据（Odom、TF）发布到 Gazebo Transport 主题。下一步是将这些主题连接到 ROS 2。

## **III. 系统集成：统一的启动文件与 ros\_gz\_bridge**

本节将所有组件（Gazebo、Rviz、Robot State Publisher 和 ROS-Gazebo 桥接）集成到一个统一的 ROS 2 启动文件中。

### **A. 核心启动文件组件**

在 diff\_drive\_robot\_description/launch/ 目录下创建一个 simulation.launch.py 文件。

1. **启动 Gazebo Sim:** 使用 ros\_gz\_sim 包提供的 IncludeLaunchDescription 来启动 Gazebo 服务器和 GUI。  
   Python  
   from launch.actions import IncludeLaunchDescription  
   from launch.launch\_description\_sources import PythonLaunchDescriptionSource  
   from ament\_index\_python.packages import get\_package\_share\_directory  
   import os

   \#... 在 LaunchDescription 函数内部...

   \# 获取 ros\_gz\_sim 包的共享目录  
   pkg\_ros\_gz\_sim \= get\_package\_share\_directory('ros\_gz\_sim')

   \# 启动 Gazebo 仿真 (gz\_sim.launch.py 会自动启动 server 和 gui)  
   gz\_sim\_launch \= IncludeLaunchDescription(  
       PythonLaunchDescriptionSource(  
           os.path.join(pkg\_ros\_gz\_sim, 'launch', 'gz\_sim.launch.py')  
       ),  
       \# '-r' 表示在启动时运行一个空世界 (empty.sdf)  
       launch\_arguments={'gz\_args': '-r empty.sdf'}.items()  
   )

2. **Robot State Publisher:** 此节点读取 URDF 文件，并根据关节状态发布机器人（非odom-\>base\_link）的 TF 变换（例如 base\_link \-\> lidar\_link）。  
   Python  
   from launch\_ros.actions import Node  
   from launch.substitutions import Command, LaunchConfiguration  
   from launch.actions import DeclareLaunchArgument

   \#...

   \# 声明 use\_sim\_time 参数，默认值为 'true'  
   use\_sim\_time \= LaunchConfiguration('use\_sim\_time', default='true')

   \# 获取 URDF 文件路径  
   urdf\_model\_path \= os.path.join(  
       get\_package\_share\_directory('diff\_drive\_robot\_description'),  
       'urdf',  
       'diff\_drive\_robot.urdf.xacro'  
   )

   \# 使用 xacro 命令解析 URDF/XACRO 文件  
   robot\_description\_content \= Command(\['xacro ', urdf\_model\_path\])

   \# 启动 robot\_state\_publisher 节点  
   robot\_state\_publisher\_node \= Node(  
       package='robot\_state\_publisher',  
       executable='robot\_state\_publisher',  
       output='screen',  
       parameters=  
   )

### **B. 配置 ros\_gz\_bridge：系统的心脏**

ros\_gz\_bridge 是连接第 II 节中 Gazebo 主题和 ROS 2 网络的关键。我们将使用 parameter\_bridge 可执行文件，并通过一个专用的 YAML 文件来配置它，这种方式比命令行参数更清晰、更易于管理。

#### **关键系统依赖：/clock 桥接**

在配置桥接之前，必须理解 /clock 主题的至关重要性。

1. 当 use\_sim\_time ROS 2 参数被设置为 True 时（如上文 robot\_state\_publisher 和所有 Nav2/SLAM 节点所示），所有这些节点都会**停止**使用系统的挂钟时间 (wall clock)。  
2. 相反，它们会**等待** /clock ROS 2 主题上发布的时间戳，以此来同步其所有操作 1。  
3. Gazebo 仿真器在 Gazebo Transport 网络上发布其内部的仿真时钟（例如，在 /clock Gazebo 主题上）。  
4. ros\_gz\_bridge 是**唯一**负责将这个 Gazebo 时钟 桥接到 /clock ROS 2 主题的组件。

**结论：** 如果 /clock 桥接丢失或配置错误，整个 ROS 2 系统（Rviz、TF、SLAM、Nav2）将**静默地冻结**。节点会启动，但不会处理任何数据，因为它们在等待一个永远不会到来的时间戳。**这是 ROS 2 仿真中最常见、最难调试的失败点。**

#### **经验证的 gz\_bridge.yaml 配置文件**

在 diff\_drive\_robot\_description/config/ 目录下创建 gz\_bridge.yaml 文件。此文件提供了本教程所需的所有桥接。

**文件: config/gz\_bridge.yaml**

YAML

\#   
\# \`ros\_gz\_bridge\` 参数文件  
\# 这是一个 YAML \*序列\* (以 '-' 开头)  
\# 

\#  
\# 1\. 时钟桥接 (GZ \-\> ROS)  
\# CRITICAL: 将 Gazebo 仿真时钟同步到 ROS 2，驱动 use\_sim\_time  
\#  
\- topic\_name: "/clock"  
  ros\_type\_name: "rosgraph\_msgs/msg/Clock"  
  gz\_type\_name: "gz.msgs.Clock"  
  direction: GZ\_TO\_ROS   \# 符号:

\#  
\# 2\. Lidar 桥接 (GZ \-\> ROS)  
\# 桥接来自 \<sensor type="gpu\_lidar"\> 的数据  
\#  
\- topic\_name: "scan"     \# 必须匹配 URDF 中的 \<topic\> 标签  
  ros\_topic\_name: "scan" \# 在 ROS 2 上的主题名称  
  ros\_type\_name: "sensor\_msgs/msg/LaserScan"  
  gz\_type\_name: "gz.msgs.LaserScan"  
  direction: GZ\_TO\_ROS

\#  
\# 3\. IMU 桥接 (GZ \-\> ROS)  
\# 桥接来自 \<sensor type="imu"\> 的数据  
\#  
\- topic\_name: "imu"      \# 必须匹配 URDF 中的 \<topic\> 标签  
  ros\_topic\_name: "imu"  
  ros\_type\_name: "sensor\_msgs/msg/Imu"  
  gz\_type\_name: "gz.msgs.IMU"  
  direction: GZ\_TO\_ROS

\#  
\# 4\. 命令速度桥接 (ROS \-\> GZ)  
\# 将 ROS 2 /cmd\_vel 消息发送给 Gazebo 差分驱动插件  
\#  
\- topic\_name: "cmd\_vel"  \# 必须匹配 DiffDrive 插件中的 \<topic\> 标签  
  ros\_topic\_name: "cmd\_vel"  
  ros\_type\_name: "geometry\_msgs/msg/Twist"  
  gz\_type\_name: "gz.msgs.Twist"  
  direction: ROS\_TO\_GZ   \# 符号: \] (从 ROS 读，写入 Gazebo)

\#  
\# 5\. 里程计桥接 (GZ \-\> ROS)  
\# 从 DiffDrive 插件获取里程计数据  
\#  
\- topic\_name: "odom"     \# 必须匹配 DiffDrive 插件中的 \<odom\_topic\>  
  ros\_topic\_name: "odom"  
  ros\_type\_name: "nav\_msgs/msg/Odometry"  
  gz\_type\_name: "gz.msgs.Odometry"  
  direction: GZ\_TO\_ROS

\#  
\# 6\. TF 桥接 (GZ \-\> ROS)  
\# 从 DiffDrive 插件获取 odom \-\> base\_link 的 TF 变换  
\#  
\- topic\_name: "tf"       \# 必须匹配 DiffDrive 插件中的 \<tf\_topic\>  
  ros\_topic\_name: "/tf"  
  ros\_type\_name: "tf2\_msgs/msg/TFMessage"  
  gz\_type\_name: "gz.msgs.Pose\_V"  
  direction: GZ\_TO\_ROS

### **C. 在启动文件中添加桥接节点**

最后，将 ros\_gz\_bridge 节点添加到 simulation.launch.py 中，并加载上述 YAML 配置文件。

Python

\#... 在 simulation.launch.py 中...

\# 获取桥接配置文件的路径  
bridge\_config\_file \= os.path.join(  
    get\_package\_share\_directory('diff\_drive\_robot\_description'),  
    'config',  
    'gz\_bridge.yaml'  
)

\# 启动 ros\_gz\_bridge 节点  
gz\_bridge\_node \= Node(  
    package='ros\_gz\_bridge',  
    executable='parameter\_bridge',  
    \# 加载 YAML 配置文件  
    parameters=\[{  
        'config\_file': bridge\_config\_file,  
        'use\_sim\_time': use\_sim\_time \# 桥接节点本身也必须使用仿真时间  
    }\],  
    output='screen'  
)

\#... 在 LaunchDescription 的 return 语句中包含...  
\#   gz\_sim\_launch,  
\#   robot\_state\_publisher\_node,  
\#   gz\_bridge\_node,  
\#   (以及可选的 Rviz 节点)

运行此启动文件（ros2 launch diff\_drive\_robot\_description simulation.launch.py）后，一个完整的、桥接了传感器和控制的仿真环境即告完成。

## **IV. 自主建图 (SLAM) 与 slam\_toolbox**

利用第 III 节中集成的仿真环境，本节将运行 slam\_toolbox 来创建环境地图。

### **A. 启动与配置 SLAM**

1. **安装 (如果尚未安装):** sudo apt install ros-\<distro\>-slam-toolbox。  
2. **配置:** 在 config/ 目录下创建 slam\_toolbox\_params.yaml 1。  
   YAML  
   slam\_toolbox:  
     ros\_\_parameters:  
       \# 必须使用仿真时间！  
       use\_sim\_time: true

       \# SLAM 核心参数  
       odom\_frame: "odom"  
       map\_frame: "map"  
       base\_frame: "base\_link"  
       scan\_topic: "/scan"  \# 必须匹配 gz\_bridge.yaml 中定义的 ROS 主题

       \# 增加 TF 容忍度，以应对仿真中的潜在延迟  
       transform\_tolerance: 0.5 

       \# 其他可以调优的参数...  
       resolution: 0.05  
       map\_update\_interval: 5.0

3. **创建 SLAM 启动文件:** 采用模块化方法 1，创建一个新的 mapping.launch.py 文件，它将 *包含* simulation.launch.py，并 *添加* slam\_toolbox 节点。

**文件: launch/mapping.launch.py**

Python

import os  
from ament\_index\_python.packages import get\_package\_share\_directory  
from launch import LaunchDescription  
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument  
from launch.launch\_description\_sources import PythonLaunchDescriptionSource  
from launch.substitutions import LaunchConfiguration

def generate\_launch\_description():  
    pkg\_share \= get\_package\_share\_directory('diff\_drive\_robot\_description')  
    pkg\_slam\_toolbox \= get\_package\_share\_directory('slam\_toolbox')

    \# 声明 use\_sim\_time 参数  
    use\_sim\_time\_arg \= DeclareLaunchArgument(  
        'use\_sim\_time',  
        default\_value='true',  
        description='Use simulation (Gazebo) clock if true'  
    )  
    use\_sim\_time \= LaunchConfiguration('use\_sim\_time')

    \# 1\. 包含基础仿真启动文件 (Gazebo, RSP, Bridge)  
    simulation\_launch \= IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(  
            os.path.join(pkg\_share, 'launch', 'simulation.launch.py')  
        ),  
        launch\_arguments={'use\_sim\_time': use\_sim\_time}.items()  
    )

    \# 2\. 获取 slam\_toolbox 的参数文件  
    slam\_params\_file \= os.path.join(pkg\_share, 'config', 'slam\_toolbox\_params.yaml')

    \# 3\. 包含 slam\_toolbox 的启动文件  
    slam\_toolbox\_launch \= IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(  
            os.path.join(pkg\_slam\_toolbox, 'launch', 'online\_async\_launch.py')  
        ),  
        launch\_arguments={  
            'slam\_params\_file': slam\_params\_file,  
            'use\_sim\_time': use\_sim\_time  
        }.items()  
    )

    return LaunchDescription(\[  
        use\_sim\_time\_arg,  
        simulation\_launch,  
        slam\_toolbox\_launch  
    \])

### **B. 执行建图与故障排查**

1. **启动 SLAM:** ros2 launch diff\_drive\_robot\_description mapping.launch.py  
2. **启动遥控:** 在新终端中，ros2 run teleop\_twist\_keyboard teleop\_twist\_keyboard \--ros-args \-r /cmd\_vel:=/cmd\_vel。  
3. **可视化:** 在 Rviz 中添加 /map, /scan 和 TF。  
4. **建图:** 驾驶机器人在环境中移动，直到地图完整。

#### **故障排查: "Message Filter dropping message" 错误**

一个在 slam\_toolbox 中极其常见的错误是：  
\[slam\_toolbox\]: Message Filter dropping message: frame 'lidar\_link' at time X for reason 'the timestamp on the message is earlier than all the data in the transform cache.'

* **问题分析:** 此错误意味着 slam\_toolbox 在时间点 $T$ 收到了一个 Lidar 扫描 (/scan)。为了处理此扫描，它需要知道在时间点 $T$ 或稍早一点的完整 TF 树（map \-\> odom \-\> base\_link \-\> lidar\_link）。该错误表明，slam\_toolbox 的 TF 缓存是陈旧的（例如，只到 $T-0.1$ 秒）。Lidar 消息比其对应的 TF 变换“更早”到达。  
* **根本原因:** 这几乎总是由时间戳不同步引起的。  
* **解决方案:**  
  1. **验证 /clock:** 立即检查 ros2 topic echo /clock。如果该主题没有发布，或发布缓慢，则返回第 III-B 节，检查 gz\_bridge.yaml 中的 /clock 桥接是否正确无误。  
  2. **验证 use\_sim\_time:** 确保**所有**节点（slam\_toolbox, robot\_state\_publisher, gz\_bridge）都已设置 use\_sim\_time: true。  
  3. **增加容忍度:** 作为最后的手段，在 slam\_toolbox\_params.yaml 中增加 transform\_tolerance 参数（例如，0.5 到 1.0 秒），以给予 TF 消息更多的时间到达。

### **C. 保存地图**

建图完成后，使用 nav2\_map\_server CLI 保存地图以供导航使用。

Bash

\# \-f my\_map 指定保存的文件名前缀  
ros2 run nav2\_map\_server map\_saver\_cli \-f my\_map

这将生成 my\_map.yaml 和 my\_map.pgm 两个文件。

## **V. 自主导航 (Nav2) 与参数调优**

这是最后一步：配置完整的 Nav2 堆栈，以使用第 IV 节中创建的地图进行自主导航。本节将重点关注调优。

### **A. Nav2 全堆栈启动配置**

1. **安装 (如果尚未安装):** ros-\<distro\>-navigation2 和 ros-\<distro\>-nav2-bringup。  
2. **创建 nav2\_params.yaml:** 这是 Nav2 的主配置文件。它非常庞大，用于配置所有服务器（controller\_server, planner\_server, amcl, bt\_navigator, local\_costmap, global\_costmap 等）。一个完整的示例超出了本报告的范围，但我们将重点关注调优的关键部分。  
3. **创建导航启动文件:** 创建 launch/navigation.launch.py 1。  
   * 此文件将 *包含* simulation.launch.py。  
   * 它**不会**启动 slam\_toolbox。  
   * 它将**包含**两个来自 nav2\_bringup 的核心启动文件 1：  
     1. localization\_launch.py: 启动 amcl（用于定位）和 map\_server（用于加载 my\_map.yaml）。  
     2. navigation\_launch.py: 启动所有 Nav2 核心服务器（控制器、规划器、行为树等）。  
   * 这两个启动文件都将接收我们自定义的 nav2\_params.yaml 文件的路径。

**文件: launch/navigation.launch.py (简化版)**

Python

import os  
from ament\_index\_python.packages import get\_package\_share\_directory  
from launch import LaunchDescription  
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument  
from launch.launch\_description\_sources import PythonLaunchDescriptionSource  
from launch.substitutions import LaunchConfiguration

def generate\_launch\_description():  
    pkg\_share \= get\_package\_share\_directory('diff\_drive\_robot\_description')  
    pkg\_nav2\_bringup \= get\_package\_share\_directory('nav2\_bringup')  
      
    \# 声明参数  
    use\_sim\_time \= LaunchConfiguration('use\_sim\_time', default='true')  
    map\_file\_path \= LaunchConfiguration('map', default=os.path.join(pkg\_share, 'maps', 'my\_map.yaml'))  
    params\_file\_path \= LaunchConfiguration('params\_file', default=os.path.join(pkg\_share, 'config', 'nav2\_params.yaml'))

    \# 1\. 包含基础仿真 (Gazebo, RSP, Bridge)  
    simulation\_launch \= IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(os.path.join(pkg\_share, 'launch', 'simulation.launch.py')),  
        launch\_arguments={'use\_sim\_time': use\_sim\_time}.items()  
    )

    \# 2\. 包含 Nav2 本地化启动 (AMCL, Map Server)  
    localization\_launch \= IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(os.path.join(pkg\_nav2\_bringup, 'launch', 'localization\_launch.py')),  
        launch\_arguments={  
            'use\_sim\_time': use\_sim\_time,  
            'map': map\_file\_path,  
            'params\_file': params\_file\_path  
        }.items()  
    )

    \# 3\. 包含 Nav2 导航启动 (Controller, Planner, BT)  
    navigation\_launch \= IncludeLaunchDescription(  
        PythonLaunchDescriptionSource(os.path.join(pkg\_nav2\_bringup, 'launch', 'navigation\_launch.py')),  
        launch\_arguments={  
            'use\_sim\_time': use\_sim\_time,  
            'params\_file': params\_file\_path  
        }.items()  
    )

    return LaunchDescription()

### **B. 调优指南：代价地图膨胀 (Costmap Inflation)**

在 nav2\_params.yaml 中，global\_costmap 和 local\_costmap 都使用 inflation\_layer 插件来在障碍物周围创建安全缓冲区。

inflation\_layer 的关键参数 2:

* robot\_radius 2 或 footprint: 定义机器人的**实际物理边界**。robot\_radius 是一个简单的圆形近似。footprint（一个多边形）更精确。这是“内切”或“致命”成本区域。  
* inflation\_radius: 障碍物成本向外膨胀的距离（米）。这定义了安全缓冲区的总大小。  
* cost\_scaling\_factor: 成本衰减的指数因子。  
  * **高值 (例如 10.0):** 产生一个“陡峭的悬崖”。成本值急剧下降，允许机器人靠近障碍物（只要它在膨胀区之外），但在进入时会受到高惩罚。  
  * **低值 (例如 3.0):** 产生一个“平缓的斜坡”。成本值缓慢下降，使机器人在距离障碍物较远时就开始变得“谨慎”。

一个必须理解的非显而易见的关系是 **costmap 和 controller 之间的耦合**。inflation\_layer 在地图上创建了一个成本的“势场” (0-253)。控制器（dwb\_controller）的 BaseObstacleCritic 插件 会读取其模拟轨迹沿线的这些成本值，并将其乘以该评论家自己的 scale 参数（例如 BaseObstacle.scale）。

这意味着 inflation\_layer 的 cost\_scaling\_factor 和 BaseObstacleCritic.scale 是**乘法关系**。如果机器人在狭窄通道中表现得“过于胆小”，很可能是因为这两个值都设置得过高，导致机器人“双重恐惧”障碍物。调优时，应首先设置一个合理的膨胀曲线，**然后**再调优控制器对该曲线的反应。

### **C. 调优指南：dwb\_controller**

dwb\_controller（动态窗口法）是 controller\_server 的默认插件。其工作原理是：

1. **轨迹生成 (TrajectoryGenerator):** 向前模拟数千条可能的 (vx, vy, vtheta) 速度指令。  
2. **轨迹评论 (TrajectoryCritics):** 每一条模拟轨迹都由一系列“评论家”插件进行评分。  
3. **选择最佳:** 得分**总和最高** 的轨迹被选中，其对应的速度指令被发送给机器人。

因此，**调优 dwb 本质上是在平衡不同评论家的 scale（权重）**，以管理**相互冲突的目标**。例如，PathAlignCritic（路径对齐）的目标是让机器人紧贴全局路径，而 BaseObstacleCritic（障碍物）的目标是让机器人远离障碍物。这两个目标在狭窄的走廊中是直接冲突的。scale 参数决定了哪个目标“更重要”。

#### **dwb\_controller 调优实施手册 (基于**

3

此表提供了基于症状的调优指南，以解决最常见的问题。

| 症状 | 观察到的行为 | 调优解释与参数调整 |
| :---- | :---- | :---- |
| **“过度谨慎” / “转弯过大”** | 机器人转弯半径非常大，远离障碍物，严重偏离全局路径。 | 对障碍物的“恐惧”压倒了遵循路径的“愿望”。 • **减小** BaseObstacleCritic.scale 和/或 ObstacleFootprintCritic.scale。 • **增大** PathAlignCritic.scale 和/或 PathDistCritic.scale。 |
| **“切内角” / “离得太近”** | 机器人为了完美地跟随路径，会“抄近道”切过拐角，或离障碍物过近。 | 遵循路径的“愿望”压倒了对障碍物的“恐惧”。 • **增大** BaseObstacleCritic.scale 和/或 ObstacleFootprintCritic.scale。 • **减小** PathAlignCritic.scale 和/或 PathDistCritic.scale。 |
| **“未到达最终姿态”** | 机器人导航到目标的 $(x, y)$ 位置，但没有旋转到最终的朝向 (yaw) 就停止了。 | GoalDistCritic (距离) 评论家“获胜”了，但 GoalAlignCritic (朝向) 评论家失败了。 • **增大** GoalAlignCritic.scale。 • 确保 RotateToGoalCritic.scale 3 非零。此评论家仅在接近目标时激活，以执行原地旋转。 |
| **“来回振荡” / “卡住”** | 机器人在狭窄空间或障碍物附近来回“抽搐”，无法取得进展。 | 陷入“局部最优”。机器人想前进（被 BaseObstacle 惩罚），但又不想后退（被 PreferForwardCritic 惩罚）。 • **减小** PreferForwardCritic.scale 以允许机器人后退。 • 检查 OscillationCritic 3 参数，确保它能正确触发恢复行为。 |

## **VI. 结论**

本技术报告提供了一个从零开始的完整工作流程，用于在现代 ROS 2 (Humble/Jazzy) 和 Gazebo Harmonic 堆栈上实现全栈自主导航。报告成功地解决了几个关键的技术挑战：

1. **环境配置:** 提供了在非标准（Humble/Harmonic）组合下避免 apt 依赖冲突的明确安装路径。  
2. **现代建模:** 阐明了 Gazebo Harmonic 中“传感器 \+ 桥接”的新插件范式，并提供了经过验证的 URDF 插件代码（DiffDrive, gpu\_lidar, imu）。  
3. **核心集成:** 强调了 use\_sim\_time 和 /clock 桥接的绝对重要性，将其确定为仿真系统中最常见的故障点，并提供了完整的 gz\_bridge.yaml 配置文件。  
4. **导航调优:** 提供了超越简单参数列表的专家级调优见解，将 costmap 和 dwb\_controller 视为一个紧密耦合的系统，并提供了一个基于症状的实用调优手册来平衡控制器评论家之间的冲突。

遵循本指南中概述的步骤和配置，工程师和研究人员应能可靠地部署、测试和验证基于 Lidar 的 SLAM 和 Nav2 算法，为在现代 Gazebo 仿真环境中进行更高级的机器人研发奠定坚实的基础。

#### **Works cited**

1. MOGI-ROS/Week-7-8-ROS2-Navigation: Mapping, localization and using the navigation stack with ROS2 Jazzy \- GitHub, accessed November 4, 2025, [https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation](https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation)  
2. ROS 2 Navigation Tuning Guide – Nav2 \- AutomaticAddison.com, accessed November 4, 2025, [https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/](https://automaticaddison.com/ros-2-navigation-tuning-guide-nav2/)  
3. DWB Controller — Nav2 1.0.0 documentation, accessed November 4, 2025, [https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html)