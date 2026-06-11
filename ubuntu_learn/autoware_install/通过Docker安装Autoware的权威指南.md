

# **通过Docker安装Autoware的权威指南**

## **引言**

在自动驾驶领域，Autoware已成为一个领先的开源软件栈。然而，由于其复杂的依赖关系，在本地环境中进行配置可能是一项艰巨的任务。Docker通过容器化技术解决了这一挑战，它将应用程序及其所有依赖项打包到一个隔离的环境中，确保了在任何系统上的一致性和可复现性 1。本指南将提供一个详尽、分步的流程，指导您如何利用官方推荐的Docker工作流来安装和设置Autoware。

---

## **第1节：系统必备条件**

在开始之前，必须确保您的主机系统满足以下要求。忽略这些步骤是导致安装失败的最常见原因。

* **操作系统**：强烈推荐使用Ubuntu 22.04 LTS。  
* **Git**：用于克隆Autoware的源代码仓库 2。  
* **NVIDIA GPU与驱动程序**：虽然可以在没有NVIDIA GPU的情况下运行Autoware的部分功能，但对于完整的仿真和感知模块，强烈建议配备NVIDIA显卡并安装最新的专有驱动程序 2。  
  * 您可以通过系统的“软件和更新”工具安装驱动，或通过命令行安装。  
  * 安装后，运行nvidia-smi命令来验证驱动程序是否已正确加载。  
* **Docker引擎**：  
  * 遵循([https://docs.docker.com/engine/install/)为您的Ubuntu版本安装Docker](https://docs.docker.com/engine/install/)为您的Ubuntu版本安装Docker)。  
  * 执行至关重要的[安装后步骤](https://docs.docker.com/engine/install/linux-postinstall/)，将您的用户添加到docker组，以避免在每个docker命令前都使用sudo 3。  
    Bash  
    sudo usermod \-aG docker $USER

  * 注销并重新登录以使更改生效。  
* **NVIDIA Container Toolkit**：此工具包允许Docker容器利用主机的NVIDIA GPU，这对于运行Autoware的计算密集型任务至关重要 2。  
  * 遵循([https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)进行安装](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)进行安装)。  
  * 安装后，重启Docker服务：  
    Bash  
    sudo systemctl restart docker

* **Rocker**：这是一个便捷的工具，用于简化启动具有复杂权限（如图形界面和设备访问）的Docker容器的命令 2。  
  * 按照[官方说明](https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rocker#manual-installation)进行安装。

---

## **第2节：选择适合您需求的Docker镜像**

Autoware提供了两种主要的Docker镜像类型，理解它们的区别对于选择正确的路径至关重要 4。

| 镜像类型 | 描述 | 包含内容 | 推荐用途 |
| :---- | :---- | :---- | :---- |
| **Runtime (运行时)** | 一个轻量级的预构建镜像，仅包含运行Autoware所需的可执行文件。 | 运行时库、可执行文件。 | **初学者、功能演示、快速入门**。不适用于代码修改或调试。 |
| **Devel (开发)** | 一个功能完备的开发环境，包含了从源码构建和修改Autoware所需的一切。 | 编译器、头文件、构建工具、源代码挂载。 | **开发者、研究人员、任何需要修改、编译或调试代码的用户**。 |

**对于任何严肃的开发或调试工作，您必须选择Devel镜像。** 本指南将主要关注Devel镜像的安装流程。

---

## **第3节：安装Autoware开发环境（推荐）**

此流程将引导您建立一个功能齐全的开发环境，您可以在其中修改、编译和调试Autoware。

### **步骤 3.1: 克隆Autoware仓库**

首先，在您的主目录或首选工作区中克隆官方的Autoware仓库 2。

Bash

git clone https://github.com/autowarefoundation/autoware.git  
cd autoware

### **步骤 3.2: 拉取开发镜像**

接下来，从GitHub容器镜像仓库中拉取最新的autoware-universe镜像。此镜像包含了CUDA支持，适用于带NVIDIA GPU的系统 2。

Bash

docker pull ghcr.io/autowarefoundation/autoware-universe:latest-cuda

### **步骤 3.3: 启动开发容器**

使用rocker工具启动容器。此命令会将您主机上的autoware源代码目录挂载到容器内部，并配置GPU和图形界面访问 2。

Bash

rocker \--nvidia \--x11 \--user \--volume $HOME/autoware \-- ghcr.io/autowarefoundation/autoware-universe:latest-cuda

* \--nvidia: 启用NVIDIA GPU支持。  
* \--x11: 允许容器内的图形应用（如RViz2）显示在您的主机桌面上。  
* \--user: 确保容器内的用户权限与您的主机用户匹配。  
* \--volume $HOME/autoware: 这是最关键的部分。它将您主机上的autoware目录链接到容器内的/home/autoware/autoware目录。这意味着您可以在主机上用您喜欢的IDE编辑代码，然后在容器内编译和运行。

执行此命令后，您的终端提示符将变为容器内的shell。

### **步骤 3.4: 在容器内设置和构建工作空间**

现在您已在容器内部，需要完成工作空间的设置和首次构建。

1. **进入工作区目录**  
   Bash  
   cd autoware

2. 创建src目录并导入仓库  
   使用vcs工具克隆Autoware的所有子仓库到src目录中 2。  
   Bash  
   mkdir src  
   vcs import src \< autoware.repos

3. 安装ROS 2依赖项  
   使用rosdep安装所有必要的依赖包 2。  
   Bash  
   rosdep update  
   rosdep install \-y \--from-paths src \--ignore-src \--rosdistro $ROS\_DISTRO

4. 构建工作空间  
   使用colcon编译整个工作空间。--symlink-install是一个推荐选项，它通过创建符号链接而不是复制文件来加快后续的构建速度 2。  
   Bash  
   colcon build \--symlink-install \--cmake-args \-DCMAKE\_BUILD\_TYPE=Release

   构建过程需要大量内存和时间，请耐心等待。

构建成功后，您的Autoware开发环境便已准备就绪。您可以source工作空间 (source install/setup.bash) 并开始运行Autoware节点。

---

## **第4节：快速入门（仅运行时）**

如果您只想快速运行Autoware的预打包演示，而不需要进行开发，可以使用runtime镜像。

1. **克隆Autoware仓库**  
   Bash  
   git clone https://github.com/autowarefoundation/autoware.git  
   cd autoware

2. 下载演示数据  
   官方提供了一个脚本来下载所需的地图和rosbag数据 5。

./setup-dev-env.sh \-y download\_artifacts  
\`\`\`

3. 运行Runtime容器  
   使用run.sh脚本启动容器，并指定地图和数据路径 5。

./docker/run.sh \--map-path path\_to\_map \--data-path path\_to\_data  
\`\`\`  
进入容器后，您可以按照官方教程运行仿真。

---

## **第5节：故障排除**

* **Docker权限错误**: 如果您在运行docker命令时遇到“permission denied”错误，这通常意味着您的用户不在docker组中。请执行sudo usermod \-aG docker $USER，然后完全注销并重新登录 3。  
* **构建失败（内存不足）**: Autoware的构建过程需要大量RAM。如果您的系统内存不足，可能会导致构建失败或系统冻结。建议配置16-32GB的交换空间（swap） 6。  
* **构建失败（其他原因）**: 如果构建因其他原因失败，首先尝试清理旧的构建产物并重新构建 6。  
  Bash  
  rm \-rf build/ install/ log/  
  colcon build \--symlink-install \--cmake-args \-DCMAKE\_BUILD\_TYPE=Release

* **Docker/Rocker问题**: 如果容器无法启动，请首先确认Docker本身是否正常工作 6。  
  Bash  
  docker run \--rm \-it hello-world

---

## **结论**

通过遵循本指南，您应该已经成功地使用Docker搭建了一个稳定且功能强大的Autoware环境。Docker不仅简化了初始安装，还为开发、调试和协作提供了一个一致的平台。无论您是选择用于快速演示的runtime镜像，还是用于深度开发的devel镜像，容器化都是体验和贡献Autoware生态系统的推荐方式。

#### **Works cited**

1. Docker: Accelerated Container Application Development, accessed October 16, 2025, [https://www.docker.com/](https://www.docker.com/)  
2. Docker installation for development \- Autoware Documentation, accessed October 16, 2025, [https://autowarefoundation.github.io/autoware-documentation/pr-279/installation/autoware/docker-installation-devel/](https://autowarefoundation.github.io/autoware-documentation/pr-279/installation/autoware/docker-installation-devel/)  
3. Step-by-step Autoware.Auto installation guide \- Robotics. And other stuff too., accessed October 16, 2025, [https://idorobotics.com/2021/08/31/step-by-step-autoware-auto-installation-guide/](https://idorobotics.com/2021/08/31/step-by-step-autoware-auto-installation-guide/)  
4. Docker installation \- Autoware Documentation, accessed October 16, 2025, [https://autowarefoundation.github.io/autoware-documentation/pr-473/installation/autoware/docker-installation/](https://autowarefoundation.github.io/autoware-documentation/pr-473/installation/autoware/docker-installation/)  
5. Open AD Kit: containerized workloads for Autoware \- Autoware ..., accessed October 16, 2025, [https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/docker-installation/)  
6. Troubleshooting \- Autoware Documentation, accessed October 16, 2025, [https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/](https://autowarefoundation.github.io/autoware-documentation/main/support/troubleshooting/)  
7. Troubleshooting \- Autoware Documentation, accessed October 16, 2025, [https://autowarefoundation.github.io/autoware-documentation/pr-366/support/troubleshooting/](https://autowarefoundation.github.io/autoware-documentation/pr-366/support/troubleshooting/)