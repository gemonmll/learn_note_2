

# **构建先进开发环境：在 Ubuntu 22.04 上使用 Docker 对 Autoware Universe 进行源码级调试的综合指南**

## **引言**

### **目标声明**

本报告旨在提供一份详尽的、专家级的操作流程，用于为 Autoware Universe 项目建立一个容器化的开发环境。核心目标是实现从宿主机 Visual Studio Code (VS Code) 实例到 Docker 容器内运行的 C++ 节点的无缝源码级调试。

### **问题背景**

Autoware 这样的自动驾驶技术栈的复杂性，要求开发过程具备强大而灵活的调试能力。容器化技术虽然在环境一致性和依赖管理方面提供了无与伦比的优势，但其引入的抽象层也可能使传统的调试工作流变得复杂。本指南将直面并解决这一挑战，打通容器内外的调试链路。

### **技术栈验证**

本报告将首先验证并确认所选技术栈的可行性与兼容性：采用 Ubuntu 22.04 (Jammy Jellyfish) 作为宿主机操作系统，配置 NVIDIA 570 系列驱动程序，并结合 CUDA 12.3 工具包，以支持与 ROS 2 Humble/Galactic 兼容的最新 Autoware Universe 版本。

### **报告结构概述**

本文将遵循一个清晰的路线图，从宿主机系统的基础环境准备，到容器化环境的构建，直至最终的交互式调试实践演示，系统性地引导读者完成整个搭建过程。

## **第一节：宿主机系统架构与基础环境搭建**

本节旨在为我们的开发环境奠定坚实的基础。此阶段的任何配置失误都可能引发连锁反应，导致后续出现难以诊断的故障。因此，我们将采用严谨且可验证的步骤进行操作。

### **1.1. 前置条件验证与兼容性分析**

在开始任何安装之前，必须对系统环境进行严格的验证，以确保硬件和软件组件之间的兼容性。

* **硬件验证:** 确认系统中存在一块支持 CUDA 的 NVIDIA GPU 是所有后续工作的前提。可通过以下命令进行检查 1：  
  Bash  
  lspci | grep \-i nvidia

* **操作系统验证:** 确保宿主机运行的是 Ubuntu 22.04 LTS。该版本提供长期支持，保证了软件包的稳定性和可预测性。使用 hostnamectl 命令可确认当前系统版本 1。  
* **驱动与 CUDA 兼容性深度解析:** NVIDIA 驱动程序与 CUDA 工具包之间存在严格的版本依赖关系。根据官方文档，CUDA 12.3 要求 NVIDIA 驱动版本不低于 545.23.06 3。用户指定使用的 570 系列驱动程序 5 远高于此最低要求，这是一种确保系统稳定性的最佳实践。这种“向前兼容”（即较新版本的驱动支持较旧版本的 CUDA 工具包）是构建稳定GPU计算环境的核心原则。反之，较旧的驱动无法支持较新的 CUDA 工具包，这是初学者常犯的错误。

### **1.2. NVIDIA 570 驱动程序的安装与验证**

选择正确的驱动安装方法对于系统的长期稳定至关重要。

* **安装策略:** 推荐使用 Ubuntu 官方的 APT 仓库进行安装。这种方法能确保驱动与系统内核更新保持良好的集成，是稳定性的首选 8。应明确避免使用官方的 .run 文件安装程序 9，因为它会绕过系统的包管理器，可能导致未来系统升级时产生冲突，且难以维护和卸载。  
* **分步安装指南:**  
  1. 为确保一个纯净的安装环境，首先应彻底清除系统中任何残留的旧版 NVIDIA 驱动：  
     Bash  
     sudo apt \--purge remove '\*nvidia\*'  
     sudo apt autoremove

  2. 更新 APT 包列表并从 Ubuntu 的 restricted 官方仓库中安装目标驱动。对于开发工作站，nvidia-driver-570 是合适的元数据包 7。  
     Bash  
     sudo apt update  
     sudo apt install nvidia-driver-570

* **安装后验证:** 驱动安装完成后，必须重启系统以加载新的内核模块。重启后，执行 nvidia-smi 命令。如果成功输出显示驱动版本为 570.x 并列出检测到的 GPU 设备信息，则证明驱动已正确安装并成功加载 11。这是完成宿主机GPU环境配置的关键检查点。

### **1.3. Docker 引擎与 NVIDIA Container Toolkit 的集成**

标准的 Docker 环境无法直接利用宿主机的 GPU 资源。必须通过 NVIDIA Container Toolkit 这座“桥梁”来实现。

* **Docker 引擎安装:** 为确保使用最新稳定版本，应从 Docker 官方的 apt 仓库安装 Docker Engine。以下是完整的安装步骤，包括设置 GPG 密钥和添加软件源 13。  
  Bash  
  \# 添加 Docker 的官方 GPG 密钥  
  sudo apt-get update  
  sudo apt-get install ca-certificates curl  
  sudo install \-m 0755 \-d /etc/apt/keyrings  
  sudo curl \-fsSL https://download.docker.com/linux/ubuntu/gpg \-o /etc/apt/keyrings/docker.asc  
  sudo chmod a+r /etc/apt/keyrings/docker.asc

  \# 添加仓库到 Apt 源  
  echo \\  
    "deb \[arch=$(dpkg \--print-architecture) signed-by=/etc/apt/keyrings/docker.asc\] https://download.docker.com/linux/ubuntu \\  
    $(. /etc/os-release && echo "$VERSION\_CODENAME") stable" | \\  
    sudo tee /etc/apt/sources.list.d/docker.list \> /dev/null

  \# 安装 Docker 软件包  
  sudo apt-get update  
  sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

  安装完成后，建议将当前用户添加到 docker 用户组，以避免在执行 docker 命令时频繁使用 sudo 13。  
  Bash  
  sudo usermod \-aG docker $USER

  执行此命令后，需要注销并重新登录才能使组权限生效。  
* **关键的桥梁：NVIDIA Container Toolkit:** 该工具包是让 Docker 容器能够识别并使用宿主机 NVIDIA GPU 的核心中间件 11。  
* **工具包安装:** 使用官方提供的命令序列来添加 NVIDIA Container Toolkit 的仓库并安装相关软件包 16。  
  Bash  
  curl \-fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg \--dearmor \-o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \\  
  && curl \-s \-L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \\  
  sed 's\#deb https://\#deb \[signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg\] https://\#g' | \\  
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

  sudo apt-get update  
  sudo apt-get install \-y nvidia-container-toolkit

* **配置 Docker 守护进程:** 这是集成过程中的决定性一步。运行 nvidia-ctk 命令来配置 Docker，使其能够识别并使用 NVIDIA 运行时。该命令会自动修改 /etc/docker/daemon.json 文件 11。  
  Bash  
  sudo nvidia-ctk runtime configure \--runtime=docker  
  sudo systemctl restart docker

* **最终集成验证:** 为了确认从宿主机操作系统、NVIDIA 驱动、Docker 到 Container Toolkit 的整个技术栈都已正确配置，需要执行一个最终的验证测试。通过运行一个启用了 CUDA 的基础容器，并在容器内部执行 nvidia-smi 命令。如果该命令能够成功返回与宿主机一致的 GPU 信息，则证明整个GPU容器化环境已准备就绪 11。  
  Bash  
  sudo docker run \--rm \--gpus all nvidia/cuda:12.3.0-base-ubuntu22.04 nvidia-smi

整个宿主机环境的配置是一个环环相扣的依赖链。其中最常见的故障点源于内核版本、内核头文件与 NVIDIA 驱动模块之间的不匹配。NVIDIA 驱动包含一个内核模块组件，当系统通过 apt upgrade 更新 Linux 内核时，该模块必须针对新内核重新编译。这个过程由 DKMS (Dynamic Kernel Module Support) 自动处理 8。DKMS 的工作依赖于系统已安装了与当前运行内核版本（可通过 uname \-r 查看）完全匹配的 linux-headers 包。因此，一次看似无关的系统更新，如果未能正确安装对应的头文件或 DKMS 编译失败，就会导致 NVIDIA 驱动加载失败。这解释了为何 nvidia-smi 命令在系统更新后可能突然失效。本指南推荐的基于 APT 的驱动安装方式（如 nvidia-dkms-570 包 7）的优越性在于，它会自动将驱动注册到 DKMS 系统中，从而在内核更新时自动完成重新编译，极大地增强了系统的鲁棒性。

## **第二节：构建 Autoware 开发容器**

在验证了宿主机环境后，我们现在将注意力转向 Autoware 开发环境本身。我们将优先采用 Autoware 官方推荐的实践，以确保环境的可维护性和社区一致性。

### **2.1. 宿主机工作区与数据持久化策略**

为了实现代码和数据的持久化，我们需要在宿主机上建立一个合理的文件结构。

* **克隆 Autoware 源码:** 将 Autoware Foundation 官方的 autoware 仓库克隆到用户的主目录（例如 \~/autoware）。必须指定与目标 ROS 2 发行版匹配的分支，例如 humble 19。  
  Bash  
  git clone https://github.com/autowarefoundation/autoware.git \-b humble \~/autoware

* **创建持久化数据目录:** 在宿主机上创建专门的目录，用于存放那些生命周期应长于容器的数据，如地图、rosbag 记录等。这是 Docker 的核心最佳实践之一 19。  
  Bash  
  mkdir \~/autoware\_map  
  mkdir \~/autoware\_data

* **策略原理:** 将这些宿主机目录作为数据卷（volumes）挂载到容器内部，可以确保在容器停止、删除或重建后，所有重要的工作产出（如下载的地图、实验数据）都不会丢失，实现了开发环境与工作数据的分离。

### **2.2. 使用 rocker 精通容器编排**

rocker 是一个为机器人开发量身定制的工具，它极大地简化了 Docker 容器的启动和管理。

* **rocker 简介:** rocker 是 Autoware 基金会推荐的一个强大的 Docker 命令包装脚本 19。它能够将启动复杂机器人开发容器所需的冗长且易错的 docker run 参数（如 GUI 转发、GPU 集成等）自动化，并封装成简洁的命令 21。  
* **安装 rocker:** 推荐通过 ROS 的官方软件源来安装 rocker 21。  
  Bash  
  sudo apt-get install python3-rocker

* **rocker 命令详解:** 下面是用于启动 Autoware 容器的完整 rocker 命令，并对每个参数进行详细注解 19：  
  Bash  
  rocker \--nvidia \--x11 \--user \\  
         \--volume $HOME/autoware:/home/autoware/autoware \\  
         \--volume $HOME/autoware\_map:/home/autoware/autoware\_map \\  
         ghcr.io/autowarefoundation/autoware-universe:humble-latest-cuda

  * \--nvidia: rocker 的核心功能之一，自动检测宿主机 NVIDIA 驱动，并向容器注入匹配的驱动库和设置正确的运行时 21。  
  * \--x11: 启用 X11 GUI 转发，使得在容器内运行的图形化工具（如 Rviz2, rqt）能够显示在宿主机的桌面上。  
  * \--user: 确保在容器内创建的文件和目录的所有权与宿主机当前用户匹配，避免权限问题。  
  * \--volume...: 将宿主机目录挂载到容器内的指定路径，实现代码和数据的共享与持久化。  
  * ghcr.io/autowarefoundation/autoware-universe:humble-latest-cuda: 指定要使用的官方预构建、支持 CUDA 的 Autoware 开发镜像。

### **2.3. 容器初始化与依赖填充**

首次进入容器后，需要完成工作区的最后配置。

* **首次进入:** 通过 rocker 命令进入容器后，首先切换到工作区目录：  
  Bash  
  cd /home/autoware/autoware

* **导入模块源码:** 运行 vcs 工具，根据 autoware.repos 文件拉取构成 Autoware Universe 的所有上游源码仓库 19。  
  Bash  
  vcs import src \< autoware.repos

* **安装依赖:** 执行 rosdep 命令来安装源码所需的所有系统和 ROS 依赖。这一步至关重要，因为基础镜像创建后，main 分支的代码可能已经引入了新的依赖项 19。  
  Bash  
  sudo apt update  
  rosdep update  
  rosdep install \-y \--from-paths src \--ignore-src \--rosdistro $ROS\_DISTRO

采用预构建的开发镜像（ghcr.io/...）结合 vcs import 和 rosdep install 的策略，体现了一种在开发速度和灵活性之间取得精妙平衡的混合环境管理方法。从头开始在 Dockerfile 中编译整个 Autoware 依赖栈（包括 ROS, PCL 等）会极其耗时。Autoware 基金会提供的预构建基础镜像 19 已经包含了这些重量级依赖，为开发者节省了数小时的初始设置时间。然而，Autoware 的源代码每天都在快速演进，预构建的镜像不可能与每个软件包仓库的 main 分支完美同步。因此，标准工作流程是：从稳定的基础镜像开始，使用 vcs 拉取最新的源代码，然后利用 rosdep 来安装两者之间的“增量”——即自基础镜像创建以来新增的任何依赖。这种策略既利用了预构建镜像的速度优势，又保证了开发环境能跟上最新的代码变更，是 Autoware 项目专业开发实践的体现。

## **第三节：为深度调试编译 Autoware**

本节是整个指南的核心转折点。标准的 Release 构建模式会经过编译器的高度优化，导致有效的调试几乎无法进行。为了实现源码级调试，必须采用特定的编译配置。

### **3.1. CMake 构建类型的策略性选择**

CMake 提供了多种构建类型，每种都有其特定的用途。

* **Release:** 这是用于性能发布的默认选项。它启用高级别的编译器优化（例如 \-O3）并移除所有调试符号。这使得程序运行速度最快，但不适用于调试 19。  
* **Debug:** 此类型包含完整的调试符号（-g）并完全禁用编译器优化（-O0）。它最适合于调试，但生成的代码运行速度会显著变慢。在像 Autoware 这样的实时系统中，性能的巨大差异有时可能掩盖或引发与时序相关的错误 23。  
* **RelWithDebInfo (推荐):** 这是开发阶段的最佳选择。它在编译时启用高级别的优化（例如 \-O2），同时*保留*了必要的调试符号（-g）。这种模式提供了最佳的平衡：代码运行性能接近 Release 版本，同时允许调试器将执行指令精确地映射回源代码行 23。对于需要兼顾性能分析和问题排查的实时机器人应用，强烈推荐使用此模式。

### **3.2. 执行 colcon 调试构建**

基于上述策略，我们将使用 colcon 来执行带有特定 CMake 参数的构建。

* **构建命令:** 在容器的工作区根目录下，执行以下精确的 colcon build 命令：  
  Bash  
  colcon build \--symlink-install \--cmake-args \-DCMAKE\_BUILD\_TYPE=RelWithDebInfo \-DCMAKE\_EXPORT\_COMPILE\_COMMANDS=ON

* **参数详解:**  
  * \--symlink-install: 这是迭代开发中的一个关键标志。它通过创建符号链接而非复制文件的方式来安装软件包，这意味着对 Python 脚本或其他非编译型文件的修改可以立即生效，无需重新构建整个工作区 23。  
  * \--cmake-args: 该参数用于将后续的参数直接传递给底层的 CMake 构建系统 23。  
  * \-DCMAKE\_BUILD\_TYPE=RelWithDebInfo: 正如前文所述，此标志将构建类型设置为 RelWithDebInfo，以生成带调试信息的优化代码。  
  * \-DCMAKE\_EXPORT\_COMPILE\_COMMANDS=ON: 这是一个能极大提升 IDE 体验的关键附加参数。它指示 CMake 在构建目录中为每个包生成一个 compile\_commands.json 文件。该文件精确记录了编译每个源文件时所使用的编译器、标志和包含路径。VS Code 的 C/C++ 扩展可以利用这个文件，从而极大地提高 IntelliSense、代码补全和静态错误检查的准确性 23。

\--symlink-install 和 \-DCMAKE\_EXPORT\_COMPILE\_COMMANDS=ON 这两个参数的组合，其意义远不止于调试本身，它们共同构建了一个高效且“IDE友好”的开发闭环。机器人开发者的工作流通常涉及频繁的、小范围的代码修改和重新编译。--symlink-install 最大限度地减少了解释型文件的“重新构建”环节。而 \-DCMAKE\_EXPORT\_COMPILE\_COMMANDS=ON 则通过为 IDE 提供完美的上下文，增强了“编码”环节的体验，减少了开发者在代码导航和解决虚假静态分析错误上所花费的时间。当这些标志与用于调试的 RelWithDebInfo 构建相结合时，便形成了一个在速度、准确性和洞察力方面都得到优化的整体开发环境。

## **第四节：在 VS Code 中进行源码级调试的权威工作流**

本节将前面所有的准备工作整合为一个可重复的、实用的调试工作流程。

### **4.1. 配置 VS Code 集成开发环境**

首先，在宿主机的 VS Code 中安装必要的扩展，为远程开发和调试做好准备。

* **核心扩展:** 必须安装以下扩展：Dev Containers, Docker, ROS (由 Microsoft 发布) 和 C/C++ Extension Pack 27。  
  * Dev Containers: 允许 VS Code 连接到正在运行的容器内部，并将 IDE 的功能完整地延伸到容器环境中。  
  * Docker: 提供了管理 Docker 镜像、容器和仓库的便捷界面。  
  * ROS: 为 ROS 2 开发提供语法高亮、代码片段和构建任务等支持。  
  * C/C++ Extension Pack: 提供强大的 C++ IntelliSense、代码导航和调试功能。

### **4.2. 建立宿主机与容器的连接桥梁**

利用 Dev Containers 扩展，我们可以无缝地将 VS Code 的前端界面连接到后台运行的 Autoware 容器。

* **附加到容器:** 在 VS Code 中，按下 Ctrl+Shift+P 打开命令面板，然后输入并选择 Dev Containers: Attach to Running Container...。在弹出的列表中选择正在运行的 Autoware 容器。  
* **连接后体验:** 连接成功后，VS Code 将打开一个新窗口。在这个窗口中，文件浏览器、编辑器和集成终端都将直接在容器内部运行。这提供了一种无缝且功能强大的开发模式，开发者可以在熟悉的 IDE 环境中直接操作容器内的文件和进程。

### **4.3. 通过 pipeTransport 实现 GDB/gdbserver 调试会话**

我们将采用一种现代化的方法来连接调试器，这种方法比传统的端口映射更安全、更便捷。

* **核心技术:** 我们不通过网络端口暴露 gdbserver，而是利用 pipeTransport 机制，将 GDB 协议的通信数据流通过标准的 docker exec 通道进行传输 29。这种方法无需任何网络配置，且本质上更安全。  
* **创建 launch.json:** 在已附加到容器的 VS Code 窗口中，打开 Autoware 工作区（/home/autoware/autoware），并在该工作区根目录下创建 .vscode/launch.json 文件。  
* **配置详解:** 提供一份完整且带有详细注释的 launch.json 配置文件。这是本报告的技术核心，它演示了如何配置 VS Code 来启动一个通过管道连接到容器内 gdbserver 的调试会话。我们将采用直接通过 gdbserver 启动目标节点的方式，这比附加到已运行进程更为简洁和可靠，因为它避免了复杂的进程ID（PID）查找过程 29。

**表 1: 用于 Autoware 节点调试的 VS Code launch.json 配置**

| 键 (Key) | 值 (Value) | 注释 (Annotation) |
| :---- | :---- | :---- |
| name | "Debug Autoware Node (gdbserver)" | 为此调试配置指定一个描述性名称。 |
| type | "cppdbg" | 指定使用 C/C++ 调试器扩展。 |
| request | "launch" | 表明我们将启动一个新的进程进行调试。 |
| MIMode | "gdb" | 指定使用 GDB 作为调试器接口。 |
| cwd | "${workspaceFolder}" | 将当前工作目录设置为 Autoware 工作区的根目录。 |
| program | "${workspaceFolder}/install/planning\_validator/lib/planning\_validator/planning\_validator\_node" | **关键:** 这是宿主机 GDB 客户端需要加载符号表的可执行文件路径。请根据要调试的节点进行修改。 |
| pipeTransport | {...} | **远程调试设置的核心。** |
| pipeTransport.pipeCwd | "/usr/bin/" | 管道程序的工作目录。 |
| pipeTransport.pipeProgram | "/usr/bin/docker" | 用于建立管道连接的程序，即 Docker CLI。 |
| pipeTransport.pipeArgs | \`\` | 传递给 docker 的参数。exec \-i 在容器内执行命令；gdbserver \- 启动服务器监听标准输入/输出；最后是**容器内**的可执行文件路径。**请务必将 CONTAINER\_NAME\_OR\_ID 替换为实际的容器名称或ID。** |
| pipeTransport.debuggerPath | "/usr/bin/gdb" | 宿主机上 GDB 客户端的路径。 |
| sourceFileMap | {"/home/autoware/autoware": "${workspaceFolder}"} | 将容器内的源码路径映射到宿主机上的工作区路径。这对于 GDB 正确定位源文件至关重要。 |

### **4.4. 实用调试场景演练**

现在，我们将所有步骤付诸实践。

* **选择目标:** 选择一个核心的 Autoware C++ 节点作为演示对象，例如 planning\_validator\_node。  
* **设置断点:** 在 VS Code 中，打开相关的 C++ 源文件（例如 planning\_validator.cpp），在希望暂停执行的代码行号左侧单击以设置一个断点。  
* **启动 Autoware:** 在 VS Code 的一个集成终端中，启动 Autoware 的仿真环境或一个特定的启动文件。  
* **启动调试:** 切换到 VS Code 的“运行和调试”侧边栏，从下拉菜单中选择我们刚刚创建的 "Debug Autoware Node (gdbserver)" 配置，然后按 F5 键（或点击绿色的播放按钮）。  
* **调试结果:** VS Code 的界面将切换到调试模式。pipeTransport 配置会通过 docker exec 在容器内启动 gdbserver，gdbserver 接着启动目标节点。当程序的执行流程到达预设的断点时，执行将暂停。此时，开发者可以在 VS Code 的调试面板中检查变量值、查看调用堆栈、单步执行代码（步入、步过、步出），所有这些操作都在宿主机 IDE 中完成，而代码的实际执行则在容器内部。

## **第五节：操作最佳实践与故障排查**

### **5.1. 常见环境陷阱**

* **nvidia-smi 失败:** 通常是由于系统内核更新后，驱动与内核不匹配。解决方案是确保安装了正确的内核头文件，并使用 DKMS 方式重新安装驱动。  
* **Docker 权限被拒绝:** 当前用户不在 docker 用户组中。解决方案是执行 sudo usermod \-aG docker $USER，然后注销并重新登录 15。  
* **rocker GUI 失败:** 宿主机上的 DISPLAY 环境变量或 X11 权限配置不正确。  
* **编译失败:** 可能是依赖项过时，需要重新运行 rosdep install；也可能是旧的构建产物导致冲突，此时应运行 rm \-rf build install log 清理工作区。

### **5.2. 性能与构建优化**

* **集成 ccache:** 为了显著加快增量编译的速度，强烈建议在容器内安装和配置 ccache。ccache 能够缓存编译产物，当再次编译未修改的源文件时，可以直接从缓存中获取结果，从而大幅缩短编译时间 23。  
* **资源分配:** 如果使用 Docker Desktop，应适当调整其资源限制（CPU核心数、内存大小），确保为编译和运行 Autoware 分配了足够的系统资源。

## **结论**

### **成果总结**

本报告系统地阐述了从一个裸机 Ubuntu 22.04 系统出发，逐步构建一个功能完备、支持交互式调试的 Autoware Universe 开发环境的全过程。我们成功地整合了 NVIDIA GPU 驱动、CUDA 工具包、Docker 容器化技术以及 VS Code IDE，最终为这个复杂的、GPU加速的机器人应用程序搭建了一套专业的调试工作流。

### **价值主张**

通过遵循本指南建立的开发环境，代表了机器人软件开发的专业标准。它通过提供快速迭代、深度系统分析和直观的源码级调试能力，能够显著提升开发效率和代码质量。RelWithDebInfo 构建、compile\_commands.json 的生成以及通过 pipeTransport 实现的无缝调试，共同构成了一个旨在最大化开发者生产力的解决方案。

### **未来方向**

本文所构建的基础环境具有良好的可扩展性。它可以进一步被整合到持续集成（CI）流水线中，用于自动化测试和构建。此外，该环境也可以作为基础，扩展用于更复杂的多机器人仿真场景，展示了其作为专业机器人开发平台的强大潜力。

#### **Works cited**

1. CUDA Installation Guide for Linux \- NVIDIA Docs Hub, accessed October 16, 2025, [https://docs.nvidia.com/cuda/cuda-installation-guide-linux/](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)  
2. 1\. Introduction — NVIDIA Driver Installation Guide r580 documentation, accessed October 16, 2025, [https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/index.html](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/index.html)  
3. Support Matrix — NVIDIA cuDNN Backend, accessed October 16, 2025, [https://docs.nvidia.com/deeplearning/cudnn/backend/latest/reference/support-matrix.html](https://docs.nvidia.com/deeplearning/cudnn/backend/latest/reference/support-matrix.html)  
4. CUDA Toolkit 12.3 Downloads | NVIDIA Developer, accessed October 16, 2025, [https://developer.nvidia.com/cuda-12-3-0-download-archive?target\_os=Linux\&target\_arch=x86\_64\&Distribution=Ubuntu\&target\_version=22.04\&target\_type=deb\_network](https://developer.nvidia.com/cuda-12-3-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network)  
5. Data Center Driver for Ubuntu 22.04 570.86.15 | Linux 64-bit Ubuntu ..., accessed October 16, 2025, [https://www.nvidia.com/en-us/drivers/details/239769/](https://www.nvidia.com/en-us/drivers/details/239769/)  
6. Frameworks Support Matrix \- NVIDIA Docs, accessed October 16, 2025, [https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html](https://docs.nvidia.com/deeplearning/frameworks/support-matrix/index.html)  
7. nvidia-graphics-drivers-570 package : Ubuntu \- Launchpad, accessed October 16, 2025, [https://launchpad.net/ubuntu/+source/nvidia-graphics-drivers-570](https://launchpad.net/ubuntu/+source/nvidia-graphics-drivers-570)  
8. NVIDIA drivers installation \- Ubuntu Server documentation, accessed October 16, 2025, [https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/](https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/)  
9. CUDA Toolkit 12.3 Downloads | NVIDIA Developer, accessed October 16, 2025, [https://developer.nvidia.com/cuda-12-3-0-download-archive?target\_os=Linux\&target\_arch=x86\_64\&Distribution=Ubuntu\&target\_version=22.04\&target\_type=runfile\_local](https://developer.nvidia.com/cuda-12-3-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=runfile_local)  
10. The Graphics Drivers PPA updated with NVIDIA 570 for Ubuntu / Linux Mint, accessed October 16, 2025, [https://ubuntuhandbook.org/index.php/2025/03/graphics-driver-ppa-nvidia-570/](https://ubuntuhandbook.org/index.php/2025/03/graphics-driver-ppa-nvidia-570/)  
11. How to Install NVIDIA Container Toolkit and Use GPUs with Docker Containers \- GPU Mart, accessed October 16, 2025, [https://www.gpu-mart.com/blog/install-nvidia-container-toolkit](https://www.gpu-mart.com/blog/install-nvidia-container-toolkit)  
12. Can't install nvidia drivers on ubuntu 22.04, accessed October 16, 2025, [https://askubuntu.com/questions/1459950/cant-install-nvidia-drivers-on-ubuntu-22-04](https://askubuntu.com/questions/1459950/cant-install-nvidia-drivers-on-ubuntu-22-04)  
13. How to Install Docker on Ubuntu 22.04 \- Cherry Servers, accessed October 16, 2025, [https://www.cherryservers.com/blog/install-docker-ubuntu-22-04](https://www.cherryservers.com/blog/install-docker-ubuntu-22-04)  
14. Ubuntu | Docker Docs, accessed October 16, 2025, [https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)  
15. How to Install Docker on Ubuntu 22.04 | by Andrey Byhalenko | DevOps Manuals and Technical Notes | Medium, accessed October 16, 2025, [https://medium.com/devops-technical-notes-and-manuals/how-to-install-docker-on-ubuntu-22-04-b771fe57f3d2](https://medium.com/devops-technical-notes-and-manuals/how-to-install-docker-on-ubuntu-22-04-b771fe57f3d2)  
16. Installing/Configuring Nvidia Container Toolkit for Docker on Linux Mint 21.1 (Based on Ubuntu 22.04) \- GitHub Gist, accessed October 16, 2025, [https://gist.github.com/practical-dreamer/79c26931d99bc6a7f28271b3612907a9](https://gist.github.com/practical-dreamer/79c26931d99bc6a7f28271b3612907a9)  
17. Installing the NVIDIA Container Toolkit \- NVIDIA Docs Hub, accessed October 16, 2025, [https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)  
18. Can't make the nvidia-driver-570-server driver work on Ubuntu 24.04, accessed October 16, 2025, [https://askubuntu.com/questions/1551395/cant-make-the-nvidia-driver-570-server-driver-work-on-ubuntu-24-04](https://askubuntu.com/questions/1551395/cant-make-the-nvidia-driver-570-server-driver-work-on-ubuntu-24-04)  
19. Docker installation \- Autoware Universe Documentation, accessed October 16, 2025, [https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/docker-installation/](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/autoware/docker-installation/)  
20. autowarefoundation/autoware\_universe \- GitHub, accessed October 16, 2025, [https://github.com/autowarefoundation/autoware\_universe](https://github.com/autowarefoundation/autoware_universe)  
21. osrf/rocker: A tool to run docker containers with overlays and convenient options for things like GUIs etc. \- GitHub, accessed October 16, 2025, [https://github.com/osrf/rocker](https://github.com/osrf/rocker)  
22. Autoware \- Docker Image, accessed October 16, 2025, [https://hub.docker.com/r/autoware/autoware](https://hub.docker.com/r/autoware/autoware)  
23. Advanced usage of colcon \- Autoware Documentation, accessed October 16, 2025, [https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/others/advanced-usage-of-colcon/](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/others/advanced-usage-of-colcon/)  
24. How to colcon build RelWithDebInfo with no code optimizations ( \-O0 ) \- ROS Answers, accessed October 16, 2025, [https://answers.ros.org/question/390957/](https://answers.ros.org/question/390957/)  
25. Debugging ros2 libraries \- cmake \- Robotics Stack Exchange, accessed October 16, 2025, [https://robotics.stackexchange.com/questions/105590/debugging-ros2-libraries](https://robotics.stackexchange.com/questions/105590/debugging-ros2-libraries)  
26. How to — colcon documentation \- Read the Docs, accessed October 16, 2025, [https://colcon.readthedocs.io/en/released/user/how-to.html](https://colcon.readthedocs.io/en/released/user/how-to.html)  
27. VSCode setup \- Kev's Robots, accessed October 16, 2025, [https://www.kevsrobots.com/learn/learn\_ros/08\_vscode\_setup.html](https://www.kevsrobots.com/learn/learn_ros/08_vscode_setup.html)  
28. ROS 2 and VSCode | PickNik, accessed October 16, 2025, [https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html)  
29. Remote debugging with GDB | Red Hat Developer, accessed October 16, 2025, [https://developers.redhat.com/blog/2015/04/28/remote-debugging-with-gdb](https://developers.redhat.com/blog/2015/04/28/remote-debugging-with-gdb)