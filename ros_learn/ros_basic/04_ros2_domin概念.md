

# **ROS 2 Domain ID 详尽解析：从核心原理到高级网络应用场景**

## **引言**

现代机器人技术的发展早已超越了单一智能体的范畴，演变为复杂的分布式系统。这些系统中的计算节点常常分布在多台计算机甚至整个机器人集群中，从而对网络管理、通信完整性和系统可扩展性提出了严峻挑战。为了应对这些挑战，机器人操作系统第二代（ROS 2）从设计之初就将分布式通信作为其核心能力。

ROS 2 通过一个名为ROS中间件接口（RMW）的抽象层来实现其通信功能，该接口默认依赖于数据分发服务（DDS）标准 1。DDS 是一种为实时分布式系统量身打造的通信协议，其强大的功能和灵活性使其成为 ROS 2 的理想选择。在 DDS 的诸多特性中，Domain ID 是实现网络分割和隔离的基石 1。它提供了一种核心机制，允许在同一个物理网络上创建多个相互隔离的“逻辑网络”，从而有效防止不同机器人系统间的意外串扰 4。

本报告旨在对 ROS 2 中的 ROS\_DOMAIN\_ID 进行一次全面而深入的分析。报告将从 DDS 的基本通信原理入手，逐步深入到底层网络端口的计算机制，并最终扩展到实际应用中的配置方法、关键限制因素以及高级故障排查策略。其目标是为机器人开发者和系统架构师提供一份权威的技术参考，使其能够自信地设计、配置和调试复杂的多节点及多机器人 ROS 2 系统。

---

## **第 1 节：DDS Domain 在 ROS 2 中的基础作用**

在深入探讨技术细节之前，首先必须理解 Domain ID 背后的核心概念，即它为何如此重要。本节将阐述其作为逻辑通信平面的基本原理，并澄清一些常见的混淆概念。

### **1.1. 作为逻辑通信平面的“Domain”**

源于 DDS 的核心设计，一个“Domain”（域）被定义为一个独立的、自包含的通信平面，或可称之为“虚拟网络” 8。在同一个域内的实体（例如应用程序或 ROS 2 节点）可以自由地发现彼此并进行通信。然而，它们对于存在于其他域中的实体是完全无感知的，即便所有实体共享着相同的物理硬件和网络基础设施 8。

这个概念对于多租户环境至关重要。例如，在科研实验室、自动驾驶竞赛 5 或自动化工厂中，可能同时运行着多个独立的机器人应用程序。通过为每个系统分配不同的 Domain ID，可以确保它们在共享的网络上共存而互不干扰。

### **1.2. ROS 2 中间件（RMW）与 ROS\_DOMAIN\_ID**

ROS 2 通过 RMW API 实现了其对底层通信中间件的灵活性 3。尽管存在多种 RMW 实现（如 rmw\_fastrtps\_cpp、rmw\_cyclonedds\_cpp），但主流的实现都基于 DDS 标准 3。为了给用户提供一个统一的配置接口，ROS 2 引入了 ROS\_DOMAIN\_ID 这个环境变量。它作为一种上层抽象，用于控制底层 DDS 实现所使用的 domainId 5。默认情况下，所有 ROS 2 节点都会加入 Domain 0 1。

### **1.3. 隔离的核心法则**

Domain ID 的工作机制可以归结为一个非常简单的基本原则：**处于相同 Domain ID 的 ROS 2 节点可以自由地发现并相互通信，而处于不同 Domain ID 的节点则不能** 1。这种隔离是在 DDS 传输层实现的，是绝对的。从网络层面看，不同域的节点对于彼此是完全“隐形”的 14。

一个形象的比喻是：物理网络（如 Wi-Fi 或以太网）就像一栋公寓楼，而 ROS\_DOMAIN\_ID 则是每个公寓的门牌号。住在门牌号为 10 的公寓里的人可以自由交谈，但他们听不到也无法与门牌号为 20 的公寓里的人交流，反之亦然 7。

### **1.4. 关键区别：Domain ID 与 ROS 命名空间**

初学者常常将 Domain ID 与 ROS 命名空间（Namespace）混淆，因此有必要对二者进行清晰的区分。

* **Domain ID (ROS\_DOMAIN\_ID)**：工作在 DDS 传输层。它创建的是完全独立的、物理隔离的通信网络。其核心作用是实现**系统级的隔离**。两个处于不同 Domain ID 的系统在网络层面完全看不到对方 5。  
* **ROS 命名空间 (Namespace)**：工作在 ROS 应用层。它是在一个**单一 ROS 系统（即单一 Domain）内部**为节点和话题名称添加前缀。其核心作用是实现**系统内的逻辑组织** 5。

一个至关重要的观点是，在 DDS 环境中，仅使用命名空间不足以实现真正的系统隔离。即使两个机器人的话题名称通过命名空间区分开（例如 /robot1/cmd\_vel 和 /robot2/cmd\_vel），只要它们处于同一个 Domain ID，所有节点仍然会参与相同的发现过程。这意味着每个节点都会向网络中的所有其他节点（无论其命名空间如何）广播自己的存在信息。在大型系统中，这种不必要的发现流量会给网络和节点的 CPU 带来巨大的负担 12。而使用 Domain ID 则从根本上杜绝了这种发现级别的串扰，实现了更彻底、更高效的隔离。

因此，ROS\_DOMAIN\_ID 不仅仅是一个配置参数，更是一项基本的架构决策，它定义了一个机器人系统的通信边界。如何规划和分配 Domain ID（例如，是为每个机器人、每个机器人集群还是每个应用分配一个独立的 ID）是系统设计阶段必须考虑的关键问题。一个不恰当的 Domain ID 策略可能会导致后期需要进行复杂的网络重构，或者不得不依赖如 ROS 2 Router 这样的桥接工具来实现跨域通信。

---

## **第 2 节：Domain ID 实现的技术机理**

本节将揭开 ROS\_DOMAIN\_ID 的抽象面纱，深入探讨其在网络层面的具体实现机制。

### **2.1. 确定性链接：从 Domain ID 到 UDP 端口**

Domain ID 的核心功能是作为一个关键输入参数，参与一个确定性的算法，该算法用于计算节点通信时所使用的 UDP 端口号 1。正是这种机制，在物理网络上强制实现了逻辑隔离。大多数 DDS 实现都默认使用 UDP 协议，因为它具有低延迟的特性，非常适合实时系统的需求 3。

### **2.2. DDS 端口计算公式解析**

OMG DDS-RTPS（实时发布订阅协议）规范中定义了用于计算多播和单播端口的标准公式，主流的 DDS 供应商（如 eProsima Fast DDS 和 Eclipse Cyclone DDS）都遵循此规范 16。其基本形式如下：

$$Port \= PB \+ (DG \\times \\text{domainId}) \+ (PG \\times \\text{participantId}) \+ \\text{Offset}$$  
公式中每个参数的含义如下：

* **PB (PortBase)**：所有 DDS 通信的起始基础端口号。该值通常为 7400 16。  
* **DG (DomainGain)**：为每个 Domain 分配的端口块的大小。该值通常为 250 16。  
* **domainId**：即 ROS\_DOMAIN\_ID 设置的值。  
* **PG (ParticipantGain)**：为每个 DDS 参与者（Participant）分配的端口数量。该值通常为 2 16。  
* **participantId**：一个从 0 开始计数的唯一整数，用于标识在**同一台主机**和**同一个 Domain** 内的每一个 ROS 2 进程。  
* **Offset**：一些小的常量（如 d0, d1, d2, d3），用于区分不同类型的端口，例如发现端口与用户数据端口、多播端口与单播端口 16。

这种确定性的端口计算方法揭示了一个深刻的事实：ROS\_DOMAIN\_ID 不仅仅是一个逻辑标签，它更是一种**网络资源（UDP 端口）的分配机制**。当用户设置 ROS\_DOMAIN\_ID=0 时，DDS 会占用 7400 至 7649 范围内的端口；当设置为 1 时，则会占用 7650 至 7899 范围内的端口，以此类推。由于 UDP 端口是操作系统管理的有限资源（一个 16 位无符号整数，最大值为 65535），这就从根本上解释了为何 Domain ID 的数量存在上限（大约为 232），以及为何不当的设置会导致与操作系统临时端口的冲突。这并非软件的任意限制，而是端口计算公式与网络协议内在约束的直接数学结果 1。

### **2.3. 端口分配详解：多播与单播**

DDS 的通信过程巧妙地利用了多播（Multicast）和单播（Unicast）两种方式，分别用于节点发现和数据传输。

* **发现多播端口 (Discovery Multicast Ports)**：每个 Domain 都有两个众所周知的多播端口，其计算不依赖于 participantId（或可视为 participantId=0） 16。同一 Domain 内的所有节点都会监听这两个端口。当一个新节点启动时，它会通过这些端口向整个 Domain 广播其存在信息（这个过程被称为简单参与者发现协议，SPDP）。这就像一个“城市广场”，新来的人在这里高喊“我来了！”，以便让城里的其他人知道 20。  
* **用户单播端口 (User Unicast Ports)**：一旦两个节点通过多播发现彼此，它们就会交换各自唯一的单播端口信息。后续的实际数据（如话题消息）将通过这些单播端口进行高效的点对点传输 1。每个 ROS 2 进程（即 DDS Participant）都会根据其唯一的 participantId 获得一组专属的单播端口。这种方式避免了将所有数据都进行广播，大大提高了通信效率。

---

## **第 3 节：实际应用与配置方法**

本节将提供可操作的指南，指导开发者如何在不同场景下正确配置和使用 ROS\_DOMAIN\_ID。

### **3.1. 设置 ROS\_DOMAIN\_ID 环境变量**

最常用和直接的方法是通过设置环境变量来配置 Domain ID。

* **临时配置（当前终端有效）**：在 Linux 或 macOS 系统中，使用 export ROS\_DOMAIN\_ID=\<value\> 命令。在 Windows 中，则使用 set ROS\_DOMAIN\_ID=\<value\>。这种方式设置的 Domain ID 仅在当前终端会话中有效，非常适合用于临时测试和调试 4。  
* **永久配置（用户或系统级）**：为了让配置在每次打开新终端时都生效，可以将上述 export 命令添加到用户的 shell 启动脚本中，例如 \~/.bashrc 或 \~/.zshrc。这对于机器人或专用开发机是推荐的做法，可以确保通信环境的一致性 11。  
* **验证配置**：设置完成后，可以使用 echo $ROS\_DOMAIN\_ID (Linux/macOS) 或 printenv | grep ROS\_DOMAIN\_ID 命令来检查环境变量是否已正确设置 21。

### **3.2. 编程方式与逐节点配置**

除了全局的环境变量，ROS 2 还提供了在代码中为单个节点指定 Domain ID 的高级功能。

* 在 C++ 中，可以通过 rclcpp::NodeOptions 对象的 domain\_id 成员进行设置。  
* 在 Python 的 rclpy 中，同样可以在创建节点时传入 domain\_id 参数 25。

通过编程方式设置的 Domain ID 将会**覆盖**全局的 ROS\_DOMAIN\_ID 环境变量，仅对该节点生效 25。这个特性对于开发复杂的网桥或监控工具非常有用。例如，一个进程可以同时实例化两个节点：一个节点加入 Domain 10 与远端的机器人集群通信，另一个节点则加入 Domain 0 与本地的图形界面交互，整个过程无需切换环境变量。

### **3.3. 命令行参数：一个值得注意的缺失**

尽管 ROS 2 提供了丰富的命令行参数来配置节点（例如，使用 \--ros-args 配合 \-r 进行重映射，或 \-p 设置参数），但目前**没有标准的命令行参数**可以直接用来设置 ROS\_DOMAIN\_ID 27。

这是一个已知的局限性。社区中已经有相关的特性请求，希望增加类似的命令行标志，以简化自动化脚本和使配置更加明确 29。了解这一点有助于开发者避免徒劳的尝试，并选择正确的配置方法。

### **3.4. 动手实践：隔离 Talker 和 Listener**

下面通过一个简单的例子来直观地展示 Domain ID 的隔离效果。

* **场景一：正常通信**  
  1. 打开第一个终端，设置 Domain ID 并启动 talker 节点：  
     Bash  
     export ROS\_DOMAIN\_ID=42  
     ros2 run demo\_nodes\_cpp talker

  2. 打开第二个终端，设置相同的 Domain ID 并启动 listener 节点：  
     Bash  
     export ROS\_DOMAIN\_ID=42  
     ros2 run demo\_nodes\_cpp listener

  3. **预期结果**：listener 终端会持续打印出 talker 发布的消息，表明通信成功 7。  
* **场景二：通信隔离**  
  1. 在第一个终端中，仍然使用 Domain ID 42 启动 talker：  
     Bash  
     export ROS\_DOMAIN\_ID=42  
     ros2 run demo\_nodes\_cpp talker

  2. 在第二个终端中，设置一个**不同**的 Domain ID（例如 43）并启动 listener：  
     Bash  
     export ROS\_DOMAIN\_ID=43  
     ros2 run demo\_nodes\_cpp listener

  3. **预期结果**：listener 终端不会收到任何消息。此外，如果在第二个终端中执行 ROS\_DOMAIN\_ID=43 ros2 topic list，将看不到 talker 节点在 Domain 42 中发布的话题 4。

### **3.5. 多机器人部署架构**

在多机器人场景下，可以根据通信需求设计不同的 Domain ID 部署策略。

* **一机器人一域 (One-Domain-Per-Robot)**：为每一台机器人分配一个独一无二的 ROS\_DOMAIN\_ID。这种策略提供了最大程度的隔离，完全杜绝了机器人之间的意外通信。它非常适用于教学环境或机器人各自执行独立任务的场景 14。  
* **一集群一域 (One-Domain-Per-Fleet)**：为一组需要协同工作的机器人分配一个共同的 ROS\_DOMAIN\_ID。这使得集群内的机器人可以作为一个统一的分布式系统无缝通信，同时与网络上的其他机器人集群或系统保持隔离 7。  
* **混合策略**：在更复杂的系统中，可以将 Domain ID 和命名空间结合使用。例如，将所有仓库机器人划分到 Domain 5 以实现集群隔离，然后在该域内使用 /robot\_A、/robot\_B 等命名空间来区分和组织单个机器人 14。

---

## **第 4 节：约束、限制与最佳实践**

理解 ROS\_DOMAIN\_ID 的局限性对于设计稳定可靠的系统至关重要。本节将详细讨论相关的约束条件和推荐的最佳实践。

### **4.1. 选择安全且可移植的 Domain ID**

* **理论范围与推荐范围**：根据 UDP 端口计算公式和最大端口号 65535，理论上可用的最大 Domain ID 为 232 1。然而，为了确保跨平台的兼容性并避免与操作系统网络功能的潜在冲突，**强烈建议使用 0 到 101 这个保守的范围** 1。  
* **社区实践**：为了避免不同开发团队在共享网络中发生 Domain ID 冲突，ROS 2 社区甚至维护了一个共享的电子表格，供团队注册和预留他们使用的 ID 11。

### **4.2. 平台特定约束：临时端口（Ephemeral Ports）**

操作系统会保留一段高位的端口号范围，用于处理临时的出站网络连接，这个范围被称为临时端口范围。如果选择的 Domain ID 过高，DDS 计算出的端口号可能会与这个范围重叠，导致不可预测的网络问题 1。

#### **表 1：各操作系统平台下的安全 ROS\_DOMAIN\_ID 范围**

下表汇总了主流操作系统的默认临时端口范围以及由此推导出的安全 ROS\_DOMAIN\_ID 范围，为开发者提供清晰的参考 1。

| 操作系统 | 默认临时端口范围 | 安全的 ROS\_DOMAIN\_ID 范围 |
| :---- | :---- | :---- |
| Linux | 32768 \- 60999 | 0 \- 101 和 215 \- 232 |
| macOS | 49152 \- 65535 | 0 \- 166 |
| Windows | 49152 \- 65535 | 0 \- 166 |

### **4.3. 参与者约束与进程数量限制**

“参与者”（Participant）是 DDS 中的一个概念，在 ROS 2 的语境下，每个 ROS 2 进程（例如一个运行中的可执行文件）都会创建一个 DDS Participant 1。这带来了两个关于进程数量的重要限制。

1. **域间端口冲突**：每个 Domain 分配了 250 个端口，而每个 Participant 会从中占用 2 个单播端口。当在同一台主机上的某个 Domain 内运行的 Participant 数量过多时，其分配的单播端口号会不断递增，最终可能“溢出”到下一个 Domain 的多播端口范围内，造成域间干扰。这个临界值大约是**每个主机每个域 120 个 Participant (进程)** 1。  
2. **与临时端口的冲突**：当使用一个较高的 Domain ID 时，即使 Participant 数量不多，其计算出的单播端口号也可能很快就进入操作系统的临时端口范围，从而严重限制了可运行的进程数量。

这种 Domain ID 与最大进程数之间的反比关系并不直观，但对于设计高密度计算节点的系统架构师来说却至关重要。例如，一位架构师可能会为一台 Linux 机器选择 ROS\_DOMAIN\_ID=101，因为它在“安全范围”内，却没有意识到这个选择将这台机器上可运行的 ROS 2 进程数量隐性地限制在了 54 个。

#### **表 2：特定 Domain ID 下单台主机的最大 ROS 2 进程数**

下表明确列出了在一些临界 Domain ID 值下，不同操作系统上单台主机可以运行的最大 ROS 2 进程数，以防止此类设计疏忽 1。

| 操作系统 | ROS\_DOMAIN\_ID | 最大 ROS 2 进程数 | 限制原因 |
| :---- | :---- | :---- | :---- |
| Linux | 101 | 54 | 参与者单播端口与 Linux 临时端口范围（起始于 32768）发生冲突 |
| macOS / Windows | 166 | 120 | 参与者单播端口与 macOS/Windows 临时端口范围（起始于 49152）发生冲突 |
| 任何系统 | 232 | 63 | 参与者单播端口与 UDP 协议的最大端口号（65535）发生冲突 |

---

## **第 5 节：故障排查与高级网络主题**

本节将讨论当通信出现问题时如何诊断，并介绍一些超越默认配置的高级网络解决方案。

### **5.1. 诊断常见的发现失败问题**

当节点无法相互发现时，可以按以下步骤进行排查：

* **ROS\_DOMAIN\_ID 不匹配**：这是最常见的问题。确保所有需要通信的节点都配置了完全相同的 ROS\_DOMAIN\_ID，并且该环境变量已在它们的运行环境中正确导出 20。  
* **ros2 daemon 进程问题**：ROS 2 的命令行工具（如 ros2 topic list）依赖一个后台的 ros2 daemon 进程。这个守护进程也必须运行在正确的 Domain ID 上。如果它是在设置不同的 ID 时启动的，可能会导致命令行工具无法发现节点，即便节点之间通信正常。解决方案是：在设置好正确的 Domain ID 后，执行 ros2 daemon stop; ros2 daemon start 来重启守护进程 30。  
* **ROS\_LOCALHOST\_ONLY=1 的影响**：这个环境变量会强制所有 ROS 2 通信限制在本地主机（localhost），从而阻止任何跨机器的发现。在教室或共享网络环境中，这常常是导致问题的元凶 21。解决方案是取消设置该变量，或将其值设为 0。  
* **网络配置问题**：防火墙阻止了 DDS 所需的 UDP 端口，或者网络交换机禁用了多播功能，都可能导致发现失败。解决方案是检查网络和防火墙规则，确保 DDS 计算出的端口范围内的 UDP 流量（包括多播和单播）是允许通行的。

### **5.2. 超越多播：发现服务器（Discovery Server）**

默认的节点发现机制（Simple Discovery）依赖于 UDP 多播。但在某些网络环境（尤其是 Wi-Fi）中，多播包的传输并不可靠，或者被网络管理员禁用。此时，可以采用发现服务器模式作为替代方案 4。

这是一种客户端-服务器架构，其中一个或多个节点被配置为“服务器”，其他节点作为“客户端”连接到这些服务器。所有的发现信息都通过服务器进行中继，而不是通过多播广播。这种方式在不可靠的网络上更为健壮 34。配置时，客户端节点需要通过 ROS\_DISCOVERY\_SERVER 环境变量指向服务器的地址和端口 4。

### **5.3. 跨域桥接：ROS 2 路由器（ddsrouter）**

在某些高级应用中，完全的隔离可能并非所需。有时，需要在不同的 Domain 之间有选择地共享信息。eProsima 的 DDS Router（或称 ddsrouter）工具可以实现这一目标，它能作为一个可配置的桥梁，连接两个或多个不同的 Domain 37。

例如，假设有两个机器人集群，分别在 Domain 10 和 Domain 20 上运行。可以配置一个 ddsrouter，使其仅将 Domain 10 中的 /fleet\_status 话题转发到 Domain 20。这样，既实现了有限、可控的信息共享，又避免了将两个网络完全合并所带来的复杂性和风险 37。

### **5.4. 已知的兼容性问题与边缘案例**

需要注意的是，并非所有 ROS 2 生态系统中的工具都能完美地遵循 ROS\_DOMAIN\_ID 的设置。一个典型的例子是 micro-ros-agent，它用于将微控制器上的 micro-ROS 节点桥接到主 ROS 2 网络。该工具通常期望在默认的 Domain 0 上运行，使用其他 Domain ID 可能会导致其无法正常工作 35。

这对于从事嵌入式系统开发的工程师来说是一个重要的“陷阱”。它意味着连接嵌入式世界和主 ROS 2 网络的桥梁节点，往往必须固定在 Domain 0，这一点可能会对整个系统的网络架构设计产生深远影响。

---

## **结论与战略建议**

ROS\_DOMAIN\_ID 是 ROS 2 中一个功能强大、基础而又充满细节的网络隔离机制。它是确保多个 ROS 2 系统在共享网络上稳定、无干扰运行的主要工具。一个成功的 ROS 2 系统部署，离不开对 Domain ID 的深刻理解和审慎规划。

基于本报告的详尽分析，为系统架构师提出以下战略性建议：

1. **主动规划 Domain 策略**：在部署任何多节点或多机器人系统之前，应根据系统的通信需求，明确地设计 Domain ID 的分配方案。切勿依赖默认的 Domain 0，尤其是在生产或共享环境中。  
2. **优先考虑可移植性**：在选择 Domain ID 时，应优先使用 0 到 101 的范围。这能确保您的系统无需修改即可在所有主流操作系统（Linux, macOS, Windows）上正常运行。  
3. **关注进程密度限制**：在设计需要运行大量 ROS 2 进程的高性能计算节点时，必须意识到每个域大约 120 个进程的上限，并了解使用较高的 Domain ID 会进一步降低这个上限。  
4. **文档化与标准化**：在团队协作中，应将每个系统使用的 Domain ID 文档化，并强制执行统一的配置标准。这将有效避免因配置不一致而导致的、难以调试的通信故障。  
5. **了解并善用高级工具**：对于不可靠的网络环境（如 Wi-Fi），或需要实现精细的跨域通信时，应准备好超越默认配置，评估并采用如发现服务器或 ROS 2 路由器等高级工具来满足需求。

#### **Works cited**

1. The ROS\_DOMAIN\_ID — ROS 2 Documentation: Foxy documentation, accessed October 20, 2025, [https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html)  
2. Introduction to Robot Operating System 2 (ROS 2\) \- MATLAB & Simulink \- MathWorks, accessed October 20, 2025, [https://www.mathworks.com/help/ros/gs/robot-operating-system-ros2-basic-concepts.html](https://www.mathworks.com/help/ros/gs/robot-operating-system-ros2-basic-concepts.html)  
3. ROS 2 Communication | Clearpath Robotics Documentation, accessed October 20, 2025, [https://docs.clearpathrobotics.com/docs/ros2humble/ros/networking/ros2\_communication/](https://docs.clearpathrobotics.com/docs/ros2humble/ros/networking/ros2_communication/)  
4. Running ROS 2 on Multiple Machines | Husarion, accessed October 20, 2025, [https://husarion.com/tutorials/ros2-tutorials/6-robot-network/](https://husarion.com/tutorials/ros2-tutorials/6-robot-network/)  
5. ROS Domain ID \- Steven Gong, accessed October 20, 2025, [https://stevengong.co/notes/ROS-Domain-ID](https://stevengong.co/notes/ROS-Domain-ID)  
6. DDS Middleware and Network tuning \- Stereolabs, accessed October 20, 2025, [https://www.stereolabs.com/docs/ros2/dds\_and\_network\_tuning](https://www.stereolabs.com/docs/ros2/dds_and_network_tuning)  
7. Understanding DDS and the ROS\_DOMAIN\_ID Variable \- Automatic Addison, accessed October 20, 2025, [https://automaticaddison.com/understanding-dds-and-the-ros\_domain\_id-variable/](https://automaticaddison.com/understanding-dds-and-the-ros_domain_id-variable/)  
8. 3.2. Domain \- 3.4.0 \- eProsima Fast DDS, accessed October 20, 2025, [https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds\_layer/domain/domain.html?highlight=Domain](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/domain/domain.html?highlight=Domain)  
9. 1.1.1. The DCPS conceptual model \- Fast DDS \- eProsima, accessed October 20, 2025, [https://fast-dds.docs.eprosima.com/en/stable/fastdds/getting\_started/definitions.html](https://fast-dds.docs.eprosima.com/en/stable/fastdds/getting_started/definitions.html)  
10. 3.2. Domain \- 3.2.2 \- eProsima Fast DDS, accessed October 20, 2025, [https://fast-dds.docs.eprosima.com/en/v3.2.2/fastdds/dds\_layer/domain/domain.html](https://fast-dds.docs.eprosima.com/en/v3.2.2/fastdds/dds_layer/domain/domain.html)  
11. ROS 2 on-boarding guide \- ROS documentation, accessed October 20, 2025, [https://docs.ros.org/en/eloquent/Contributing/ROS-2-On-boarding-Guide.html](https://docs.ros.org/en/eloquent/Contributing/ROS-2-On-boarding-Guide.html)  
12. Specifying different \`ros\_domain\_ids\` should connect to different routers and not simply prefix topic keyexpressions · Issue \#201 · ros2/rmw\_zenoh \- GitHub, accessed October 20, 2025, [https://github.com/ros2/rmw\_zenoh/issues/201](https://github.com/ros2/rmw_zenoh/issues/201)  
13. Additional settings for developers \- Autoware Documentation, accessed October 20, 2025, [https://autowarefoundation.github.io/autoware-documentation/galactic/installation/additional-settings-for-developers/](https://autowarefoundation.github.io/autoware-documentation/galactic/installation/additional-settings-for-developers/)  
14. Multi-Robot Setup \- Create® 3 Docs \- GitHub Pages, accessed October 20, 2025, [https://iroboteducation.github.io/create3\_docs/setup/multi-robot/](https://iroboteducation.github.io/create3_docs/setup/multi-robot/)  
15. Domain ID's in ROS2 : r/AskRobotics \- Reddit, accessed October 20, 2025, [https://www.reddit.com/r/AskRobotics/comments/1gy0059/domain\_ids\_in\_ros2/](https://www.reddit.com/r/AskRobotics/comments/1gy0059/domain_ids_in_ros2/)  
16. A guide to the configuration options of Eclipse Cyclone DDS \- GitLab de FING, accessed October 20, 2025, [https://gitlab.fing.edu.uy/cocosim/code/cyclonedds/-/blob/0.6.0rc1/docs/manual/config.rst](https://gitlab.fing.edu.uy/cocosim/code/cyclonedds/-/blob/0.6.0rc1/docs/manual/config.rst)  
17. A guide to the configuration options of Eclipse Cyclone DDS \- Gitee, accessed October 20, 2025, [https://gitee.com/runsunlg/cycloneDDS/blob/master/docs/manual/config.rst](https://gitee.com/runsunlg/cycloneDDS/blob/master/docs/manual/config.rst)  
18. Multi-machine communication configuration \- Yahboom, accessed October 20, 2025, [https://www.yahboom.net/public/upload/upload-html/1733882883/Multi-machine%20communication%20configuration.html](https://www.yahboom.net/public/upload/upload-html/1733882883/Multi-machine%20communication%20configuration.html)  
19. 10.3. DomainParticipant profiles \- 3.2.2 \- eProsima Fast DDS, accessed October 20, 2025, [https://fast-dds.docs.eprosima.com/en/v3.2.2/fastdds/xml\_configuration/domainparticipant.html](https://fast-dds.docs.eprosima.com/en/v3.2.2/fastdds/xml_configuration/domainparticipant.html)  
20. publisher \- How do ROS2 nodes find each other on the same ..., accessed October 20, 2025, [https://robotics.stackexchange.com/questions/108497/how-do-ros2-nodes-find-each-other-on-the-same-computer](https://robotics.stackexchange.com/questions/108497/how-do-ros2-nodes-find-each-other-on-the-same-computer)  
21. Setup — 240AR060 \- Introduction to ROS, accessed October 20, 2025, [https://sir.upc.edu/projects/ros2tutorials/appendices/setup/index.html](https://sir.upc.edu/projects/ros2tutorials/appendices/setup/index.html)  
22. Configuring environment — ROS 2 Documentation: Foxy documentation, accessed October 20, 2025, [https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)  
23. Multiple robots · User Manual \- GitHub Pages, accessed October 20, 2025, [https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple\_robots.html](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/multiple_robots.html)  
24. Understanding DDS and the ROS\_DOMAIN\_ID Variable \- YouTube, accessed October 20, 2025, [https://www.youtube.com/watch?v=qSjNQC2AT\_4](https://www.youtube.com/watch?v=qSjNQC2AT_4)  
25. rcl: rcl\_node\_options\_t Struct Reference \- ROS Documentation, accessed October 20, 2025, [https://docs.ros2.org/beta1/api/rcl/structrcl\_\_node\_\_options\_\_t.html](https://docs.ros2.org/beta1/api/rcl/structrcl__node__options__t.html)  
26. Ability to configure domain ID in rclpy · Issue \#484 \- GitHub, accessed October 20, 2025, [https://github.com/ros2/rclpy/issues/484](https://github.com/ros2/rclpy/issues/484)  
27. ROS Command Line Arguments \- ROS2 Design, accessed October 20, 2025, [https://design.ros2.org/articles/ros\_command\_line\_arguments.html](https://design.ros2.org/articles/ros_command_line_arguments.html)  
28. 5.4.9. Passing ROS arguments to nodes via the command-line \-, accessed October 20, 2025, [https://docs.vulcanexus.org/en/jazzy/ros2\_documentation/source/How-To-Guides/Node-arguments.html](https://docs.vulcanexus.org/en/jazzy/ros2_documentation/source/How-To-Guides/Node-arguments.html)  
29. Specify Domain ID Through Command Line Argument · Issue \#267 · ros2/sros2 \- GitHub, accessed October 20, 2025, [https://github.com/ros2/sros2/issues/267](https://github.com/ros2/sros2/issues/267)  
30. Separating ROS2 environments with ROS\_DOMAIN\_ID \- The Construct, accessed October 20, 2025, [https://www.theconstruct.ai/separating-ros2-environments-ros\_domain\_id-ros2-concepts-in-practice/](https://www.theconstruct.ai/separating-ros2-environments-ros_domain_id-ros2-concepts-in-practice/)  
31. The ROS\_DOMAIN\_ID — ROS 2 Documentation: Rolling documentation, accessed October 20, 2025, [https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html)  
32. Discovery — ROS 2 Documentation: Iron documentation, accessed October 20, 2025, [https://docs.ros.org/en/iron/Concepts/Basic/About-Discovery.html](https://docs.ros.org/en/iron/Concepts/Basic/About-Discovery.html)  
33. The command "ros2 node list" did not display any nodes · Issue \#1616 \- GitHub, accessed October 20, 2025, [https://github.com/ros2/ros2/issues/1616](https://github.com/ros2/ros2/issues/1616)  
34. How to setup ROS2 Fast-DDS Discovery Server | by RoboFoundry | Medium, accessed October 20, 2025, [https://robofoundry.medium.com/how-to-setup-ros2-fast-dds-discovery-server-3843c3a4adec](https://robofoundry.medium.com/how-to-setup-ros2-fast-dds-discovery-server-3843c3a4adec)  
35. \[Solved\] Can't View Topics Remotely \- Help \- Husarion Community, accessed October 20, 2025, [https://community.husarion.com/t/solved-cant-view-topics-remotely/1620](https://community.husarion.com/t/solved-cant-view-topics-remotely/1620)  
36. 3.2. Change ROS 2 Domain to Discovery Server \- \- Vulcanexus Documentation, accessed October 20, 2025, [https://docs.vulcanexus.org/en/jazzy/rst/tutorials/cloud/change\_domain\_discovery\_server/change\_domain\_discovery\_server.html](https://docs.vulcanexus.org/en/jazzy/rst/tutorials/cloud/change_domain_discovery_server/change_domain_discovery_server.html)  
37. 3.1. Change ROS 2 Domain Id \- Vulcanexus Documentation, accessed October 20, 2025, [https://docs.vulcanexus.org/en/iron/rst/tutorials/cloud/change\_domain/change\_domain.html](https://docs.vulcanexus.org/en/iron/rst/tutorials/cloud/change_domain/change_domain.html)  
38. 3.1. Change ROS 2 Domain Id \- \- Vulcanexus Documentation, accessed October 20, 2025, [https://docs.vulcanexus.org/en/jazzy/rst/tutorials/cloud/change\_domain/change\_domain.html](https://docs.vulcanexus.org/en/jazzy/rst/tutorials/cloud/change_domain/change_domain.html)  
39. \[Solved\] ROS\_DOMAIN\_ID not being respected \- Help \- Husarion Community, accessed October 20, 2025, [https://community.husarion.com/t/solved-ros-domain-id-not-being-respected/1711](https://community.husarion.com/t/solved-ros-domain-id-not-being-respected/1711)