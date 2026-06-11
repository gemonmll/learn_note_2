好的，这篇教程的目的是让你**亲眼看到** ROS 2 安全机制是如何工作的。

这篇文章的目标是：**使用 `tcpdump` 网络抓包工具，捕获并对比启用安全前后的 ROS 2 网络流量，以直观地证明“加密”确实发生了。**

-----

### 1\. 概述 (Overview)

之前的教程只是*启用*了安全，但这篇教程要*验证*安全。我们将捕获原始网络数据包，看看启用安全前后的区别。

**⚠️ 重要提示 (针对 Fast DDS 用户)：**

  * 默认情况下，如果 `talker` 和 `listener` 在**同一台主机**上，Fast DDS (ROS 2 Jazzy 的默认 RMW) 会自动使用**共享内存 (Shared Memory Transport)** 来通信，而不是通过网络。
  * 这会导致 `tcpdump` 或 `Wireshark` **抓不到任何数据包**（因为数据根本没经过网络接口）。
  * **解决方案：**
    1.  （简单）按照上一篇教程，在**两台不同的机器**上运行 `talker` 和 `listener`。
    2.  （复杂）或者，你必须通过 XML 配置文件**禁用共享内存**，强制 Fast DDS 使用 UDP。

本教程假设你在**同一台机器**上运行，但通过不同的 `ssh` 会话模拟，并且**没有禁用共享内存**。因此，教程中的抓包命令能抓到数据，这暗示教程作者可能没有使用默认的 Fast DDS，或者使用了某种特定配置。不过，我们还是按照教程的步骤来学习。

-----

### 2\. 先决条件 (Prerequisites)

  * 本教程只在 Linux 上运行。
  * 你需要安装 `tcpdump`。

-----

### 3\. 运行演示 (Run the demo)

#### 第1步：安装 `tcpdump`

`tcpdump` 是一个命令行的网络抓包工具。(你也可以使用图形化的 `Wireshark`，原理相同)。

```bash
# 源码 (命令) - 终端 3
sudo apt update
sudo apt install tcpdump
```

#### 第2步：运行 *未加密* 的 talker/listener

我们要先看看*不安全*的通信是什么样子。

**终端 1 (运行 Talker):**

```bash
# 源码 (命令) - 终端 1
# 确保没有设置安全环境变量 (或者显式 unset)
unset ROS_SECURITY_ENABLE

# 运行 talker (注意：--enclave 参数在这里会被忽略，因为安全未启用)
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

**终端 2 (运行 Listener):**

```bash
# 源码 (命令) - 终端 2
# 同样，确保安全未启用
unset ROS_SECURITY_ENABLE
ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener
```

#### 第3步：抓取 *未加密* 的“发现”包

ROS 2 节点启动时，会通过**多播 (multicast)** 互相“发现”。

**终端 3 (运行 tcpdump):**
`tcpdump` 需要 `sudo` 权限来监听网络接口。

```bash
# 源码 (命令) - 终端 3
# -X: 打印包内容的 ASCII 和 Hex 编码
# -i any: 监听所有网络接口
# udp port 7400: 只抓取 DDS 发现协议 (DDSI-RTPS) 默认使用的 7400 端口
sudo tcpdump -X -i any udp port 7400
```

  * **预期输出 (精炼讲解)：**
    你会看到类似下面这样的输出，其中包含了**明文信息**：
    ```
    ...
    0x00d0: 2f74 616c 6b65 725f 6c69 7374 656e 6572  /talker_listener
    0x00e0: 2f74 616c 6b65 7200 2c00 2800 2100 0000  /talker.,.(.!...
    0x00f0: 656e 636c 6176 653d 2f74 616c 6b65 725f  enclave=/talker_
    0x0100: 6c69 7374 656e 6572 2f74 616c 6b65 723b  listener/talker;
    ...
    ```
  * **分析：**
      * 目标地址是 `239.255.0.1` (多播地址)。
      * 目标端口是 `7400`。
      * **关键：** 我们可以清楚地看到 ASCII 码中的明文字符串，如节点名 `/talker_listener/talker` 和 `enclave=...`。这说明**发现过程是未加密的**，网络上的任何人都可以知道你的节点名称和拓扑结构。

#### 第4步：抓取 *未加密* 的“数据”包

现在我们来抓取 `chatter` 话题的实际数据。DDS 数据包通常使用 `7400` 以上的端口。

**终端 3 (运行 tcpdump):** (按 `Ctrl+C` 停止上一个命令)

```bash
# 源码 (命令) - 终端 3
# 监听 7401 到 7500 范围的 UDP 端口
sudo tcpdump -i any -X udp portrange 7401-7500
```

  * **预期输出 (精炼讲解)：**
    你会看到类似下面这样的输出，其中包含了**明文数据**：
    ```
    ...
    0x0050: 5708 0000 0001 0000 1200 0000 4865 6c6c  W...........Hell
    0x0060: 6f20 576f 726c 643a 2032 3133 3500 0000  o.World:.2135...
    ...
    ```
  * **分析：**
      * **关键：** 我们可以清楚地看到 ASCII 码中的消息内容 "Hello World: 2135"。
      * 这证明了**话题数据是未加密的**。网络上的任何人都可以窃听你的话题内容。

-----

#### 第5步：启用加密

现在，我们停止 `talker` 和 `listener` (在终端 1 和 2 中按 `Ctrl+C`)，然后用安全的方式重启它们。

**终端 1 (运行 安全的 Talker):**

```bash
# 源码 (命令) - 终端 1
# 设置安全环境变量
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# 启动 talker
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

**终端 2 (运行 安全的 Listener):**

```bash
# 源码 (命令) - 终端 2
# 设置安全环境变量
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# 启动 listener
ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener
```

#### 第6步：抓取 *加密* 的“发现”包

我们再次运行和第3步完全相同的 `tcpdump` 命令。

**终端 3 (运行 tcpdump):**

```bash
# 源码 (命令) - 终端 3
sudo tcpdump -X -i any udp port 7400
```

  * **预期输出 (精炼讲解)：**
      * 包变得**非常大** (教程中是 596 字节，之前是 252 字节)。
      * 你可能**仍然**能看到一些明文，比如节点名 `/talker_listener/listener` 和 `enclave=...`。
      * **但是**，包中会多出很多额外的数据块，如 `DDS:Auth:PKI-DH:` 和 `dds.perm_ca.algo` 等。
  * **分析：**
      * 这些是 DDS-Security 握手包。节点在交换它们的证书和权限，进行身份验证。
      * **关键：** 尽管在默认配置下，一些发现信息（如节点名）可能仍然是明文（这取决于 `governance.xml` 策略中是否加密了发现过程），但身份验证和后续的数据交换已经是安全的了。

#### 第7步：抓取 *加密* 的“数据”包

最后，我们再次运行和第4步完全相同的 `tcpdump` 命令。

**终端 3 (运行 tcpdump):** (按 `Ctrl+C` 停止上一个命令)

```bash
# 源码 (命令) - 终端 3
sudo tcpdump -i any -X udp portrange 7401-7500
```

  * **预期输出 (精炼讲解)：**
    ```
    ...
    0x0130: 7905 d390 3201 1400 3ae5 0b60 3906 967e  y...2...:..`9..~
    0x0140: 5b17 fd42 de95 54b9 0000 0000 3401 1400  [..B..T.....4...
    0x0150: 42ae f04d 0559 84c5 7116 1c51 91ba 3799  B..M.Y..q..Q..7.
    0x0160: 0000 0000 ....
    ...
    ```
  * **分析：**
      * **关键：** 在 ASCII 码区域，你**再也看不到 "Hello World"** 了。你所能看到的只是一堆无意义的乱码。
      * 这证明了**话题数据现在是完全加密的**。网络上的窃听者无法再读取你的通信内容。