好的，这篇教程是建立在之前“设置安全”教程的基础上的，讲解了如何**在两台不同的机器之间**配置和运行安全的 ROS 2 节点。

这篇文章的目标是：**让两台不同的机器（Alice 和 Bob）使用加密和身份验证进行安全通信。**

-----

### 1\. 背景 (Background)

  * 之前的教程都是在同一台机器上（`localhost`）运行 `talker` 和 `listener`。
  * 本教程将其扩展到两台机器，以展示安全功能的真正价值（防止网络窃听和未经授权的访问）。
  * **场景假设：**
      * **机器 `Alice`**：这是我们之前运行 `sros2` 命令的机器。它拥有**完整**的密钥库 (`demo_keystore`)，包括 CA 的私钥。
      * **机器 `Bob`**：这是一台新的、干净的机器。

-----

### 2\. 在 `Bob` 机器上创建 Keystore 目录

首先，我们需要在 `Bob` 机器上创建一个目录，用来存放从 `Alice` 复制过来的安全文件。

```bash
# 源码 (命令) - 在 Alice 机器上远程登录到 Bob
ssh Bob

# 源码 (命令) - 在 Bob 机器上创建父目录
mkdir ~/sros2_demo

# 源码 (命令) - 登出 Bob
exit
```

*(注意：教程原文标题 "Create the second keystore" 容易让人误解。这一步并不是要*生成*一个新的 keystore，而只是为复制文件*创建目标文件夹\*。)\*

-----

### 3\. 复制文件 (从 `Alice` 到 `Bob`)

现在，`Bob` 机器需要获取运行 `talker` 节点所需的安全文件。

**核心概念：**
`Bob` 机器要运行 `talker` 节点，它**至少**需要：

1.  `talker` 节点的 Enclave（包含 `talker` 的私钥、公共证书、权限文件）。
2.  CA 的**公共**证书 (`ca.cert.pem`)，用来验证 `listener` 的身份。
3.  域治理文件 (`governance.p7s`)。

教程中为了简单起见，采用了复制**整个 `demo_keystore` 目录**的方法，这能确保 `Bob` 拥有所有需要的文件。

在**机器 `Alice`** 上执行：

```bash
# 源码 (命令) - 在 Alice 机器上，进入 sros2_demo 目录
cd ~/sros2_demo

# 源码 (命令) - 使用 scp 递归地将整个 demo_keystore 目录复制到 Bob 机器的 sros2_demo 目录下
# 替换 USERNAME@Bob 为你登录 Bob 机器的实际用户名和主机名
scp -r demo_keystore USERNAME@Bob:~/sros2_demo/
```

**⚠️ 教程警告：**
请注意，这种方法（复制整个 keystore）会将**CA 的私钥** (`demo_keystore/private/ca.key.pem`) 也复制到 `Bob` 机器上。从安全角度来看，这**不是**最佳实践。`Bob` 机器*运行*节点**不需要** CA 的私钥（CA 私钥只在*生成*新证书时才需要）。更安全的部署方式（“部署指南”中会提到）是只分发必要的 Enclave 和公共证书。但对于本教程，复制整个目录是最简单的方法。

-----

### 4\. 启动节点 (跨机器)

现在两台机器都拥有了相同的 `demo_keystore`。我们可以开始运行了。

**在 机器 `Bob` 上 (运行 Talker):**

```bash
# 源码 (命令) - 登录到 Bob
ssh Bob

# 源码 (命令) - 在 Bob 上设置环境变量 (路径与 Alice 相同)
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# 源码 (命令) - 在 Bob 上运行 talker，并指定其 enclave
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

**在 机器 `Alice` 上 (运行 Listener):**

(在 `Alice` 机器的**另一个**终端中)

```bash
# 源码 (命令) - 在 Alice 上设置环境变量
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# 源码 (命令) - 在 Alice 上运行 listener，并指定其 enclave
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```

**结果：**
你现在会看到，`Alice` 机器上的 `listener` 成功接收到了来自 `Bob` 机器的 `talker` 发送的消息。这些消息在两台机器之间的网络传输是**经过加密和身份验证**的。