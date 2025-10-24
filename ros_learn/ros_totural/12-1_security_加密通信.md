好的，这篇教程是 ROS 2 安全（SROS2）的入门指南。

这篇文章的目标是：**使用 `sros2` 工具包，启用并运行一个加密通信的 ROS 2 talker/listener 示例。**

-----

### 1\. 背景 (Background)

  * `sros2` 包提供了在 ROS 2 中启用 DDS-Security (DDS 安全标准) 的工具。
  * 它支持跨平台（Linux, macOS, Windows）和跨语言（C++, Python）。
  * **核心概念：** 安全是基于底层的 DDS 中间件（RMW）实现的。`sros2` 工具就是用来生成和管理这些中间件所需的密钥和证书文件。
  * **注意：** 安全通信**不支持**跨不同的 DDS 厂商（例如，安全的 Fast DDS 无法与安全的 Cyclone DDS 通信）。

-----

### 2\. 安装 (Installation)

  * 通常，标准的 ROS 2 Jazzy `desktop` 安装已包含安全功能。

  * **如果从源码编译：** 你需要先安装 OpenSSL。

    (以 Linux 为例)

    ```bash
    # 源码 (命令)
    sudo apt update
    sudo apt install libssl-dev
    ```

  * **如果从源码编译 Fast DDS (默认中间件)：** 编译时需要一个特殊的 CMake 标志来启用安全插件。

    ```bash
    # 源码 (命令) - 编译时开启 SECURITY 标志
    colcon build --symlink-install --cmake-args -DSECURITY=ON \
        --packages-select fastrtps rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp rmw_fastrtps_shared_cpp
    ```

-----

### 3\. 运行演示 (Run the demo)

这里是教程的核心步骤，我们将一步步创建一个安全的通信环境。

#### 第1步：创建安全文件目录

你需要一个文件夹来存放所有的密钥、证书和配置文件。

```bash
# 源码 (命令) - Linux/MacOS
mkdir ~/sros2_demo
cd ~/sros2_demo
```

(Windows 用户使用 `md C:\dev\ros2\sros2_demo` 并 `cd` 进去)

#### 第2步：生成密钥库 (Keystore)

"Keystore"（密钥库）是一个根目录，它包含管理安全网络所需的所有公共（public）和私有（private）密钥和证书。

```bash
# 源码 (命令)
# 语法: ros2 security create_keystore <目录名>
ros2 security create_keystore demo_keystore
```

执行后，会在 `~/sros2_demo/` 下创建一个名为 `demo_keystore` 的目录。

#### 第3步：生成密钥和证书 (Enclave)

"Enclave"（安全飞地）是为**特定 ROS 2 节点**生成的一组唯一的安全文件。

我们需要为 `talker` 和 `listener` 两个节点分别创建 enclaves。

```bash
# 源码 (命令)
# 语法: ros2 security create_enclave <密钥库目录> <enclave的路径/名称>

# 为 talker 创建
ros2 security create_enclave demo_keystore /talker_listener/talker

# 为 listener 创建
ros2 security create_enclave demo_keystore /talker_listener/listener
```

  * `demo_keystore`: 指定使用哪个密钥库（在第2步创建的）。
  * `/talker_listener/talker`: 这是一个逻辑路径（也叫“身份”，Identity）。运行节点时，我们需要告诉节点它应该使用这个身份。
  * 执行后，`demo_keystore` 目录下会生成一个 `enclaves` 子目录，里面包含了 `/talker_listener/talker` 和 `/talker_listener/listener` 各自的安全配置文件。

#### 第4步：配置环境变量

为了让 ROS 2 节点找到并使用这些安全文件，必须设置三个环境变量。

```bash
# 源码 (命令) - Linux/MacOS

# 1. 告诉 ROS 2 密钥库在哪里
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore

# 2. 启用安全功能
export ROS_SECURITY_ENABLE=true

# 3. 设置安全策略为 "Enforce"
# Enforce: 强制安全。如果节点没有配置安全，它将无法启动或通信。
# (另一个选项是 "Permissive": 允许，但不强制)
export ROS_SECURITY_STRATEGY=Enforce
```

**重要：** 这三个环境变量必须在**每一个**你希望运行安全节点的终端中设置。

#### 第5步：运行安全的 talker/listener

现在我们来启动节点。

**终端 1 (运行 Talker):**

```bash
# 源码 (命令) - 终端 1
# 确保已设置上述三个环境变量 (ROS_SECURITY_...)

# 运行 talker，并使用 --enclave 标志指定它的“身份”
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

**终端 2 (运行 Listener):**

```bash
# 源码 (命令) - 终端 2
# 确保也设置了上述三个环境变量

# 运行 listener，并指定它对应的“身份”
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```

  * `--ros-args --enclave /talker_listener/talker`：这个参数告诉 `talker` 节点去 `ROS_SECURITY_KEYSTORE` 路径下查找 `/talker_listener/talker` 这个 enclave，并加载其安全配置。
  * 此时，`talker` 和 `listener` 正在进行**身份验证**和**加密通信**。如果你使用 Wireshark 等工具抓包，会发现 `chatter` 话题的内容是加密的。

#### 第6步：在安全环境中使用 ros2 命令行

`ros2` 命令行工具（如 `ros2 node list`）本身也是 ROS 2 节点。如果网络是安全的，它们也需要一个“身份”才能加入网络。

**终端 3 (运行 ros2 cli):**

```bash
# 源码 (命令) - 终端 3
# 1. 设置标准的环境变量
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# 2. (关键) 设置身份覆盖变量
# 我们让 ros2 cli "借用" listener 的身份和权限
export ROS_SECURITY_ENCLAVE_OVERRIDE=/talker_listener/listener

# 3. (可选但推荐) 使用 --no-daemon 
# 避免使用可能没有安全权限的后台守护进程
ros2 node list --no-daemon --spin-time 3

# 你应该能看到 /listener 和 /talker

ros2 topic list --no-daemon --spin-time 3
# 你应该能看到 /chatter, /parameter_events, /rosout
```

-----

### 4\. 教程小测验 (Take the Quiz\!)

教程最后提供了几个思考题，这有助于你理解 `ROS_SECURITY_ENABLE` 和 `ROS_SECURITY_STRATEGY` 的作用：

  * **场景1：** 你在一个**没有**设置任何安全环境变量的终端里运行 `listener`。

      * **结果：** `listener` 节点启动了，但**收不到任何消息**。因为它没有启用安全，无法解密 `talker` 发来的加密数据。

  * **场景2：** 你只设置了 `ROS_SECURITY_ENABLE=true`，但**没有**设置 `ROS_SECURITY_KEYSTORE` 和 `ROS_SECURITY_STRATEGY`。

      * **结果：** `listener` 启动了（因为策略默认为 `Permissive`，允许启动），但也**收不到消息**。它虽然启用了安全，但找不到密钥文件，无法解密。

  * **场景3：** 你设置了 `ROS_SECURITY_ENABLE=true` 和 `ROS_SECURITY_STRATEGY=Enforce`，但**没有**设置 `ROS_SECURITY_KEYSTORE`。

      * **结果：** `listener` **启动失败**。因为策略是 `Enforce`（强制），它要求必须配置好安全。当它找不到 `KEYSTORE` 时，ROS 2 会抛出错误并阻止节点启动。