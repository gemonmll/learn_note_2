好的，这篇教程深入讲解了 ROS 2 安全的另一个核心方面：**访问控制 (Access Controls)**。

之前的教程关注的是“认证”（你是谁？）和“加密”（通信内容保密），而这篇教程关注的是\*\*“授权”（你被允许做什么？）\*\*。

这篇文章的目标是：**通过编辑权限策略文件 (`permissions.xml`)，精确限制一个节点（talker）只能在特定的话题（`/chatter`）上发布消息，并阻止其在其他话题上发布。**

-----

### 1\. 背景 (Background)

  * 我们将使用在“设置安全”教程中创建的 `~/sros2_demo/demo_keystore`。
  * 核心机制：每个 Enclave（安全飞地）目录中都有一个 `permissions.xml` 文件（和它签名后的版本 `permissions.p7s`）。这个 XML 文件定义了该 Enclave（即使用该 Enclave 的节点）的权限。
  * 默认情况下，`sros2` 生成的权限是比较宽松的。
  * 在本教程中，我们将为 `talker` 节点创建一个**严格**的策略：只允许在 `/chatter` 话题上发布。
  * 我们将测试当 `ROS_SECURITY_STRATEGY`（安全策略）设置为 `Enforce`（强制）时，如果 `talker` 试图发布到*其他*话题（例如通过 `remap` 重映射），它将被系统阻止。

-----

### 2\. 任务1：修改 `permissions.xml`

我们将手动编辑 `talker` 节点的权限策略文件。

**a. 备份并打开文件**

```bash
# 源码 (命令) - 进入 talker 节点的 enclave 目录
cd ~/sros2_demo/demo_keystore/enclaves/talker_listener/talker

# 源码 (命令) - 备份当前的权限文件 ( .p7s 是已签名的, .xml 是源文件)
mv permissions.p7s permissions.p7s~
mv permissions.xml permissions.xml~

# 源码 (命令) - 创建一个新的 permissions.xml 文件
vi permissions.xml
```

**b. 粘贴新的权限内容**

将以下 XML 内容粘贴到新的 `permissions.xml` 文件中并保存。

```xml
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_permissions.xsd">
  <permissions>
    <grant name="/talker_listener/talker">
      <subject_name>CN=/talker_listener/talker</subject_name>
      <validity>
        <not_before>2021-06-01T16:57:53</not_before>
        <not_after>2031-05-31T16:57:53</not_after>
      </validity>

      <allow_rule>
        <domains>
          <id>0</id> </domains>
        
        <publish>
          <topics>
            <topic>rt/chatter</topic> 
            
            <topic>rt/rosout</topic>
            <topic>rt/parameter_events</topic>
            <topic>*/talker/*</topic> </topics>
        </publish>
        
        <subscribe>
          <topics>
            <topic>rt/parameter_events</topic>
            <topic>*/talker/*</topic>
          </topics>
        </subscribe>
      </allow_rule>
      
      <allow_rule>
        <domains>
          <id>0</id>
        </domains>
        <publish>
          <topics>
            <topic>ros_discovery_info</topic>
          </topics>
        </publish>
        <subscribe>
          <topics>
            <topic>ros_discovery_info</topic>
          </topics>
        </subscribe>
      </allow_rule>

      <default>DENY</default>
    </grant>
  </permissions>
</dds>
```

  * **精炼讲解：**
      * `rt/chatter`: 这是 ROS 话题 `/chatter` 在 DDS 层的实际名称（`rt` 代表 `ros topic`）。
      * `rt/rosout`, `rt/parameter_events`: 这是所有节点运行所必需的内务话题。
      * `<default>DENY</default>`: 这是关键。它规定，除了上面 `<allow_rule>` 明确列出的，其他一切行为（比如发布到 `rt/not_chatter`）都是被**禁止**的。

-----

### 3\. 任务2：重新签署策略文件

DDS 中间件不读取 `.xml` 文件，它只读取**签名后**的 `.p7s` 文件。我们必须使用 CA 的**私钥**来签署我们刚刚修改的 `permissions.xml`。

```bash
# 源码 (命令) - (确保你仍
# 在 ~/sros2_demo/demo_keystore/enclaves/talker_listener/talker 目录下)

# 使用 openssl smime 命令进行签名
openssl smime -sign -text \
  -in permissions.xml \
  -out permissions.p7s \
  --signer permissions_ca.cert.pem \
  -inkey ~/sros2_demo/demo_keystore/private/permissions_ca.key.pem
```

  * `-in permissions.xml`: 输入我们修改后的 XML。
  * `-out permissions.p7s`: 输出 DDS 要读取的签名文件。
  * `--signer permissions_ca.cert.pem`: 签名者是 CA 的公共证书。
  * `-inkey ~/sros2_demo/demo_keystore/private/permissions_ca.key.pem`: **(核心)** 使用 CA 的**私钥**来执行签名。

-----

### 4\. 任务3：启动并测试节点

现在我们来测试新的访问控制策略。

**a. 设置环境变量** (在所有终端中)

```bash
# 源码 (命令)
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce # 必须是 Enforce 才能阻止违规操作
```

**b. 启动 Listener** (在终端 1)
(Listener 的权限我们没改，它启动正常)

```bash
# 源码 (命令) - 终端 1
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```

**c. 启动 Talker (正常情况)** (在终端 2)
`talker` 默认发布到 `/chatter`，我们的策略允许 `rt/chatter`，所以它能成功启动和发布。

```bash
# 源码 (命令) - 终端 2
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

  * **结果：** `listener` 成功接收到 "Hello World" 消息。

**d. 启动 Talker (违规情况)** (在终端 2)
现在，停止 `talker` (按 `Ctrl+C`)。我们尝试使用 ROS 的 `remap` 功能，强迫 `talker` 将 `/chatter` 话题重命名为 `/not_chatter`。

```bash
# 源码 (命令) - 终端 2 (违规测试)
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker \
    --remap chatter:=not_chatter
```

  * **结果：** `talker` 节点**启动失败**（或立即崩溃）。
  * **分析：**
    1.  `talker` 节点尝试在 DDS 层发布到 `rt/not_chatter` 话题。
    2.  DDS 中间件加载了 `permissions.p7s` 文件。
    3.  它检查 `talker` 的 `<allow_rule>`，发现里面只有 `rt/chatter`，没有 `rt/not_chatter`。
    4.  默认策略是 `<default>DENY</default>`。
    5.  由于安全策略是 `Enforce`（强制），DDS 拒绝了 `talker` 的发布请求，导致节点启动失败。

-----

### 5\. (可选) 使用模板 (Use the templates)

手动编写 XML 很复杂。`sros2` 包提供了 `create_permission` 命令，可以使用模板来生成权限文件。

**a. 获取模板文件**
(模板文件在 `sros2` 的源码仓库中)

```bash
# 源码 (命令) - 克隆 sros2 仓库到 /tmp 目录
git clone https://github.com/ros2/sros2.git /tmp/sros2
```

**b. 使用命令生成权限**
这个命令会自动处理 XML 内容并**自动调用 `openssl` 签名**。

```bash
# 源码 (命令) - 为 talker 生成权限
ros2 security create_permission demo_keystore \
    /talker_listener/talker \
    /tmp/sros2/sros2/test/policies/sample.policy.xml

# 源码 (命令) - 为 listener 生成权限
ros2 security create_permission demo_keystore \
    /talker_listener/listener \
    /tmp/sros2/sros2/test/policies/sample.policy.xml
```

  * `demo_keystore`: 密钥库路径。
  * `/talker_listener/talker`: Enclave 路径。
  * `.../sample.policy.xml`: 使用的策略模板（这个模板只允许 `/chatter` 话题）。

这个命令会自动在 `.../talker` 和 `.../listener` 目录下生成正确的 `permissions.xml` 和 `permissions.p7s` 文件，效果与我们手动操作相同。