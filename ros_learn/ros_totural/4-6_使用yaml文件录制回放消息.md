相比ros2 bag更轻量级的方法（echo topic方法）

好的，这篇教程是关于如何使用 YAML 文件来\*\*录制和重放（发布）\*\*ROS 2 消息的。

这篇文档是为你准备的学习摘要：

### 核心目标 🎯

解决一个痛点：在命令行上发布像 `geometry_msgs/msg/Twist` 这样的**复杂消息**非常麻烦（因为你必须手动输入冗长的结构）。

**解决方案**：

1.  用 `ros2 topic echo` 把一条消息**录制**成一个易于编辑的 `YAML` 文件。
2.  用 `ros2 topic pub` 从这个 `YAML` 文件\*\*重放（发布）\*\*这条消息。

-----

### 步骤 1：录制消息 (Echo) 📝

你可以使用 `ros2 topic echo` 命令，加上 `--once` 标志（表示只抓取一条消息），然后使用输出重定向 `>` 将其保存到文件中。

**示例命令：**
假设你想从 `/cmd_vel` 话题录制一条消息：

```bash
# 监听 /cmd_vel 话题，只抓取一条消息，并将其内容保存到 cmd_vel.yaml 文件中
ros2 topic echo --once /cmd_vel > cmd_vel.yaml
```

**生成的 `cmd_vel.yaml` 文件内容：**
你会得到一个像这样的文件，格式清晰，易于阅读和修改。

```yaml
linear:
   x: 1.0
   y: 0.0
   z: 0.0
angular:
   x: 0.0
   y: 0.0
   z: 0.0
---
```

**注意**：`---` 是 YAML 格式中的一个文档分隔符。

-----

### 步骤 2：重放消息 (Pub) 📤

现在，你可以使用 `ros2 topic pub` 命令，配合新的 `--yaml-file` 选项，从该文件发布消息。

**示例命令：**

```bash
# 发布到 /cmd_vel 话题，指定消息类型，并使用 --yaml-file 指向你的文件
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist --yaml-file cmd_vel.yaml
```

这个命令会读取 `cmd_vel.yaml` 的内容，将其作为一条消息发布出去，然后命令**自动退出**。

-----

### 高级技巧：一次发布多条消息

如果你想按顺序发布多条消息，只需编辑你的 `.yaml` 文件，使用 `---` 分隔符来堆叠多条消息即可。

**示例 `cmd_vel.yaml`（包含 3 条消息）：**

```yaml
linear:
   x: 1.0
   y: 0.0
   z: 0.0
angular:
   x: 0.0
   y: 0.0
   z: 0.0
---
linear:
   x: 2.0
   y: 0.0
   z: 0.0
angular:
   x: 0.0
   y: 0.0
   z: 0.0
---
linear:
   x: 3.0
   y: 0.0
   z: 0.0
angular:
   x: 0.0
   y: 0.0
   z: 0.0
---
```

当你运行**与之前完全相同**的 `pub` 命令时：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist --yaml-file cmd_vel.yaml
```

它会自动按顺序发布文件中的所有 3 条消息，然后退出。

### 总结

这是一种比 `ros2 bag` 更轻量级的方法，非常适合**调试或测试**时，需要快速、重复地发送特定复杂消息的场景。