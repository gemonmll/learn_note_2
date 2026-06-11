非常好 👍，你现在已经有一个完整的 `launch.json`，
接下来我们要让它 **自动：**

1️⃣ 用 `ros2 launch` 启动 `pure_pursuit` 节点（带 gdb 参数并停在入口）
2️⃣ 然后用 VSCode attach 上去进行调试

---

## 🧩 完整方案分三步走

---

### **第一步：修改 ROS2 launch 文件**

把你的 `sim_pure_pursuit_launch.py` 改成下面这样👇：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'sim_config.yaml'
    )

    # gdb 启动 pure_pursuit
    pure_pursuit = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[config],
        # 启动 GDB 并停在程序入口
        prefix=['xterm', '-e', 'gdb', '--args']
    )

    waypoint_visualizer_node = Node(
        package='pure_pursuit',
        executable='waypoint_visualizer',
        name='waypoint_visualizer_node',
        parameters=[config]
    )

    ld = LaunchDescription()
    ld.add_action(pure_pursuit)
    ld.add_action(waypoint_visualizer_node)
    return ld
```

> 🚀 注意：
>
> 1. 这会在新终端里启动 `gdb`，你会看到 `(gdb)` 提示符。
> 2. 它不会立刻运行节点（你必须在 gdb 里输入 `run` 才启动）。

---

### **第二步：在 Docker 里允许 GDB attach**

确保你的容器是这样启动的（关键部分是 `--cap-add=SYS_PTRACE`）：

```bash
docker run -it --rm \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  -v /home/jzm/workspace/github_repo/f1tenth_ws/src:/sim_ws/src \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --name ros2_debug \
  ros:foxy
```

> ⚠️ 如果没加这两个参数，VSCode 不能 attach。
>
> 一旦容器已运行无法修改权限，必须重新 run。

---

### **第三步：VSCode 调试步骤**

你已经有这两个配置：

* **ROS2 Debug - via Launch File**（启动 gdb 模式）
* **Attach to pure_pursuit node**（附加到节点）

下面是使用顺序👇：

---

#### 🧭 Step 1：在 VSCode 打开你的容器项目

确保你在容器中运行 VSCode（推荐用 Remote - Containers）。
这样 VSCode 内部和 ROS2 环境是一致的。

---

#### 🧭 Step 2：启动 gdb 节点（停在入口）

在 VSCode 的终端执行：

```bash
ros2 launch pure_pursuit sim_pure_pursuit_launch.py
```

你会看到一个新的终端窗口（xterm）弹出，里面是：

```
GNU gdb (Ubuntu 9.2-0ubuntu1~20.04) ...
(gdb)
```

在这里 **暂时不要输入 run**，保持 gdb 停着。

---

#### 🧭 Step 3：在 VSCode Attach 调试

1. 切换到 VSCode 调试视图（Ctrl+Shift+D）
2. 在上方选择配置 `"Attach to pure_pursuit node"`
3. 按下绿色 ▶️（开始调试）
4. VSCode 会提示你选择进程，选中 `pure_pursuit`

   （在 Docker 内部通常能看到 `/sim_ws/install/pure_pursuit/lib/pure_pursuit/pure_pursuit`）

---

#### 🧭 Step 4：回到 gdb 窗口，执行 run

在弹出的 xterm 的 `(gdb)` 提示符下输入：

```
run
```

此时程序开始运行，但 VSCode 已经附加到它。
如果你在源码中设置了断点，程序会在对应位置停下来 🎯

---

### ✅ 调试完成后

1. 输入 `Ctrl+C` 停止节点
2. 在 xterm 的 gdb 里输入 `quit` 退出
3. VSCode 会自动断开 attach

---

## 🧠 提示：可选优化

如果你想让 gdb 自动停在 `main()` 而不是空着等待，可改成：

```python
prefix=['xterm', '-e', 'gdb', '-ex', 'break main', '-ex', 'run', '--args']
```

这样一启动就：

* 自动下断点到 main
* 自动执行 run
* 停在 main 等待你 attach VSCode

---

是否要我帮你补一个完整的 `.vscode/tasks.json`，自动执行 `source_ros2_env` 与构建任务？
那样按 F5 就能一键构建 + 调试整个节点。
