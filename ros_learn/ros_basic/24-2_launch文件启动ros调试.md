非常好，这个错误图说明问题很清楚：

```
GDB failed with message: "/opt/ros/foxy/bin/ros2": not in executable format: file format not recognized
```

---

### 💡 原因分析

`/opt/ros/foxy/bin/ros2` 这个文件并不是一个 ELF 可执行二进制文件，而是一个 **Python 脚本**。
你可以验证这一点：

```bash
head -n 1 /opt/ros/foxy/bin/ros2
```

输出类似：

```bash
#!/usr/bin/env python3
```

所以 GDB 报错的原因是：

> GDB 只能调试原生 ELF 可执行文件（例如 C/C++ 编译出的二进制），不能直接调试 Python 入口脚本。

换句话说：

* `ros2 launch` 是 Python 代码；
* GDB 不认识 Python 文件格式；
* 所以 “file format not recognized”。

---

### ⚙️ 解决方法（推荐的几种）

#### ✅ 方法 1：使用 `cppdbg` 的 **中间层 wrapper**

你可以让 `program` 指向 `/usr/bin/env`，并在 `args` 中调用 `ros2 launch ...`：

```jsonc
{
  "name": "ROS2 Debug - via Launch File",
  "type": "cppdbg",
  "request": "launch",
  "program": "/usr/bin/env",
  "args": [
    "bash", "-c",
    "source /opt/ros/foxy/setup.bash && source /sim_ws/install/setup.bash && ros2 launch pure_pursuit sim_pure_pursuit_launch.py"
  ],
  "stopAtEntry": false,
  "cwd": "${workspaceFolder}",
  "MIMode": "gdb",
  "externalConsole": true
}
```

这样：

* 实际执行的是 bash 命令；
* bash 会加载 ROS 环境，然后运行 `ros2 launch ...`；
* 这样不会触发 GDB 对 Python 文件的错误。

不过这种方式 **GDB 仍然不会 attach 到节点上**，它只是运行整个系统。

👉 若你希望真正断点调试 C++ 节点，需要结合下一种。

---

#### ✅ 方法 2：在 launch 文件中让目标节点以 GDB 模式运行

修改你的 `sim_pure_pursuit_launch.py`：

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    debug = os.getenv('DEBUG_PURE_PURSUIT', '0') == '1'

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        prefix=['gdb', '-ex', 'run', '--args'] if debug else [],
    )

    return LaunchDescription([pure_pursuit_node])
```

然后：

```bash
DEBUG_PURE_PURSUIT=1 ros2 launch pure_pursuit sim_pure_pursuit_launch.py
```

这时节点会自动进入 GDB 调试。

如果你想从 VS Code 一键执行：

```jsonc
{
  "name": "ROS2 Debug Pure Pursuit",
  "type": "cppdbg",
  "request": "launch",
  "program": "/usr/bin/env",
  "args": [
    "bash", "-c",
    "source /opt/ros/foxy/setup.bash && source /sim_ws/install/setup.bash && DEBUG_PURE_PURSUIT=1 ros2 launch pure_pursuit sim_pure_pursuit_launch.py"
  ],
  "MIMode": "gdb",
  "externalConsole": true
}
```

这样：

* ROS2 launch 文件正常运行；
* `pure_pursuit` 节点在 GDB 中；
* 你可以在 GDB 控制台下输入 `b`、`run`、`next` 等命令调试。

---

#### ✅ 方法 3：先运行 launch，再 attach（最稳定）

1. 正常启动：

   ```bash
   ros2 launch pure_pursuit sim_pure_pursuit_launch.py
   ```
2. 查找目标节点 PID：

   ```bash
   ps -ef | grep pure_pursuit
   ```
3. 在 VS Code 中配置 attach：

   ```jsonc
   {
     "name": "Attach to running pure_pursuit",
     "type": "cppdbg",
     "request": "attach",
     "processId": "${command:pickProcess}",
     "MIMode": "gdb"
   }
   ```

这种方式最常用，也最稳定，因为不会因为 ROS 启动器是 Python 而出错。

---

### ✅ 推荐组合

| 需求                               | 推荐方案 |
| -------------------------------- | ---- |
| 想一次性跑整个 launch + GDB attach 某个节点 | 方法 2 |
| 想先跑 launch 再 attach 节点           | 方法 3 |
| 只想运行 launch，不调试                  | 方法 1 |

---

是否希望我帮你基于你现有的 `sim_pure_pursuit_launch.py`（如果你贴出它的内容）改成「支持调试开关」版本？
这样你可以用环境变量控制是否进入 GDB，非常方便。
