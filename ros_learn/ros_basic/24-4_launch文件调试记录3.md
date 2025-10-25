完美 ✅，你要的目标其实是——

> 在 **VSCode** 里点击 “▶ 启动调试” 按钮，
> 自动运行 `ros2 launch pure_pursuit sim_pure_pursuit_launch.py`，
> 并让 VSCode 的 **gdb 调试器** 附加到 `pure_pursuit` 节点（带符号、能断点）。

---

我们来一步步配置出一个 **VSCode + ROS2 Launch 联动调试方案**。
最终你只要点一下 F5，就能进入到节点源码里单步调试。

---

## 🧩 一、问题背景

你的原始 VSCode `launch.json` 是直接运行：

```json
"program": "/sim_ws/install/pure_pursuit/lib/pure_pursuit/pure_pursuit"
```

这样能调试单个可执行文件 ✅，
但不会加载 launch 文件里的：

* 参数文件；
* 话题 remap；
* 其他节点（如 `waypoint_visualizer`）。

所以我们要让它：

> 启动 launch 文件 → 自动 attach 到其中的纯 Pursuit 节点。

---

## 🚀 二、推荐方案（最稳定）

我们用 **“复合调试 (compound)”** 机制：

1️⃣ **第一个配置**：运行 `ros2 launch`
2️⃣ **第二个配置**：GDB 附加到 `pure_pursuit` 进程

---

### 🔧 `.vscode/launch.json`

请改成下面的完整版本👇：

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "ROS2 Launch - pure_pursuit",
      "type": "cppdbg",
      "request": "launch",
      "program": "/usr/bin/bash",
      "args": [
        "-c",
        "source /opt/ros/foxy/setup.bash && source /sim_ws/install/setup.bash && ros2 launch pure_pursuit sim_pure_pursuit_launch.py"
      ],
      "cwd": "${workspaceFolder}",
      "MIMode": "gdb",
      "stopAtEntry": false,
      "externalConsole": true,
      "setupCommands": [
        {
          "description": "Enable pretty-printing for gdb",
          "text": "-enable-pretty-printing",
          "ignoreFailures": true
        }
      ]
    },
    {
      "name": "Attach to pure_pursuit node",
      "type": "cppdbg",
      "request": "attach",
      "program": "/sim_ws/install/pure_pursuit/lib/pure_pursuit/pure_pursuit",
      "processId": "${command:pickProcess}",
      "MIMode": "gdb",
      "setupCommands": [
        {
          "description": "Enable pretty-printing for gdb",
          "text": "-enable-pretty-printing",
          "ignoreFailures": true
        }
      ],
      "cwd": "${workspaceFolder}"
    }
  ],

  "compounds": [
    {
      "name": "Launch and Debug pure_pursuit (ROS2 Launch)",
      "configurations": [
        "ROS2 Launch - pure_pursuit",
        "Attach to pure_pursuit node"
      ],
      "stopAll": true
    }
  ]
}
```

---

## 🧠 三、调试步骤

1️⃣ 启动 VSCode
2️⃣ 打开左侧调试栏（Ctrl + Shift + D）
3️⃣ 在顶部选择：

```
Launch and Debug pure_pursuit (ROS2 Launch)
```

4️⃣ 按 F5 或点击 ▶️

VSCode 会：

* 启动 `ros2 launch pure_pursuit sim_pure_pursuit_launch.py`
* 启动后弹出选择进程窗口
* 你选中那个 `/sim_ws/install/pure_pursuit/lib/pure_pursuit/pure_pursuit`
* VSCode 的 GDB 自动 attach

此时你在源码中设置的断点会变红 ✅，
进入节点逻辑后会直接停下来。

---

## ⚙️ 四、优化建议

### 1️⃣ 自动选中进程

你可以把 `processId` 改成自动匹配纯追踪节点：

```json
"processId": "${command:pickProcess}"
```

改为：

```json
"processId": "${command:pickProcess|pure_pursuit}"
```

如果你用的是新版 VSCode，可直接搜索 `pure_pursuit`。

### 2️⃣ 无图形环境（Docker）

确保容器是：

```bash
--cap-add=SYS_PTRACE --security-opt seccomp=unconfined
```

否则 GDB attach 会失败。

---

## 💡 五、可选：延时 attach（自动化）

如果 `pure_pursuit` 启动稍慢，
可以用一个延迟版本：

```json
"processId": "`${command:pickProcess}`",
"preLaunchTask": {
    "label": "delay-attach",
    "type": "shell",
    "command": "sleep 3"
}
```

---

## ✅ 六、最终效果

* 点击 ▶️
* Launch 文件启动所有 ROS2 节点；
* VSCode 弹出进程列表；
* 选择 `pure_pursuit`；
* VSCode 停在断点处。

---

是否希望我帮你写成「**全自动 attach（无需手动选进程）**」的版本？
它会自动检测 `pure_pursuit` 进程并 attach，无需你点任何东西。
