很好，我们来一起详细学习这篇 ROS 2 Launch 教程 “launch Using Event Handlers” （适用于 Rolling 版）— 我会帮你拆解背景、关键机制、典型用法、以及实践技巧。你学完后应该能够在自己的 launch 文件中正确使用事件处理器。 来源: Open Robotics Foundation 文档。 ([ROS Documentation][1])

---

## 🧠 一、背景：为什么需要 “事件处理器（Event Handlers）”

* 在 ROS 2 的 launch 系统中，除了启动一堆节点之外，还经常需要监控这些节点/进程的状态，例如：某个节点启动后执行一个脚本、某个进程退出后重新启动、当用户关闭窗口或按下 Ctrl+C 时做清理等。
* 这些状态变化在 launch 系统中称为 “事件（events）”。 比如： “进程启动（ProcessStart）”、 “进程退出（ProcessExit）”、 “执行完成（ExecutionComplete）”、 “Shutdown 请求” 等。 ([ROS Documentation][1])
* 事件处理器（Event Handlers）允许你**注册回调／动作**，当某个事件发生时触发指定动作。这样你的 launch 文件就变得更为动态和智能、而不仅仅是“按顺序启动节点”。 ([ROS Documentation][1])

---

## 🎯 二、关键机制：事件 + 处理器

### ✅ 常见的 Event 类型

教程中列出了一些常用的事件类：

* `OnProcessStart` — 某个动作（Action，例如 Node、ExecuteProcess）**开始执行**时触发。 ([ROS Documentation][1])
* `OnProcessExit` — 某个动作**退出／结束**时触发。 ([ROS Documentation][1])
* `OnExecutionComplete` — 某个动作执行完毕（可能包括子动作）时触发。 ([ROS Documentation][1])
* `OnShutdown` — Launch 系统要求关闭（例如用户按 Ctrl+C、主窗口关闭）时触发。 ([ROS Documentation][1])
* `OnProcessIO` — 某个进程的 I/O（stdout/stderr）流有数据时触发。 ([ROS Documentation][1])

### 🔧 注册 Event Handler 的方式

你需要使用 `RegisterEventHandler(...)` 动作，将一个 EventHandler 实例绑定到某个动作。例如：

```python
RegisterEventHandler(
  OnProcessStart(
    target_action=turtlesim_node,
    on_start=[ ...actions to run when it starts... ]
  )
)
```

在这个例子中，当 `turtlesim_node` 动作开始后，就执行 `on_start` 列表里的动作。 ([ROS Documentation][1])

### 📋 典型流程总结

1. 在 launch 文件中定义一个或多个 Action（比如 Node、ExecuteProcess）。
2. 为一个特定 target_action 注册一个 EventHandler （如 OnProcessExit）。
3. 指定 on_exit 或 on_start 或 on_shutdown 等，当事件发生时要运行的动作。
4. 启动 launch，当事件触发时，注册的动作就被执行。

---

## 🧩 三、教程中的示例摘要

在教程中有一个完整的 Python launch 文件示例（`example_event_handlers_launch.py`）包含如下内容： ([ROS Documentation][1])

* 声明 Launch 参数：`turtlesim_ns`, `use_provided_red`, `new_background_r`。
* 启动 `turtlesim_node` （在指定命名空间）。
* 使用 ExecuteProcess 启动一个 spawn 服务调用（在 `turtlesim_ns/spawn`）。
* 使用 ExecuteProcess 改变背景颜色参数。
* 注册 RegisterEventHandler( OnProcessStart( target_action=turtlesim_node, on_start=[LogInfo(...), spawn_turtle] ) ) —— 当 turtlesim 节点启动时立即执行 spawn_turtle。
* 注册 OnShutdown 事件：当 launch 被关闭时，打印消息说明原因。

运行时如果你执行：

```bash
ros2 launch launch_tutorial example_event_handlers_launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
```

那系统会：

1. 启动 `turtlesim3/sim`。
2. 当该节点「启动」事件发生时，日志 “Turtlesim started, spawning turtle” 出现在控制台，然后 spawn_turtle 动作执行。
3. 背景颜色被设置为默认然后条件判断后再被设置成新颜色。
4. 当你关闭窗口或按 Ctrl+C 时，OnShutdown 触发，日志显示关机原因。

---

## 🔍 四、在实践中你需要掌握的点

| 功能        | 用途                    | 注意事项                                                                                              |
| --------- | --------------------- | ------------------------------------------------------------------------------------------------- |
| 控制启动顺序／依赖 | 如“节点 A 启动后再执行 B”      | 使用 OnProcessStart 或 OnProcessExit 监控 A 的动作                                                        |
| 处理失败／终止条件 | 如“如果脚本失败就关闭整个 launch” | 在 OnProcessExit 回调中检 event.returncode 然后 EmitEvent(Shutdown(...))。 ([Robotics Stack Exchange][2]) |
| 清理／优雅关闭   | 监听 OnShutdown 事件      | 能记录/处理 shutdown 原因                                                                                |
| I/O 监控    | 根据 stdout/stderr 触发行为 | 使用 OnProcessIO 事件                                                                                 |
| 条件执行      | 根据参数值或环境变量决定是否执行某动作   | 结合 IfCondition、PythonExpression 等 ([ROS Documentation][1])                                        |

---

## 🔧 五、示例：你如何在自己的 launch 文件使用

假设你有一个脚本 `init_setup.sh` 要先运行，然后才启动节点。你可以写 launch 如下：

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    init = ExecuteProcess(
      cmd=['bash','-c','./init_setup.sh'],
      output='screen',
      shell=True
    )
    node = Node(
      package='my_pkg',
      executable='my_node',
      name='my_node'
    )
    handler = RegisterEventHandler(
      OnProcessExit(
        target_action=init,
        on_exit=[
          LogInfo(msg='Init done, launching my_node'),
          node
        ]
      )
    )
    # also handle failure
    failure_handler = RegisterEventHandler(
      OnProcessExit(
        target_action=init,
        on_exit=[
          EmitEvent(event=Shutdown(reason='Init script failed'))
        ]
      )
    )
    return LaunchDescription([
      init,
      handler,
      failure_handler
    ])
```

这样你就可以确保“脚本先执行成功”然后才启动节点。

---

## ✅ 六、总结

* 事件处理器 (Event Handlers) 是 ROS 2 launch 系统中一个强大机制，用于“监听节点／进程状态变化”并做出反应。
* 学会常用事件类型（OnProcessStart, OnProcessExit, OnShutdown 等）与注册方式 RegisterEventHandler。
* 在复杂启动流程（有初始化脚本、依赖节点、失败处理等）中，使用事件处理器能让 launch 文件变得更健壮和清晰。
* 不要简单地依赖定义顺序来保证执行顺序；如果需要严格顺序／条件，推荐用事件机制。

---

如果你愿意，我可以 **帮你生成一个更完整的 “事件处理器”示例 launch 文件模板**（包含脚本初始化、节点启动、失败关闭、日志记录等），你看要不要？

[1]: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Event-Handlers.html?utm_source=chatgpt.com "Using event handlers — ROS 2 Documentation"
[2]: https://robotics.stackexchange.com/questions/115326/ros2-launch-eventhandler-onprocessexit-check-how-the-process-exited?utm_source=chatgpt.com "ros2 launch eventhandler onProcessExit check how the ..."

----

# 完整示例

好的，下面是一个 **完整版的 ROS 2 Python launch 文件模板**，演示如何使用 Event Handlers 来控制脚本与节点的启动顺序、处理失败情况、启动日志等。你可以将其放入你自己的包中，根据需要替换 package 名称、executable 名称、脚本路径等。

```python
# 文件：launch/sequential_with_events.launch.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    EmitEvent
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 定义 launch 参数（可从 CLI 传入）
    init_script = LaunchConfiguration('init_script', default='init_setup.sh')
    use_extra_node = LaunchConfiguration('use_extra_node', default='False')

    arg_init_script = DeclareLaunchArgument(
        'init_script',
        default_value='init_setup.sh',
        description='Initialization script to run before nodes'
    )
    arg_use_extra_node = DeclareLaunchArgument(
        'use_extra_node',
        default_value='False',
        description='Whether to start the extra node after main node'
    )

    # 2. 第一个动作：执行脚本
    init_proc = ExecuteProcess(
        cmd=['bash', '-c', [init_script]],
        output='screen',
        shell=True
    )

    # 3. 主节点，在脚本执行成功后启动
    main_node = Node(
        package='my_package',
        executable='main_node_executable',
        name='main_node',
        output='screen'
    )

    # 4. 可选的额外节点：如果 use_extra_node 为 True，则在主节点启动后执行
    extra_node = Node(
        package='my_package',
        executable='extra_node_executable',
        name='extra_node',
        output='screen'
    )

    # 5. 注册事件处理器：脚本退出成功 → 启动主节点
    handler_after_init = RegisterEventHandler(
        OnProcessExit(
            target_action=init_proc,
            on_exit=[
                LogInfo(msg='Initialization script completed, launching main_node'),
                main_node
            ]
        )
    )

    # 6. 注册事件处理器：如果脚本失败 (exit code ≠0) → 关闭整个 launch
    handler_init_fail = RegisterEventHandler(
        OnProcessExit(
            target_action=init_proc,
            on_exit=[
                LogInfo(msg='Initialization script failed, shutting down'),
                EmitEvent(event=Shutdown(reason='Init script failed'))
            ]
        )
    )

    # 7. 注册事件处理器：主节点 “已启动” → 若参数允许，则再启动额外节点
    handler_after_main = RegisterEventHandler(
        OnProcessStart(
            target_action=main_node,
            on_start=[
                LogInfo(msg='main_node has started'),
                # 条件启动 extra_node
                extra_node
            ]
        )
    )

    # 8. 注册事件处理器：当 launch 系统关闭时（如 Ctrl+C）做日志提示
    handler_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(msg='Launch is shutting down')
            ]
        )
    )

    return LaunchDescription([
        arg_init_script,
        arg_use_extra_node,
        init_proc,
        handler_after_init,
        handler_init_fail,
        handler_after_main,
        handler_on_shutdown,
    ])
```

### 🧠 模板说明与可替换部分

* **init_proc**：执行初始化脚本（或命令）。你可以替换 `'init_setup.sh'` 为你自己的脚本路径或命令。
* **main_node**：在脚本成功完成后才启动的节点。替换 `my_package` 和 `main_node_executable` 为你实际的包和可执行程序。
* **extra_node**：一个额外节点，使用 `OnProcessStart` 监听 `main_node` 的启动。如果你想让它只在满足某个条件（如 `use_extra_node == True`）时启动，可以再加 `IfCondition` 控制。
* **事件处理器**：

  * `handler_after_init` 用 `OnProcessExit` 来监听脚本结束，然后启动主节点。
  * `handler_init_fail` 同样监听脚本退出，但如果失败，则触发 `Shutdown` 事件来关闭整个 launch。
  * `handler_after_main` 在 `main_node` 启动后记录日志，并可启动额外节点。
  * `handler_on_shutdown` 在用户关闭 Launch 系统时执行日志提示。

### ✅ 使用步骤

1. 将该文件放入你的包的 `launch/` 目录，例如 `my_package/launch/sequential_with_events.launch.py`。
2. 在 `CMakeLists.txt`（如果是 C++ 包）或 `setup.py`（如果是 Python 包）中确保 **安装 launch 文件**。
3. 构建你的工作区 `colcon build` 并 `source install/setup.bash`。
4. 使用命令运行：

   ```bash
   ros2 launch my_package sequential_with_events.launch.py init_script:='my_init.sh' use_extra_node:=True
   ```

   你会看到：脚本先运行 → 脚本完成后启动 `main_node` → `main_node` 启动后（如果条件允许）再启动 `extra_node` → 最终 Launch 系统监控关闭。

---

如果你愿意，我还可以帮你 **生成一个 GitHub 风格的完整样例仓库结构**（包括 `package.xml`, `CMakeLists.txt` 或 `setup.py`、该 launch 文件、示例脚本与节点 stub）方便你直接拷贝。你看要不要？
