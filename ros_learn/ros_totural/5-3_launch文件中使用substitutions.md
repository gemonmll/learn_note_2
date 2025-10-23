> 注意学习<ros learn> 41 42 namespace理解 启动顺序理解

---

## 一、背景与目标

* **目标**：学习在 ROS 2 launch 文件中如何使用 *substitutions（替换／变量机制）*。 ([ROS Documentation][1])
* **背景**：当你编写 launch 文件（使用 Python／XML／YAML）时，经常需要在不同情境下复用相同文件、但配置略有变化。Substitutions 提供一种机制，让你在 launch 文件中使用变量、环境值、路径查找、逻辑判断等，从而让 launch 文件更加灵活、可复用。 ([ROS Documentation][1])
* **教程难度**：Intermediate（中级） ([ROS Documentation][1])
* **推荐时间**：约 15 分钟。 ([ROS Documentation][1])

---

## 二、关键概念：Substitutions 是什么？

在 ROS 2 的 launch 系统中，“substitution” 指的是在 launch 描述中（XML/YAML/Python）插入某些“变量”或“表达式”，这些会在实际执行 launch 时被**动态计算**或替换。 ([ROS Documentation][1])
这些变量／表达式可以包括：

* launch 配置（launch arguments）
* 环境变量
* 路径查找（例如某包的共享目录）
* Python 表达式（在条件逻辑中使用）

使用 substitutions 可以让你的 launch 文件更通用，不必为每一种情况都写一个完全不同的文件。

---

## 三、教程中展示的步骤和示例

教程分了几个步骤。下面我结合每一步简要说明，并注明你在实践时应注意的地方。

### 步骤 1：创建并设置包

你先创建一个名为 `launch_tutorial` 的包（可以用 ament_python 或 ament_cmake）并在其里创建 `launch/` 目录。 ([ROS Documentation][1])
然后确保 launch 文件被安装（对于 ament_cmake 包，在 `CMakeLists.txt` 中加入类似 `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)`）。 ([ROS Documentation][1])

> **注意**：如果你用的是 Python 包（ament_python），你需要在 `setup.py` 中通过 `data_files=[('share/<pkg_name>/launch', glob('launch/*'))]` 来安装 launch 文件。 ([ROS Documentation][1])

---

### 步骤 2：父 launch 文件（Parent launch file）

该文件用于 **包含（include）另一个 launch 文件**，并通过 substitutions／变量向它传递参数。 ([ROS Documentation][1])

以 XML 为例，示例中有：

```xml
<let name="background_r" value="200" />
<include file="$(find-pkg-share launch_tutorial)/launch/example_substitutions_launch.xml">
  <let name="turtlesim_ns" value="turtlesim2" />
  <let name="use_provided_red" value="True" />
  <let name="new_background_r" value="$(var background_r)" />
</include>
```

这里关键点：

* `$(find-pkg-share launch_tutorial)` 是一个 substitution，用于查找包 `launch_tutorial` 的 share 目录。
* `<let>` 标签将值赋给变量（launch 配置）
* `<include>` 将另一个 launch 文件纳入，并传递变量/值。

在 Python 版也有类似：使用 `FindPackageShare`, `PathJoinSubstitution`, `LaunchConfiguration` 等。 ([ROS Documentation][1])

> **注意**：通过这种方式，父 launch 文件可以很灵活地配置子 launch 文件的行为（例如命名空间、背景颜色、是否启用某功能等）。

---

### 步骤 3：Substitutions 示例的 launch 文件

这是被包含的子 launch 文件，展示了如何在 launch 描述中使用各种 substitution。 ([ROS Documentation][1])

例如，在 XML 示例中：

```xml
<arg name="turtlesim_ns" default="turtlesim1" />
<arg name="use_provided_red" default="False" />
<arg name="new_background_r" default="200" />

<node pkg="turtlesim" namespace="$(var turtlesim_ns)" exec="turtlesim_node" name="sim" />

<executable cmd="ros2 service call $(var turtlesim_ns)/spawn turtlesim_msgs/srv/Spawn '{x: 5, y: 2, theta: 0.2}'" />
...
<timer period="2.0">
  <executable cmd="ros2 param set $(var turtlesim_ns)/sim background_r $(var new_background_r)"
    if="$(eval '$(var new_background_r) == 200 and $(var use_provided_red)')" />
</timer>
```

几个重点 substitution 类型：

* `$(var turtlesim_ns)` —— 使用变量 `turtlesim_ns` 的值。
* `$(eval '...')` —— 使用 Python 表达式进行条件判断。
* `$(find-pkg-share <pkg>)` —— 查找包的安装路径。
* 在 Python 示例中，也有 `LaunchConfiguration('turtlesim_ns')`, `PythonExpression([...])` 等。

> **注意**：这种机制让你可以用同一份 launch 文件，根据传入的参数／变量，启动不同配置的系统。

---

### 步骤 4：构建 & 运行

构建包（`colcon build`），然后使用 `ros2 launch launch_tutorial example_main_launch.xml` 或 `.py` 或 `.yaml` 来运行。 ([ROS Documentation][1])
你还可以通过 `--show-args` 来查看该 launch 文件可接受的 launch-argument（例如 `turtlesim_ns`, `use_provided_red`, `new_background_r`）及默认值。 ([ROS Documentation][1])

> **注意**：当你从终端传入 launch 参数时语法为例如：
>
> ```bash
> ros2 launch launch_tutorial example_substitutions_launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200
> ```

---

## 四、你应掌握的使用场景和技巧

* 当你有多个类似的系统配置，仅差别在于命名空间、背景颜色、节点数目等，你可以用 substitutions 写一份通用的 launch 文件，而不是多个几乎相同的文件。
* 想要在 launch 文件内部使用路径查找（例如包资源、launch 文件被包含的位置）时，使用 `FindPackageShare`（Python）或 `$(find-pkg-share ...)`（XML/YAML）。
* 想要基于条件启动某些节点或进程时，可以用 `$(eval ...)` 或 `PythonExpression()`。
* 想要在主 launch 文件中向子 launch 文件传递变量／参数时，使用 `<let>`（XML）或用 `launch_arguments`（Python）。
* 当你发布自己的 ROS 2 包并希望别人通过 `ros2 launch your_pkg your_launch_file` 来运行时，别忘了在 package 里**安装 launch 文件**，并在 CMakeLists 或 setup.py 中做相应配置。

---

## 五、总结

这篇教程教给你的核心是：

> **Substitutions = 让 launch 文件更灵活、可复用、参数化**。
> 你现在懂得了它的几种典型用法、语法以及为什么要用它。

如果你愿意，我可以帮你 **写一个基于这个教程的“可参数化 Launch 文件模板”**，例如带命名空间、参数传入、条件逻辑、路径查找，能够适配你自己的机器人系统。要不要？

[1]: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-Substitutions.html "Using substitutions — ROS 2 Documentation: Rolling  documentation"


# python 版本
好的，下面是基于 ROS 2 教程 “Using substitutions in launch files” 的 **Python 版本 launch 文件示例**（摘自官方文档）并附上关键说明。 ([ROS Documentation][1])

---

### 📝 Python 版 launch 文件示例

**文件：`example_main_launch.py`**

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
# … 可能还需其他 import …

def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('launch_tutorial'),
                'launch',
                'example_substitutions.launch.py'
            ]),
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])
```

**文件：`example_substitutions_launch.py`**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # 定义可传入的 launch 参数
    turtlesim_ns = LaunchConfiguration('turtlesim_ns', default='turtlesim1')
    use_provided_red = LaunchConfiguration('use_provided_red', default='False')
    new_background_r = LaunchConfiguration('new_background_r', default='200')

    return LaunchDescription([
        DeclareLaunchArgument('turtlesim_ns', default_value='turtlesim1'),
        DeclareLaunchArgument('use_provided_red', default_value='False'),
        DeclareLaunchArgument('new_background_r', default_value='200'),

        Node(
            package='turtlesim',
            namespace=turtlesim_ns,
            executable='turtlesim_node',
            name='sim'
        ),

        # 例如执行 service 去 spawn 第二只 turtlesim 实例…
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                [turtlesim_ns, '/spawn'], 'turtlesim_msgs/srv/Spawn',
                '{x: 5, y: 2, theta: 0.2}'
            ],
            shell=True
        ),

        # 例如根据条件改变 background_r 参数
        TimerAction(
            period=2.0,
            actions=[ ExecuteProcess(
                condition=IfCondition(
                    PythonExpression([
                        new_background_r, ' == 200 and ', use_provided_red
                    ])
                ),
                cmd=[
                  'ros2', 'param', 'set',
                  [turtlesim_ns, '/sim', 'background_r'], new_background_r
                ],
                shell=True
            )]
        ),
    ])
```

---

### 🔍 关键说明

* `FindPackageShare('launch_tutorial')`：查找包 `launch_tutorial` 的 share 目录。 ([ROS Documentation][1])
* `PathJoinSubstitution([...])`：将多个路径片段组合成完整路径。
* `TextSubstitution(text=str(...))`：将字面量包为替换值，以便与其他 substitution 一起使用。
* `LaunchConfiguration('turtlesim_ns')`：表示可通过命令行或父 launch 文件传入的参数。
* `DeclareLaunchArgument(...)`：定义该参数、其默认值及说明。
* `PythonExpression([...])`：用于构造条件表达式，例如判断 `new_background_r == 200 and use_provided_red`。
* `IncludeLaunchDescription(...)`：在父 launch 文件中包含子 launch 文件，并传入 `launch_arguments`。
* 使用 `.items()` 是因为 `launch_arguments` 要求一个可迭代的键／值对结构。

---

### ✅ 为什么使用这种方式（Substitutions）？

* 让 launch 文件 **可复用／可配置**：不同参数可传入不同配置而无需改动源码。
* 简化路径查找逻辑：例如查找包目录、拼接路径、指定文件。
* 支持条件逻辑、环境变量、命令-行参数，从而增强启动系统的灵活性。
* 支持父子 launch 文件结构，便于大型系统拆分与组合。

---

如果你愿意，我可以 **为你生成一个完整的可运行示例包**（包括 `package.xml`、`CMakeLists.txt`／`setup.py`、上述两个 launch 文件），你可以直接放到你的 ROS2 工作区运行。你看要吗？

[1]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html?utm_source=chatgpt.com "Using substitutions — ROS 2 Documentation: Humble ..."
