> 注意学习<ros learn>下的 43 rviz下的替换机制

好的，我们来一起学习这篇 ROS 2 Launch 教程 **“Managing large projects”**（或“Using ROS2 Launch for large projects”）([ROS Documentation][1])
我会帮你梳理文章结构、关键概念、示例用法以及实践中可应用的技巧。

---

## 🧩 一、背景 & 目的

* 教程名称：*Managing large projects*（大型工程中使用 Launch 文件的最佳实践）([ROS Documentation][1])
* **目标**：掌握如何在较大的 ROS2 系统中（多个节点、多个配置、不同子系统）**组织、复用、参数化** Launch 文件，使其灵活、可维护。 ([ROS Documentation][1])
* **适用场景**：当一个机器人系统包含多个模块（例如感知、控制、仿真、多机器人、TF 广播/监听、RViz 可视化等）时。教程中以多个 turtlesim 实例 + TF 广播器 + RViz 为例。 ([ROS Documentation][1])

---
好的，我们来详细学习这篇教程的每一个部分，并附上相应的示例代码。

这篇教程的目标是教你如何使用 ROS 2 的 `launch` 系统来管理大型项目，核心思想是**模块化**和**可重用性**。

-----

### 1\. 顶层组织 (Top-level organization)

**概念：**
不要把所有的节点都放在一个巨大的 `launch` 文件中。相反，应该创建多个功能单一、职责分明的 `launch` 文件（例如，一个用于启动模拟器，一个用于启动 TF 广播，一个用于启动 RViz）。然后，创建一个“顶层”的 `launch` 文件，使用 `IncludeLaunchDescription` 动作将这些子 `launch` 文件“包含”进来，组合成一个完整的系统。

**示例 (`launch_turtlesim_launch.py`):**
这是顶层文件，它不直接启动任何节点，而是“包含”了其他几个 `launch` 文件。

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 找到 launch_tutorial 包的 share 目录下的 launch 文件夹
    launch_dir = PathJoinSubstitution([
        FindPackageShare('launch_tutorial'),
        'launch'
    ])

    return LaunchDescription([
        # 包含第一个 turtlesim 世界
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'turtlesim_world_1_launch.py'])
        ),
        # 包含第二个 turtlesim 世界
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'turtlesim_world_2_launch.py'])
        ),
        # 包含 TF 广播和监听
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'broadcaster_listener_launch.py']),
            # 传入参数，覆盖子文件中的默认值
            launch_arguments={'target_frame': 'carrot1'}.items()
        ),
        # 包含 mimic 节点
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'mimic_launch.py'])
        ),
        # 包含固定坐标系广播
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'fixed_broadcaster_launch.py'])
        ),
        # 包含 RViz
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'turtlesim_rviz_launch.py'])
        ),
    ])
```

-----

### 2\. 参数 (Parameters)

#### 2.1 在 Launch 文件中设置参数

**概念：**
可以直接在 `launch` 文件中为节点定义和设置参数。这适用于参数较少或希望在 `launch` 文件中直接暴露的参数。

  - `DeclareLaunchArgument`: 声明一个启动参数，可以设置默认值。
  - `LaunchConfiguration`: 获取 `DeclareLaunchArgument` 声明的参数的值。

**示例 (`turtlesim_world_1_launch.py`):**
这个文件启动一个 `turtlesim` 节点，并将其背景色参数（R, G, B）通过 `DeclareLaunchArgument` 暴露出来。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明三个启动参数，并设置默认值
        DeclareLaunchArgument('background_r', default_value='0'),
        DeclareLaunchArgument('background_g', default_value='84'),
        DeclareLaunchArgument('background_b', default_value='122'),

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            parameters=[{
                # 使用 LaunchConfiguration 来获取参数值
                'background_r': LaunchConfiguration('background_r'),
                'background_g': LaunchConfiguration('background_g'),
                'background_b': LaunchConfiguration('background_b'),
            }]
        ),
    ])
```

#### 2.2 从 YAML 文件加载参数

**概念：**
当参数非常多时，将它们全部写在 `launch` 文件中会很混乱。更好的方法是将参数保存在一个 `.yaml` 配置文件中，然后在 `launch` 文件中加载这个文件。

**示例 (YAML 文件):**
首先，创建一个 `config/turtlesim.yaml` 文件：

```yaml
# 注意：这里的 /turtlesim2/sim 是节点的完整名称（包含命名空间）
/turtlesim2/sim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
```

**示例 (`turtlesim_world_2_launch.py`):**
这个 `launch` 文件启动了第二个 `turtlesim` 节点，并从 `turtlesim.yaml` 加载参数。

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 构造 YAML 文件的完整路径
    config_file = PathJoinSubstitution([
        FindPackageShare('launch_tutorial'),
        'config',
        'turtlesim.yaml'
    ])

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='turtlesim2',  # 注意这里指定了命名空间
            name='sim',
            parameters=[config_file] # 直接将 YAML 文件路径作为参数
        ),
    ])
```

#### 2.3 在 YAML 中使用通配符

**概念：**
如果你想为多个节点设置相同的参数，但又不想在 YAML 中为每个节点（可能在不同的命名空间下）都重复写一遍，你可以使用通配符 `/**`。

**示例 (修改 `turtlesim.yaml`):**
将 `config/turtlesim.yaml` 修改为：

```yaml
/**:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
```

这样，任何通过 `parameters=[config_file]` 加载这个 `yaml` 文件的 `Node`，都会被设置这些参数，无论它的命名空间和节点名称是什么。

-----

### 3\. 命名空间 (Namespaces)

**概念：**
命名空间（`namespace`）是 ROS 2 中用于组织节点和话题的一种方式，可以避免名称冲突。在 `Node` 定义中设置 `namespace` 参数，该节点的所有话题、服务等都会被放到这个命名空间下。

**示例 (来自 `turtlesim_world_2_launch.py`):**
在 2.2 的示例中，我们为第二个 `turtlesim` 节点设置了命名空间：

```python
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='turtlesim2', # 设置命名空间
            name='sim',
            parameters=[config_file]
        ),
```

这个节点启动后，它的节点全名将是 `/turtlesim2/sim`，它的话题将是 `/turtlesim2/turtle1/pose` 等，而不是默认的 `/sim` 和 `/turtle1/pose`。

-----

### 4\. 复用节点 (Reusing nodes)

**概念：**
你可以在同一个 `launch` 文件中多次启动同一个可执行文件（`executable`），只要给它们指定不同的 `name` 即可。

**示例 (`broadcaster_listener_launch.py`):**
这个文件启动了两个 `turtle_tf2_broadcaster` 节点，分别广播 `turtle1` 和 `turtle2` 的 TF 坐标变换。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        # 启动第一个广播器
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1', # 节点名 1
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        # 启动第二个广播器 (复用)
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2', # 节点名 2
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        # 启动监听器
        Node(
            package='turtle_tf2_py',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')}
            ]
        ),
    ])
```

-----

### 5\. 参数覆盖 (Parameter overrides)

**概念：**
在顶层 `launch` 文件中“包含”子 `launch` 文件时（使用 `IncludeLaunchDescription`），可以覆盖子 `launch` 文件中 `DeclareLaunchArgument` 声明的参数的默认值。

**示例 (来自 `launch_turtlesim_launch.py`):**
`broadcaster_listener_launch.py` (见上文) 声明了一个 `target_frame` 参数，默认值是 `turtle1`。
在顶层 `launch` 文件中，我们包含它时，通过 `launch_arguments` 将其覆盖为 `carrot1`。

```python
        # ...
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'broadcaster_listener_launch.py']),
            # 覆盖子 launch 文件中的 'target_frame' 参数
            launch_arguments={'target_frame': 'carrot1'}.items()
        ),
        # ...
```

这样，`broadcaster_listener_launch.py` 中启动的 `listener` 节点实际使用的 `target_frame` 将是 `carrot1` 而不是 `turtle1`。

-----

### 6\. 重映射 (Remapping)

**概念：**
`remapping` 允许你更改节点订阅或发布的话题名称。这在你想将两个原本不兼容的节点连接在一起时非常有用。

**示例 (`mimic_launch.py`):**
`mimic` 节点默认订阅 `/input/pose` 话题，并发布到 `/output/cmd_vel` 话题。我们希望它跟随 `/turtle2/pose`，并控制 `/turtlesim2/turtle1/cmd_vel`。

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                # 将 /input/pose 重映射为 /turtle2/pose
                ('/input/pose', '/turtle2/pose'),
                # 将 /output/cmd_vel 重映射为 /turtlesim2/turtle1/cmd_vel
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

-----

### 7\. 配置文件 (Config files)

**概念：**
对于 RViz、`robot_state_publisher` 等节点，我们通常不是通过 ROS 参数来配置它们，而是通过专门的配置文件（如 `.rviz` 文件或 `URDF` 文件）。我们可以使用 `Node` 的 `arguments` 参数将这些配置文件作为命令行参数传递给节点。

**示例 (`turtlesim_rviz_launch.py`):**
这个文件启动 RViz，并使用 `-d` 参数加载一个特定的 `.rviz` 配置文件。

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 找到 turtle_tf2_py 包中的 rviz 配置文件
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('turtle_tf2_py'),
        'rviz',
        'turtle_rviz.rviz'
    ])

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # 将 '-d' 和 rviz 配置文件路径作为命令行参数传递
            arguments=['-d', rviz_config_file]
        ),
    ])
```

-----

### 8\. 环境变量 (Environment Variables)

**概念：**
`launch` 文件可以读取系统的环境变量，这对于区分不同机器人或不同用户的配置很有用。

**示例 (`fixed_broadcaster_launch.py`):**
这个文件启动一个 TF 广播器，并使用 `USER` 环境变量作为其节点名称的前缀。

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 声明一个 node_prefix 参数，默认值是 "USER" 环境变量 + "_"
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='prefix for node name'
        ),
        Node(
            package='turtle_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            # 节点名将是 'username_fixed_broadcaster'
            name=[LaunchConfiguration('node_prefix'), 'fixed_broadcaster'],
        ),
    ])
```

-----

### 运行前的准备：更新 setup.py

**概念：**
`launch` 文件（`.py`）和配置文件（`.yaml`, `.rviz`）默认不会被 `colcon build` 安装。你必须修改包的 `setup.py` 文件，告诉构建系统将这些文件安装到 `share` 目录中。

**示例 (`setup.py`):**
你需要添加 `import os` 和 `from glob import glob`，并修改 `data_files` 字段：

```python
import os
from glob import glob
from setuptools import setup

package_name = 'launch_tutorial'

setup(
    # ... 其他 setup 配置 ...
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加下面这两行
        # 安装 launch/ 目录下的所有文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # 安装 config/ 目录下的所有 .yaml 文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    # ...
)
```

完成这些步骤并构建 (`colcon build`) 后，你就可以通过 `ros2 launch launch_tutorial launch_turtlesim_launch.py` 来运行你的顶层 `launch` 文件了。


### 运行 Launch 文件

* 对于 Python 包：在 `setup.py` 使用 `data_files=[('share/<pkg>/launch', glob('launch/*.launch.py'))]` 安装。
* 对于 C++ 包：在 CMakeLists 中使用 `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)`。 ([ROS Documentation][1])
* 然后使用 `ros2 launch <package> <launch_file>` 执行。

---

## ✅ 四、实践应用建议

* **模块化**：将系统拆分为子 Launch 文件，每个负责一个子系统或功能。顶层文件仅负责“组合”。
* **参数与命名空间化**：通过参数化 namespace、config 文件、参数值，提高可复用性。
* **资源分离**：将配置（YAML）、RViz、TF、参数放在资源目录，通过 substitutions 加载。
* **环境切换**：通过 launch arguments +环境变量控制不同运行场景（仿真、真实、多机器人）。
* **版本化 & 安装**：确保你的 package 安装了 launch 文件（share 目录），便于他人使用。

---

## 📌 总结

这篇教程告诉我们，在大型 ROS2 项目中，**良好的 Launch 文件架构**尤为重要。
不是“写一个巨大的 launch 文件”，而是“拆分 + 参数化 +资源化 +复用”。
掌握这些技巧，你的系统就更易于维护、扩展与复用。

---

如果你愿意，我可以帮你 **写一个基于该教程的“多模块 Launch 文件结构示例”**（例如仿真机器人场景：感知模块 + 控制模块 +可视化模块，各个用子 Launch 文件），你看要不要？

[1]: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html?utm_source=chatgpt.com "Managing large projects — ROS 2 Documentation"


好的，下面是一个基于 ROS 2 Launch 教程 “Using ROS2 Launch for Large Projects” 的 **多模块 launch 文件结构示例**。你可以根据自己的项目（多个子系统、仿真 vs 真实、多机器人等）改动。
你也可以直接把它作为「项目骨架」来使用。

---

## 目录结构（假设包名为 `my_robot_system`）

```
my_robot_system/
├── launch/
│   ├── top_level_launch.py
│   ├── perception_launch.py
│   ├── control_launch.py
│   ├── simulation_launch.py
│   ├── rviz_launch.py
│   └── config/
│       ├── params_robot1.yaml
│       ├── params_robot2.yaml
│       └── rviz_config.rviz
├── package.xml
├── CMakeLists.txt (或 setup.py，如果为 Python 包)
└── src/
    └── …（节点代码）
```

---

## Launch 文件示例

### `launch/top_level_launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_dir = PathJoinSubstitution(
        [FindPackageShare('my_robot_system'), 'launch']
    )
    return LaunchDescription([
        # 启动仿真环境（若使用仿真）
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'simulation_launch.py']),
            launch_arguments={'use_simulation': 'True'}.items()
        ),
        # 启动感知子系统
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'perception_launch.py']),
            launch_arguments={'robot_ns': 'robot1'}.items()
        ),
        # 启动控制子系统
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'control_launch.py']),
            launch_arguments={'robot_ns': 'robot1'}.items()
        ),
        # 启动可视化
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'rviz_launch.py']),
            launch_arguments={'robot_ns': 'robot1'}.items()
        ),
    ])
```

---

### `launch/simulation_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim = LaunchConfiguration('use_simulation', default='False')
    declare_sim = DeclareLaunchArgument(
        'use_simulation', default_value='False',
        description='Whether to launch simulation environment'
    )
    sim_node = Node(
        package='gazebo_ros',   # 假设使用 Gazebo 仿真
        executable='gzserver',
        name='gazebo_server',
        output='screen',
        condition=launch.conditions.IfCondition(use_sim)
    )
    return LaunchDescription([declare_sim, sim_node])
```

---

### `launch/perception_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns', default='robot1')
    declare_ns = DeclareLaunchArgument(
        'robot_ns', default_value='robot1',
        description='Robot namespace'
    )
    params_file = PathJoinSubstitution([
        FindPackageShare('my_robot_system'),
        'launch/config',
        'params_{}.yaml'.format(robot_ns.perform({}))  # 简化写法示意
    ])
    sensor_node = Node(
        package='my_robot_system',
        executable='sensor_node',
        namespace=robot_ns,
        name='sensor',
        output='screen',
        parameters=[params_file]
    )
    return LaunchDescription([declare_ns, sensor_node])
```

---

### `launch/control_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns', default='robot1')
    declare_ns = DeclareLaunchArgument(
        'robot_ns', default_value='robot1',
        description='Robot namespace'
    )
    controller_node = Node(
        package='my_robot_system',
        executable='controller_node',
        namespace=robot_ns,
        name='controller',
        output='screen',
        parameters=[{'use_sim': True}]  # 可覆写的参数
    )
    return LaunchDescription([declare_ns, controller_node])
```

---

### `launch/rviz_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare('my_robot_system'),
        'launch/config',
        'rviz_config.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    return LaunchDescription([rviz_node])
```

---

## 🧠 说明

* 顶层 `top_level_launch.py` 只负责 “组合子系统”，可复用性高。
* 每个子系统（仿真、感知、控制、可视化）独立 launch 文件，便于维护。
* 参数与命名空间（`robot_ns`）通过 LaunchArguments 或配置文件 (YAML) 传入，提升灵活性。
* 使用 `FindPackageShare` + `PathJoinSubstitution` 加载资源（如 config 文件、RViz 配置）。
* 命名空间：所有节点都使用 `namespace=robot_ns`，这样支持多机器人复用。
* 覆写／参数化：例如感知节点加载 `params_robot1.yaml`，控制节点有可覆盖参数。

---

如果你愿意，我可以 **生成一个完整的 GitHub 风格代码仓库结构（包括 package.xml、CMakeLists/ setup.py、config YAML、上述 launch 文件）**给你，你可以直接克隆使用。你看要不要？
