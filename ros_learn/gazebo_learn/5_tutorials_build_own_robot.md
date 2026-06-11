以下是对这篇文章 “Gazebo Fortress — *Building your own robot*” 的详细整理、解析和源码摘录（中文说明 + 原文代码）。我会按文章结构：介绍 SDF → 构建世界 → 构建模型（底盘、轮子、框架、关节）来说明，同时附上原文代码片段。

---

## 一、简介／SDF 是什么

文章开头说明，在本教程中我们将使用 SDF 描述语言来构建一个简单的双轮机器人。 ([gazebosim.org][1])
SDF 即 “SDFormat (Simulation Description Format)” — 一个用于机器人仿真、可视化、控制环境中描述对象与环境的 XML 格式。([gazebosim.org][1])
因此，使用 Gazebo Fortress 时，我们使用 SDF 文件来定义环境、模型、物理属性、可视属性、碰撞属性、连结（joints）等。

---

## 二、构建一个世界（Building a world）

文章下一步说明先创建一个世界文件（例如 `building_robot.sdf`），把简单的地面、太阳光源加入。([gazebosim.org][1])
以下是原文代码片段（有略微缩进调整以便阅读）：

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="car_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics">
        </plugin>
        <plugin
            filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands">
        </plugin>
        <plugin
            filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>
        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>
        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                        </plane>
                    </geometry>
                </collision>
                <visual name="visual">
                    <geometry>
                        <plane>
                            <normal>0 0 1</normal>
                            <size>100 100</size>
                        </plane>
                    </geometry>
                    <material>
                        <ambient>0.8 0.8 0.8 1</ambient>
                        <diffuse>0.8 0.8 0.8 1</diffuse>
                        <specular>0.8 0.8 0.8 1</specular>
                    </material>
                </visual>
            </link>
        </model>
    </world>
</sdf>
```

代码说明：

* `<physics>` 块定义物理引擎参数（步长 `max_step_size=0.001` 秒，实时因子 `1.0`）([gazebosim.org][1])
* 插件（plugin）部分载入物理、用户命令、场景广播系统。
* `<light>` 定义一个定向光源“sun”，用于模拟太阳光。
* `<model name="ground_plane">` 定义一个静态模型作为地面。链接 link 含有碰撞体（plane）和视觉体（plane size 100×100m）。

运行方式：

```
ign gazebo building_robot.sdf
```

你应当看到一个空世界，仅有地面和平面光源。([gazebosim.org][1])

---

## 三、构建模型（Building a model）

文章接下来介绍如何在同一世界文件中加入机器人模型（model 标签）。([gazebosim.org][1])

### 3.1 定义模型（Defining the model）

代码片段：

```xml
<model name='vehicle_blue' canonical_link='chassis'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
    ...
```

说明：

* `name='vehicle_blue'` 给模型命名。
* `canonical_link='chassis'` 指定模型的 “canonical” 链接（即模型的主要参考框架）为 `chassis`。如果不定义，首个 `<link>` 将被选为 canonical。([gazebosim.org][1])
* `<pose relative_to='world'>0 0 0 0 0 0</pose>` 定义模型在世界坐标系下的位置和姿态（X Y Z R P Y 均为零，表示在原点）。([gazebosim.org][1])

### 3.2 链接（Links）：底盘（Chassis）

代码：

```xml
<link name='chassis'>
    <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
    <inertial>
        <mass>1.14395</mass>
        <inertia>
            <ixx>0.095329</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.381317</iyy>
            <iyz>0</iyz>
            <izz>0.476646</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <box>
                <size>2.0 1.0 0.5</size>
            </box>
        </geometry>
        <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.0 0.0 1.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <box>
                <size>2.0 1.0 0.5</size>
            </box>
        </geometry>
    </collision>
</link>
```

说明：

* 链接 `chassis` 表示机器人底盘部分。
* `<pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>` 将底盘在模型框架中位于 x=0.5m, y=0, z=0.4m。
* `<inertial>` 定义质量 `1.14395 kg` 及惯性张量。文章中指出，这些惯性张量可通过工具计算。([gazebosim.org][1])
* `<visual>` 定义可视几何体为一个 box (2.0×1.0×0.5 m)，颜色为蓝色 (0,0,1)。
* `<collision>` 定义碰撞几何体，使用相同尺寸的 box。文章提醒：可视几何与碰撞几何可以不同，为了简化碰撞检测可使用更简单模型。([gazebosim.org][1])

### 3.3 添加轮子（Left wheel & Right wheel）

首先左轮（Left wheel）代码：

```xml
<link name='left_wheel'>
    <pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
        <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```

说明：

* 名为 `left_wheel` 的链接，其坐标系相对于 `chassis`：x = -0.5 m, y = 0.6 m, z = 0, 旋转角为 -1.5707 rad (~ ‑90°)绕 x 轴。原因是轮子被侧放。([gazebosim.org][1])
* 惯性：质量 1kg，惯性张量给出。
* 可视与碰撞体几何为一个圆柱 (radius 0.4m, length 0.2m)，材质为红色 (1,0,0)。
  文章提示：因为轮子放置在底盘左侧后面，所以偏移为 -0.5, 0.6。([gazebosim.org][1])

然后右轮（Right wheel）代码：

```xml
<link name='right_wheel'>
    <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
        <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <cylinder>
                <radius>0.4</radius>
                <length>0.2</length>
            </cylinder>
        </geometry>
    </collision>
</link>
```

说明：右轮与左轮类似，仅 y 坐标为 -0.6m，表示在底盘右侧。([gazebosim.org][1])

### 3.4 定义任意框架（Arbitrary frame）

文章接着说明，SDF 1.7+ 支持定义任意帧 (frame)。例如为“caster”轮（万向轮）定义一个 frame。代码：

```xml
<frame name="caster_frame" attached_to='chassis'>
    <pose>0.8 0 -0.2 0 0 0</pose>
</frame>
```

说明：

* 名为 `caster_frame`，连接到 `chassis` 链接。
* 未使用 `relative_to`，因此 pose 相对于 `attached_to` 的 `chassis`。位置 x=0.8m, y=0, z=-0.2m。([gazebosim.org][1])

### 3.5 添加 caster 轮（Caster wheel）

代码：

```xml
<link name='caster'>
    <pose relative_to='caster_frame'/>
    <inertial>
        <mass>1</mass>
        <inertia>
            <ixx>0.016</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.016</iyy>
            <iyz>0</iyz>
            <izz>0.016</izz>
        </inertia>
    </inertial>
    <visual name='visual'>
        <geometry>
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
        <material>
            <ambient>0.0 1 0.0 1</ambient>
            <diffuse>0.0 1 0.0 1</diffuse>
            <specular>0.0 1 0.0 1</specular>
        </material>
    </visual>
    <collision name='collision'>
        <geometry>
            <sphere>
                <radius>0.2</radius>
            </sphere>
        </geometry>
    </collision>
</link>
```

说明：

* 链接 `caster`，其 pose 相对于 `caster_frame`。因为 `<pose/>` 为空，表示与 frame 同位（identity）。
* 惯性质量 1kg，惯性张量均为 0.016。
* 可视/碰撞几何为球体（radius 0.2m），材质绿色 (0,1,0)。([gazebosim.org][1])

### 3.6 连接链接 — 关节（Joints）

文章说明必须用 `<joint>` 标签将链接连接起来，以定义怎样运动。([gazebosim.org][1])

#### 左轮关节

代码：

```xml
<joint name='left_wheel_joint' type='revolute'>
    <pose relative_to='left_wheel'/>
    <parent>chassis</parent>
    <child>left_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
        </limit>
    </axis>
</joint>
```

说明：

* 名为 `left_wheel_joint`，类型 `revolute`（一个旋转自由度）。
* `<pose relative_to='left_wheel'/>` 指定关节位置相对于左轮链接。
* `parent=chassis`, `child=left_wheel`：父链接为底盘，子链接为左轮。
* `<axis><xyz …>0 1 0</xyz>` 定义旋转轴为 y‑轴（相对于 `__model__` 框架）。
* 关节极限为负／正无限（‑∞ 到 +∞）用浮点表示。([gazebosim.org][1])

#### 右轮关节

代码：

```xml
<joint name='right_wheel_joint' type='revolute'>
    <pose relative_to='right_wheel'/>
    <parent>chassis</parent>
    <child>right_wheel</child>
    <axis>
        <xyz expressed_in='__model__'>0 1 0</xyz>
        <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
        </limit>
    </axis>
</joint>
```

说明：与左轮类似，仅子链接换为 `right_wheel`。([gazebosim.org][1])

#### Caster 轮关节

代码：

```xml
<joint name='caster_wheel' type='ball'>
    <parent>chassis</parent>
    <child>caster</child>
</joint>
```

说明：

* 名为 `caster_wheel`，类型 `ball`（球形关节，有 3 个旋转自由度）。
* 父链接 `chassis`，子链接 `caster`。
* 无额外轴定义。([gazebosim.org][1])

---

## 四、结论部分

文章结尾指出，运行命令 `ign gazebo building_robot.sdf` 后，你应当看到一个两轮机器人模型。([gazebosim.org][1])
然后指出：你可以继续学习更多 SDF 标签的细节。([gazebosim.org][1])

---

## 五、完整源码（整合）

下面我整合一个完整的 `building_robot.sdf` 示例（基于文章内容），注：省略了部分重复或不必要标签，仅保留关键模型结构。你可以作为起点，然后根据需要修改／扩展。

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="car_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="vehicle_blue" canonical_link="chassis">
      <pose relative_to="world">0 0 0 0 0 0</pose>

      <!-- chassis link -->
      <link name="chassis">
        <pose relative_to="__model__">0.5 0 0.4 0 0 0</pose>
        <inertial>
          <mass>1.14395</mass>
          <inertia>
            <ixx>0.095329</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.381317</iyy>
            <iyz>0</iyz>
            <izz>0.476646</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>2.0 1.0 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.0 0.0 1.0 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>2.0 1.0 0.5</size>
            </box>
          </geometry>
        </collision>
      </link>

      <!-- left wheel -->
      <link name="left_wheel">
        <pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>

      <!-- right wheel -->
      <link name="right_wheel">
        <pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.043333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.043333</iyy>
            <iyz>0</iyz>
            <izz>0.08</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1.0 0.0 0.0 1</ambient>
            <diffuse>1.0 0.0 0.0 1</diffuse>
            <specular>1.0 0.0 0.0 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.4</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
      </link>

      <!-- caster frame -->
      <frame name="caster_frame" attached_to="chassis">
        <pose>0.8 0 -0.2 0 0 0</pose>
      </frame>

      <!-- caster wheel -->
      <link name="caster">
        <pose relative_to="caster_frame"/>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.016</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.016</iyy>
            <iyz>0</iyz>
            <izz>0.016</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.0 1.0 0.0 1</ambient>
            <diffuse>0.0 1.0 0.0 1</diffuse>
            <specular>0.0 1.0 0.0 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.2</radius>
            </sphere>
          </geometry>
        </collision>
      </link>

      <!-- joints -->
      <joint name="left_wheel_joint" type="revolute">
        <pose relative_to="left_wheel"/>
        <parent>chassis</parent>
        <child>left_wheel</child>
        <axis>
          <xyz expressed_in="__model__">0 1 0</xyz>
          <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
          </limit>
        </axis>
      </joint>

      <joint name="right_wheel_joint" type="revolute">
        <pose relative_to="right_wheel"/>
        <parent>chassis</parent>
        <child>right_wheel</child>
        <axis>
          <xyz expressed_in="__model__">0 1 0</xyz>
          <limit>
            <lower>-1.79769e+308</lower>
            <upper>1.79769e+308</upper>
          </limit>
        </axis>
      </joint>

      <joint name="caster_wheel" type="ball">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>

    </model>

  </world>
</sdf>
```

你可以将上面内容保存为 `building_robot.sdf`，然后在终端运行：

```
ign gazebo building_robot.sdf
```

如果一切正常，你将在仿真中看到地面 + 由底盘、两个大轮子和一个小球形万向轮组成的机器人。

---

## 六、一些补充说明 & 提示

* 当你修改机器人尺寸、质量、惯性、几何形状、位置时，应注意保持惯性矩阵与质量分布的合理性，否则物理仿真可能会不稳定。文章中提到惯性矩阵可通过工具计算。([gazebosim.org][1])
* 本示例的关节极限设置为 “无限” （‐∞ 到 +∞），意味着轮子能无限转动。如果你计划给电机／控制器加上限制或减速器，可能需要更具体的上下限。
* 使用 `<frame>` 定义自定义参考帧（如 caster_frame）是 SDF 1.7+ 的新特性，有助于更灵活地定位和组织链接。([gazebosim.org][1])
* 可视几何（visual）和碰撞几何（collision）并不必须完全一致。碰撞几何可以简化以提高仿真效率。文章中有提示。([gazebosim.org][1])
* 虽然此文使用的是 Gazebo Fortress (与 Ignition Gazebo 相关）环境，但基本 SDF 概念也适用于后续版本。文章也在开头说明。([gazebosim.org][1])

---

如果你愿意，我可以 **完整翻译整篇文章**（包含所有注释和代码），或者 **扩展**：例如加入传感器、驱动控制、加入 ROS2 接口，再或者 **详细分析**各个 SDF 标签含义。你希望哪一种？

[1]: https://gazebosim.org/docs/fortress/building_robot/ "Building your own robot — Gazebo fortress documentation"
