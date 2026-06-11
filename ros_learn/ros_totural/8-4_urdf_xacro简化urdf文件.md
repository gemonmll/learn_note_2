没问题，我们来学习一下如何使用 Xacro 来“清理”和简化你的 URDF 文件。

Xacro (XML Macros) 是一种宏语言，它能帮助你创建更简洁、可读性更强、更易于维护的 URDF 文件。想象一下，你不需要再手动复制粘贴大量重复的 XML 代码块，也不需要手动计算所有坐标了。

以下是教程的核心内容：

### 1\. 什么是 Xacro？

Xacro 是一种宏语言，它允许你：

  * **使用常量（Properties）:** 定义一次尺寸（如 `width="0.2"`），然后在多个地方引用它。
  * **进行数学运算:** 在 URDF 中直接计算值（如 `${width / 2}`）。
  * **创建宏（Macros）:** 定义可重用的 XML 模板（如为“腿”或“轮子”创建一个模板），然后用不同的参数多次调用它。

### 2\. 如何使用 Xacro？

Xacro 文件通常以 `.xacro` 或 `.urdf.xacro` 结尾。

#### 2.1 声明 Xacro 命名空间

为了让文件能被正确解析，你必须在 `robot` 标签中添加 `xmlns:xacro` 命名空间。你的 Xacro 文件开头应该如下所示：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  </robot>
```

#### 2.2 生成 URDF

Xacro 文件本身不能被 ROS 直接使用，它必须先被转换成一个标准的 `.urdf` 文件。

  * **命令行转换:**

    ```bash
    xacro model.xacro > model.urdf
    ```

  * **在 Launch 文件中自动转换 (推荐)**:
    你可以在 ROS 2 的 Launch 文件中设置，让 `robot_state_publisher` 节点在启动时自动运行 Xacro 并加载模型。

    （教程中提供了两种 launch 方法，一种是使用 `Command` 替换，另一种是使用 `urdf_launch` 包，后者更简洁。）

### 3\. Xacro 的核心功能

#### 3.1 常量 (Properties)

这可以消除代码中的“魔术数字”（magic numbers）并减少冗余。

  * **定义:** 使用 `<xacro:property>` 标签
  * **使用:** 使用 `${property_name}`

**示例：** 假设你的 `base_link` 在 `visual` 和 `collision` 中都使用了相同的圆柱体。

**原始 URDF (冗余):**

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
</link>
```

**使用 Xacro (简洁):**

```xml
<xacro:property name="bodylen" value="0.6" />
<xacro:property name="width" value="0.2" />

<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="${bodylen}" radius="${width}"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="${bodylen}" radius="${width}"/>
    </geometry>
  </collision>
</link>
```

现在，如果你想改变 `base_link` 的尺寸，只需要修改顶部的 `property` 值即可。

#### 3.2 数学运算

你可以在 `${}` 内部执行数学运算：

```xml
<xacro:property name="wheeldiam" value="0.1" />
<xacro:property name="width" value="0.2" />

<cylinder radius="${wheeldiam / 2}" length="0.1"/>

<origin xyz="${(width + 0.02) * -1} 0 0.25" />
```

教程还提到，你可以使用 `sin`, `cos` 等函数。

#### 3.3 宏 (Macros)

这是 Xacro 最强大的功能，用于创建可重用的代码块。

  * **定义:** `<xacro:macro name="macro_name" params="param1 param2 ..."> ... </xacro:macro>`
  * **使用:** `<xacro:macro_name param1="value1" param2="value2" />`

**示例 1: 简单的参数化宏 (默认惯性)**

假设你厌倦了为每个连杆复制粘贴 `inertial` 标签，你可以创建一个宏：

```xml
<xacro:macro name="default_inertial" params="mass">
  <inertial>
    <mass value="${mass}" />
    <inertia ixx="1e-3" ixy="0.0" ixz="0.0" iyy="1e-3" iyz="0.0" izz="1e-3" />
  </inertial>
</xacro:macro>

<link name="my_link">
  <xacro:default_inertial mass="10"/>
</link>
```

**示例 2: 块参数 (Block Parameters)**

你甚至可以传递整个 XML 块作为参数。

```xml
<xacro:macro name="blue_shape" params="name *shape">
  <link name="${name}">
    <visual>
      <geometry>
        <xacro:insert_block name="shape" />
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <xacro:insert_block name="shape" />
      </geometry>
    </collision>
  </link>
</xacro:macro>

<xacro:blue_shape name="base_link">
  <cylinder radius=".42" length=".01" />
</xacro:blue_shape>
```

### 4\. 实际应用：创建 R2D2 的腿

教程展示了一个非常实用的 `leg` 宏，它结合了所有技巧来创建机器人的左腿和右腿，而代码只写了一遍。

**代码（简化版）：**

```xml
<xacro:property name="leglen" value="0.6" />
<xacro:property name="width" value="0.2" />
<xacro:property name="pi" value="3.14159" />

<xacro:macro name="leg" params="prefix reflect">
  <link name="${prefix}_leg">
    <visual>
      <geometry>
        <box size="${leglen} 0.1 0.2"/>
      </geometry>
      <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
      <material name="white"/>
    </visual>
    </link>

  <joint name="base_to_${prefix}_leg" type="fixed">
    <parent link="base_link"/>
    <child link="${prefix}_leg"/>
    <origin xyz="0 ${reflect * (width + 0.02)} 0.25" />
  </joint>
</xacro:macro>

<xacro:leg prefix="right" reflect="1" />
<xacro:leg prefix="left" reflect="-1" />
```

### 总结

Xacro 是编写 URDF 的现代标准方式。它通过**属性（常量）**、**数学运算**和\*\*宏（模板）\*\*极大地提高了 URDF 文件的可维护性和可读性，让你能够构建复杂模型而不会迷失在 XML 的海洋中。

要查看 Xacro 文件的最终效果（教程中的 `08-macroed.urdf.xacro`），你可以运行：

```bash
ros2 launch urdf_tutorial display.launch.py model:=urdf/08-macroed.urdf.xacro
```