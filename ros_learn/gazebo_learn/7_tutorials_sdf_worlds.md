好的，下面我将按 **格式清晰**、结构分层整理这篇 Ignition Gazebo (Fortress) 教程 “SDF Worlds” 的内容，包含：节标题、讲解、代码片段。这样你能更易阅读和理解。

---

## 教程：SDF Worlds

出处：官方文档 “SDF worlds” ([gazebosim.org][1])

---

### 1. 定义一个世界（Defining a world）

**说明**：每个 SDF 世界文件以以下最基本结构开始：

````xml
<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="world_demo">
        …
    </world>
</sdf>
``` :contentReference[oaicite:2]{index=2}  
- `<?xml version="1.0" ?>`：XML 声明。  
- `<sdf version="1.8">`：指定 SDF 格式版本。  
- `<world name="world_demo"> … </world>`：定义世界主体，`name` 属性给出名称。  
- 世界中所有内容（物理、光源、模型、插件等）都放在 `<world>` 标签内。  
``````


### 2. 物理引擎配置（Physics）  
**说明**：控制仿真物理系统的步长、实时因子等。示例如下：  
```xml
<physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
</physics>
``` :contentReference[oaicite:3]{index=3}  
- `name="1ms"`：给物理系统命名（这里表示“每步 1毫秒”）。  
- `type="ignored"`：物理库类型（如 ODE、Bullet、Simbody、Dart），“ignored” 表示暂不指定具体物理库。  
- `<max_step_size>`：每仿真步的最大时间（秒）。例如 `0.001` 表示 1ms。  
- `<real_time_factor>`：仿真时间与真实时间的比例。`1.0` 表示同步。  

**注意**：步长设小可提高精度，但会增加计算负担。
``````
---

### 3. 插件（Plugins）  
**说明**：插件是加载共享库来增强仿真系统功能。示例如下：  
```xml
<plugin
    filename="gz‑sim‑physics‑system"
    name="gz::sim::systems::Physics">
</plugin>

<plugin
    filename="gz‑sim‑user‑commands‑system"
    name="gz::sim::systems::UserCommands">
</plugin>

<plugin
    filename="gz‑sim‑scene‑broadcaster‑system"
    name="gz::sim::systems::SceneBroadcaster">
</plugin>
``` :contentReference[oaicite:4]{index=4}  
- 第一个插件 `"Physics"`：负责物理系统模拟。  
- 第二个 `"UserCommands"`：支持创建/移动/删除模型等用户操作。  
- 第三个 `"SceneBroadcaster"`：将场景信息广播给 GUI 等。  
你可以根据需要添加更多插件（例如传感器系统、导航系统等）。
``````
---

### 4. GUI 配置（GUI）  
**说明**：控制仿真界面的视图、摄像机、渲染引擎等。示例如下：  
```xml
<gui fullscreen="0">
    <!-- 3D scene plugin -->
    <plugin filename="MinimalScene" name="3D View">
        <gz‑gui>
            <title>3D View</title>
            <property type="bool" key="showTitleBar">false</property>
            <property type="string" key="state">docked</property>
        </gz‑gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-6 0 6 0 0.5 0</camera_pose>
        <camera_clip>
            <near>0.25</near>
            <far>25000</far>
        </camera_clip>
    </plugin>

    <plugin filename="GzSceneManager" name="Scene Manager">
        <gz‑gui>
            <property key="resizable" type="bool">false</property>
            <property key="width" type="double">5</property>
            <property key="height" type="double">5</property>
            <property key="state" type="string">floating</property>
            <property key="showTitleBar" type="bool">false</property>
        </gz‑gui>
    </plugin>
</gui>
``` :contentReference[oaicite:5]{index=5}  
**解释**：  
- `<gui fullscreen="0">`：指是否全屏（0=否）。  
- 第一个插件 `MinimalScene` / `3D View`：控制场景渲染、摄像机位置、背景色。  
- 第二个插件 `Scene Manager`：控制窗口尺寸、是否可调整、浮动状态等。  
- `<engine>`：可选渲染引擎，如 `ogre2`。  
- `<camera_pose>`：摄像机初始位置 (x, y, z, roll, pitch, yaw)。  
- `<camera_clip>`：摄像机裁剪范围。  

此外，教程还提到有 “World control” 和 “World stats” 等插件，用于控制仿真播放及查看统计。 :contentReference[oaicite:6]{index=6}
``````
---

### 5. 光源（Light）  
**说明**：添加场景中的光源，如“太阳光”。示例如下：  
```xml
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
``` :contentReference[oaicite:7]{index=7}  
**解释**：  
- `type="directional"` 表示定向光源（类似太阳）。  
- `cast_shadows="true"`：是否投射阴影。  
- `<pose>`：光源位置与姿态。  
- `<diffuse>` / `<specular>`：漫反射 / 镜面反射颜色。  
- `<attenuation>`：光衰减设置：包括 `range`（作用距离），`constant`、`linear`、`quadratic` 衰减系数。  
- `<direction>`：光线方向。
``````
---

### 6. 添加模型（Adding models）  
**说明**：将模型加入世界中有两种常见方式：

#### 6.1 使用 URI 方式  
```xml
<include>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
``` :contentReference[oaicite:8]{index=8}  
**解释**：  
- `<include>` 标签用于加载外部模型。  
- `<uri>` 可以是网络地址（如 Fuel 仓库）或者本地 `model://` 格式。  
- 模型会在仿真加载时自动载入。
``````
#### 6.2 本地引用方式  
```xml
<include>
    <uri>model://Coke</uri>
</include>
````

然后需要设置环境变量，例如：

````
export IGN_GAZEBO_RESOURCE_PATH="$HOME/world_tutorial"
``` :contentReference[oaicite:9]{index=9}  
**解释**：  
- `model://Coke` 表示本地模型目录 `Coke`。  
- 环境变量 `IGN_GAZEBO_RESOURCE_PATH`（或类似）用于指定模型资源路径。  
- 本地引用能更好管理自定义模型。

**注意**：如果要加载多个实例，还可以为每个 `<include>` 指定 `<name>` 和 `<pose>`，如下：
```xml
<include>
    <name>Coke0</name>
    <pose>0 0 0 0 0 0</pose>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
<include>
    <name>Coke1</name>
    <pose>0 0.1 0 0 0 0</pose>
    <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
``` :contentReference[oaicite:10]{index=10}  
``````
---

### 7. 完整示例概览  
以下是一个 **简化的** world 文件示例，将前面各块合并起来：

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="world_demo">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz‑sim‑physics‑system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz‑sim‑user‑commands‑system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz‑sim‑scene‑broadcaster‑system" name="gz::sim::systems::SceneBroadcaster"/>

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <gz‑gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </gz‑gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>-6 0 6 0 0.5 0</camera_pose>
        <camera_clip>
          <near>0.25</near>
          <far>25000</far>
        </camera_clip>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <gz‑gui>
          <property key="resizable" type="bool">false</property>
          <property key="width" type="double">5</property>
          <property key="height" type="double">5</property>
          <property key="state" type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </gz‑gui>
      </plugin>
    </gui>

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

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
    </include>

  </world>
</sdf>
````

你可以保存为 `world_demo.sdf`，然后运行：

```bash
ign gazebo world_demo.sdf
```

（或使用 `gz sim world_demo.sdf`，取决于你的 Ignition/Gazebo 版本）

---

如果你觉得“格式不对”的话：请告诉我 **你希望的格式**（比如：Markdown 表格、带注释版本、只代码、小节概述、中文翻译、原文对照）— 我可以按照你指定的格式重新整理。你希望改成哪种？

[1]: https://gazebosim.org/docs/latest/sdf_worlds/?utm_source=chatgpt.com "SDF worlds — Gazebo ionic documentation"
