下面是对 Gazebo Fortress（即 Gazebo Sim “Fortress” 版本）文档中 “Actors” 一节（“Actors — Gazebo Fortress documentation”）的较为完整、详细的学习整理，包含关键源码片段，并按结构进行分节说明。你可以把它当做学习笔记／实践指南。若需我提取更完整的示例 SDF 文件或进一步解释每个标签的含义，我也可以帮你。

---

## 概览

在 Gazebo Sim（Fortress版本）中，“actor”（译作“演员”或“动作模型”）指的是一种 **动画模型（animated model）**，即模型里包含动画帧或者预设轨迹，并且**不受物理力（如重力、碰撞）影响**。([Gazebo][1])
文档开篇说明：

> “Animations are very useful if we want to have entities following a predefined path in simulation without being affected by the physics. …” ([Gazebo][1])
> 也就是说，“actor”适合用于那些想要在仿真中沿固定轨迹移动、或做骨骼动画（如人走路）但不希望被物理交互（重力、碰撞）影响的场景。 ([Gazebo][1])

在本节中，作者将介绍两种动画方式：

* Skeleton animation（骨骼动画，模型内部链接之间的运动）
* Trajectory animation（轨迹动画，将整个 actor 模型沿路径移动）
  并且说明可以组合使用。 ([Gazebo][1])

下面我按文档结构细分讲解。

---

## 1. Actor 的基本特性

文档中列出了 actor 的几个关键特性： ([Gazebo][1])

* 不受物理力作用：既不会被重力拉下，也不会因为接触而受到影响。
* 支持骨骼动画（skeleton animation），可导入 COLLADA (.dae) 或 BVH (.bvh) 文件。 ([Gazebo][1])
* 支持在 SDF 文件中直接定义“轨迹动画”（trajectory animation）。 ([Gazebo][1])

此外，文档指出骨骼动画和轨迹动画可分开使用或组合使用。 ([Gazebo][1])

理解这些非常重要：如果你只是想让一个模型“走路”但静止在一个地点，你可能只用骨骼动画；如果你还想让这个模型“走来走去”，那就用轨迹动画（并可能同时有骨骼动画）。

---

## 2. 骨骼动画 (Skeleton)

### 2.1 支持的格式

文档指出，Gazebo Sim 支持两种骨骼动画文件格式：

* COLLADA (.dae) 文件格式。 ([Gazebo][1])
* Biovision Hierarchy (.bvh) 文件格式。 ([Gazebo][1])

### 2.2 示例 SDF 片段

文档给出了一个 SDF 的示例，说明如何在 world 文件中添加一个 actor，其骨骼动画来自一个 COLLADA 文件。摘录如下： ([Gazebo][1])

```xml
<actor name="actor_walking">
    <skin>
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
        <scale>1.0</scale>
    </skin>
    <animation name="walk">
        <filename>https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae</filename>
    </animation>
</actor>
```

**说明**：

* `<actor name="actor_walking">`：定义了一个 actor 模型，名称为 actor_walking。
* `<skin>` 标签：定义 actor 的外观（模型网格/皮肤）。这里指定一个 COLLADA (.dae) 文件，并用 `<scale>` 指定缩放比例。
* `<animation name="walk">`：定义一个动画，名称为 “walk”，并同样指定动画来源文件（这里同样是 walk.dae）。
* 注意：当使用 COLLADA 文件时，动画和模型可以一同在 skin 文件中或 animation 文件中载入。文档中指出：“When a COLLADA file is used within the `<skin>` tags its animation is loaded.” ([Gazebo][1])

### 2.3 说明

* 通过 `<skin>` 载入的 dae 文件不仅仅是模型，也可以带有骨骼动画。
* `<scale>` 控制模型大小。
* `<animation>` 是可选的，如果想定义多个动画（比如走路、挥手）也可。文档中提到“we can combine different skins with different animations as long as they have compatible skeletons.” ([Gazebo][1])
* 在仿真中看到效果后，模型就会“动”起来（骨骼动画生效）但如果没有轨迹，它只是原地动画。

---

## 3. 路径（轨迹）动画 (Scripted trajectory)

如果你希望 actor 不只是原地动动手脚，而是沿世界坐标移动，那就要使用轨迹动画。

### 3.1 说明

文档说明如下：

> “This is the high level animation of actors, which consists of specifying a series of poses to be reached at specific times. Gazebo Sim takes care of interpolating the motion between them so the movement is fluid.” ([Gazebo][1])
> “Animations that have displacement on the X axis … will have the skeleton animated while following a trajectory.” ([Gazebo][1])

也就是说：你在 SDF 中定义一系列时间点（waypoints），每个 time 对应一个 pose（位置+朝向）。仿真引擎会在这些点之间插值，实现平滑移动。

### 3.2 示例 SDF 片段

文档给出的示例（基于上面的 actor）如下： ([Gazebo][1])

```xml
<script>
    <loop>true</loop>
    <delay_start>0.000000</delay_start>
    <auto_start>true</auto_start>

    <trajectory id="0" type="walk" tension="0.6">
        <waypoint>
            <time>0</time>
            <pose>0 0 1.0 0 0 0</pose>
        </waypoint>
        <waypoint>
            <time>2</time>
            <pose>2.0 0 1.0 0 0 0</pose>
        </waypoint>
        <waypoint>
            <time>2.5</time>
            <pose>2 0 1.0 0 0 1.57</pose>
        </waypoint>
        <waypoint>
            <time>4</time>
            <pose>2 2 1.0 0 0 1.57</pose>
        </waypoint>
        <waypoint>
            <time>4.5</time>
            <pose>2 2 1.0 0 0 3.142</pose>
        </waypoint>
        <waypoint>
            <time>6</time>
            <pose>0 2 1 0 0 3.142</pose>
        </waypoint>
        <waypoint>
            <time>6.5</time>
            <pose>0 2 1 0 0 -1.57</pose>
        </waypoint>
        <waypoint>
            <time>8</time>
            <pose>0 0 1.0 0 0 -1.57</pose>
        </waypoint>
        <waypoint>
            <time>8.5</time>
            <pose>0 0 1.0 0 0 0</pose>
        </waypoint>
    </trajectory>
</script>
```

### 3.3 说明细节

* `<script>` 标签位于 `<actor>` 标签内部。
* `<loop>`：若设置为 true，轨迹会循环执行。文档提示：“for a fluid continuous motion, make sure the last waypoint matches the first one, as we will do.” ([Gazebo][1])
* `<delay_start>`：从仿真启动后，延迟多少秒开始执行此脚本。
* `<auto_start>`：如果 true，仿真启动后就会自动开始；如果 false，可能由插件或触发事件启动。
* `<trajectory>`：定义一个轨迹，带有属性：

  * `id`：轨迹编号（这里为 “0”）
  * `type`：应当对应之前在 `<animation>` 标签中定义的动画名称（这里为 “walk”） — 这样骨骼动画和轨迹动画能够同步。
  * `tension`：平滑参数，范围 0 到 1。文档说明： “The default tension value is zero, which equates to a Catmull-Rom spline, which may cause the animation to overshoot waypoints. A tension value of one will cause the animation to stick to the waypoints.” ([Gazebo][1])
* `<waypoint>` 标签定义一个时间点：

  * `<time>`：从脚本开始计时，单位秒。
  * `<pose>`：位置 + 朝向。格式通常为 “x y z roll pitch yaw”。在示例中：`0 0 1.0 0 0 0` 表示起点在 x=0,y=0,z=1，高度 1 米，朝向(0,0,0)。
* 通过多个 waypoint，actor 会移动形成一个“走”或“循环”路径。示例中走了一个“方形”路径。
* 文档同时说明：顺序不必按照时间排序（但按照逻辑定义比较清晰），“The order in which waypoints are defined is not important, they will follow the given times.” ([Gazebo][1])
* 插值：轨迹会被平滑处理，“the trajectory is smoothed as a whole. … you’ll get a fluid motion, but the exact poses contained in the waypoints might not be reached.” ([Gazebo][1])

---

## 4. 综合示例 +使用提示

把骨骼动画和轨迹动画结合起来，就可以创建一个 actor：既有“走路”内部骨骼动作，又沿场景路径移动。文档中（虽然没有再给一个合体示例）通过上面的两个部分展示了。

### 使用提示：

* 如果你只需要让模型在一点做动作（例如挥手、跳跃等），那你可以省略 `<trajectory>` 部分，仅定义 `<skin>` + `<animation>`。
* 如果要让模型沿路径移动，那就至少定义 `<script>` + `<trajectory>`。如果还希望骨骼动画同步（比如走路动作），那么 `<animation>` 的 `type` 名称应当与 `<trajectory type="...">` 一致（示例中都是 “walk”）。
* 确保 actor 的高度（z 值）设置合适，以免模型“镶”进地面或悬空。示例中高度 1.0 米。
* 若使用循环 (`<loop>true</loop>`)，最后一个 waypoint 最好与起点匹配，以防跳跃回起点造成运动突变。
* 轨迹的 tension 值可以调节路径平滑度：0 = Catmull-Rom 可能会“越过”waypoints，1 = 会“严格”经过 waypoints。
* 导入的 COLLADA 文件中若含骨骼动画，skin 标签就能载入动画；如果动画与模型分离，也可以用 `<animation>` 指定不同的文件。
* 在仿真中观察 actor 时，要注意：即使 actor 在移动，它**不会**与其他物体发生物理碰撞或受重力影响（除非你把它当作普通物理模型而不是 actor） — 这是 actor 的用途。

---

## 5. 附加说明 & 链接

* 文档提到，actor 的 SDF 元素定义可参考 SDF 官方规格。 ([Gazebo][1])
* 文档还提供了一个 “Video walk-through” 链接。 ([Gazebo][1])
* 也指出：你可以访问 Ignition Robotics Fuel 平台查找可用的 actor 模型和动画。 ([Gazebo][1])

---

如果你愿意，我可以帮你 **下载**（或提取）一个完整的示例 world 文件（.sdf）包含 actor 模型 +动画 +轨迹，然后我们可以一步步分析每一行 SDF。你看要不要？

[1]: https://gazebosim.org/docs/fortress/actors/ "Actors — Gazebo fortress documentation"
