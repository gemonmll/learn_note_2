下面我帮你完整、详细地梳理一下 Gazebo Fortress（即 Ignition Gazebo 的 “Fortress” 版本）文档中 **“Sensors”** 一节（网址：[https://gazebosim.org/docs/fortress/sensors/](https://gazebosim.org/docs/fortress/sensors/)）的内容。文章主要介绍如何在仿真中为机器人／模型添加传感器，包括 IMU、接触（Contact）传感器、激光雷达（Lidar）传感器，并给出了示例 SDF 插件/传感器定义及一个小的 C++ 订阅节点代码。下面我按文档的结构来讲解，并 **附上关键源代码**。

---

## 1. 前言 & 使用说明

文档开头说明这是 Gazebo Fortress 版本的说明。([Gazebo][1])
并指出本教程是从“Moving the robot (移动机器人)”教程继续。([Gazebo][1])
目标：让你学会将传感器加到机器人或其他模型，并展示三种传感器：IMU、Contact、Lidar。([Gazebo][1])
还指出你可以查看最终世界、许多示例以及完整的 sensors 库。([Gazebo][1])

---

## 2. 预备说明（Preliminaries）

在你在 SDF 文件中新增插件之前，需要确保世界（world）文件中已有一些 “逻辑默认” 插件加载（即仿真系统、场景广播系统等），否则添加传感器可能功能不完整。([Gazebo][1])
文档给了一个示例：

```xml
<sdf version='1.9'>
  <world name='demo'>
    <plugin
        filename="ignition-gazebo-physics-system"
        name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
        filename="ignition-gazebo-scene-broadcaster-system"
        name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <!-- … 其它内容 … -->
```

这样做的目的是确保 Gazebo 的物理系统、场景广播系统等正常运行。([Gazebo][1])
然后你才在 `<world>` 中加入传感器插件及相应的 `<sensor>` 标签。

---

## 3. IMU 传感器（IMU sensor）

### 3.1 功能说明

IMU（惯性测量单元）输出机器人链接（link）的：

* `orientation` 四元数；
* `angular_velocity` 三轴；
* `linear_acceleration` 三轴。([Gazebo][1])

### 3.2 如何定义插件与传感器

在 SDF 世界文件中，你首先定义 IMU 系统插件（plugin），然后在模型的某个 link 下插入 `<sensor>` 标签。示例如下：

```xml
<plugin filename="libignition-gazebo-imu-system.so"
        name="ignition::gazebo::systems::Imu">
</plugin>
```

然后在模型某个 link（如 chassis）里：

```xml
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```

（注：`<always_on>`、`<update_rate>`、`<visualize>`、`<topic>`等，并非所有传感器都支持所有标签。）([Gazebo][1])

### 3.3 如何读取数据

启动仿真：

```
ign gazebo sensor_tutorial.sdf
```

然后在另一个终端监听 IMU 话题：

```
ign topic -e -t /imu
```

你会看到类似方向、角速度、线加速度的数据在输出。([Gazebo][1])
当你用键盘让机器人前进，IMU 数值会变化。([Gazebo][1])

---

## 4. 接触传感器（Contact sensor）

### 4.1 功能说明

接触传感器用于检测模型是否与其他物体发生接触／碰撞。这在避免机器人撞坏自身或环境时很有用。([Gazebo][1])

### 4.2 在 SDF 中使用：示例

首先，定义一个壁（wall）模型，如下：

```xml
<model name='wall'>
    <static>true</static>
    <pose>5 0 0 0 0 0</pose> <!-- pose 相对于 world -->
    <link name='box'>
        <pose/>
        <visual name='visual'>
            <geometry>
                <box>
                    <size>0.5 10.0 2.0</size>
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
                    <size>0.5 10.0 2.0</size>
                </box>
            </geometry>
        </collision>
    </link>
</model>
```

然后定义 plugin：

```xml
<plugin filename="libignition-gazebo-contact-system.so"
        name="ignition::gazebo::systems::Contact">
</plugin>
```

接着在 `box` link 下添加 `<sensor>`：

```xml
<sensor name='sensor_contact' type='contact'>
    <contact>
        <collision>collision</collision>
    </contact>
</sensor>
```

最后，还加一个 “触碰触发” 插件（TouchPlugin）用于发布消息：

```xml
<plugin filename="libignition-gazebo-touchplugin-system.so"
        name="ignition::gazebo::systems::TouchPlugin">
    <target>vehicle_blue</target>
    <namespace>wall</namespace>
    <time>0.001</time>
    <enabled>true</enabled>
</plugin>
```

其中：

* `<target>`：与壁发生接触的模型名称（如 `vehicle_blue`）([Gazebo][1])
* `<namespace>`：该触碰消息所属的话题命名空间（在本例中为 `/wall/touched`）([Gazebo][1])

### 4.3 使用方式

运行仿真：

```
ign gazebo sensor_tutorial.sdf
```

在另一个终端监听 `/wall/touched` 话题：

```
ign topic -e -t /wall/touched
```

当机器人与壁发生碰撞时，你应该看到消息 `data: true`。([Gazebo][1])
接着，可以用 `TriggeredPublisher` 插件让机器人停止运动：

```xml
<plugin filename="libignition-gazebo-triggered-publisher-system.so"
        name="ignition::gazebo::systems::TriggeredPublisher">
    <input type="ignition.msgs.Boolean" topic="/wall/touched">
        <match>data: true</match>
    </input>
    <output type="ignition.msgs.Twist" topic="/cmd_vel">
        linear: {x: 0.0}, angular: {z: 0.0}
    </output>
</plugin>
```

这样，当 `/wall/touched` 收到 `data: true` 时，机器人通过 `/cmd_vel` 接收到速度命令 `0`（停止）以防撞坏。([Gazebo][1])

---

## 5. 激光雷达传感器（Lidar sensor）

### 5.1 功能说明

LiDAR（Light Detection and Ranging）用于探测机器人周围环境的障碍物、距离信息。此处用于在机器人靠近壁之前就检测距离，从而避免碰撞。([Gazebo][1])

### 5.2 SDF 定义步骤

**第一步**：为机器人模型定义一个 `frame`（坐标系）来挂载 LiDAR：

```xml
<frame name="lidar_frame" attached_to='chassis'>
    <pose>0.8 0 0.5 0 0 0</pose>
</frame>
```

其中 `lidar_frame` 相对于机器人 `chassis` 的 pose。([Gazebo][1])

**第二步**：在 `<world>` 下加入 LiDAR 系统插件：

```xml
<plugin
  filename="libignition-gazebo-sensors-system.so"
  name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
</plugin>
```

此插件负责管理仿真中的传感器系统。([Gazebo][1])

**第三步**：在 `chassis` link 下加入 LiDAR `<sensor>` 标签：

```xml
<sensor name='gpu_lidar' type='gpu_lidar'>
    <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
    <topic>lidar</topic>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>640</samples>
                <resolution>1</resolution>
                <min_angle>-1.396263</min_angle>
                <max_angle>1.396263</max_angle>
            </horizontal>
            <vertical>
                <samples>1</samples>
                <resolution>0.01</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
    </ray>
    <always_on>1</always_on>
    <visualize>true</visualize>
</sensor>
```

解释关键标签：

* `name` 和 `type`：标识传感器名称和类型（此处为 `gpu_lidar`）。([Gazebo][1])
* `<pose relative_to='lidar_frame'>`：位置／朝向相对于 `lidar_frame`。([Gazebo][1])
* `<topic>`：输出话题名称，此处 `lidar`。([Gazebo][1])
* `<update_rate>`：数据更新频率（Hz）。([Gazebo][1])
* `<ray>`：激光射线定义，包括 `<scan>`（水平、垂直方向）和 `<range>`。

  * `<horizontal>`：激光在水平方向的采样数、分辨率、最小角／最大角。([Gazebo][1])
  * `<vertical>`：垂直方向设定。([Gazebo][1])
  * `<range>`：射线的最小、最大距离以及线性分辨率。([Gazebo][1])
* `<always_on>` 和 `<visualize>`：是否一直激活、是否在 GUI 可视化。([Gazebo][1])

### 5.3 读取 LiDAR 数据 &避障逻辑

启动仿真：

```
ign gazebo sensor_tutorial.sdf
```

在另一个终端监听 `/lidar` 话题：

```
ign topic -e -t /lidar
```

LiDAR 消息包含如下字段：

````txt
message LaserScan
{
  Header header;
  string frame;
  Pose world_pose;
  double angle_min;
  double angle_max;
  double angle_step;
  double range_min;
  double range_max;
  uint32 count;
  double vertical_angle_min;
  double vertical_angle_max;
  double vertical_angle_step;
  uint32 vertical_count;
  repeated double ranges;
  repeated double intensities;
}
``` :contentReference[oaicite:27]{index=27}  
其中 `ranges` 数组就是每条激光射线测得的距离。:contentReference[oaicite:28]{index=28}
``````
随后文档给出了一个简单的 C++ “避障节点”示例代码：

```cpp
ignition::transport::Node node;
std::string topic_pub = "/cmd_vel";
ignition::msgs::Twist data;
auto pub = node.Advertise<ignition::msgs::Twist>(topic_pub);

void cb(const ignition::msgs::LaserScan &_msg)
{
  bool allMore = true;
  for (int i = 0; i < _msg.ranges_size(); i++)
  {
    if (_msg.ranges(i) < 1.0)
    {
      allMore = false;
      break;
    }
  }
  if (allMore) // if all ranges are bigger than 1.0
  {
    data.mutable_linear()->set_x(0.5);
    data.mutable_angular()->set_z(0.0);
  }
  else
  {
    data.mutable_linear()->set_x(0.0);
    data.mutable_angular()->set_z(0.5);
  }
  pub.Publish(data);
}

int main(int argc, char **argv)
{
    std::string topic = "/lidar";
    if (!node.Subscribe(topic, cb))
    {
      std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
      return -1;
    }
    ignition::transport::waitForShutdown();
    return 0;
}
``` :contentReference[oaicite:29]{index=29}

这个节点的逻辑是：  
- 若所有激光射线的距离都大于 1.0 m，则机器人继续前进（线速度 0.5，角速度 0.0）；  
- 否则（发现前方有障碍物且距离 < 1.0 m）则停止前进，原地转动（线速度 0.0，角速度 0.5）以避开障碍。  
这样结合仿真中的 LiDAR 数据，就能实现简单的避障行为。
``````
### 5.4 统一启动（Ignition Launch）  
为了简化操作，不必分别在两个终端运行世界仿真 + 节点，还可以用一个 launch 文件同时启动，如下：

```xml
<?xml version='1.0'?>
<ignition version='1.0'>
  <executable name='sensor-world'>
    <command>ign gazebo sensor_tutorial.sdf</command>
  </executable>
  <executable name='lidar_node'>
    <command>./build/lidar_node</command>
  </executable>
</ignition>
````

保存为 `sensor_launch.ign`，然后运行：

```
ign launch sensor_launch.ign
```

这样即可一键启动仿真世界和避障节点。([Gazebo][1])
启动后在 GUI 点击 Play 按钮，就能看到机器人移动并避开墙壁。([Gazebo][1])

---

## 6. 总结与扩展建议

* 本教程展示了如何在 Gazebo Fortress 中为机器人／模型加入三类常见传感器：IMU、Contact、LiDAR。
* 提供了 SDF 传感器定义、插件定义、订阅节点代码，以及 launch 文件示例。
* 对每种传感器的作用、关键标签（如 `update_rate`, `topic`, `always_on`, `horizontal.samples` 等）都有说明。
* 如果你想更深入的话，可以查看文档中提到的 “完整的 sensors 库”、各种示例 world 以及 ROS2 集成示例。([Gazebo][1])

---

如果你愿意，我可以帮你 **下载**或者 **摘录**该文档里更多 “完整示例 world 文件” 或者 “完整订阅节点源码（含 CMakeLists）”，你看怎么样？

[1]: https://gazebosim.org/docs/fortress/sensors/ "Sensors — Gazebo fortress documentation"
