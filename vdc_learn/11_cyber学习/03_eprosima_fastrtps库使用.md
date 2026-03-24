可以。我先把名字统一一下：

**你要学的这个库现在官方主名是 Fast DDS，旧名叫 Fast RTPS。它是 eProsima 做的 C++ DDS 实现。** eProsima 在 2020 年把 Fast RTPS 更名为 Fast DDS；官方文档也明确说它同时提供高层 DDS API 和更底层的 RTPS 层。([eProsima][1])

你如果是为了“学会怎么用”，最省时间的路线不是先啃 RTPS 细节，而是按这个顺序：

**类型定义 → Participant → Topic → Publisher/DataWriter → Subscriber/DataReader → Listener 回调 → QoS/发现/共享内存。**
Fast DDS 官方“simple C++ publisher/subscriber”就是按这条线来教的。([Fast DDS][2])

---

## 1. 先建立一个最小心智模型

Fast DDS 里最常见的对象关系可以记成：

```text
DomainParticipant
 ├─ Topic
 ├─ Publisher
 │   └─ DataWriter
 └─ Subscriber
     └─ DataReader
```

含义是：

* **DomainParticipant**：当前进程在某个 DDS domain 里的入口
* **Topic**：通信主题名 + 类型
* **DataWriter**：发布数据
* **DataReader**：订阅数据

这就是你平时说的 pub/sub，只是名字更 DDS 化。官方 getting started 也是围绕这些实体展开。([Fast DDS][2])

---

## 2. 最小可运行流程

先看发布端最小流程：

1. 创建 `DomainParticipant`
2. 注册数据类型
3. 创建 `Topic`
4. 创建 `Publisher`
5. 创建 `DataWriter`
6. 调用 `write()` 发数据

订阅端对应：

1. 创建 `DomainParticipant`
2. 注册数据类型
3. 创建 `Topic`
4. 创建 `Subscriber`
5. 创建 `DataReader`
6. 用 `Listener` 或 `take()/read()` 收数据

这就是你最需要先掌握的主线。([Fast DDS][2])

---

## 3. 推荐你先学哪套 API

Fast DDS 有两层思路：

* **高层 DDS API**：最适合上手，日常开发最常用
* **更底层 RTPS 层**：更接近协议和发现机制，控制更细，但上手更陡

官方首页明确写了它有“两层 API”：高层 DDS 和低层 RTPS。对你现在这个阶段，我建议**先只学 DDS API**。([Fast DDS][3])

---

## 4. 一个“能看懂结构”的最小 C++ 示例

下面这份不是照抄官方大例子，而是我给你压缩过的**学习版骨架**。你先看懂对象怎么连起来。

### 发布端

```cpp
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

class HelloWorld
{
public:
    uint32_t index = 0;
    std::string message;
};

// 实际项目里这个类型通常由 Fast DDS-Gen 从 IDL 生成
class HelloWorldPubSubType
{
    // 这里只是占位说明，真实代码要继承 TopicDataType
};

int main()
{
    DomainParticipantQos pqos;
    pqos.name("participant_pub");

    DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos);

    // 真实用法：TypeSupport type(new HelloWorldPubSubType());
    // type.register_type(participant);

    Topic* topic = participant->create_topic(
        "HelloTopic",
        "HelloWorld",
        TOPIC_QOS_DEFAULT);

    Publisher* publisher =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

    DataWriter* writer =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, nullptr);

    HelloWorld data;
    data.index = 1;
    data.message = "hello fastdds";

    // writer->write(&data);

    return 0;
}
```

### 订阅端

```cpp
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>

using namespace eprosima::fastdds::dds;

class ReaderListener : public DataReaderListener
{
public:
    void on_data_available(DataReader* reader) override
    {
        // 真实用法：
        // HelloWorld data;
        // SampleInfo info;
        // if (reader->take_next_sample(&data, &info) == ReturnCode_t::RETCODE_OK)
        // {
        //     if (info.valid_data) { ... }
        // }
    }
};

int main()
{
    DomainParticipantQos pqos;
    pqos.name("participant_sub");

    DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, pqos);

    Topic* topic = participant->create_topic(
        "HelloTopic",
        "HelloWorld",
        TOPIC_QOS_DEFAULT);

    Subscriber* subscriber =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

    ReaderListener listener;
    DataReader* reader =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT, &listener);

    while (true) { /* wait */ }
    return 0;
}
```

这个骨架的重点不是直接编译，而是让你先知道：

* **participant 是入口**
* **topic 是名字 + 类型**
* **writer/reader 才是实际收发端**

---

## 5. 真正项目里，类型怎么定义

Fast DDS 里最标准的方式是：

1. 写一个 **IDL**
2. 用 **Fast DDS-Gen** 生成 C++ 类型支持代码
3. 在程序里注册这个类型

官方文档把 Fast DDS-Gen 单独列出来，并说明它可以生成发布订阅应用所需代码。([Fast DDS][4])

例如你会写一个 `HelloWorld.idl`：

```idl
struct HelloWorld
{
    unsigned long index;
    string message;
};
```

然后用生成工具产出：

* 数据结构 C++
* `PubSubType`
* 序列化/反序列化支持代码

这一步很重要，因为 DDS 不是随便扔一个 C++ 对象指针进去，它需要知道**类型描述和序列化规则**。

---

## 6. 你上手时最先该会的 5 个操作

### 第一，会创建 participant

这是一切的入口。

```cpp
DomainParticipant* participant =
    DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
```

这里的 `0` 是 domain id。
同一个 domain 里的实体才更容易自动发现彼此。官方 simple app 也是从这里开始。([Fast DDS][2])

### 第二，会注册类型

没注册类型，topic 建不起来，writer/reader 也没法正常匹配。

### 第三，会创建 topic

Topic 至少有两个关键元素：

* topic 名
* type 名

双方要匹配，通常这两者都得一致。

### 第四，会创建 writer 和 reader

就是实际发和收。

### 第五，会写 Listener

新手最常用的是 `on_data_available()`，因为它最符合“收到消息就回调”的直觉。

---

## 7. 你现在最该先学的 QoS

别一上来把几十种 QoS 全背。先盯 4 个：

* **Reliability**
* **Durability**
* **History**
* **Depth**

这是最影响“为什么匹配不上”“为什么收不到历史数据”“为什么队列丢数据”的几项。Fast DDS 文档把 QoS policies 放在 DDS layer 里系统讲。([Fast DDS][5])

你先这么理解就够了：

* **Reliability**

  * `BEST_EFFORT`：尽力而为
  * `RELIABLE`：更可靠，但控制流更多
* **Durability**

  * 新订阅者能不能看到旧数据
* **History**

  * 保存历史策略
* **Depth**

  * 缓冲多少条

---

## 8. 发现机制怎么理解到“会用”的程度

你前面已经在问 RTPS 发现了。放回 Fast DDS 里，最重要的是知道：

* 默认会做自动发现
* 默认的发现机制叫 **Simple Discovery**
* 它遵循 RTPS 的 PDP/EDP 流程，也就是 participant discovery + endpoint discovery

Fast DDS 官方 discovery 文档就是这么定义的。([Fast DDS][6])

所以你看到“两个程序没连上”，先排查这几项：

* domain id 是否一致
* topic 名是否一致
* type 是否一致
* QoS 是否兼容
* 网络/locator/transport 是否可达

---

## 9. 同机性能：共享内存怎么开

Fast DDS 官方有 **Shared Memory Transport** 文档，而且明确说：
如果你想让 discovery 也通过共享内存，就需要禁用默认 builtin transports，再显式配置共享内存传输。官方还给了 `HelloWorldExampleSharedMem` 例子。([Fast DDS][7])

这意味着：

* 默认情况下，不一定“全都走共享内存”
* 你可以配置成更偏本机高性能通信

如果你之后想研究 Apollo/Cyber 这种“本机快路径”，这一块很值得看。
再往前一步，Fast DDS 还提供 **Zero-Copy / Data-sharing delivery** 场景文档，用来进一步减少拷贝。([Fast DDS][8])

---

## 10. 给你一条最省时间的学习路线

### 第 1 步：先跑官方 simple C++ pub/sub

目标：知道最基本对象怎么连起来。
官方有专门的“Writing a simple C++ publisher and subscriber application”。([Fast DDS][2])

### 第 2 步：学 IDL + Fast DDS-Gen

目标：学会定义自己的消息类型，不再只看 HelloWorld。([Fast DDS][4])

### 第 3 步：只学 4 个核心 QoS

目标：知道为什么匹配失败、为什么消息丢、为什么新 reader 看不到旧数据。([Fast DDS][5])

### 第 4 步：学 Discovery 和 transport

目标：知道为什么同机、跨机、不同网段表现不同。
先看 Simple Discovery，再看 Shared Memory Transport。([Fast DDS][6])

### 第 5 步：再决定要不要深入 RTPS API

目标：只有你真要做协议层、性能调优或中间件开发时才下潜。

---

## 11. 如果你是 Apollo/Cyber 视角来学，最该关注什么

你不用把 Fast DDS 当成“另一个业务框架”，而是把它当：

**一个标准 DDS/RTPS 通信库，用来帮你理解发现、QoS、共享内存、可靠传输这些机制在真实库里怎么落地。**

你最该对照看的点是：

* `Participant` 对应“通信域入口”
* `Writer/Reader` 对应“发布订阅端点”
* `QoS` 对应“匹配和传输语义”
* `Discovery` 对应“服务发现”
* `SHM / Zero-copy` 对应“本机性能路径”

---

## 12. 我建议你接下来这样学

先别一下子啃整套文档。最有效的是这三步：

1. 跑通一个 HelloWorld pub/sub
2. 自己写一个 `idl`，发你定义的结构体
3. 改 `Reliability/History/Depth` 看现象变化

这样你会很快把“库怎么用”和“底层为什么这样设计”连起来。

如果你愿意，我下一条直接给你一套**可编译的最小 Fast DDS C++ 示例**：
包含 `HelloWorld.idl`、生成命令、`publisher.cpp`、`subscriber.cpp`、`CMakeLists.txt`，你可以直接在 Ubuntu 上编。

[1]: https://www.eprosima.com/news/fast-rtps-is-now-fast-dds?utm_source=chatgpt.com "Fast RTPS is now Fast DDS!"
[2]: https://fast-dds.docs.eprosima.com/en/3.x/fastdds/getting_started/simple_app/simple_app.html?utm_source=chatgpt.com "1.3. Writing a simple C++ publisher and subscriber application"
[3]: https://fast-dds.docs.eprosima.com/?utm_source=chatgpt.com "Fast DDS - eProsima"
[4]: https://fast-dds.docs.eprosima.com/en/2.x/notes/notes.html?utm_source=chatgpt.com "Version 2.14.6 (latest) - Fast DDS"
[5]: https://fast-dds.docs.eprosima.com/en/3.x/notes/versions.html?utm_source=chatgpt.com "Dependencies and compatibilities - 3.5.0 - Fast DDS - eProsima"
[6]: https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/dynamic_types/examples.html?utm_source=chatgpt.com "14.9. Dynamic HelloWorld Examples - Fast DDS - eProsima"
[7]: https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/transport/shared_memory/shared_memory.html?utm_source=chatgpt.com "6.4. Shared Memory Transport - Fast DDS - eProsima"
[8]: https://fast-dds.docs.eprosima.com/en/latest/fastdds/use_cases/zero_copy/zero_copy.html?utm_source=chatgpt.com "15.9. Zero-Copy communication - 3.5.0 - Fast DDS - eProsima"

-----

# 附录 eprosima fastrtps fastdds区别
你这里大概率是想问 **Fast RTPS 和 eProsima 是什么关系**。

结论很简单：

**eProsima 是公司/组织名，Fast RTPS 是它做的一个中间件库；后来 Fast RTPS 改名成了 Fast DDS。** eProsima 在 2020 年官方宣布把 **Fast RTPS** 更名为 **Fast DDS**。([eProsima][1])

可以这样对应：

* **eProsima**：厂商/项目方
* **Fast RTPS**：它早期的产品名
* **Fast DDS**：Fast RTPS 的新名字，也是现在官方主叫法

再细一点说，Fast DDS 官方文档把它定义为：
**eProsima Fast DDS 是 DDS 标准的 C++ 实现**，底层使用 **RTPS** 作为通信协议，并提供 API 和传输能力。([Fast DDS][2])

所以你以后看到这些说法，其实基本是一条线上的东西：

* `eProsima Fast RTPS`
* `Fast RTPS`
* `eProsima Fast DDS`
* `Fast DDS`

其中前两个偏旧称呼，后两个是现在的正式叫法。官方旧版文档也明确写过：**Fast RTPS 是 RTPS 协议的 C++ 实现**；而新版文档则转成了 **Fast DDS** 这个名字，并强调它是 DDS 实现。([Fast DDS][3])

你可以把它理解成：

```text
eProsima
  └── Fast RTPS  （旧名）
        └── Fast DDS （新名）
```

顺手纠正一下概念：

* **RTPS**：协议/线协议
* **Fast RTPS / Fast DDS**：实现这个协议并提供 DDS 能力的库
* **eProsima**：做这个库的公司

所以不是“Fast RTPS 和 eProsima”并列两个同类东西，
而是 **Fast RTPS 是 eProsima 的产品**。

放到你现在 Cyber/Apollo 的语境里，也可以这么看：

* Cyber 里提到 RTPS 时，常常是在说一种通信层/发现机制
* 如果底层接了 eProsima 的实现，那通常就是接的 **Fast RTPS / Fast DDS** 这一套

如果你想，我可以继续给你讲：
**Fast RTPS、DDS、RTPS 三个词到底怎么区分**。

[1]: https://www.eprosima.com/news/fast-rtps-is-now-fast-dds?utm_source=chatgpt.com "Fast RTPS is now Fast DDS!"
[2]: https://fast-dds.docs.eprosima.com/?utm_source=chatgpt.com "Fast DDS - eProsima"
[3]: https://fast-dds.docs.eprosima.com/_/downloads/en/v1.9.3/pdf/?utm_source=chatgpt.com "Release 1.9.3 eProsima - FastRTPS Documentation"
