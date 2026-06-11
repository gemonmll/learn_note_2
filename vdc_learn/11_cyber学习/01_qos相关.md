在 Apollo Cyber 里，**QoS 不是一个独立“网络协议”层**，而是挂在 `RoleAttributes` 上的一组传输语义配置。创建 Writer/Reader 时，如果你没显式设置 QoS，`Transport::CreateTransmitter/CreateReceiver` 会自动把 `QOS_PROFILE_DEFAULT` 填进去；然后再按 `OptionalMode` 选择走 `INTRA`、`SHM`、`RTPS` 或 `HYBRID` 这几种传输路径。也就是说，**QoS 是“怎么发、怎么缓存、怎么匹配”的规则，RTPS 是其中跨进程/跨机通信时可用的一种底层传输实现**。([GitHub][1])

更具体一点，Cyber 里的 QoS 配置至少包含这些典型字段：`history`、`depth`、`mps`、`reliability`、`durability`。从 Apollo 的相关 issue 和代码片段能看到，`QosProfileConf::CreateQosProfile(...)` 就是在组装这些字段；另外某些特殊话题会用专门的 QoS，比如 `tf_static` 使用 `QOS_PROFILE_TF_STATIC`，说明 Cyber 的 QoS 是“按 channel/role 配置的”，而不是全局唯一策略。([GitHub][2])

你可以把它理解成：

* `history`：保存历史消息的策略，常见是只保留最近若干条。
* `depth`：队列深度，和 `history=keep_last` 这类策略配合使用。
* `reliability`：更偏“尽力而为”还是“可靠投递”。
* `durability`：新加入的订阅者能不能收到之前保留的数据。
* `mps`：Apollo 自己这里的一个消息相关限制项，通常和单条消息大小/传输约束有关；它不是 DDS 标准里最常见的核心四项，但在 Cyber 的 QoS 结构里存在。([GitHub][2])

从实现路径看，Cyber 的发送接收层大概是这样串起来的：

1. `Node::CreateWriter/CreateReader`
2. 组装 `RoleAttributes`
3. 附带 `qos_profile`
4. 进入 `Transport`
5. 按模式创建 `Intra/SHM/Rtps/Hybrid` 的 transmitter/receiver
6. 最终由对应 dispatcher/participant 去完成数据分发和发现。([GitHub][1])

所以你问“**cyber 中的 QoS 是如何实现的**”，答案可以概括为：

**它本质上是通过 `RoleAttributes.qos_profile` 驱动传输层行为，在创建 Reader/Writer 时下沉到具体的 transmitter/receiver 实现中；本机进程内可走 INTRA，本机跨进程常走 SHM，跨机或统一发现机制会走 RTPS，而 HYBRID 则是把几种机制组合起来。** ([GitHub][1])

至于你问“**底层 RTPS 是什么协议**”：

RTPS 指的是 **Real-Time Publish-Subscribe**，更正式的名字是 **DDSI-RTPS**，是 OMG 为 DDS 定义的**互操作线协议**。它的作用是让不同实现的 DDS 系统在网络上传递发布/订阅数据、做 participant/discovery、reader/writer 匹配等。OMG 官方文档明确写了：DDSI-RTPS 是 DDS 的 interoperability wire protocol。([omg.org][3])

结合 Apollo 来说，Cyber RT 的 RTPS 层和 eProsima Fast RTPS/Fast DDS 关系很深。Apollo 官方仓库里的 issue 多次直接提到 **Cyber RT depends on fastrtps**，也能看到代码/报错里出现 `RTPSParticipant`、`createParticipant` 等对象和调用，这说明 Cyber 的 RTPS 通信不是自己从 UDP 裸写一套，而是建立在 RTPS 中间件实现之上的。([GitHub][4])

你可以把两者关系记成一句话：

**QoS 是策略，RTPS 是线协议；Cyber RT 用 QoS 决定消息语义，用 RTPS 负责远程节点发现与网络传输。**

再通俗一点：

* QoS 决定“这条消息丢一两条行不行、要不要缓存、后来者能不能补拿”
* RTPS 决定“节点怎么在网上互相发现、包怎么编码、怎么发出去、怎么匹配到对端”

如果你愿意，我下一条可以直接按 Apollo 源码给你画一张“`CreateWriter -> Transport -> RtpsTransmitter/HybridTransmitter -> Participant`”的数据流图，并顺手解释 `reliability/durability/depth` 在 Apollo 里通常各自会带来什么现象。

[1]: https://github.com/ApolloAuto/apollo/blob/master/cyber/transport/transport.h "apollo/cyber/transport/transport.h at master · ApolloAuto/apollo · GitHub"
[2]: https://github.com/ApolloAuto/apollo/issues/13331?utm_source=chatgpt.com "Not compatible with protobuf3 · Issue #13331 · ApolloAuto/ ..."
[3]: https://www.omg.org/spec/DDSI-RTPS/2.2/ "About the DDS Interoperability Wire Protocol Specification Version 2.2"
[4]: https://github.com/ApolloAuto/apollo/issues/8289 "Compile apollo Cyber RT in aarch64 platform, need fastrtps modification · Issue #8289 · ApolloAuto/apollo · GitHub"


---------