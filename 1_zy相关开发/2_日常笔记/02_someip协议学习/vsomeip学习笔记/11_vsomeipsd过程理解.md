# 第 4 周：事件 / 事件组 / 订阅 / 通知

## 本周目标

这一周的目标不是“会跑 sample”就结束，而是把下面 5 件事说清楚：

1. `offer_event`、`request_event`、`subscribe`、`notify` 分别是谁调用，作用是什么。
2. 为什么 `request_event` 不等于 `subscribe`。
3. 为什么 `ET_FIELD` 在刚订阅后可能立刻收到一个“当前值”。
4. 为什么字段值没变化时，通知可能不会重复发。
5. 订阅链路里 routing 和 SD 分别干了什么。

## 先记住 4 个 API

### 服务端

- `offer_event`
  把某个 event/field 注册到 routing 中，声明“这个服务会发布这个事件”。
- `notify`
  更新事件当前 payload，并把通知发给订阅者。

### 客户端

- `request_event`
  告诉 routing：“我关心这个 event，它属于哪些 eventgroup，它是 field 还是普通 event。”
- `subscribe`
  真正发起对 eventgroup 的订阅。

一句话理解：

`request_event` 是“登记关注对象”，`subscribe` 是“发起订阅动作”。

## 本周主 sample

建议你以这两个文件作为本周阅读入口：

- 服务端: `examples/notify-sample.cpp`
- 客户端: `examples/subscribe-sample.cpp`

关键 ID 在 `examples/sample-ids.hpp` 里：

- service: `0x1234`
- instance: `0x5678`
- event: `0x8778`
- eventgroup: `0x4465`

## 从 sample 看最小闭环

### 1. 服务端先注册 event，再 offer service

`examples/notify-sample.cpp` 里最值得先看的 4 处：

- `43-46`
  `offer_event(...)`，把 `SAMPLE_EVENT_ID` 绑定到 `SAMPLE_EVENTGROUP_ID`，类型是 `ET_FIELD`
- `83-88`
  `offer_service(...)`，真正把服务实例 offer 出去
- `117-127`
  `on_set(...)` 收到请求后更新 payload，并立即 `notify(...)`
- `147-175`
  周期性构造 payload 并 `notify(...)`

这说明：

- `offer_event` 不是网络广播，它更像是“在 routing 中注册本服务持有的事件”
- `offer_service` 才是服务实例可用性的对外声明
- `notify` 才是事件数据真正发生变化后的分发动作

### 2. 客户端先 request_event，再 subscribe

`examples/subscribe-sample.cpp` 里最关键的是：

- `40-43`
  先 `request_event(...)`，再 `subscribe(...)`
- `58-61`
  应用注册成功后 `request_service(...)`
- `69-105`
  `on_message(...)` 统一接收通知，并根据 payload 长度触发 GET/SET 请求

你可以把客户端流程记成：

1. `request_service`
2. 等服务 available
3. `request_event`
4. `subscribe`
5. 在消息回调里接收通知

## API 到实现的主链

### 1. `offer_event` / `request_event` 最终都会走到 `register_event`

入口在 `implementation/runtime/src/application_impl.cpp`：

- `1361-1376`
  `application_impl::offer_event(...)`
- `1384-1389`
  `application_impl::request_event(...)`

这两个 API 最终都会调用：

- `routing_->register_event(...)`

真正干活的是：

- `implementation/routing/src/routing_manager_impl.cpp:3959`

这里会做几件重要的事：

- 创建或更新 `event` 对象
- 记录 event 属于哪些 eventgroup
- 记录 event 类型是 `ET_EVENT` 还是 `ET_FIELD`
- 解析可靠性配置
- 建立 event 与 service/instance 的关联

所以第 4 周一个很重要的认知是：

`request_event` 和 `offer_event` 本质上都是“注册 event 元数据”，只是一个站在消费者角度，一个站在提供者角度。

### 2. `subscribe` 是另一条链，不等同于 `request_event`

订阅入口在：

- `implementation/routing/src/routing_manager_impl.cpp:513`

这里能直接看到一个很重要的事实：

- 如果没有 `discovery_`，会报
  `SOME/IP eventgroups require SD to be enabled!`

也就是说，在 vSomeIP 里，eventgroup 的订阅链路依赖 SD 组件。

这也是你本周必须回答的问题之一：

为什么本地 request/response 可以不靠 SD，但事件订阅链路仍然会依赖 SD 组件？

### 3. 服务端可通过 subscription handler 决定接不接受订阅

服务端注册订阅回调的典型例子在：

- `test/network_tests/subscribe_notify_tests/subscribe_notify_test_service.cpp:80-82`

回调分发入口在：

- `implementation/runtime/src/application_impl.cpp:1023-1054`

这里的逻辑很直接：

- 如果应用注册了 subscription handler，就调用它
- handler 返回 `true`，表示接受订阅
- handler 返回 `false`，表示拒绝订阅
- 如果没注册 handler，默认接受

所以服务端不是只能“被动接受订阅”，它可以基于 client、security 信息、环境等做准入控制。

### 4. 为什么刚订阅就可能收到当前值

这条链非常值得你重点看：

- `routing_manager_impl.cpp:539`
  `notify_one_current_value(...)`
- `routing_manager_impl.cpp:4402-4418`
  只对 `field` 做“当前值推送”
- `event.cpp:351-365`
  `notify_one(...)` 真正单播当前值

结论：

- 只有 `ET_FIELD` 才有“当前值”这个语义
- 新订阅者成功订阅后，可能立即收到 field 的当前 payload
- 如果当前值还没被服务端 set 过，日志会提示“不会收到 initial notification”

这正是 field 和普通 event 的核心区别之一。

## `notify` 真正发生了什么

### 1. 应用层只是把 payload 交给 routing

入口：

- `implementation/runtime/src/application_impl.cpp:875-889`

`application_impl::notify(...)` 和 `notify_one(...)` 做的事情很简单：

- 复制 payload
- 调用 `routing_->notify(...)` 或 `routing_->notify_one(...)`

### 2. routing 中的 event 对象负责缓存和分发

在 client 侧 routing 实现里：

- `implementation/routing/src/routing_manager_client.cpp:3029-3039`

这里会找到对应的 `event` 对象，然后调用：

- `event->set_payload(...)`

真正值得精读的是 `implementation/routing/src/event.cpp`：

- `138-188`
  `set_payload(...)`
- `312-321`
  `notify(...)`
- `351-365`
  `notify_one(...)`
- `374-389`
  `prepare_update_payload_unlocked(...)`

这里能看出两个关键行为：

1. `ET_FIELD` 且 `cycle == 0` 时，如果 payload 没变化，默认不会重复发
2. 第一次成功设置 payload 后，field 才算“有当前值”

尤其是：

- `event.cpp:376-378`

这几行是理解“字段值没变化为什么不重发”的核心。

## 本周必须搞懂的 4 个概念

### 1. event 和 eventgroup 不是一回事

- event 是具体的一条通知数据
- eventgroup 是订阅粒度

也就是说，客户端不是直接“订阅某个 event”，而是订阅某个 eventgroup。
是否只接收特定 event，则是 subscribe 时额外传入 `_event` 参数做过滤。

### 2. `request_event` 和 `subscribe` 不是重复动作

- `request_event` 解决“我关心谁、它是什么类型、属于哪些 group”
- `subscribe` 解决“我现在要订阅哪个 group”

只 `request_event` 不 `subscribe`，你通常收不到通知。

### 3. `ET_EVENT` 和 `ET_FIELD` 的语义不同

- `ET_EVENT`
  更像一次次独立事件
- `ET_FIELD`
  更像“带当前值的状态量”

`ET_FIELD` 具备两个很重要的特征：

- 新订阅者可能收到当前值
- 值没变化时可以不重复推送

### 4. subscription handler 是服务端治理点

它不是“消息处理回调”，而是“订阅准入回调”。

如果以后你要做：

- 白名单订阅
- 条件拒绝订阅
- 按 client 区分订阅权限

入口基本都在这一层。

## 本周推荐阅读顺序

按下面顺序读，效率会比较高：

1. `examples/sample-ids.hpp`
2. `examples/notify-sample.cpp`
3. `examples/subscribe-sample.cpp`
4. `interface/vsomeip/application.hpp`
   重点看 `offer_event`、`request_event`、`subscribe`、`notify`
5. `implementation/runtime/src/application_impl.cpp`
6. `implementation/routing/src/routing_manager_impl.cpp`
   重点看 `subscribe`、`register_event`、`notify_one_current_value`
7. `implementation/routing/src/event.cpp`
8. `test/network_tests/subscribe_notify_tests/subscribe_notify_test_service.cpp`

## 本周练习

### 练习 1：先只做静态阅读

请你自己回答下面 5 个问题：

1. 为什么服务端既要 `offer_event`，又要 `offer_service`？
2. 为什么客户端既要 `request_event`，又要 `subscribe`？
3. 为什么 field 订阅成功后可能立刻收到一条通知？
4. 为什么有些 `notify` 不会真的发到订阅者？
5. 为什么订阅链路里能看到 SD 的参与？

### 练习 2：跑 sample 观察现象

如果你在自己的终端里运行，可以用现成二进制：

```bash
cd /home/jzm/workspace/vsomeip/build/examples

# 终端 1
env VSOMEIP_CONFIGURATION=../../config/vsomeip-local.json \
VSOMEIP_APPLICATION_NAME=service-sample \
./notify-sample

# 终端 2
env VSOMEIP_CONFIGURATION=../../config/vsomeip-local.json \
VSOMEIP_APPLICATION_NAME=client-sample \
./subscribe-sample
```

观察重点：

- 服务端会周期性 `notify`
- 客户端会打印 notification 的 payload
- 服务端每 10 秒会 `offer / stop_offer` 切换一次
- 客户端可观察 availability 的变化

说明：

我在当前 Codex 沙箱里验证了命令路径是对的，但实际运行会被沙箱限制住本地 routing socket，所以这里的“跑起来”请在你自己的终端里完成。

### 练习 3：做两个最小改动

建议你自己动手改这两个点：

1. 把 `notify-sample.cpp` 里的 `ET_FIELD` 改成 `ET_EVENT`
   然后观察新订阅者是否还会收到“当前值”语义
2. 把周期通知的 payload 改成固定不变
   然后观察 field 语义下是否还会持续转发

你改完之后，再回头对照 `event.cpp:374-389`，理解会非常快。

## 本周补充观察

### 关于 event ID 的写法

仓库里你会同时看到：

- 代码里用 `0x8778`
- 配置里用 `0x0778` 或 `0x778`

从 `interface/vsomeip/vsomeip_sec.h:117-120` 可以确认：

- SOME/IP 内部把 member id 小于 `0x8000` 的看作 method
- 大于等于 `0x8000` 的看作 event

但这个仓库里的 sample 和不同测试配置，写法并不完全统一。
所以你在阅读具体用例时，最稳妥的方式不是死记规则，而是：

- 先看 sample 代码里实际传给 `offer_event/request_event/notify` 的 ID
- 再看该用例对应配置和测试是否使用同一套写法

这个点先知道“有坑”就够了，第 4 周不建议在这里花太久。

## 本周结束时，你应该能做到

如果这一周学完，你应该可以不看资料，自己讲清楚下面这段话：

“服务端先用 `offer_event` 在 routing 中注册 event，再用 `offer_service` 对外提供服务。客户端先 `request_event` 声明自己关心哪些 event，再 `subscribe` 某个 eventgroup。订阅成功后，如果这是 `ET_FIELD` 且当前值已经设置过，新订阅者可能立刻收到一条当前值通知。后续服务端 `notify` 时，routing 会根据 event 类型、payload 是否变化、订阅关系和目标端点决定是否真正分发通知。” 

如果你愿意，第 5 周我建议我们继续学：

- `subscription_status_handler`
- selective event 和 `notify_one`
- multicast eventgroup
- debounce/filter
