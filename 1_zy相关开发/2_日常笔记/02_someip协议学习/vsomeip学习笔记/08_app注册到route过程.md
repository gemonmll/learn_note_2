这里其实有两种“注册”，很容易混在一起：

- `注册进 runtime`：只是把 `application` 对象放到本进程里的 `runtime_impl::applications_` 里。
- `注册到 routing manager`：这是通信层面的真正“上线”，成功后才会出现 `ST_REGISTERED`，然后你的 `on_state` 才会被触发。

**先回答你的两个问题**

`client 在哪里注册进入 runtime？`  
在 `vsomeip::runtime::get()->create_application()` 这一步。`runtime_impl::create_application()` 会创建 `application_impl`，然后放进 `applications_` map 里 [runtime_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/runtime_impl.cpp:44)。  
所以严格说，**不是 `init()` 或 `request_service()` 时才进 runtime**，而是 `create_application()` 时就已经被 runtime 持有了。

`runtime 在哪里回调了 on_state？`  
严格说，**不是 runtime 回调的**。真正调用 `on_state` 的是 `routing_manager_client`，它通过 `routing_manager_host` 接口回调 `application_impl::on_state()` [routing_manager_host.hpp](/home/jzm/workspace/vsomeip/implementation/routing/include/routing_manager_host.hpp:37)。  
而 `application_impl` 正好实现了这个接口 [application_impl.hpp](/home/jzm/workspace/vsomeip/implementation/runtime/include/application_impl.hpp:41)。

**整条链路按顺序是这样的**

```text
runtime::get()->create_application()
  -> runtime_impl::create_application()
  -> new application_impl
  -> runtime 保存到 applications_

app->register_state_handler(user_on_state)
  -> application_impl::handler_ = user_on_state

app->init()
  -> 加载配置、拿 client id
  -> new routing_manager_client(this, ...)
     这里 this 就是 application_impl

app->start()
  -> 启动 dispatcher 线程
  -> routing_->start()
  -> 连接 routing manager
  -> register_application(client)

routing_manager_client::register_application()
  -> host_->on_state(ST_REGISTERED)

application_impl::on_state()
  -> 把你的 state handler 丢进 handlers_ 队列
  -> 唤醒 main_dispatch 线程

main_dispatch()
  -> 真正执行你注册的 user_on_state

user_on_state(ST_REGISTERED)
  -> app->request_service(...)
```

**关键代码点**

- `create_application()` 创建并保存 `application_impl`：[runtime_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/runtime_impl.cpp:44)
- `register_state_handler()` 只是把用户回调保存起来：[application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:892)
- `init()` 里创建 `routing_manager_client(this, ...)`，所以后面 routing 可以回调 application：[application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:253)
- `routing_manager_base` 构造时把 `host_` 记下来：[routing_manager_base.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_base.cpp:26)
- `start()` 会启动 dispatcher，并调用 `routing_->start()`：[application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:355)
- routing 连上后会调用 `register_application()`：[routing_manager_client.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_client.cpp:2342)
- 注册成功后触发 `host_->on_state(ST_REGISTERED)`：[routing_manager_client.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_client.cpp:1729)
- `application_impl::on_state()` 不直接调用用户代码，而是入队，交给 dispatcher：[application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:1504)
- dispatcher 线程真正执行 handler：[application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:1739)

**`request_service` 又是怎么走的**

示例里用户回调是这样写的：[request-sample.cpp](/home/jzm/workspace/vsomeip/examples/request-sample.cpp:81)

- `on_state(ST_REGISTERED)` 里调用 `app_->request_service(...)`
- `application_impl::request_service()` 转给 `routing_->request_service(client_, ...)` [application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:608)
- `routing_manager_client::request_service()` 先把请求记到 `requests_`，如果当前已经是 `ST_REGISTERED`，就立刻发 `request_service_command` 给 routing manager [routing_manager_client.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_client.cpp:428)
- 真正发包在 `send_request_services()`：[routing_manager_client.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_client.cpp:1765)
- routing 端收到后，在 `routing_manager_stub` 里解包并继续处理 `host_->request_service(...)`：[routing_manager_stub.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_stub.cpp:460)

还有一个细节：  
`request_service()` 就算早于 `ST_REGISTERED` 调用，也不会丢，它会先缓存，等注册成功后在 `send_pending_commands()` 里补发 [routing_manager_client.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_client.cpp:1949)。

如果你愿意，我下一条可以继续帮你把 `app->start()` 到 `host_->on_state(ST_REGISTERED)` 这段画成更细的“时序图版”，把线程和对象都标出来。


----

## 问题 再哪里区分的server和client，还是在on_state阶段不区分，都是app来处理的

对，**`on_state` 阶段不区分 server 和 client**。  
在 vsomeip 里，先只有一个统一的 `application` 概念；`on_state(ST_REGISTERED)` 表示的只是：

`这个 app 已经成功注册到 routing manager 了，可以开始干活了`

不是：

`这个 app 现在是 server`
或
`这个 app 现在是 client`

**server/client 是在哪里“区分”出来的**

区分不是靠 `on_state`，而是靠你注册完成后做的动作：

- 调 `offer_service(...)`，你就在扮演 service provider / server
- 调 `request_service(...)`，你就在扮演 service consumer / client
- 调 `register_message_handler(...)`，你是在接收请求/消息
- 调 `register_availability_handler(...)`，你是在等服务可用

对应代码很直接：

- `application_impl::offer_service()` 走到 `routing_->offer_service(...)`  
  [application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:598)
- `application_impl::request_service()` 走到 `routing_->request_service(...)`  
  [application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:608)

**所以 server/client 更像“角色”，不是“类”**
同一个 `application` 既可以 request 别人的服务，也可以同时 offer 自己的服务。也就是说，一个 app 可以同时是 client 和 server。

**为什么说 `on_state` 本身不区分角色**

因为 `application_impl::on_state()` 收到的只有一个 `state_type_e`，没有任何 “我是 server 还是 client” 的参数：[application_impl.cpp](/home/jzm/workspace/vsomeip/implementation/runtime/src/application_impl.cpp:1504)

而且真正触发它的是 routing 层：

- 注册成功后 `routing_manager_client` 调 `host_->on_state(ST_REGISTERED)`  
  [routing_manager_client.cpp](/home/jzm/workspace/vsomeip/implementation/routing/src/routing_manager_client.cpp:1739)

这里的 `host_` 实际就是 `application_impl`，不是 runtime 里的某个“client专用对象”或“server专用对象”。

**看示例最直观**

client 示例：
- 在 `on_state(ST_REGISTERED)` 里调用 `request_service(...)`  
  [request-sample.cpp](/home/jzm/workspace/vsomeip/examples/request-sample.cpp:81)

server 示例：
- 在 `on_state(ST_REGISTERED)` 里只是标记“已注册”
- 后面线程被放开后再去 `offer_service(...)`  
  [response-sample.cpp](/home/jzm/workspace/vsomeip/examples/response-sample.cpp:70)  
  [response-sample.cpp](/home/jzm/workspace/vsomeip/examples/response-sample.cpp:101)  
  [response-sample.cpp](/home/jzm/workspace/vsomeip/examples/response-sample.cpp:60)

所以可以把它理解成：

```text
on_state(ST_REGISTERED)
    = “我这个 app 已经连上 routing 体系了”

之后：
    offer_service  -> server 角色
    request_service -> client 角色
```

一句话总结：  
**vsomeip 不在 `on_state` 阶段区分 server/client；`on_state` 只反映 app 的注册状态，真正的 server/client 角色是由 app 后续调用 `offer_service` 还是 `request_service` 决定的。**

如果你愿意，我下一条可以继续把这两个路径画成一张对照时序图：

1. `app->start() -> ST_REGISTERED -> offer_service`
2. `app->start() -> ST_REGISTERED -> request_service`

这样会更不容易绕。