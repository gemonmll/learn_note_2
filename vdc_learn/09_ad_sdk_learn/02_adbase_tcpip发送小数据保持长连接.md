你把 `TcpFrameServer` 的实现贴出来后，3 个 test 里那些“睡 1s / 6s / 8s”、“断开后第一次 Send 不判断、过几秒后 Send 才 false”的现象就很容易解释了：**这套实现有后台线程 + epoll + keepalive/ack + 自己维护的连接集合 `gfd_set_`**，而且**server 的 Disconnect 只影响 server，不会立刻让 client 端马上感知**（TCP 的典型特性）。

下面我用“实现细节 → test 行为”的方式把关键点讲清楚。

---

## 先抓住这个 Server 的核心机制

### 1) `TryBind()` 会启动两个线程

* `bg_thread_`：跑 `BackgroundThread()`，负责 `accept/recv/解析 frame/入队`
* `ack_thread_`：跑 `AckThread()`，周期性给 `ack_fd_set_` 里的连接发 keepalive 前缀（`kTcpKeepAlivePrefix`），间隔 `kAckIntervalMs`

所以 server 是“异步”的：你在测试里 `Send` 之后，server 要等后台线程调度、epoll 唤醒、recv 到数据并解析完，`Recv()` 才能拿到 frame。

👉 这就是 test 里多处 `sleep_for(1s)` 的主要原因：**给后台线程时间 accept/recv/解析/入队**。

---

### 2) `BackgroundThread()` 用 epoll 管理所有 fd

它把：

* 监听 fd：`sockfd_`
* 中断 fd：`intrfb_`（eventfd）
* 每个 client 的 cfd

都加进 epoll。

`Disconnect()` 时会：

* `running_ = false`
* `eventfd_write(intrfb_, 1)` 让 epoll 立刻醒来并退出线程
* close `sockfd_` / `epfd_`
* join 两个线程
* `valid_ = false`

---

### 3) Server 维护了一个“当前在线连接集合” `gfd_set_`

* accept 成功：`gfd_set_.emplace(cfd)`
* client 断开（recv 返回 0）时：`gfd_set_.erase(cfd)`
* `Send()` 时会先检查：`id` 是否在 `gfd_set_`，不在就直接 `return false`

所以对 server 来说，“连接是否有效”的判断主要靠 **`gfd_set_`**。

---

## 现在回到 3 个 test：差异为什么会出现？

### A. `TcpFrameSocketTestBasic`：断开后 Recv 超时失败

断开后它做了两段验证：

1. `server.Disconnect(); EXPECT_FALSE(server.IsValid());`

* 你这里 `Disconnect()` 会 join 线程并把 `valid_ = false`，所以立刻成立。

2. `EXPECT_FALSE(client.Recv(&frame, 100ms));`

* server 已经关闭并且不再发送任何东西，client `Recv` 自然超时返回 false。
* 它验证的是：**client 的 Recv 不会卡死，而且断开后不会凭空收到数据**。

它后面还有：

```cpp
client.Send(...); // 不检查返回值
EXPECT_FALSE(client.Recv(..., 100ms));
```

这里不检查 `Send` 返回值很常见：因为 TCP 断开后“第一次 send 是否立刻失败”在不同实现/时机下不稳定（看 client 的实现、是否有缓存、是否立刻感知到 RST/FIN 等）。所以 Basic 用更稳定的断言：**收不到**。

---

### B. `TcpFrameSocketTestReconnected`：重点在“最终 send 必须失败 + server 重启后可恢复”

这个 test 的关键断言是：

```cpp
client.Send(...);          // 断开后不久，没断言
sleep(8s);
EXPECT_FALSE(client.Send(...));  // 过一段时间，必须明确失败
```

为什么“过 8 秒才必须失败”？

* server 的 `Disconnect()` 只是 server 本地关掉 socket。
* client 端不一定立刻知道（TCP 经典问题：没有读写就不一定发现对端已死）。
* 所以很多 client 实现会：

  * 断开后仍尝试发送（内核可能先接受写入，随后在某次重传/探测后才报错）
  * 或者内部有重连/重试窗口，短时间内 `Send` 仍返回 true/或未立即失败
* 但测试希望验证：**超过某个超时窗口（这里 8s）后，client 必须明确认为连接不可用，Send 返回 false**。

然后它让 server 再次 `TryBind()`（模拟“server 重新起来”）：

```cpp
EXPECT_TRUE(server.TryBind());
sleep(2s);
EXPECT_TRUE(client.Send(send_msg2...));
sleep(1s);
EXPECT_TRUE(server.Recv(&frame));
```

这里的断言体现的是：**client 具备“恢复连接/自动重连/或无需显式重连也能重新建立发送通路”的能力**（具体是哪一种取决于 `TcpFrameClient` 代码，但 test 语义是这个）。

最后：

```cpp
EXPECT_FALSE(client.Recv(&frame));
```

只是因为 server 这次没回包，所以 client 收不到。

---

### C. `TcpFrameSocketTestServerLatestUp`：重点在“server 晚上线”

这条是“启动顺序”测试：

1. 一开始只有 client，没有 server：

* `TryConnect()` 必须失败
* `Send()` 必须失败
* `IsValid()` 必须 false

2. 后面 server 上线 `TryBind()` 后：

* client 再 `TryConnect()` 能成功
* 发送能成功
* server 能收到

最后又做了“断开后最终 send 失败”的验证（睡 6 秒）

👉 这测试的是：**client 能处理 server 迟到（先失败，后成功）**。

---

## 为什么要 sleep 1s/2s/6s/8s？

结合你 server 的实现，sleep 的作用主要是：

1. **让后台线程完成 accept/epoll 注册**
   `TryBind()` 返回时线程刚启动，epoll 还没跑到 accept 那一步。马上 `client.TryConnect()` 有时会 race（尤其 CI/负载高的时候）。所以 sleep(1s)。

2. **让数据从 kernel 收到并被解析成 frame 入队**
   server 的收包解析在 `BackgroundThread()` 做，`Recv()` 只是从队列取。所以要 sleep 给它入队时间。

3. **等待 client 的“连接失效判定窗口”**
   6s/8s 很像 client 侧内部的 keepalive/重连/超时阈值（你 server 这边的 ack interval 也暗示系统里有周期性探测机制）。
   test 用“过窗口后必须失败”来避免 TCP 行为不确定导致的 flaky。

---

## 额外：你这段 Server 代码里有一个明显 bug（会导致偶发不稳定）

在 `accept` 这里：

```cpp
socklen_t addr_size;
int cfd = accept(sockfd_, (sockaddr*)&cli_addr, &addr_size);
```

`addr_size` **没有初始化**。按 POSIX 约定，调用前应该：

```cpp
socklen_t addr_size = sizeof(cli_addr);
```

否则 accept 可能返回 EINVAL 或导致地址长度异常（不一定每次都炸，但会引入不稳定）。

这类问题在 CI 上会让“sleep 多一点”看起来像修好了，但本质是 bug。

---

## 如果你把 TcpFrameClient 也贴出来

我可以把 6s/8s 的“窗口”精确对齐到它的：

* keepalive 参数（`TCP_KEEPIDLE/KEEPINTVL/KEEPCNT`）
* 内部重连间隔/超时
* Send 返回 true/false 的具体条件（比如是否基于 socket fd、是否异步写队列、是否检测 EPIPE/ECONNRESET）

这样你就能知道 test 里的这些时间是“拍脑袋”还是“和实现强绑定”。

你要不要把 `TcpFrameClient::Send/TryConnect/Recv/IsValid` 那个文件也贴一下？我可以顺便帮你判断这些 test 有没有潜在 flaky 点，以及怎么改得更稳（不用大量 sleep）。

-------

这个问题问得非常专业 👍
答案是：

> ✅ **两边都可以**
> 🔥 但在工程上通常“谁更容易感知异常，谁负责发心跳”

我们系统讲清楚。

---

# 一、理论上：谁发都可以

TCP 是双向的。

只要**任意一方周期性发送一点数据**，连接就会：

* 保持活跃
* 不会被 NAT 干掉
* 能尽早检测对端是否挂掉

所以：

| 方式       | 是否可行             |
| -------- | ---------------- |
| server 发 | ✅                |
| client 发 | ✅                |
| 双方都发     | ✅                |
| 都不发      | ❌（长时间 idle 可能被断） |

---

# 二、工程上怎么选？

关键问题是：

> 谁更关心“连接是否存活”？

---

## 🧠 场景分析

---

### 场景 1️⃣：client 是主动请求方

例如：

* client 采集数据
* client 定期上报
* server 只是接收

👉 这种场景通常 **client 发心跳**

因为：

* client 更关心 server 是否在线
* client 可以决定是否重连
* server 可能连接很多 client，不想频繁发

---

### 场景 2️⃣：server 主动推送

例如：

* server 是消息中心
* server 给 client 推送通知
* client 只是接收

👉 这种场景通常 **server 发心跳**

因为：

* server 更关心 client 是否还活着
* server 要维护在线列表

---

### 场景 3️⃣：长连接网关 / NAT 后面

👉 双方都发更安全

因为：

* 中间 NAT 可能 idle timeout
* 你无法控制网络设备行为

---

# 三、回到你这套代码

你现在的实现是：

```cpp
TcpFrameServer 有 AckThread()
TcpFrameClient 未知
```

所以现在是：

> 🔵 server 负责发 keepalive

为什么这么设计？

从代码看：

```cpp
ack_fd_set_.emplace(cfd);
```

只有在 server 收到过数据的连接，才会加入心跳列表。

说明设计意图是：

> 谁曾经发过数据，server 才维护它的连接状态。

这是一个“server 控制连接有效性”的架构。

---

# 四、TCP keepalive 和 应用层心跳的区别

---

## 1️⃣ TCP Keepalive（内核级）

通过：

```cpp
setsockopt(..., SO_KEEPALIVE)
```

优点：

* 自动
* 不需要写代码

缺点：

* 默认超时时间很长（2小时）
* 粒度不可控（除非手动调参数）

---

## 2️⃣ 应用层心跳（你现在的）

就是：

```cpp
send(fd, kTcpKeepAlivePrefix)
```

优点：

* 可控频率
* 可自定义协议
* 更快检测异常

缺点：

* 需要自己写代码

---

# 五、到底推荐谁发？

### 工业建议：

| 系统类型      | 推荐       |
| --------- | -------- |
| 简单系统      | client 发 |
| 推送型系统     | server 发 |
| 大规模分布式    | 双向发      |
| 车载/自动驾驶系统 | 双向发（更稳）  |

---

# 六、如果只选一个，我建议？

👉 一般建议 **client 发**

原因：

1. client 更主动
2. client 更关心重连
3. server 连接很多 client 时压力小
4. 易扩展

---

# 七、为什么你现在代码选 server 发？

可能因为：

* server 要管理在线连接池
* server 需要知道 client 是否掉线
* client 端实现更简单

---

# 八、核心原则

不是“谁必须发”，而是：

> 谁更需要及时知道对方死了，谁发。

---

# 九、再深入一点（你现在这个 test 现象）

你看到：

```cpp
sleep(6s);
EXPECT_FALSE(client.Send(...));
```

说明：

* client 没立即发现 server 断开
* 过了一段时间才检测到

如果改成 client 主动发心跳：

* client 会更早发现 server 挂掉
* test 里可能不需要 sleep 6s

---

# 十、总结一句话

```text
TCP 不会主动告诉你“对方死了”
必须有人定期发数据，错误才会暴露
```

---

如果你愿意，我可以给你画一个：

* 双向心跳 vs 单向心跳 时序图
* NAT 超时 + 心跳恢复图
* 为什么 send 不会立刻失败的内核机制图

你想看哪个？
