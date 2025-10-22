
---
# 捕获变量的生命周期

---

非常好的问题👏，你这句问得非常关键，说明你已经注意到了 **多线程与对象生命周期** 的核心问题。

我们来完整解释下：

> “在主线程捕获的 `node`，如何保证在子线程中运行过程中不会被销毁？”

---

## 🧩 一、`node` 是什么类型？

在 ROS 2 C++ 中，创建节点通常是这样的：

```cpp
auto node = rclcpp::Node::make_shared("my_node");
```

这个函数返回类型是：

```cpp
std::shared_ptr<rclcpp::Node>
```

也就是说，`node` 是一个 **智能指针（`shared_ptr`）**，
用于自动管理 `rclcpp::Node` 对象的生命周期。

---

## 🧠 二、shared_ptr 的核心机制

`std::shared_ptr` 的本质是一个 **引用计数智能指针**。
它记录有多少个地方持有该对象的“共享所有权”。

例如：

```cpp
auto a = std::make_shared<int>(10);
auto b = a;  // 引用计数 +1
```

当最后一个 `shared_ptr` 销毁时，对象才会释放。

---

## 🧵 三、在你的例子中，生命周期是这样的：

代码片段：

```cpp
std::promise<void> stop_async_spinner;
std::thread async_spinner_thread(
  [stop_token = stop_async_spinner.get_future(), node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      executor.spin_until_future_complete(stop_token);
  });
```

我们重点看 **捕获 `[ ..., node ]` 这一部分**：

当你写下：

```cpp
[node]
```

相当于 **按值捕获** `node`。

⚙️ 即：

* 主线程中有一个 `std::shared_ptr<rclcpp::Node> node`
* 捕获时，会**拷贝一份** `shared_ptr`（而不是拷贝整个 Node）
* 所以引用计数 +1

---

## 🔁 四、生命周期的具体过程

1️⃣ 主线程创建节点：

```cpp
auto node = rclcpp::Node::make_shared("my_node");
```

此时引用计数为 `1`。

2️⃣ 启动子线程并捕获：

```cpp
std::thread t([node]() { ... });
```

此时捕获的 `node`（副本）使引用计数变为 `2`。

3️⃣ 主线程可能继续做事，甚至销毁原始变量：

```cpp
node.reset();  // 引用计数 -1，但对象仍然活着
```

只要子线程还持有 `node`，对象仍然存在。

4️⃣ 子线程结束后，捕获的 `node` 离开作用域：

```cpp
t.join();  // 子线程退出
```

子线程中的 `node` 被销毁，引用计数 -1，变为 0 → 对象析构。

✅ **所以，只要子线程的 lambda 捕获了 node 的 shared_ptr，
Node 对象的生命周期一定能延续到线程结束。**

---

## 🔒 五、如果你捕获的是裸指针，会出事

比如这样：

```cpp
auto node = rclcpp::Node::make_shared("my_node");
std::thread t([node_ptr = node.get()]() {
    node_ptr->do_something();
});
```

这种捕获方式是危险的：

* 捕获的是裸指针；
* 如果主线程 `node.reset()` 提前释放；
* 子线程再访问 `node_ptr` 就是 **悬垂指针（dangling pointer）**。

💀→ 典型的多线程崩溃场景。

---

## 🧱 六、正确使用 shared_ptr 的理由

| 捕获方式                  | 生命周期安全性  | 说明                    |
| --------------------- | -------- | --------------------- |
| `[node]`              | ✅ 安全     | 捕获 shared_ptr 的副本     |
| `[&node]`             | ⚠️ 可能不安全 | 捕获引用，若主线程 reset() 就悬空 |
| `[node = node.get()]` | ❌ 不安全    | 捕获裸指针，可能悬空            |

---

## ✅ 七、总结一句话

> `node` 是一个 `std::shared_ptr`，lambda 捕获它时会复制这个智能指针，从而增加引用计数，保证节点对象在子线程运行期间不会被销毁。

---

## 🔍 延伸（额外技巧）

有时候我们会显式使用 `std::weak_ptr` 来避免循环引用，比如：

```cpp
std::weak_ptr<rclcpp::Node> weak_node = node;
std::thread t([weak_node]() {
    if (auto n = weak_node.lock()) {
        // node 还活着
        n->get_logger();
    } else {
        // node 已销毁
    }
});
```

这种方式适合长期后台线程，防止 node 永远不被销毁。

---

是否希望我帮你画一张图（引用计数随捕获/释放变化的流程图），让这个生命周期更清楚？

---
# 样例学习

非常好的问题！👏
你这一段代码其实展示了 **ROS 2 异步 spin 模式** 的高级写法 ——
让一个独立线程去 spin（执行回调），而主线程可以自由做别的事。

我们来逐行深度解析这一段 👇

---

## 🔹完整代码

```cpp
std::promise<void> stop_async_spinner;
std::thread async_spinner_thread(
  [stop_token = stop_async_spinner.get_future(), node]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin_until_future_complete(stop_token);
  });
```

---

## 🧩 1️⃣ `std::promise<void>` 与 `std::future<void>`

这两个是 **C++ 标准库的同步通信工具**。
可以理解成一个「线程间的信号机制」：

| 对象                   | 谁持有 | 作用             |
| -------------------- | --- | -------------- |
| `std::promise<void>` | 主线程 | 未来某个时刻可以“发出信号” |
| `std::future<void>`  | 子线程 | 可以等待信号被触发      |

调用关系：

```cpp
auto fut = promise.get_future();  // 获取关联的 future
// 之后在别的线程：
promise.set_value();  // 发信号
// 对应的 future 会“完成”（即 ready）
```

在这里，这两个对象被用来**优雅地停止 spin 循环**。

---

## 🧩 2️⃣ lambda 捕获部分

```cpp
[stop_token = stop_async_spinner.get_future(), node]()
```

🔸 捕获了：

* 一个 `stop_token`（也就是 `std::future<void>`）
* 一个 `node`（共享指针）

> 注意这是**按值捕获**（拷贝进入 lambda 中），
> 确保线程内部有自己的副本，不受外部作用域影响。

---

## 🧩 3️⃣ `rclcpp::executors::SingleThreadedExecutor executor;`

在 ROS 2 中，`Executor` 是负责执行回调的核心机制。
`SingleThreadedExecutor` 表示：

* 所有回调都在一个线程中顺序执行。
* 一般用于简单情况或调试（复杂应用中也有 `MultiThreadedExecutor`）。

---

## 🧩 4️⃣ `executor.add_node(node);`

把当前节点加入到这个 executor 的管理列表中。

换句话说：
「这个 executor 负责监控 node 的定时器、订阅者、服务等事件。」

---

## 🧩 5️⃣ `executor.spin_until_future_complete(stop_token);`

这行非常关键 👇

### 🧠 普通 `spin()` 是：

```cpp
executor.spin();
```

➡️ 会**一直循环等待事件**，直到节点被销毁或被中断（Ctrl+C）。

### 🧠 而 `spin_until_future_complete(future)` 是：

```cpp
executor.spin_until_future_complete(future);
```

➡️ 它会**持续处理回调事件**，
直到这个 `future` 对象被“标记为完成”（`promise.set_value()` 被调用）。

等价逻辑如下：

```cpp
while (!future.ready()) {
  执行所有待处理回调;
}
```

因此，你可以在主线程中这样停止异步 spin：

```cpp
stop_async_spinner.set_value();  // 通知 future 完成
async_spinner_thread.join();     // 等待线程退出
```

这就比直接调用 `executor.cancel()` 更安全、更优雅。

---

## 🧩 6️⃣ `std::thread async_spinner_thread(...)`

这一行创建了一个新的线程。
里面执行的 lambda 逻辑是：

> “在这个线程里创建 executor，运行 node 的所有回调事件，直到接到 stop 信号。”

这样，**spin 不会阻塞主线程**。
主线程就可以自由执行其它逻辑（例如等待输入、发布命令、调用 service 等）。

---

## ⚙️ 举个完整的例子

```cpp
auto node = std::make_shared<MyNode>();
std::promise<void> stop_async_spinner;

// 启动异步 spin
std::thread async_spinner_thread(
  [stop_token = stop_async_spinner.get_future(), node]() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin_until_future_complete(stop_token);
  });

// 主线程可以做自己的事
std::this_thread::sleep_for(std::chrono::seconds(5));
RCLCPP_INFO(node->get_logger(), "Stopping spin...");

// 通知异步线程退出
stop_async_spinner.set_value();
async_spinner_thread.join();
RCLCPP_INFO(node->get_logger(), "Stopped cleanly");
```

✅ 输出：

```
[INFO] [main]: Stopping spin...
[INFO] [main]: Stopped cleanly
```

---

## 🧩 总结

| 概念                             | 含义                               |
| ------------------------------ | -------------------------------- |
| `std::promise` / `std::future` | 线程间的信号机制                         |
| `executor`                     | ROS 2 回调调度器                      |
| `spin_until_future_complete()` | 一直 spin，直到指定的 future 完成          |
| 异步 spin 的好处                    | 主线程不会被阻塞，可以自由执行逻辑                |
| 停止方法                           | `stop_async_spinner.set_value()` |

---

要不要我帮你写一个完整的「异步 spin + 服务调用 + 优雅退出」的可调试示例？
可以直接放进你的 VSCode 环境里调试。

