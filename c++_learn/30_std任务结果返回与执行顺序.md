非常好的问题 💡，这就进入线程池与 **数据返回（任务结果传递）** 的关键部分——
在 C++ 中，线程执行完任务后如何把结果“带回来”？
这就要用到 **`std::future`** 和 **`std::promise`**，这也是线程池中最经典的一环。

我们分两步讲清楚：
1️⃣ 普通线程怎么返回结果。
2️⃣ 在线程池中怎么实现任务结果传递（用 `std::future` 包装任务）。

---

## 🧩 一、普通线程无法直接返回结果

举个例子：

```cpp
int add(int a, int b) { return a + b; }

int main() {
    std::thread t(add, 1, 2);  // ❌ 线程不能直接返回结果
    t.join();
}
```

线程执行完后你拿不到返回值。

---

## ✅ 二、解决方案：`std::promise` + `std::future`

`std::promise` 负责“设置值”，
`std::future` 负责“取值”。

你可以这样做：

```cpp
#include <iostream>
#include <thread>
#include <future>

int add(int a, int b) {
    return a + b;
}

int main() {
    std::promise<int> prom;
    std::future<int> fut = prom.get_future();

    std::thread t([&prom]() {
        int result = add(1, 2);
        prom.set_value(result);  // 设置结果
    });

    std::cout << "Result: " << fut.get() << std::endl; // 获取结果
    t.join();
}
```

输出：

```
Result: 3
```

这就像是你往信箱里塞了一封信（`promise`），
另一个线程在等着看信（`future`）。

---

## ⚙️ 三、线程池中返回结果的方式（使用 `std::packaged_task`）

`std::packaged_task` 是对 `promise` + `future` 的封装。
可以把一个函数任务包装成可执行对象，执行后自动存储结果。

👇 举个简化版线程池的例子：

```cpp
#include <iostream>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>

class ThreadPool {
public:
    ThreadPool(size_t n) : stop(false) {
        for (size_t i = 0; i < n; ++i) {
            workers.emplace_back([this] {
                for (;;) {
                    std::function<void()> task;

                    {   // 任务获取区
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }

                    task();  // 执行任务
                }
            });
        }
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result<F, Args...>::type> {
        using return_type = typename std::invoke_result<F, Args...>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> res = task->get_future();

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace([task]() { (*task)(); });
        }
        condition.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : workers)
            worker.join();
    }

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};
```

---

## 🎯 四、示例：在线程池中提交任务并获取结果

```cpp
int main() {
    ThreadPool pool(4);

    auto f1 = pool.enqueue([](int x) { return x * 2; }, 5);
    auto f2 = pool.enqueue([](std::string s) { return s + " world"; }, "hello");

    std::cout << f1.get() << std::endl;  // 10
    std::cout << f2.get() << std::endl;  // hello world
}
```

输出：

```
10
hello world
```

每个任务都可以返回不同类型的结果，
通过 `std::future`，你可以在主线程中同步等待结果，
或者异步查询状态（`future.wait()` / `wait_for()`）。

---

## 🦾 五、在 ROS 中怎么用线程池传数据？

在 ROS 环境下，线程池最常用于：

* 并行执行多个回调（例如多个传感器数据融合）
* 异步处理计算任务（如路径规划、SLAM 子模块）
* 后台任务调度（日志、地图更新）

你可以这样结合：

```cpp
auto future = pool.enqueue([msg] {
    // 处理ROS消息
    auto result = heavy_compute(msg);
    return result;
});

// 在主线程中获取结果（例如发布新topic）
auto output = future.get();
pub.publish(output);
```

如果你不想阻塞主线程，还可以在回调里用 `std::future::wait_for()` 来轮询是否完成。

---

是否希望我帮你 **写一个“ROS节点中使用线程池处理消息”的具体例子**？
比如订阅图像或雷达数据，然后用线程池异步处理后再发布结果。

---
## 第二部分
太棒了 👍 我们就来动手做一个完整示例：
👉 **在 ROS 节点中使用线程池异步处理消息，然后发布结果。**

这个例子非常贴近实战，比如你在做 **图像、雷达或路径规划任务** 时就能直接用。

---

## 🎯 示例场景

* 订阅一个 `std_msgs::Int32` 类型话题 `/input_data`
* 用线程池异步处理（例如计算平方）
* 把结果发布到 `/output_data`

---

## 🚀 代码示例

假设你使用的是 **ROS 1 + C++14/17**（例如 melodic / noetic），
下面是完整节点实现：

```cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <iostream>
#include <vector>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>

// ---------------------- 线程池定义 ----------------------
class ThreadPool {
public:
    explicit ThreadPool(size_t n) : stop(false) {
        for (size_t i = 0; i < n; ++i) {
            workers.emplace_back([this] {
                for (;;) {
                    std::function<void()> task;

                    { // 任务队列访问保护
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { return stop || !tasks.empty(); });
                        if (stop && tasks.empty()) return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }

                    task(); // 执行任务
                }
            });
        }
    }

    template<class F, class... Args>
    auto enqueue(F&& f, Args&&... args)
        -> std::future<typename std::invoke_result<F, Args...>::type> {
        using return_type = typename std::invoke_result<F, Args...>::type;

        auto task = std::make_shared<std::packaged_task<return_type()>>(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
        );

        std::future<return_type> res = task->get_future();

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (stop)
                throw std::runtime_error("enqueue on stopped ThreadPool");
            tasks.emplace([task]() { (*task)(); });
        }
        condition.notify_one();
        return res;
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for (std::thread &worker : workers)
            worker.join();
    }

private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;
};

// ---------------------- ROS节点 ----------------------

class ThreadPoolNode {
public:
    ThreadPoolNode() : pool(4) { // 启动4个线程
        ros::NodeHandle nh;
        pub_ = nh.advertise<std_msgs::Int32>("/output_data", 10);
        sub_ = nh.subscribe("/input_data", 10, &ThreadPoolNode::callback, this);
    }

    void callback(const std_msgs::Int32::ConstPtr &msg) {
        int input = msg->data;

        // 将计算任务提交到线程池
        auto future = pool.enqueue([input]() -> int {
            ROS_INFO("Processing %d on thread %ld", input, std::this_thread::get_id());
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 模拟耗时任务
            return input * input; // 计算平方
        });

        // 异步获取结果并发布
        std::thread([this, f = std::move(future)]() mutable {
            int result = f.get();
            std_msgs::Int32 msg_out;
            msg_out.data = result;
            pub_.publish(msg_out);
            ROS_INFO("Published result: %d", result);
        }).detach();
    }

private:
    ThreadPool pool;
    ros::Subscriber sub_;
    ros::Publisher pub_;
};

// ---------------------- main ----------------------

int main(int argc, char **argv) {
    ros::init(argc, argv, "threadpool_node");
    ThreadPoolNode node;
    ros::spin();
    return 0;
}
```

---

## 📦 编译

在你的 `CMakeLists.txt` 里添加：

```cmake
add_executable(threadpool_node src/threadpool_node.cpp)
target_link_libraries(threadpool_node ${catkin_LIBRARIES})
```

---

## 🧠 运行方式

```bash
roscore &
rosrun your_package threadpool_node
```

发布消息测试：

```bash
rostopic pub /input_data std_msgs/Int32 "data: 5"
rostopic echo /output_data
```

输出示例：

```
[INFO] Processing 5 on thread 140370898151168
[INFO] Published result: 25
```

---

## ⚙️ 实战扩展方向

你可以轻松改造成更复杂的 ROS 节点：

* 🚘 SLAM：每帧图像异步处理特征点匹配。
* 🧠 规划：后台异步计算路径候选。
* 🛰️ 感知融合：多传感器数据并行融合。
* 💾 日志/地图更新异步写入磁盘。

---

是否希望我帮你 **扩展成“图像消息（sensor_msgs::Image）异步处理的版本”**？
比如：订阅图像 → 在线程池中灰度化或缩放 → 发布处理结果。

---

