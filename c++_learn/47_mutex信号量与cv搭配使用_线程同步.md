好呀，我们把 `std::condition_variable`（下面简称 `cv`）完整捋一遍，再配一个**超级简单的完整例子**，你自己复制就能编译运行的那种。

---

## 一、`condition_variable` 是干嘛的？

一句话：

> **cv = 让一个线程“睡觉等条件”，另一个线程“改好数据后把它叫醒”。**

它通常配合这三样东西一起用：

* `std::mutex`：保护共享数据
* `std::unique_lock<std::mutex>`：管理锁
* `std::condition_variable`：用来等待/唤醒

典型场景：
线程 A 等待某个条件 `ready == true`
线程 B 处理完数据，设置 `ready = true` 然后 `notify_one()` 把 A 叫醒。

---

## 二、最经典的用法模式

必须记住这个套路（真正项目里几乎都长这样）：

### 1）等待方（wait 一侧）

```cpp
std::mutex mtx;
std::condition_variable cv;
bool ready = false;

void waiting_thread() {
    std::unique_lock<std::mutex> lock(mtx);

    // 等待条件 ready == true
    cv.wait(lock, [] { return ready; });

    // 能走到这里，说明：
    // 1）已经被唤醒
    // 2）ready == true 条件满足
}
```

> `cv.wait(lock, pred)` 会：
>
> 1. 在内部自动 `unlock()` mutex，并让线程睡眠
> 2. 被唤醒后，再自动重新 `lock()`
> 3. 调用 `pred()`，只有返回 true 才真正从 wait 返回
> 4. 否则继续等待

所以：

* **不会死锁**
* **能防止虚假唤醒（spurious wakeup）**

---

### 2）通知方（notify 一侧）

```cpp
void notifying_thread() {
    {
        std::lock_guard<std::mutex> lock(mtx);
        ready = true;   // 改变条件
    }                   // 这里自动解锁

    cv.notify_one();    // 通知一个等待线程
    // 或者 cv.notify_all(); 通知所有等待线程
}
```

习惯写法是：

1. 先针对共享变量 `ready` 加锁并修改
2. 解锁（或出了作用域自动解锁）
3. 再调用 `notify_one()` 或 `notify_all()`

---

## 三、给你一个完整的简单例子

> 场景：
>
> * 子线程干点“活”（比如睡 1 秒模拟计算）
> * 干完后设置 `ready = true` 并 `notify_one()`
> * 主线程一开始调用 `cv.wait()` 阻塞，直到被叫醒

你可以直接放到 `main.cpp` 里编译运行试试。

```cpp
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

std::mutex mtx;
std::condition_variable cv;
bool ready = false;   // 条件
int result = 0;       // 共享数据

// 工作线程：做一些计算，然后通知主线程
void worker() {
    std::cout << "[worker] start working...\n";

    // 模拟干活：比如计算、IO 等
    std::this_thread::sleep_for(std::chrono::seconds(1));

    {
        // 修改共享数据前，先上锁
        std::lock_guard<std::mutex> lock(mtx);
        result = 42;
        ready = true;
        std::cout << "[worker] work done, result = " << result << "\n";
    } // 自动解锁

    // 通知等待的线程
    cv.notify_one();
    std::cout << "[worker] notified main thread.\n";
}

int main() {
    std::thread t(worker);   // 启动子线程

    std::cout << "[main] waiting for result...\n";

    // 1. 先拿锁
    std::unique_lock<std::mutex> lock(mtx);

    // 2. 然后开始等待条件 ready == true
    cv.wait(lock, [] { return ready; });

    // 能走到这里，说明 ready 为 true，并且锁已重新持有
    std::cout << "[main] got notification, result = " << result << "\n";

    t.join();
    std::cout << "[main] exit.\n";
    return 0;
}
```

### 你可以观察几个关键点：

1. **主线程**：

   * 先 `unique_lock lock(mtx);`
   * 再 `cv.wait(lock, []{ return ready; });`
   * 在等待过程中，mutex 是被自动释放的，让 worker 能拿到锁修改 `ready` 和 `result`。

2. **子线程 `worker`**：

   * `lock_guard` 上锁修改 `ready` 和 `result`
   * 出了作用域自动解锁
   * `cv.notify_one()` 通知主线程：条件已经满足了

3. **控制流**：

   * 主线程刚开始会卡在 `wait()` 上
   * 约 1 秒后，worker 线程“干完活”，设置 `ready = true` 并 `notify_one()`
   * 主线程被唤醒、检查 `ready == true`，然后继续执行，打印结果

---

## 四、`wait(lock, pred)` 和 `wait(lock)` 的区别

你会看到有两种写法：

### 1）推荐写法（带 predicate）✅

```cpp
cv.wait(lock, [] { return ready; });
```

优点：

* 自动循环等待，直到条件满足
* 能防止虚假唤醒（有时线程会在没有 notify 的情况下被唤醒）

等价于：

```cpp
while (!ready) {
    cv.wait(lock);
}
```

### 2）原始写法（不带 predicate）⛔ 不推荐直接用

```cpp
cv.wait(lock);  // 只负责“睡觉 + 缩放锁”，不检查条件
```

真实场景必须配合 while：

```cpp
while (!ready) {
    cv.wait(lock);
}
```

---

## 五、你可以这么记（超简易版）

**1⃣ 三件套：**

```cpp
std::mutex mtx;
std::condition_variable cv;
bool ready = false;
```

**2⃣ 等待方：**

```cpp
std::unique_lock<std::mutex> lock(mtx);
cv.wait(lock, []{ return ready; });
// 这里开始条件已满足
```

**3⃣ 通知方：**

```cpp
{
    std::lock_guard<std::mutex> lock(mtx);
    ready = true;
}
cv.notify_one();  // 或 notify_all()
```

> **核心记忆：**
>
> * 共享变量一定要用 mutex 保护（`ready`、队列、buffer 等）
> * 等待用 `cv.wait(lock, 条件)`
> * 修改条件后用 `notify_one()` / `notify_all()`

---

如果你愿意，我们可以再写一个**生产者-消费者的小例子**：
比如一个 `std::queue<int>`，生产者 `push` 数据，消费者 `wait` 等队列非空，然后 `pop`。
这个例子会比上面稍微复杂一点，但就特别贴近日常工程实际用了。你要不要我再帮你写一个？
