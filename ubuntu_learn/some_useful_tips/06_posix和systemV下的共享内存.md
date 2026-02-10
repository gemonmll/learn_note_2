在这份代码里，“共享内存”不是普通文件，而是**内核里的共享内存对象/段**，然后被映射到进程的虚拟地址空间。

具体位置取决于你用的是哪一套：

**POSIX shm（`shm_open`）**
- 名字来自 `SHM_PATH`：`rscl_shm_test`
- 在 Linux 上，通常会出现在 ` /dev/shm/rscl_shm_test `（前提是 `/dev/shm` 挂载了）
- 代码位置：`testcases/base_tests/system_tests/base/shm_base.h`

**System V shm（`shmget`）**
- 标识是 key：`0x12173`（`SHM_KEY`）
- 这是内核里的 shm 段，没有文件路径
- 常用 `ipcs -m` 才能看到（如果你需要我可以帮你查）

**大小**
- `kShmSize = 102400` 字节  
- 代码位置：`testcases/base_tests/system_tests/base/shm_base.h`

如果你想，我可以告诉你在运行时怎么确认它是否创建成功，以及查看/清理的方法。


好代码示例 👍，你这个基本把 **POSIX SHM** 和 **System V SHM** 的核心用法都跑通了。我帮你从「概念 → API 对比 → 你这份代码逐段讲 → 实战注意点 → 选型建议」系统捋一遍，学完你基本就能在工程里自由选型和改造了。

---

## 1️⃣ POSIX SHM vs System V SHM：本质区别

| 对比项   | POSIX SHM       | System V SHM       |
| ----- | --------------- | ------------------ |
| 诞生年代  | 新（POSIX 标准）     | 老（UNIX 早期 IPC）     |
| 创建方式  | `shm_open()`    | `shmget()`         |
| 标识方式  | 路径字符串（类似文件）     | 整数 key             |
| 映射方式  | `mmap()`        | `shmat()`          |
| 删除方式  | `shm_unlink()`  | `shmctl(IPC_RMID)` |
| 权限模型  | 文件权限语义          | SysV IPC 权限        |
| 是否像文件 | 是（在 `/dev/shm`） | 否（内核对象）            |
| 推荐程度  | ✅ 更现代           | ⚠️ 老项目常见           |

一句话总结：

> **POSIX SHM = “像文件一样的共享内存”**
> **System V SHM = “传统内核 IPC 对象”**

---

## 2️⃣ POSIX SHM：你的代码在干嘛？

### 🔹 初始化（Sender / Receiver）

```cpp
sender_fd_ = shm_open(SHM_PATH, O_CREAT | O_RDWR | O_EXCL, 0666);
```

👉 创建共享内存对象（本质是 `/dev/shm/xxx`）

如果已存在：

```cpp
sender_fd_ = shm_open(SHM_PATH, O_RDWR, 0666);
```

然后：

```cpp
ftruncate(sender_fd_, kShmSize);
```

👉 设置共享内存大小（非常关键，不然 mmap 会失败）

映射：

```cpp
data_ = (char*)mmap(0, kShmSize, PROT_WRITE, MAP_SHARED, sender_fd_, 0);
```

👉 映射到进程虚拟地址空间

---

### 🔹 写数据（Sender::Run）

```cpp
memcpy(data_, &msg_info, sizeof(MsgInfo));
```

👉 把结构体直接写进共享内存

---

### 🔹 读数据（Receiver::Run）

```cpp
memcpy(&msg_info, data_, sizeof(MsgInfo));
```

👉 从共享内存拷贝出来

---

### 🔹 释放

```cpp
munmap(data_, kShmSize);
close(fd);
shm_unlink(SHM_PATH);
```

👉 `shm_unlink` 才是真正删除共享内存对象

⚠️ 很多人容易忘：

> **close ≠ 删除共享内存，shm_unlink 才是**

---

## 3️⃣ System V SHM：你的代码在干嘛？

### 🔹 初始化

```cpp
sender_fd_ = shmget(SHM_KEY, kShmSize, 0666 | IPC_CREAT | IPC_EXCL);
```

👉 创建共享内存段（key 是整数）

已存在时：

```cpp
sender_fd_ = shmget(SHM_KEY, 0, 0666);
```

---

### 🔹 映射

```cpp
data_ = (char*)shmat(sender_fd_, nullptr, 0);
```

👉 attach 到进程地址空间

---

### 🔹 释放

```cpp
shmdt(data_);
shmctl(sender_fd_, IPC_RMID, 0);
```

👉 标记删除（等最后一个进程 detach 后真正释放）

---

## 4️⃣ 你这份代码的几个专业点评（很关键）

### ✅ 优点

* POSIX / System V 两套 IPC 都写了，对比学习非常好
* 使用 `mmap` / `shmat` 正确
* 使用 mutex 保护本进程内写操作
* 用 seq + timestamp 验证数据一致性 👍

---

### ⚠️ 潜在问题 & 改进点

### ❌ 1. 进程间不同步（读写竞争）

现在是：

```cpp
memcpy(data_, &msg_info, sizeof(MsgInfo));
```

没有跨进程同步，可能出现：

* 读一半旧数据，一半新数据
* 读到写一半的结构体

👉 正确做法：
用 **POSIX semaphore / System V semaphore / futex** 做同步

例如 POSIX：

```cpp
sem_wait(sem);
memcpy(data_, &msg_info, sizeof(MsgInfo));
sem_post(sem);
```

---

### ❌ 2. timeout 用 system_clock + count() 有坑

```cpp
auto start_time = system_clock::now().time_since_epoch().count();
```

⚠️ 问题：

* `count()` 单位不保证是 ns
* system_clock 可能回拨

👉 建议：

```cpp
auto start = std::chrono::steady_clock::now();
if (std::chrono::steady_clock::now() - start > std::chrono::seconds(timeout_s)) ...
```

---

### ❌ 3. POSIX shm_open 错误 key 使用不统一

你这里：

```cpp
if (key != "") shm_open(key.c_str(), ...);
else shm_open(SHM_PATH, ...);

但 fallback 永远用 SHM_PATH
```

👉 如果 key 不同，sender / receiver 可能连不上同一个 shm

---

### ❌ 4. System V attach 计数方案不安全

```cpp
uint32_t* tmp = (uint32_t*)(data_ + sizeof(MsgInfo));
(*tmp)++;
```

⚠️ 问题：

* 多进程同时 ++ 有 race
* 没有原子操作
* 崩溃可能导致计数错误，shm 永远删不掉

👉 建议用：

* 原子操作
* 或者统一由“创建者进程”负责 IPC_RMID

---

## 5️⃣ 实战选型建议（工程里怎么选）

### ✅ 新项目 / 自动驾驶 / 高性能 IPC

👉 **推荐 POSIX SHM + POSIX semaphore**

优点：

* 语义更清晰
* `/dev/shm` 可视化
* 容易 debug
* 和 mmap 统一

---

### ⚠️ 老系统 / 历史代码 / RTOS 风格

👉 System V SHM 可能更常见
但新项目不建议引入

---

## 6️⃣ 给你一个“工程级推荐架构”

典型设计：

```text
[共享内存]
|-- Header
|   |-- seq
|   |-- timestamp
|   |-- data_size
|-- payload buffer

[同步机制]
POSIX semaphore:
  sem_full  // 数据就绪
  sem_empty // buffer 空闲
```

写端：

```cpp
sem_wait(sem_empty);
memcpy(shm, data, size);
sem_post(sem_full);
```

读端：

```cpp
sem_wait(sem_full);
memcpy(local, shm, size);
sem_post(sem_empty);
```

👉 这样就变成一个可靠的“共享内存队列”。

---

## 7️⃣ 一句话帮你建立直觉

> **POSIX SHM = mmap + 文件语义，现代、好维护**
> **System V SHM = 老式 IPC，维护成本高**
> **共享内存本身不解决同步问题，必须配合 semaphore / mutex / futex**

---

## 8️⃣ 如果你愿意，我可以帮你做一版：

👉「**POSIX SHM + semaphore 的标准生产级模板代码**」，可以直接拿去用在 Apollo / 感知 / 规划模块里。

你是打算用 shm 做 **传感器数据共享**，还是 **模块间高频通信**？
