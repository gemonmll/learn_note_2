Docker 容器里的 **shm size** 指的是容器可用的 **共享内存（shared memory）** 空间大小，也就是挂载在容器内的 **`/dev/shm`**（通常是 tmpfs）的容量。

### 它有什么用？

`/dev/shm` 是进程间共享内存（POSIX shm、System V shm 等）常用的地方，也经常被一些库/程序拿来当“超快的内存盘”。典型受影响场景：

* **Chrome / Chromium / Playwright / Puppeteer**：经常因为 `/dev/shm` 太小导致崩溃或卡死（常见报错里会提到 shm、dev/shm、renderer 等）。
* **Python multiprocessing / shared_memory**、某些 **数据库/缓存/中间件**、以及会用共享内存做 IPC 的程序。
* **机器学习/数据处理**：DataLoader 多进程、某些框架会用 shm 做数据交换或临时缓冲。

### 默认是多少？为什么会不够？

很多环境下 Docker 默认给容器的 `/dev/shm` 只有 **64MB**（尤其是 `docker run` 不指定时经常这样），对上面这些程序来说很容易不够。

你可以在容器里检查：

```bash
df -h /dev/shm
mount | grep shm
```

### 怎么设置/增大？

运行容器时加参数：

**1）直接指定 shm 大小（最常用）**

```bash
docker run --shm-size=1g ...
```

**2）用 tmpfs 挂载（更灵活）**

```bash
docker run --tmpfs /dev/shm:rw,size=1g ...
```

**docker-compose：**

```yaml
services:
  app:
    shm_size: "1g"
```

### 设多大合适？

* 一般 Web 自动化（Chrome）常用：**`--shm-size=1g`** 或 **512m** 起步
* 需要大量 IPC/多进程共享数据：按峰值需求上调（2g/4g…）

### 补充：它和容器内存限制的关系

* `--shm-size` 只管 `/dev/shm` 这块 tmpfs 的上限。
* 它占用的是 **宿主机内存**（不是磁盘），实际用多少占多少。
* 如果你还设置了 `--memory`，要注意整体内存预算；有些情况下 shm 的使用也会受到 cgroup/内存压力影响（不同版本/配置表现可能有差异）。

如果你告诉我你具体的场景（比如跑 Chrome、跑 PyTorch DataLoader、还是数据库），我可以给一个更贴近的 shm 推荐值和排错思路。
----
共享内存参数
----

不一定。**进程间通信（IPC）有很多种方式，shm（共享内存）只是其中一种，而且是“最快但需要同步机制”的那一类。**

### 常见 IPC 方式（不只 shm）

* **管道/匿名管道（pipe）**：父子进程常用，单向字节流
* **命名管道（FIFO）**：文件系统里一个特殊文件
* **Unix Domain Socket / TCP Socket**：本机或跨机器都能用（很多服务就是 socket）
* **消息队列（POSIX mqueue / System V msg）**
* **信号（signal）**：只适合通知，不适合传大量数据
* **文件/内存映射（mmap）**：用文件或匿名映射共享页
* **共享内存（shm）**：POSIX shm（`shm_open`）/ System V shm（`shmget`）

### shm 的特点

* **优点**：速度快、适合大块数据交换（避免复制）
* **缺点**：需要额外的**同步手段**（如 mutex / semaphore / futex），否则会数据竞争；生命周期管理也更麻烦

### 在 Docker 里，/dev/shm 和“IPC 是否走 shm”的关系

* `/dev/shm` 主要影响 **POSIX 共享内存**（很多实现会把共享内存对象放在 `tmpfs` 上，对应 `/dev/shm`）。
* **但并不是所有 IPC 都用 `/dev/shm`**：比如管道、socket、信号都不依赖 `/dev/shm` 的大小。
* 有些程序（比如 Chrome）会大量用共享内存或类似机制，因此容器里 `/dev/shm` 小会出问题。

### 怎么判断你的程序到底用没用 shm？

你可以在容器里看：

* `/dev/shm` 下有没有很多文件（POSIX shm 往往会出现形如 `sem.*` 或其他名字）

  ```bash
  ls -lh /dev/shm
  ```
* 看进程打开了哪些共享内存对象：

  ```bash
  lsof | grep /dev/shm
  ```
* 看 System V 共享内存段：

  ```bash
  ipcs -m
  ```

如果你说下你问这个是因为哪种现象（比如 Chrome 崩、Python 多进程卡、还是某服务报共享内存不足），我可以更具体地告诉你那类程序通常走哪种 IPC，以及该怎么配 `--shm-size` 或 `--ipc`。
