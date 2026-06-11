```cpp
    std::string log_name;
    if (is_pub) {
        log_name = "zmq_pubsub_c" + server_addr + "-" + std::to_string(port) + "_d" +
                   std::to_string(data_size) + "_f" +
                   std::to_string(send_freq) + ".log";
    } else {
        log_name = "zmq_pubsub_s" + std::to_string(port) + "_d" +
                   std::to_string(data_size) + "_o" +
                   std::to_string(time_offset) + ".log";
    }
    std::string log_path = "/tmp/rscl_test_log";
    GetEnvironmentVariable("test_log_path", &log_path);
    log_path += "/";
    AixLog::Log::init(
        {/// Log normal (i.e. non-special) logs to SinkCout
         std::make_shared<AixLog::SinkCout>(
             AixLog::Severity::trace,
             "%Y-%m-%d %H-%M-%S.#ms [#severity] (#tag_func) #message"),
         /// Log error and higher severity messages to cerr
         std::make_shared<AixLog::SinkCerr>(
             AixLog::Severity::error,
             "%Y-%m-%d %H-%M-%S.#ms [#severity] (#tag_func) #message"),
         /// Log special logs to native log (Syslog on Linux, Android Log on
         /// Android, EventLog on Windows, Unified logging on Apple)
         std::make_shared<AixLog::SinkNative>("aixlog",
                                              AixLog::Severity::trace),
         std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace,
                                            log_path + log_name)});

    std::string addr = "tcp://" + server_addr + ":" + std::to_string(port);
    if (is_pub) {
        zmq_sock = std::make_shared<ZmqPublisher>();
    } else {
        auto zmq_sub = std::make_shared<ZmqSubscriber>();
        zmq_sock = zmq_sub;
    } aixlog 这是如何使用的
```

这段代码主要做了三件事：
**① 生成日志文件名 → ② 初始化 AixLog 日志系统 → ③ 初始化 ZMQ Pub/Sub 对象。**
你问的是 **AixLog 怎么用**，我给你从“你这段代码在干嘛”到“实际怎么在工程里正确使用”一次讲透。

---

## ✅ 1️⃣ 你这段 AixLog 初始化代码在做什么？

```cpp
AixLog::Log::init(
    {
        std::make_shared<AixLog::SinkCout>(
            AixLog::Severity::trace,
            "%Y-%m-%d %H-%M-%S.#ms [#severity] (#tag_func) #message"),

        std::make_shared<AixLog::SinkCerr>(
            AixLog::Severity::error,
            "%Y-%m-%d %H-%M-%S.#ms [#severity] (#tag_func) #message"),

        std::make_shared<AixLog::SinkNative>("aixlog",
                                             AixLog::Severity::trace),

        std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace,
                                           log_path + log_name)
    }
);
```

意思是：
👉 **初始化 4 个日志输出目标（Sink）**

| Sink         | 作用                   |
| ------------ | -------------------- |
| `SinkCout`   | 普通日志输出到 `stdout`     |
| `SinkCerr`   | error 以上输出到 `stderr` |
| `SinkNative` | 系统日志（Linux = syslog） |
| `SinkFile`   | 输出到文件                |

---

## ✅ 2️⃣ 日志等级（Severity）

```cpp
AixLog::Severity::trace
AixLog::Severity::debug
AixLog::Severity::info
AixLog::Severity::warning
AixLog::Severity::error
AixLog::Severity::fatal
```

含义类似：

| 等级      | 典型用途  |
| ------- | ----- |
| trace   | 超细节调试 |
| debug   | 调试    |
| info    | 正常信息  |
| warning | 警告    |
| error   | 错误    |
| fatal   | 致命错误  |

例如：

```cpp
LOG(INFO) << "server start";
LOG(ERROR) << "bind failed";
LOG(DEBUG) << "send_freq=" << send_freq;
```

---

## ✅ 3️⃣ AixLog 实际怎么打日志？

初始化之后，你在代码任意地方可以这样用：

```cpp
#include <aixlog.hpp>

LOG(INFO) << "ZMQ server start, port=" << port;
LOG(DEBUG) << "data_size=" << data_size;
LOG(ERROR) << "connect failed";
LOG(WARNING) << "timeout happened";
```

如果你用的是 tag：

```cpp
LOG(INFO, "ZMQ") << "pub connected";
LOG(DEBUG, "NET") << "addr=" << addr;
```

输出示例（你设定的格式）：

```text
2026-02-10 12-01-03.123 [info] (main) ZMQ server start, port=9000
```

---

## ✅ 4️⃣ 你这个日志文件名拼接逻辑在干嘛？

客户端：

```cpp
zmq_pubsub_c127.0.0.1-9000_d1024_f50.log
```

服务端：

```cpp
zmq_pubsub_s9000_d1024_o5.log
```

👉 非常适合压测、多参数对比实验，日志不会互相覆盖 👍
这个设计挺工程化的，很明显是为了性能测试/时延测试准备的。

---

## ✅ 5️⃣ `GetEnvironmentVariable("test_log_path", &log_path);` 是干嘛的？

意思是：

* 默认日志目录：

  ```cpp
  /tmp/rscl_test_log
  ```
* 如果环境变量存在：

  ```bash
  export test_log_path=/home/jzm/logs
  ```

  那日志就写到：

  ```text
  /home/jzm/logs/zmq_pubsub_xxx.log
  ```

👉 非常实用的工程技巧 👍

---

## ✅ 6️⃣ 常见正确使用模板（你可以直接套）

### 初始化一次（main 里）

```cpp
void InitLog(const std::string& path) {
    AixLog::Log::init({
        std::make_shared<AixLog::SinkCout>(AixLog::Severity::info),
        std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace, path)
    });
}
```

---

### 业务中打日志

```cpp
LOG(INFO) << "Publisher start at " << addr;
LOG(DEBUG) << "send_freq=" << send_freq;
LOG(ERROR) << "ZMQ send failed";
```

---

## ✅ 7️⃣ 你这个 ZMQ + AixLog 的组合是典型测试工具结构

整体设计思路非常像：

* 用命令行参数控制：

  * pub / sub
  * 数据量
  * 频率
* 日志自动区分不同参数实验结果
* ZMQ 做通信性能压测

👉 很像：**网络性能测试工具 / 延迟测试工具 / 中间件benchmark工具**

---

## ✅ 8️⃣ 我帮你总结一句话版本

> AixLog 是一个轻量级 C++ 日志库，你这段代码初始化了 4 种日志输出目标（终端、错误输出、系统日志、文件），之后可以通过 `LOG(INFO) << xxx;` 在程序中随时打日志，日志会自动写入你生成的带参数信息的文件名中，方便测试分析。

---

## ✅ 9️⃣ 如果你愿意，我可以帮你：

* ✅ 改一版 **更规范的日志初始化封装**
* ✅ 加上 **--log_level / --log_path 命令行参数**
* ✅ 加一个 **性能统计日志（QPS、延迟P99）**
* ✅ 给你补一套 ZMQ Pub/Sub + 日志 benchmark 模板工程

你这个工具已经很“工程味”了，再稍微改改就能当团队内部压测工具用了 😄


-----
## 提问1 error和其他等级日志不是一个文件中吗

好问题，这个点很多人第一次看 AixLog 都会有点懵 👀
**结论先说：在你这份配置里，error 和其他等级日志是会同时出现在“同一个文件”里的。**

我们拆开看你现在的配置：

```cpp
AixLog::Log::init({
    std::make_shared<AixLog::SinkCout>(
        AixLog::Severity::trace,
        "..."),

    std::make_shared<AixLog::SinkCerr>(
        AixLog::Severity::error,
        "..."),

    std::make_shared<AixLog::SinkNative>(
        "aixlog", AixLog::Severity::trace),

    std::make_shared<AixLog::SinkFile>(
        AixLog::Severity::trace,
        log_path + log_name)
});
```

---

## ✅ 1️⃣ 现在的实际行为是怎样的？

每条日志都会**按等级“广播”给所有 Sink**，只要等级 ≥ 该 Sink 的阈值，就会写进去。

所以：

### 🔹 对于 `LOG(INFO)`

| Sink               | 是否输出         |
| ------------------ | ------------ |
| SinkCout (trace)   | ✅ 输出到 stdout |
| SinkCerr (error)   | ❌ 不输出        |
| SinkNative (trace) | ✅ 输出到 syslog |
| SinkFile (trace)   | ✅ 写入日志文件     |

👉 INFO 会写进**文件**

---

### 🔹 对于 `LOG(ERROR)`

| Sink               | 是否输出 |
| ------------------ | ---- |
| SinkCout (trace)   | ✅    |
| SinkCerr (error)   | ✅    |
| SinkNative (trace) | ✅    |
| SinkFile (trace)   | ✅    |

👉 ERROR 会 **同时出现在：终端 + 错误终端 + 系统日志 + 文件**

---

## ✅ 2️⃣ 所以你的问题答案是：

> ❓ error 和其他等级日志不是一个文件中吗？
> ✔️ **是的，现在它们都在同一个文件中。**

因为：

```cpp
std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace, log_path + log_name)
```

`trace` 表示：
👉 **trace 及以上全部等级都写入同一个文件**

---

## ✅ 3️⃣ 如果你想把 error 单独存一个文件（常见需求）

比如：

* 普通日志：`xxx.log`
* 错误日志：`xxx.err.log`

可以这样改：

```cpp
AixLog::Log::init({
    std::make_shared<AixLog::SinkCout>(AixLog::Severity::info),

    std::make_shared<AixLog::SinkFile>(
        AixLog::Severity::trace,
        log_path + log_name),

    std::make_shared<AixLog::SinkFile>(
        AixLog::Severity::error,
        log_path + log_name + ".err")
});
```

这样效果是：

| 等级    | xxx.log | xxx.err |
| ----- | ------- | ------- |
| INFO  | ✅       | ❌       |
| DEBUG | ✅       | ❌       |
| ERROR | ✅       | ✅       |
| FATAL | ✅       | ✅       |

👉 这是生产环境里很常见的日志拆分方式。

---

## ✅ 4️⃣ 进阶一点：只让 error 进 error 文件

如果你不想 error 在普通日志文件中出现（完全隔离）：

```cpp
AixLog::Log::init({
    std::make_shared<AixLog::SinkFile>(
        AixLog::Severity::trace,
        log_path + log_name),

    std::make_shared<AixLog::SinkFile>(
        AixLog::Severity::error,
        log_path + log_name + ".err",
        /* filter? */)
});
```

不过 AixLog 本身是“≥ 阈值就写”，**不支持直接排除**，如果你想 error 不进普通文件，需要：

* 改普通 Sink 的等级为 `warning` 以下
* 或者改库内部过滤（一般不推荐）

---

## ✅ 5️⃣ 实战建议（工程经验）

你这个是 **压测 / benchmark 工具**，我建议：

* 普通日志一个文件（trace/info/debug）
* error 单独一个文件
* 文件名带参数（你已经做得很好了）

推荐组合：

```cpp
AixLog::Log::init({
    std::make_shared<AixLog::SinkCout>(AixLog::Severity::info),

    std::make_shared<AixLog::SinkFile>(
        AixLog::Severity::trace,
        log_path + log_name),

    std::make_shared<AixLog::SinkFile>(
        AixLog::Severity::error,
        log_path + log_name + ".err")
});
```

---

## 🧠 一句话帮你记住

> AixLog 是“多路输出 + 等级阈值”模型，**error 不是独占文件，除非你额外再加一个 SinkFile 专门收 error。**

---

如果你愿意，把你完整的日志需求说一下（比如：只保留 error？调试阶段要不要 debug？线上要不要 trace？），我可以直接给你配一套“开发态 / 线上态”两套日志配置模板。



---
## sinkfile 和 sinknative区别

好问题 👍，这俩一看名字就有点“抽象”，但其实很实在。
我先给你一句话版结论，再展开讲清楚：

> **`SinkNative` = 打到“系统原生日志系统”里（Linux 上就是 syslog）**
> **`SinkFile` = 打到你指定的文件里（普通日志文件）**

下面掰开揉碎说 👇

---

## ✅ 1️⃣ SinkFile 是什么？（最常用）

```cpp
std::make_shared<AixLog::SinkFile>(
    AixLog::Severity::trace,
    log_path + log_name
)
```

👉 含义很简单：

* 把日志写进文件：

  ```text
  /tmp/rscl_test_log/zmq_pubsub_c127.0.0.1-9000_d1024_f50.log
  ```
* 只要日志等级 ≥ `trace`，都会写进去

你后面分析性能、延迟、丢包、错误，全靠这个文件。

**这是你这个工具最重要的日志出口。**

---

## ✅ 2️⃣ SinkNative 是什么？（系统级日志）

```cpp
std::make_shared<AixLog::SinkNative>(
    "aixlog",
    AixLog::Severity::trace
)
```

### 在 Linux 上等价于：👉 syslog

也就是说：

```bash
journalctl | grep aixlog
```

或者：

```bash
/var/log/syslog
/var/log/messages
```

可以看到这些日志（取决于发行版和日志系统）。

---

## ✅ 3️⃣ 为什么要有 SinkNative 这种东西？

这是为 **“系统服务 / 守护进程 / 后台进程”** 设计的。

### 🎯 场景 1：程序没有终端

比如：

* 你的程序是 systemd service
* 在 Docker 容器里跑
* 在车载系统 / 嵌入式系统后台跑

这时候：

* 没有 `cout`
* 日志文件可能被删
* 但系统日志一定存在

👉 SinkNative = **最后一道保险**

---

### 🎯 场景 2：运维统一收集

很多公司：

* 统一采集 syslog
* 送到 ELK / Loki / Splunk
* 做告警 & 可观测性

你程序只要打 SinkNative：

👉 日志自动进入全局监控系统，不用你额外对接。

---

## ✅ 4️⃣ SinkNative vs SinkFile 对比

| 对比项  | SinkFile      | SinkNative              |
| ---- | ------------- | ----------------------- |
| 写到哪里 | 指定路径的文件       | 系统日志（syslog / journald） |
| 可控性  | 很强（文件名、路径自己定） | 受系统管理                   |
| 是否常驻 | 你程序控制         | 系统服务常驻                  |
| 适合场景 | 压测、调试、离线分析    | 服务进程、运维监控               |
| 是否必须 | ⭐⭐⭐⭐⭐         | ⭐⭐（很多工具可以不用）            |

---

## ✅ 5️⃣ 你这个工具，SinkNative 有必要吗？

说实话，**对你这个 ZMQ 压测工具来说：**

👉 `SinkNative` 不是必须的。
它更像是：

> “这份代码是从服务程序模板里抄出来的”

在测试工具里，一般更关心：

* 文件日志（SinkFile）
* 控制台日志（SinkCout / SinkCerr）

如果你嫌它多余，可以这样精简：

```cpp
AixLog::Log::init({
    std::make_shared<AixLog::SinkCout>(AixLog::Severity::info),
    std::make_shared<AixLog::SinkFile>(AixLog::Severity::trace,
                                       log_path + log_name)
});
```

---

## ✅ 6️⃣ 什么时候我建议你保留 SinkNative？

如果你的程序将来会：

* 跑在车载系统
* 作为后台服务
* 需要接入公司日志平台
* 出问题要靠运维排查

👉 保留 SinkNative，很值。

---

## 🧠 一句话帮你记住

> * **SinkFile：给“开发/测试/分析”的人看的日志文件**
> * **SinkNative：给“系统/运维/监控平台”看的系统日志**

---

## 🚀 进阶建议（工程范）

你可以做成“模式切换”：

```cpp
if (debug_mode) {
    // 开发调试
    SinkCout + SinkFile
} else {
    // 线上服务
    SinkNative + SinkFile(error)
}
```

---

如果你想，我可以帮你整理一份：
👉 **“本地调试 / 压测工具 / 线上服务”三种日志配置最佳实践模板**，你直接 copy 就能用。
