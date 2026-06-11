`./temp.sh` 和 `source ./temp.sh`（或 `. ./temp.sh`）最大的区别在于脚本 **运行在哪个 shell 环境里** ——是否在**当前 shell**中执行。

下面用最清晰、最实用的方式解释。

---

# ✅ 1. `./temp.sh`

### ✅ **在一个新的子 Shell 中执行**

运行时会启动一个新的 bash 进程，脚本里的变量、函数、`cd` 操作都 **不会影响当前 shell**。

### ✅ 前提：脚本必须有执行权限

```bash
chmod +x temp.sh
./temp.sh
```

### ✅ 行为示例

**temp.sh：**

```bash
#!/bin/bash
x=10
cd /
```

**运行：**

```bash
./temp.sh
echo $x     # 空
pwd         # 仍然是原来的目录，而不是 /
```

---

# ✅ 2. `source ./temp.sh`（或 `. ./temp.sh`）

### ✅ **在当前 Shell 中执行（无子 Shell）**

脚本里面的所有内容都在当前 shell 里生效。

### ✅ 不需要可执行权限（只要能读）

### ✅ 行为示例：

**temp.sh：**

```bash
x=10
cd /
```

**运行：**

```bash
source temp.sh
echo $x     # 10
pwd         # 现在是 / 目录
```

---

# ✅ 3. 对比总结（最常用区别）

| 指令               | 是否产生子 shell   | 是否影响当前环境变量 | 是否改变当前工作目录 | 是否需要执行权限 |
| ---------------- | ------------- | ---------- | ---------- | -------- |
| `./temp.sh`      | ✅ 是           | ❌ 否        | ❌ 否        | ✅ 需要     |
| `source temp.sh` | ❌ 否（当前 shell） | ✅ 是        | ✅ 是        | ❌ 不需要    |

---

# ✅ 4. 什么时候用哪个？

## ✅ 用 `./temp.sh`

当脚本只是执行任务，不需要修改当前 shell 环境时：

* 代码编译
* 启动服务
* 跑某些程序

## ✅ 用 `source temp.sh`

当脚本需要改变当前 shell 的环境时：

* 设置环境变量（如 ROS、Anaconda、虚拟环境）
* 修改 PATH
* 定义函数供当前终端使用

例如 ROS 的 setup.bash 就必须用 source：

```bash
source /opt/ros/noetic/setup.bash
```

---

# ✅ 5. 一句话记忆

> **./script 是启动一个新 shell；source script 是改变你现在的 shell。**

---

如果你愿意，我可以给你一个可复制的小 demo，让你亲自运行观看两者行为的区别。
