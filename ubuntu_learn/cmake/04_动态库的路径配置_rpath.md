在 Linux 下，程序 **编译时链接动态库 (.so)** 与 **运行时加载动态库** 是两个阶段。很多人编译通过后运行报错：

```
error while loading shared libraries: xxx.so: cannot open shared object file: No such file or directory
```

这是因为 **运行时动态库搜索路径没配置好**。

下面给你最完整、实战可用的总结，适合你现在做 SNPE/QNN/8155 相关跨平台开发。

---

# 🎯 运行时应该如何配置动态库路径？

Linux 下动态库搜索顺序大概是：

1. **LD_LIBRARY_PATH（最高优先级）**
2. **程序的 rpath/runpath（编译时写入 ELF）**
3. `/etc/ld.so.cache`（来自 `/etc/ld.so.conf.d/*`）
4. 默认目录：`/lib/`、`/usr/lib/`

你可以使用下面几种方式。

---

# ✅ 方法 1：使用 `LD_LIBRARY_PATH`（最常用、最灵活）

适用于 **开发阶段** 或 **运行在 docker / 主机上** 时临时测试。

```bash
export LD_LIBRARY_PATH=/your/path/to/libs:$LD_LIBRARY_PATH
./your_program
```

例如你有：

```
./runtime/libsnpe.so
./runtime/libother.so
```

运行前执行：

```bash
export LD_LIBRARY_PATH=$PWD/runtime:$LD_LIBRARY_PATH
```

👉 **适合开发调试，不用 root**
👉 缺点：需要每次 export 或写入脚本

建议写入 `run.sh`：

```bash
#!/bin/bash
export LD_LIBRARY_PATH=$(pwd)/runtime:$LD_LIBRARY_PATH
./your_app
```

---

# ✅ 方法 2：使用 `runpath/rpath`（编译时写到 ELF 中）

编译时通过 CMake 或 g++ 注入 rpath，让程序在运行时自动找到库，非常适合 **部署到8155/QNX/嵌入式环境**。

## g++ 写法

```bash
g++ main.cpp -L./runtime -lyourlib -Wl,-rpath,'$ORIGIN/runtime'
```

`$ORIGIN` 指程序自身所在目录。

## CMake 写法（推荐）

```cmake
set(CMAKE_INSTALL_RPATH "$ORIGIN/../runtime")
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
```

或者：

```cmake
set_target_properties(your_app PROPERTIES
    INSTALL_RPATH "$ORIGIN/../runtime"
)
```

👉 程序运行时自动从相对路径加载 `.so`，不用 export。

非常适合发给别人、放到 docker、部署到 8155。

---

# ✅ 方法 3：写入 `/etc/ld.so.conf.d/*.conf`（适合长期安装）

例如你将库放在：

```
/opt/snpe/lib
```

创建文件：

```
sudo vim /etc/ld.so.conf.d/snpe.conf
```

内容：

```
/opt/snpe/lib
```

更新缓存：

```bash
sudo ldconfig
```

👉 稳定可靠，系统级。
👉 缺点：需要 root，不适合便携式软件。

---

# 哪种方式你应该用？

| 场景                        | 推荐方案                           |
| ------------------------- | ------------------------------ |
| **本地 Ubuntu / Docker 调试** | `LD_LIBRARY_PATH`              |
| **可执行文件 + 库一起打包发布**       | `RPATH/RUNPATH`（强烈推荐）          |
| **系统级安装（如部署到机顶盒/服务器）**    | `/etc/ld.so.conf` + `ldconfig` |
| **QNX / 8155 / SNPE 部署**  | `RPATH/RUNPATH` 或在启动脚本 export  |

---

# ⚠️ 常见踩坑

### ❌ 仅链接时加 `-L` 不会影响运行时

很多人以为：

```bash
-L./runtime
```

运行时也能找到库 ❌

`-L` **只影响编译阶段**

---

### ❌ 修改 `~/.bashrc` 不生效（程序在 IDE、systemd 中跑）

IDE 不读 bashrc
systemd 不读 bashrc
QNX 不读 bashrc

运行脚本必须自己设置 `LD_LIBRARY_PATH`。

---

### ❌ 相对路径用错

`LD_LIBRARY_PATH=./runtime` 只在当前目录有效。
推荐使用 `$ORIGIN` 这样绝对可靠。

---

# ✨ 最推荐你的做法（适合你当前开发流程）

你在用 docker + 主机调试 SNPE/QNN，强烈建议让可执行文件带 rpath：

---

## **最佳做法（强烈推荐）**

### CMake 配置：

```cmake
set_target_properties(your_app PROPERTIES
    BUILD_RPATH "$ORIGIN/../runtime"
    INSTALL_RPATH "$ORIGIN/../runtime"
)
```

你的目录结构建议：

```
app/
 ├─ your_app
 └─ runtime/
      ├─ libsnpe.so
      ├─ libqnn.so
      └─ ...
```

这样你哪里运行都不怕找不到库：

```bash
./app/your_app
```

不需要任何 export！完美部署。

---

# 如果你愿意，我可以帮你：

* 给你 **完整 CMake 工程模板（含 rpath 配置）**
* Docker 中如何挂载动态库路径
* 8155（QNX/Android）动态库路径配置方案
* SNPE/QNN 在 Linux 下 demo 工程模板

你想要哪个？
