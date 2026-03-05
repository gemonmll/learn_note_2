# cmake只想编译一个target,什么情况下需要不是用cmake缓存，指令是怎样的

在 **CMake** 里，如果你**只想编译某一个 target**，通常有两种思路：

1️⃣ **使用已有的 CMake cache（最常见）**
2️⃣ **不使用之前的 CMake cache，重新配置一次**

下面分别说明 **什么时候需要不用 cache**，以及 **具体指令**。

---

# 一、只编译一个 target（使用已有 cache）

如果项目已经 **cmake configure 过一次**，并且 `build/` 目录里存在 `CMakeCache.txt`，直接用：

```bash
cmake --build build --target my_target
```

或者（等价）：

```bash
cd build
make my_target
```

如果是 **Ninja**：

```bash
ninja my_target
```

✔ 这种情况 **会使用 CMakeCache.txt**

适用场景：

* 已经 configure 过
* 只是想编译某个 target
* 不需要改变配置选项

---

# 二、什么时候需要不用 CMake cache

以下情况 **建议不用旧 cache（重新 configure）**：

### 1️⃣ 修改了 toolchain / compiler

例如：

* 改了 gcc → clang
* 使用 cross compile
* 改 toolchain file

旧 cache 会保存：

```
CMAKE_C_COMPILER
CMAKE_CXX_COMPILER
```

如果继续用 cache 会出错。

---

### 2️⃣ 修改了重要 CMake option

例如：

```cmake
option(BUILD_TESTS ON)
option(USE_CUDA OFF)
```

如果之前 configure 是：

```
-DUSE_CUDA=OFF
```

现在想：

```
-DUSE_CUDA=ON
```

最好清 cache。

---

### 3️⃣ 修改了 CMakeLists 结构

例如：

* 新增 target
* 删除 target
* 改 project()

有时 cache 会残留旧信息。

---

### 4️⃣ generator 改变

例如：

```
Ninja → Unix Makefiles
```

---

# 三、不使用 cache 的方式

### 方法 1（最推荐）：删除 build 目录

```bash
rm -rf build
cmake -S . -B build
cmake --build build --target my_target
```

---

### 方法 2：删除 cache

只删 cache 文件：

```bash
rm build/CMakeCache.txt
```

然后重新 configure：

```bash
cmake -S . -B build
```

---

### 方法 3：强制重新 configure

```bash
cmake -S . -B build -U *
```

或

```bash
cmake --fresh -S . -B build
```

（新版本 CMake 推荐）

---

# 四、完整示例（只编译一个 target）

### 重新 configure + 只编译某个 target

```bash
cmake -S . -B build
cmake --build build --target my_target -j
```

---

### Ninja 示例

```bash
cmake -S . -B build -G Ninja
cmake --build build --target my_target
```

---

# 五、一个常见误区

很多人以为：

```
cmake --build build --target my_target
```

会 **只编译这个 target 的源码**

其实 **不是**。

CMake 会编译：

```
my_target + 所有依赖 target
```

例如：

```
my_app
 ├─ libA
 └─ libB
```

执行：

```bash
cmake --build build --target my_app
```

实际编译：

```
libA
libB
my_app
```

---

✅ **总结**

| 场景                       | 是否使用 cache |
| ------------------------ | ---------- |
| 已 configure，只编译某个 target | 使用 cache   |
| 修改 compiler/toolchain    | 不用 cache   |
| 修改重要 cmake option        | 建议不用       |
| generator 改变             | 必须不用       |
| 新增/删除 target             | 建议不用       |

---

如果你愿意，我可以再讲一个 **CMake 高级技巧：只 configure 一个 target（超大项目加速编译）**，很多像 **LLVM / TensorFlow** 的项目都会用这个技巧。


------
# 问题2 CCACHE_DISABLE=1 cmake --build . --target unittest_rpc_ad_diag -j$(nproc)这个ccache_disable是什么意思

命令：

```bash
cd build
CCACHE_DISABLE=1 cmake --build . --target unittest_rpc_ad_diag -j$(nproc)
```

这里的 **`CCACHE_DISABLE=1`** 是一个 **环境变量**，用于控制 **ccache** 的行为。

简单说：

> **`CCACHE_DISABLE=1` 表示临时关闭 ccache，不使用编译缓存。**

---

# 一、ccache 是什么

**ccache** 是一个 **C/C++ 编译缓存工具**。

当你编译代码时，它会缓存：

```
源码 + 编译参数 + 编译器
```

对应的 **编译结果（.o 文件）**。

如果下次编译内容一样，它会：

```
直接从缓存取结果
```

而不是重新编译。

所以编译会非常快。

---

# 二、`CCACHE_DISABLE=1` 的作用

设置：

```bash
CCACHE_DISABLE=1
```

表示：

```
完全绕过 ccache
```

流程变成：

```
gcc/clang 直接编译
```

而不是：

```
ccache -> gcc/clang
```

也就是说：

| 状态                 | 行为         |
| ------------------ | ---------- |
| 默认                 | 使用 ccache  |
| `CCACHE_DISABLE=1` | 不使用 ccache |

---

# 三、为什么有人要关闭 ccache

常见原因有 4 个：

### 1️⃣ 调试编译问题

有时候你修改了：

```
宏
include
编译参数
```

但 **ccache 命中缓存**，导致：

```
代码没重新编译
```

结果出现：

```
编译结果和代码不一致
```

关闭 ccache 可以确认问题。

---

### 2️⃣ 测试真实编译时间

很多 CI 或 benchmark 会关掉：

```
CCACHE_DISABLE=1
```

因为：

```
ccache 会影响编译时间统计
```

---

### 3️⃣ 避免缓存污染

例如：

* 不同 toolchain
* 不同编译选项
* cross compile

有时 cache 会不稳定。

---

### 4️⃣ 编译某个 target 强制重新编译

例如你想确保：

```
unittest_rpc_ad_diag
```

是 **真实重新编译**。

---

# 四、这个写法的含义

你的命令等价于：

```
临时设置环境变量
↓
运行 cmake build
```

作用范围 **只对这一条命令有效**。

等价写法：

```bash
export CCACHE_DISABLE=1
cmake --build . --target unittest_rpc_ad_diag
```

但这种会影响整个 shell。

---

# 五、命令完整含义

```bash
cd build
CCACHE_DISABLE=1 cmake --build . --target unittest_rpc_ad_diag -j$(nproc)
```

逐项解释：

| 部分                              | 含义             |
| ------------------------------- | -------------- |
| `cd build`                      | 进入 build 目录    |
| `CCACHE_DISABLE=1`              | 禁用 ccache      |
| `cmake --build .`               | 调用底层构建系统       |
| `--target unittest_rpc_ad_diag` | 只编译这个 target   |
| `-j$(nproc)`                    | 使用 CPU 核心数并行编译 |

---

# 六、如何确认项目是否在用 ccache

可以看：

```bash
which gcc
```

如果输出类似：

```
/usr/lib/ccache/gcc
```

说明 gcc 被 **ccache wrapper** 替换了。

或者看：

```bash
ccache -s
```

会显示：

```
cache hit
cache miss
```

---

✅ **一句话总结**

```
CCACHE_DISABLE=1 = 临时关闭 ccache 编译缓存
```

保证这次编译 **一定真实执行编译器**。

---

如果你愿意，我可以再讲一个 **CMake + ccache 的坑（很多大项目 CI 都会踩）**：
为什么 **改了代码但 target 不重新编译**，90% 的情况其实是 ccache + dependency tracking 的组合问题。
