非常好！👏 你现在问到的是 ROS 2 C++ 包中几乎**每个接口库**都会有的一个文件：
`visibility_control.h`。

这个文件看起来有点复杂，其实核心作用就一句话：

---

## 🧠 **它用来控制共享库（.so / .dll）中哪些符号对外可见。**

---

下面我一步步帮你解释清楚它为什么存在、每一段代码做什么、什么时候会用到。

---

## 🌍 一、为什么需要 visibility control？

当你编译一个库（例如 ROS 2 的 `libcustom_action_cpp.so`）时，
编译器会把所有函数、类、全局变量都“导出”（export）到符号表中。

但这会带来问题：

* 库变大（导出太多无用符号）
* 链接速度变慢
* 符号冲突（比如两个库都导出了同名的内部函数）
* 在 Windows 下，如果不显式导出，还**无法在 DLL 中使用这些函数**

因此，C++ 提供了**符号可见性机制（symbol visibility）**：

* “public” → 对外暴露（别人可以链接、调用）
* “hidden” → 仅库内部使用（别人看不到）

ROS 2 为了让包在 **Linux + Windows + macOS** 都能正常工作，
每个包会有一个 `visibility_control.h` 文件来做统一封装。

---

## ⚙️ 二、这个文件是怎么工作的

我们来看主要逻辑段：

### 1️⃣ Windows 与非 Windows 分支

```cpp
#if defined _WIN32 || defined __CYGWIN__
```

如果在 Windows 系统（或 Cygwin 环境）下编译：
使用 `__declspec(dllexport)` / `__declspec(dllimport)`
来控制符号导出导入。

在 Linux / macOS 下：
使用 GCC 的 `__attribute__((visibility("default")))`。

---

### 2️⃣ 定义导出 / 导入宏

在 Windows 下：

```cpp
#ifdef __GNUC__
  #define CUSTOM_ACTION_CPP_EXPORT __attribute__ ((dllexport))
  #define CUSTOM_ACTION_CPP_IMPORT __attribute__ ((dllimport))
#else
  #define CUSTOM_ACTION_CPP_EXPORT __declspec(dllexport)
  #define CUSTOM_ACTION_CPP_IMPORT __declspec(dllimport)
#endif
```

这两种写法只是兼容不同编译器的语法。
效果是一样的——告诉编译器“这个符号要导出/导入”。

---

### 3️⃣ 编译库时与使用库时的区别

```cpp
#ifdef CUSTOM_ACTION_CPP_BUILDING_DLL
  #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_EXPORT
#else
  #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_IMPORT
#endif
```

* 当你**编译库**时（CMake 定义了 `CUSTOM_ACTION_CPP_BUILDING_DLL` 宏），
  宏 `CUSTOM_ACTION_CPP_PUBLIC` 就变成 **导出 (export)**。

* 当你**使用库**时（没有定义该宏），
  宏 `CUSTOM_ACTION_CPP_PUBLIC` 就变成 **导入 (import)**。

👉 这样一来，你在头文件里只要写：

```cpp
class CUSTOM_ACTION_CPP_PUBLIC MyClass { ... };
```

无论编译还是链接，都能自动切换 export/import 模式。

---

### 4️⃣ 非 Windows 系统分支

```cpp
#define CUSTOM_ACTION_CPP_EXPORT __attribute__ ((visibility("default")))
#define CUSTOM_ACTION_CPP_IMPORT
#if __GNUC__ >= 4
  #define CUSTOM_ACTION_CPP_PUBLIC __attribute__ ((visibility("default")))
  #define CUSTOM_ACTION_CPP_LOCAL  __attribute__ ((visibility("hidden")))
#else
  #define CUSTOM_ACTION_CPP_PUBLIC
  #define CUSTOM_ACTION_CPP_LOCAL
#endif
```

GCC 和 Clang 默认导出所有符号，
但我们可以通过：

```bash
-fvisibility=hidden
```

隐藏所有符号，
然后再手动给想公开的函数加上：

```cpp
__attribute__((visibility("default")))
```

也就是说：

* `CUSTOM_ACTION_CPP_PUBLIC` → 明确导出
* `CUSTOM_ACTION_CPP_LOCAL` → 明确隐藏

---

## 🧩 三、它是怎么在 CMake 中启用的？

在 ROS 2 的 CMakeLists.txt 里，通常会这样写：

```cmake
add_library(custom_action_cpp SHARED src/custom_action.cpp)
target_compile_definitions(custom_action_cpp
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
```

这行定义告诉编译器：

> 在编译这个库时，定义 `CUSTOM_ACTION_CPP_BUILDING_DLL`。

于是：

* 头文件中的 `CUSTOM_ACTION_CPP_PUBLIC` = `__attribute__((visibility("default")))`
* 外部包在链接这个库时，则会自动识别为导入。

---

## 🧰 四、举个简单例子

假设你写一个库：

```cpp
// my_lib/visibility_control.h
#if defined _WIN32
  #define MY_LIB_PUBLIC __declspec(dllexport)
#else
  #define MY_LIB_PUBLIC __attribute__ ((visibility("default")))
#endif
```

使用：

```cpp
// my_lib/my_class.hpp
#include "visibility_control.h"

class MY_LIB_PUBLIC MyClass {
public:
  void hello();
};
```

CMake：

```cmake
add_library(my_lib SHARED src/my_class.cpp)
target_compile_definitions(my_lib PRIVATE "MY_LIB_BUILDING_DLL")
```

这就确保：

* 当你编译 `my_lib` 时，它导出 `MyClass`。
* 当别人用 `find_package(my_lib)` 时，它自动作为导入符号。

---

## ✅ 五、总结（背下来这几句话）

| 概念                               | 说明                                         |
| -------------------------------- | ------------------------------------------ |
| visibility control               | 控制库导出哪些符号                                  |
| Windows                          | 用 `__declspec(dllexport/dllimport)`        |
| Linux/macOS                      | 用 `__attribute__((visibility("default")))` |
| `CUSTOM_ACTION_CPP_BUILDING_DLL` | 区分编译库 / 使用库                                |
| `PUBLIC`                         | 可见（导出）                                     |
| `LOCAL`                          | 隐藏（内部使用）                                   |

---

# 第二部分 示例学习

---

非常棒 👍
你现在已经理解到 **CMake 与符号可见性机制的交叉点**了，
我们现在来把“导出 vs 导入”这段话讲得**完全通透**。

---

## 🧩 问题背景

你看到的这段代码：

```cmake
add_library(custom_action_cpp SHARED src/custom_action.cpp)

target_compile_definitions(custom_action_cpp
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
```

核心问题在这里：

> 为什么库在编译时定义 `CUSTOM_ACTION_CPP_BUILDING_DLL`，
> 而“外部包”在使用这个库时，就自动变成了“导入”模式？

---

## 🚀 一、宏定义控制“编译角色”

我们回忆一下在 `visibility_control.h` 里的关键部分 👇

```cpp
#ifdef CUSTOM_ACTION_CPP_BUILDING_DLL
  #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_EXPORT
#else
  #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_IMPORT
#endif
```

这段逻辑其实在说：

| 情况       | 定义                                   | 含义               |
| -------- | ------------------------------------ | ---------------- |
| 当库自己在编译时 | 定义了 `CUSTOM_ACTION_CPP_BUILDING_DLL` | 把公共符号“导出”到 `.so` |
| 当别人用这个库  | 没定义这个宏                               | 把公共符号“导入”进来使用    |

---

## ⚙️ 二、CMake 是如何切换这两个状态的

### ✅ 1️⃣ 编译库时：

你用：

```cmake
add_library(custom_action_cpp SHARED src/custom_action.cpp)
target_compile_definitions(custom_action_cpp
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
```

这行命令告诉编译器：

> “编译 `custom_action_cpp` 这个目标文件时，请定义宏 `CUSTOM_ACTION_CPP_BUILDING_DLL`。”

于是：

```cpp
#ifdef CUSTOM_ACTION_CPP_BUILDING_DLL
#define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_EXPORT
#else
#define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_IMPORT
#endif
```

👉 会走 **上半支**（导出）。

所有标记为 `CUSTOM_ACTION_CPP_PUBLIC` 的类和函数，就会变成：

```cpp
__attribute__((visibility("default")))
```

或者在 Windows 上是：

```cpp
__declspec(dllexport)
```

所以这些符号会**写进动态库的符号表里**（对外可见）。

---

### ✅ 2️⃣ 外部包使用时：

假设有一个外部包 `custom_action_user`，它的 CMakeLists 里会写：

```cmake
find_package(custom_action_cpp REQUIRED)
add_executable(user_node src/user_node.cpp)
ament_target_dependencies(user_node custom_action_cpp)
```

当 CMake 自动为 `user_node` 添加依赖时，它会：

* 链接到 `libcustom_action_cpp.so`
* 包含头文件 `custom_action_cpp/...`
* 但**不会定义** `CUSTOM_ACTION_CPP_BUILDING_DLL`

于是：

```cpp
#ifdef CUSTOM_ACTION_CPP_BUILDING_DLL
#define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_EXPORT
#else
#define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_IMPORT
#endif
```

👉 会走 **下半支**（导入）。

符号变成：

```cpp
__declspec(dllimport)
```

或在 Linux 上其实等价于“普通函数声明”（不用特殊标记）。

这样编译器/链接器就知道：

> “这个函数不是我来实现的，它在外部动态库里，我只需要从外部加载。”

---

## 🔍 三、所以“自动识别”的本质是：

> “有没有定义 `CUSTOM_ACTION_CPP_BUILDING_DLL` 宏”
> 是编译时由 CMake 决定的。

不是编译器自动发现，而是 CMake **显式地帮你定义或不定义**这个宏。

| 角色     | 宏是否定义                                  | 结果   |
| ------ | -------------------------------------- | ---- |
| 构建库本身  | ✅ 已定义 `CUSTOM_ACTION_CPP_BUILDING_DLL` | 导出符号 |
| 外部包使用库 | ❌ 未定义                                  | 导入符号 |

---

## 🧠 四、为什么要这样做（尤其在 Windows 上）

Linux 下其实可以偷懒，不写 visibility 宏也能用。
但在 Windows 上，动态库 `.dll` **必须明确指定哪些符号导出 / 导入**，
否则：

* 别人用这个 `.dll` 时会链接失败（找不到符号）；
* 或者整个库被静态复制，失去动态加载效果。

所以 ROS 2 使用这个跨平台宏机制来统一管理。
无论在 Windows 还是 Linux，行为一致。

---

## ✅ 五、一句话总结

| 你在做什么   | 宏定义状态                               | 结果                     |
| ------- | ----------------------------------- | ---------------------- |
| **编译库** | 定义 `CUSTOM_ACTION_CPP_BUILDING_DLL` | 把公共符号导出到 `.so`         |
| **使用库** | 没定义这个宏                              | 把符号识别为外部导入（从 `.so` 里取） |

👉 CMake 通过 `target_compile_definitions(... PRIVATE ...)` 来控制这一点。
这就是为什么说：“**外部包在链接这个库时，会自动识别为导入**。”

---

要不要我给你做一个最小的两包例子？
一个包 `lib_pkg` 导出函数，一个包 `app_pkg` 使用这个库，
你能清楚看到编译日志里宏切换、符号导出和导入的全过程。


----
# 不写的问题
----
非常棒的问题！👏
这行看似“无关紧要”，其实对 **动态库的导出行为** 非常关键。
我们来完整讲清楚 👇

---

## 🧩 背景：`CUSTOM_ACTION_CPP_BUILDING_DLL` 是干嘛的？

在你的头文件 `visibility_control.h` 里有这一段逻辑：

```cpp
#ifdef CUSTOM_ACTION_CPP_BUILDING_DLL
  #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_EXPORT
#else
  #define CUSTOM_ACTION_CPP_PUBLIC CUSTOM_ACTION_CPP_IMPORT
#endif
```

也就是说：

| 宏是否定义                                  | 含义         | 导出行为                                                                      |
| -------------------------------------- | ---------- | ------------------------------------------------------------------------- |
| ✅ 定义了 `CUSTOM_ACTION_CPP_BUILDING_DLL` | 我正在“编译这个库” | 用 `__attribute__((visibility("default")))` 或 `__declspec(dllexport)` 导出符号 |
| ❌ 没定义                                  | 我在“使用这个库”  | 用 `__declspec(dllimport)`（Windows）或空（Linux）导入符号                           |

---

## 🧱 情况 1：如果你写了 👇

```cmake
add_library(custom_action_cpp SHARED src/custom_action.cpp)
target_compile_definitions(custom_action_cpp PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
```

👉 意思是告诉编译器：

> “现在我正在 **编译这个库本身**，请把标记了 `CUSTOM_ACTION_CPP_PUBLIC` 的符号导出来。”

于是：

```cpp
CUSTOM_ACTION_CPP_PUBLIC void say_hello();
```

会被编译成一个导出的符号（可以被其他程序链接到）。

---

## 🧱 情况 2：如果你 **没写** 这行（即没定义宏）

那 `CUSTOM_ACTION_CPP_PUBLIC` 会被解析成 `CUSTOM_ACTION_CPP_IMPORT`。

也就是：

* 在 **Windows** 上：等价于 `__declspec(dllimport)`
  → 这告诉编译器“我只是在用这个库”，而不是在生成它
  → 结果是：库里的函数不会被导出，外部程序也无法链接这些符号
  ✅ 编译通过，但 ❌ 生成的 `.dll` 没有可导出的符号

* 在 **Linux (GCC)** 上：`CUSTOM_ACTION_CPP_IMPORT` 是空宏
  → 编译器就不会做任何导出
  → 如果你启用了 `-fvisibility=hidden`（ROS 2 默认会），那这些符号会被隐藏
  ✅ 编译通过，但 ❌ 外部程序链接时报 `undefined reference`

---

## 🧪 举例验证

假设有文件：

```cpp
// custom_action.cpp
#include "custom_action_cpp/visibility_control.h"

CUSTOM_ACTION_CPP_PUBLIC void say_hello() {
  printf("Hello\n");
}
```

如果没定义 `CUSTOM_ACTION_CPP_BUILDING_DLL`：

```bash
nm -D libcustom_action_cpp.so | grep say_hello
```

结果：🟥 **找不到符号**

加上后：

```bash
nm -D libcustom_action_cpp.so | grep say_hello
```

结果：✅

```
0000000000001130 T _Z9say_hellov
```

---

## ✅ 结论总结：

| 情况                                   | 结果                     |
| ------------------------------------ | ---------------------- |
| 定义了 `CUSTOM_ACTION_CPP_BUILDING_DLL` | 函数被导出，外部包可链接（正确做法）     |
| 没定义                                  | 函数被当作导入（不会导出），外部包链接时报错 |
| 在 Linux 上没开隐藏选项时                     | 仍然可能“侥幸成功”，但不规范，容易冲突   |

---

要不要我帮你画一张图解释这个“导出 / 导入”宏切换的编译流程？
图会显示头文件宏怎么影响 `.so` / `.dll` 的导出符号。
