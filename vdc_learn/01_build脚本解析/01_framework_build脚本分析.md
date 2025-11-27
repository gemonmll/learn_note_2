下面我帮你 **完整分析这个 build.sh 脚本的逻辑、用途、执行流程、每一行的功能、以及它和 CMake 的关系**。
我会以“新人也能看懂”的方式解释。

---

# ✅ **1. 这个脚本的作用是什么？**

它是一个 **统一编译框架（unified build framework）**：

### **主要功能：**

* 自动设置项目路径
* 根据传入参数选择 toolchain（例如 linux / aarch64 / qnx …）
* 调用 bootstrap.sh 来准备环境
* 执行 build_project（通常调用 cmake + make 或 ninja）
* 支持 `./build.sh clean` 清除 target 目录

这个脚本充当 **编译入口**，类似：

```bash
./build.sh linux debug moduleA
```

---

# ✅ **2. 路径变量解析**

```bash
readonly BUILD_PATH=$(dirname $(readlink -f $0))
```

* `$0` 表示当前脚本（build.sh）
* `readlink -f` 获取 **绝对路径**
* `dirname` 获取所在目录

举例：
`build.sh` 位于 `/home/user/project/build/scripts/`
则：

```
BUILD_PATH=/home/user/project/build/scripts
```

---

### 项目根目录

```bash
readonly PROJECT_ROOT=$(readlink -f $BUILD_PATH/..)
```

也就是：

```
/home/user/project/build/scripts/.. = /home/user/project/build
```

进一步：

```
PROJECT_ROOT = /home/user/project/build
```

---

### 其它目录变量

```bash
SCRIPT_FOLDER=$PROJECT_ROOT/build/scripts
CMAKE_FOLDER=$PROJECT_ROOT/build/cmake
SOURCE_FOLDER=$PROJECT_ROOT
TARGET_FOLDER=$PROJECT_ROOT/target
RUNNING_FOLDER=$PROJECT_ROOT/running
```

也就是说：

```
project
├── build/
│   ├── scripts/        <-- shell工具
│   ├── cmake/          <-- toolchain
├── target/             <-- 编译生成目录
├── running/            <-- 可运行文件目录
```

📌 **这是典型的 CMake out-of-source build 结构。**

---

# ✅ **3. main() 函数流程（非常重要）**

逐行分析：

---

## **① 处理参数**

```bash
if [ $# -eq 1 ] && [ "$1" = "clean" ]; then
    clean
    return 0
fi
```

支持：

```
./build.sh clean
```

清空 target。

---

## **② 控制参数数量**

```
Usage: build.sh [toolchain_name] [debug/release] [module_name]
```

说明：

| 参数             | 作用                              |
| -------------- | ------------------------------- |
| toolchain_name | 使用的交叉编译工具链（如 linux/aarch64/qnx） |
| debug/release  | 编译模式                            |
| module_name    | 指定模块                            |

---

## **③ 构造 toolchain 路径**

```bash
toolchain_path="$CMAKE_FOLDER/$1.cmake"
```

例如：

```
./build.sh aarch64 debug
```

则：

```
toolchain_path = build/cmake/aarch64.cmake
```

---

## **④ 处理默认 toolchain**

```bash
DEFAULT_TOOLCHAIN_NAME=linux
```

如果没传 toolchain 参数：

```
Using default toolchain 'linux'
```

---

# 🔥 **4. 最关键的部分：调用 build_project()**

```bash
build_project "$PROJECT_ROOT" "$TARGET_FOLDER" "$toolchain_path" $(to_upper_case "$build_type") "$module_name"
```

含义：

| 参数           | 含义                            |
| ------------ | ----------------------------- |
| 项目路径         | /project/build（或 project 根目录） |
| 编译输出路径       | target/                       |
| toolchain 文件 | linux.cmake / aarch64.cmake   |
| build_type   | DEBUG 或 RELEASE（注意自动转大写）      |
| module_name  | 编译某个模块或 all                   |

**这个函数不在本文件中，而是来自 bootstrap.sh。**

---

# 🔍 **5. source "$SCRIPT_FOLDER/bootstrap.sh" 的作用**

这一句：

```bash
source "$SCRIPT_FOLDER/bootstrap.sh"
```

表示加载另一个脚本，里面定义：

* bootstrap()
* build_project()
* to_upper_case()
* clean()
* update_timestamp()

你现在看到的 `build.sh` 其实只是 **入口文件**，真正工作在 bootstrap.sh。

---

# 🧠 **6. 这个框架的大致编译流程（推断）**

流程大概是：

```
build.sh
 → 调用 bootstrap.sh
     → bootstrap 准备环境（设置 PATH/工具链变量）
 → update_timestamp
 → build_project
       → 调用 cmake -DCMAKE_TOOLCHAIN_FILE=xxx.cmake
       → make / ninja
 → 输出到 target/
```

这是典型车厂/视觉项目会使用的框架。

---

# 📌 **7. 总结（最重要）**

这个 `build.sh` 实现了：

### 🔧 自动构建框架

* 统一入口
* 支持不同工具链
* 支持不同构建模式
* 支持模块编译

### 📁 自动路径管理

* 自动获取各种路径
* 自动设置 target 输出目录
* 自动处理 running 目录

### 🧱 自动调用 CMake

* 通过 toolchain.cmake 设置交叉编译环境

### 🧹 提供 clean 功能

* 清除 target

---

# 如果你愿意，我还能帮你：

✔ 分析 bootstrap.sh
✔ 分析 toolchain 文件结构（你查找库的地方就在这里）
✔ 教你如何扩展 build.sh（如新增工具链、模块）
✔ 教你如何根据这个框架写自己的 CMakeLists.txt

需要我继续分析哪个部分？
