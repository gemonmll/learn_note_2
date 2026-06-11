下面我给你 **全面讲解 CMake `file(GLOB ...)` 和 `file(GLOB_RECURSE ...)` 的用法**，包含语法、示例、最佳实践、注意事项，一次讲清楚。

---

# 🚀 **1. 基本语法：GLOB**

用于匹配 **当前目录** 下的文件（不递归）。

```cmake
file(GLOB <变量名>  "模式1" "模式2" ...)
```

### 示例：

获取当前目录下所有 `.cpp` 文件：

```cmake
file(GLOB SRC_LIST "*.cpp")
```

输出示例（内部相当于）：

```
SRC_LIST = A.cpp;B.cpp;C.cpp
```

你可以这样用：

```cmake
add_library(MyLib ${SRC_LIST})
```

---

# 🚀 **2. 递归版本：GLOB_RECURSE**

用于 **递归扫描子目录**。

```cmake
file(GLOB_RECURSE <变量名> "模式1" "模式2" ...)
```

### 示例：

递归查找当前目录及子目录所有 `.cpp`：

```cmake
file(GLOB_RECURSE SRC_LIST "*.cpp")
```

匹配目录结构：

```
src/
  A.cpp
  sub/
     B.cpp
     C.cpp
  sub2/
     D.cpp
```

生成：

```
SRC_LIST = A.cpp;sub/B.cpp;sub/C.cpp;sub2/D.cpp
```

---

# 🔥 **3. 推荐加：CONFIGURE_DEPENDS（自动刷新）**

CMake 官方推荐这样写：

```cmake
file(GLOB_RECURSE SRC_LIST CONFIGURE_DEPENDS
    "*.cpp"
    "*.c"
)
```

### 作用：

让 CMake 在 **新增 / 删除文件时自动重新生成构建文件**
否则，你必须手动删除 build 目录并重新运行 cmake。

这是现代 CMake 最推荐的用法。

---

# ✨ **4. 常见模式（通配符）**

| 模式                   | 匹配内容                    |
| -------------------- | ----------------------- |
| `"*.cpp"`            | 所有 cpp 文件               |
| `"*.h"`              | 头文件                     |
| `"src/*.cpp"`        | src 目录下的 cpp，非递归        |
| `"src/**/*.cpp"`     | ❌ CMake 不支持这种（不同于 Bash） |
| `"src/*.?pp"`        | 匹配 pp/kpp/cpp 等         |
| `"*/CMakeLists.txt"` | 匹配所有子目录内的某文件            |

要递归匹配，必须用：

```cmake
file(GLOB_RECURSE ...)
```

---

# 📌 **5. 使用示例：自动收集源码**

### 📍 收集所有源文件：

```cmake
file(GLOB_RECURSE ALL_SRCS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)
```

### 📍 收集所有头文件（IDE 显示用）：

```cmake
file(GLOB_RECURSE ALL_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)
```

### 📍 组成你的库：

```cmake
add_library(${LIB_NAME} STATIC
    ${ALL_SRCS}
    ${ALL_HEADERS}
)
```

---

# ⚠️ **6. 注意事项（关键点）**

### ⚠️ (1) 不要用 GLOB 来收集 *安装文件*

因为 glob 结果可能变化，不适合 install() 配置。

### ⚠️ (2) CONFIGURE_DEPENDS 很重要

否则新增文件不会被编译，容易坑。

### ⚠️ (3) GLOB 比 aux_source_directory 好太多

你之前用：

```cmake
aux_source_directory(./ DIR_MAIN)
```

但它 **不能递归**，也不推荐。

GLOB_RECURSE 才是现代 CMake 正确做法。

---

# 🎯 **7. 最佳实践模板（可以直接用）**

```cmake
# 自动扫描 C/C++ 源码
file(GLOB_RECURSE PROJECT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
)

# 自动扫描头文件
file(GLOB_RECURSE PROJECT_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)

add_library(${PROJECT_NAME} STATIC
    ${PROJECT_SOURCES}
    ${PROJECT_HEADERS}
)
```

---

# 📘 如果你告诉我你的目录结构，我可以帮你写最完整的 GLOB 脚本

例如：

```
src/
  core/
  replay/
  util/
  generated/
  middleware/
```

我可以帮你写：

* 忽略 middleware
* 单独 glob generated 文件
* 全局 glob 源码但排除某些子目录

如：

```cmake
file(GLOB_RECURSE SRCS
    CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# 排除某目录
list(FILTER SRCS EXCLUDE REGEX ".*/middleware/.*")
```

如果需要告诉我目录结构，我给你最强版本脚本。
