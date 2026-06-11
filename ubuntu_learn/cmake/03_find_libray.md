```cpp
find_library(MY_IPC_LIBRARY NAMES AdIpc PATHS ${PROJECT_SOURCE_DIR}/middleware/lib_linux NO_DEFAULT_PATH)
 find_library(MY_FRAMEWORK_LIBRARY NAMES AdFrameWork PATHS ${PROJECT_SOURCE_DIR}/middleware/lib_linux NO_DEFAULT_PATH) f
 ind_library(MY_BASE_LIBRARY NAMES AdBase PATHS ${PROJECT_SOURCE_DIR}/middleware/lib_linux NO_DEFAULT_PATH)帮我学习下
```

下面我用 **深入浅出 + 示例图解** 的方式，带你彻底学懂 CMake 里的 `find_library()`。

---

# 🔥 **1. find_library() 是干什么的？**

它的作用很简单：

👉 **从指定目录中查找某个动态库或静态库，并得到它的绝对路径。**

---

# 🔍 **2. 你使用的代码含义（逐行讲解）**

```cmake
find_library(MY_IPC_LIBRARY 
    NAMES AdIpc 
    PATHS ${PROJECT_SOURCE_DIR}/middleware/lib_linux 
    NO_DEFAULT_PATH)
```

我们来拆解：

---

## 🧩 **① 变量名：MY_IPC_LIBRARY**

```cmake
MY_IPC_LIBRARY
```

查找到的库路径会存到这个 CMake 变量中。

比如：

```
/home/user/project/middleware/lib_linux/libAdIpc.so
```

CMake 会自动生成完整路径，你后面 `target_link_libraries` 就能直接使用。

---

## 🧩 **② NAMES：要找的库名字**

```cmake
NAMES AdIpc
```

这表示你要找的是下面这些常见格式之一：

| 文件名           | CMake 自动匹配 |
| ------------- | ---------- |
| `libAdIpc.so` | ✔          |
| `libAdIpc.a`  | ✔          |
| `AdIpc.lib`   | ✔          |
| `AdIpc.dll`   | ✔          |

你 **只需写裸名 “AdIpc”**，CMake 会自动补全前缀 `lib` 和扩展名 `.so`, `.a`, `.dll`, `.lib`。

---

## 🧩 **③ PATHS：告诉 CMake 去哪里找**

```cmake
PATHS ${PROJECT_SOURCE_DIR}/middleware/lib_linux
```

作用：

📌 **指定一个查找目录，不去其它路径找。**

比如结构是：

```
middleware
└── lib_linux
    ├── libAdIpc.so
    ├── libAdFrameWork.so
    └── libAdBase.so
```

---

## 🧩 **④ NO_DEFAULT_PATH：只在你指定的路径找**

```cmake
NO_DEFAULT_PATH
```

否则 CMake 还会搜索系统路径：

* `/usr/lib`
* `/usr/local/lib`
* `/lib`
* CMake 自带路径

为了避免错误链接系统中的同名库，一定要加：

✔ 正确使用：**完全强制在 middleware 里找**

---

# 📌 **3. find_library 返回什么？**

假设你有：

```
middleware/lib_linux/libAdBase.so
```

那么：

```cmake
message(${MY_BASE_LIBRARY})
```

输出：

```
/home/user/project/middleware/lib_linux/libAdBase.so
```

如果没找到：

```
MY_BASE_LIBRARY-NOTFOUND
```

---

# 🔥 **4. 你三条查找语句完整解释**

```cmake
find_library(MY_IPC_LIBRARY       NAMES AdIpc       PATHS ... NO_DEFAULT_PATH)
find_library(MY_FRAMEWORK_LIBRARY NAMES AdFrameWork PATHS ... NO_DEFAULT_PATH)
find_library(MY_BASE_LIBRARY      NAMES AdBase      PATHS ... NO_DEFAULT_PATH)
```

等价于：

👉 去 **middleware/lib_linux** 目录找这三个库：

* `libAdIpc.so`
* `libAdFrameWork.so`
* `libAdBase.so`

找到它们 → 变量存为绝对路径
没找到 → `xxx-NOTFOUND`

---

# ✍ 如何检查是否找到？

添加：

```cmake
message(STATUS "IPC       = ${MY_IPC_LIBRARY}")
message(STATUS "Framework = ${MY_FRAMEWORK_LIBRARY}")
message(STATUS "Base      = ${MY_BASE_LIBRARY}")
```

---

# ⚠ 若没找到，则 CMake 报错（推荐）

```cmake
if (NOT MY_BASE_LIBRARY)
    message(FATAL_ERROR "Not found: AdBase")
endif()
```

---

# 🎓 **5. 学习总结（最重要的部分）**

| CMake             | 解释                |
| ----------------- | ----------------- |
| `find_library()`  | 查找库（返回绝对路径）       |
| `NAMES`           | 裸名，自动匹配 lib & 扩展名 |
| `PATHS`           | 指定查找目录            |
| `NO_DEFAULT_PATH` | 不搜索系统目录           |
| 返回 `_NOTFOUND`    | 表示没找到             |

---

# 📚 想继续学习吗？

我可以继续教你：

* 🔧 `find_library()` 和 `find_path()`、`find_file()`、`find_package()` 的区别
* 🏗 如何写自己的 `FindAdBase.cmake` 模块
* ⚙ 交叉编译环境中如何查找库
* 🧪 如何避免 CMake 错误链接到 host 机器库

你需要哪个方向？
