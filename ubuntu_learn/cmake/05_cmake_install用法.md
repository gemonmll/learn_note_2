下面给你 **最清晰、最通用的 CMake install 写法**，包括：

* 可执行文件安装
* 动态库安装
* 资源文件安装
* RPATH 配置（让运行时自动找到库）

所有例子都是真实可用的，直接复制即可。

---

# ✅ 一、最基础的安装方式

假设你的项目目录结构如下：

```
project/
 ├─ src/
 │   └─ main.cpp
 ├─ include/
 └─ CMakeLists.txt
```

### 1. 安装可执行程序

```cmake
add_executable(my_app src/main.cpp)

install(TARGETS my_app
    RUNTIME DESTINATION bin
)
```

安装后会出现在：

```
<install_prefix>/bin/my_app
```

默认 install_prefix 是 `/usr/local`，也可以指定：

```bash
cmake -DCMAKE_INSTALL_PREFIX=./install ..
make install
```

---

# ✅ 二、安装动态库 .so

假设：

```cmake
add_library(my_lib SHARED src/my_lib.cpp)

install(TARGETS my_lib
    LIBRARY DESTINATION lib
)
```

安装后目录：

```
<install_prefix>/lib/libmy_lib.so
```

---

# ✅ 三、安装头文件

```cmake
install(DIRECTORY include/ DESTINATION include)
```

会把 include 下的所有头复制到：

```
<install_prefix>/include/
```

---

# ✅ 四、安装运行所需的动态库（例如第三方库）

如果你的项目结构：

```
project/
 ├─ bin/
 │    my_app
 ├─ runtime/
 │    libxxx.so
```

你希望安装后结构为：

```
install/
 ├─ bin/my_app
 ├─ lib/libxxx.so
```

写法：

```cmake
install(DIRECTORY runtime/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so*"
)
```

---

# ⚡ 五、最推荐模板（含 RPATH，适合你 SNPE/QNN 开发）

你当前的典型结构如下：

```
project/
 ├─ src/
 ├─ CMakeLists.txt
 ├─ runtime/    # 放所有 .so
 └─ build/
```

你希望安装结果：

```
install/
 ├─ bin/my_app
 ├─ lib/*.so
```

并让程序运行时能自动找到 `lib/*.so`。

---

## 🔥 完整 CMakeLists 示例（推荐复制）

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)

# =========================================
# Build executable
# =========================================
add_executable(my_app
    src/main.cpp
)

# =========================================
# RPATH 设置（强烈推荐）
# 让程序安装后自动优先搜索 <prefix>/lib
# $ORIGIN 指 my_app 所在目录，即 bin/
# =========================================
set_target_properties(my_app PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
    BUILD_RPATH "$ORIGIN/../lib"
)

# =========================================
# Install 可执行文件
# =========================================
install(TARGETS my_app
    RUNTIME DESTINATION bin
)

# =========================================
# Install 自己写的动态库 (如有)
# =========================================
# install(TARGETS my_lib
#     LIBRARY DESTINATION lib
# )

# =========================================
# Install 第三方 .so 库（例如 SNPE/QNN） 
# runtime/*.so → install/lib
# =========================================
install(DIRECTORY runtime/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so*"
)

# =========================================
# Install your header files
# =========================================
install(DIRECTORY include/
    DESTINATION include
)
```

---

# 🧪 六、实际使用

```bash
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make -j
make install
```

安装后目录结构：

```
install/
 ├─ bin/my_app
 ├─ lib/libsnpe.so
 ├─ lib/libqnn.so
 ├─ include/...
```

现在直接运行：

```
./install/bin/my_app
```

👉 **不需要 LD_LIBRARY_PATH**
👉 程序自动会从 `$ORIGIN/../lib` 加载所有 .so
👉 最适合你做的 SNPE/QNN/8155/Linux 部署

---

# 如果你愿意，我可以根据你的项目目录，帮你写一个完整可运行的 CMakeLists（含 SNPE/QNN 链接）。
