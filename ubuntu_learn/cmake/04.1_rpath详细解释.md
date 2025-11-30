直接说结论：**rpath 就是把“动态库搜索路径”写进可执行文件/so 自己里面**，这样程序运行时不用再依赖 `LD_LIBRARY_PATH` 或系统路径，也能自动找到 `.so`。
你现在这种「一堆 so 跟着程序走」的场景，用 rpath 是最舒服的方案。

下面我分几块讲：

---

## 1. rpath 是什么，和 LD_LIBRARY_PATH 有啥关系？

Linux 下加载动态库时，搜索顺序大致是：

1. `LD_PRELOAD`
2. `LD_LIBRARY_PATH`
3. **可执行文件 / so 里的 RPATH / RUNPATH**
4. `/etc/ld.so.cache`（来自 `/etc/ld.so.conf.d/*.conf`）
5. `/lib`、`/usr/lib` 等默认目录

`rpath` 就是嵌在 ELF 里的一个字段（动态段里的 `DT_RPATH` 或 `DT_RUNPATH`），告诉 loader：**除了系统默认目录之外，还要到这些路径里找库**。

和 `LD_LIBRARY_PATH` 对比一下：

| 特性   | rpath               | LD_LIBRARY_PATH   |
| ---- | ------------------- | ----------------- |
| 配置位置 | 写在 ELF 里（编译/链接时决定）  | 运行前设置环境变量         |
| 部署   | 带着程序走即可             | 每次运行要 export，或写脚本 |
| 灵活性  | 固定，改路径需重新链接或用工具修改   | 改环境变量就行           |
| 推荐场景 | 要打包发布、嵌入式、不能控制环境变量时 | 调试、临时测试           |

**你做 SNPE/QNN/8155 这类打包发布的场景，首选 rpath。**

---

## 2. RPATH vs RUNPATH（只要有个印象就行）

现代编译器一般会生成 **RUNPATH**（`DT_RUNPATH`），老一些的是 **RPATH**（`DT_RPATH`）。
区别主要在：**优先级和继承行为略有不同**，但对于你日常使用来说：

* 用 CMake / g++ 的 `-Wl,-rpath,xxx` 就行
* 把路径写进去，程序能找到库就够了

知道这点即可：**CMake 默认会打开 `new dtags`，生成 RUNPATH，这一般是好事。**

---

## 3. 怎么看一个程序有没有 rpath？

随便编个可执行文件，然后用：

```bash
readelf -d ./my_app | grep -i rpath
```

可能看到类似：

```text
0x000000000000001d (RUNPATH)            Library rpath: [$ORIGIN/../lib]
```

或者：

```text
0x000000000000000f (RPATH)              Library rpath: [/opt/snpe/lib]
```

说明已经内置好了搜索路径。

---

## 4. 不用 CMake，只用 g++ 时如何设置 rpath？

### 4.1 最简单例子

```bash
g++ main.cpp -L./lib -lmylib \
    -Wl,-rpath,/absolute/path/to/lib
```

这里 `-Wl,` 是把参数传给链接器（ld），`-rpath` 告诉 ld 往 ELF 里写入动态库路径。

### 4.2 用 `$ORIGIN` 支持相对路径（非常重要）

如果你打包结构是：

```
app/
 ├─ bin/my_app
 └─ lib/libmylib.so
```

希望无论拷到哪里运行，都能自动从 `bin/../lib` 找 so，可以这么写：

```bash
g++ main.cpp -L./lib -lmylib \
    -Wl,-rpath,'$ORIGIN/../lib'
```

注意：

* **必须加单引号 `'`**，否则 `$ORIGIN` 会被 shell 展开成空。
* `$ORIGIN` 在运行时由 loader 解析为“**当前可执行文件所在目录**”，而不是当前工作目录。

---

## 5. 在 CMake 里如何用 rpath（核心部分）

CMake 对 rpath 的控制分两类：

* **构建时的 rpath**（在 build 目录里运行可执行文件时用）
* **安装后的 rpath**（`make install` 之后的可执行文件用）

### 5.1 最常用的 per-target 写法（推荐）

假设目标结构：

```
install/
 ├─ bin/my_app
 └─ lib/*.so
```

CMake：

```cmake
add_executable(my_app src/main.cpp)

# 让 my_app 在安装后，从 bin/../lib 找动态库
set_target_properties(my_app PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib"
)
```

如果你在 build 目录就要跑（还没 install），也希望它从某个地方找 so：

```cmake
set_target_properties(my_app PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/lib"   # 举例
    INSTALL_RPATH "$ORIGIN/../lib"
)
```

> 小结：
>
> * `BUILD_RPATH` → 对编译目录里的 `my_app` 生效
> * `INSTALL_RPATH` → 对 `make install` 之后的 `my_app` 生效

### 5.2 全局配置写法（多个 target 通用）

如果项目里一堆可执行文件 / 库，都希望走同一个 rpath，可以用全局变量：

```cmake
# 安装后所有 target 的 rpath
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")

# 构建时的 rpath，一般用绝对路径
set(CMAKE_BUILD_RPATH "${CMAKE_BINARY_DIR}/lib")

# 不要在 install 的时候把 rpath 清空
set(CMAKE_SKIP_INSTALL_RPATH OFF)

# 构建时要保留 rpath（默认就是 OFF，一般不用改）
set(CMAKE_SKIP_BUILD_RPATH OFF)
```

CMake 会把这些值应用到你后面定义的 target 上。

---

## 6. 典型完整示例（你可以直接套用）

假设工程结构：

```
project/
 ├─ src/
 │    main.cpp
 ├─ runtime/         # 放外部 .so，比如 snpe, qnn 等
 └─ CMakeLists.txt
```

你希望：

* build 时：在 build 目录跑，直接从 `<build>/runtime` 找 so
* install 后：结构如下，并自动从 `../lib` 找：

  ```
  install/
   ├─ bin/my_app
   └─ lib/*.so
  ```

**CMakeLists 示例：**

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)

add_executable(my_app
    src/main.cpp
)

# ---------- RPATH 设置 ----------
# 构建目录里运行：到 <build>/runtime 查找 so
set_target_properties(my_app PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/runtime"
    INSTALL_RPATH "$ORIGIN/../lib"
)

# ---------- Install 可执行文件 ----------
install(TARGETS my_app
    RUNTIME DESTINATION bin
)

# ---------- Install 第三方 so 到 lib ----------
install(DIRECTORY runtime/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so*"
)
```

使用：

```bash
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make -j
make install

# 运行
../install/bin/my_app
# 不需要再设置 LD_LIBRARY_PATH
```

---

## 7. rpath 常见坑 & 排查方法

### 7.1 明明设置了，运行时还是说找不到库？

排查步骤：

1. 看 ELF 里有没有 rpath：

   ```bash
   readelf -d ./my_app | grep -i rpath
   ```

2. 确认路径正确（比如 `$ORIGIN/../lib`），且对应目录下确实有库。

3. 看 loader 实际找库的过程：

   ```bash
   LD_DEBUG=libs ./my_app
   ```

   会打印它依次尝试的路径，能看到是否用了 rpath。

---

### 7.2 CMake 把 rpath 给“洗掉”了？

有些项目会设置：

```cmake
set(CMAKE_SKIP_INSTALL_RPATH ON)
```

这会导致 **安装后的 rpath 被清空**，所以 `readelf -d` 看不到 rpath。

解决：改成 `OFF` 或不要设置它：

```cmake
set(CMAKE_SKIP_INSTALL_RPATH OFF)
```

---

### 7.3 安装完想改路径，不想重新编译？

可以用工具修改 ELF 里的 rpath（不改源码）：

* `chrpath`（简单场景，支持 RPATH）
* `patchelf`（更强大）

示例（以 patchelf 为例）：

```bash
patchelf --set-rpath '$ORIGIN/../lib' ./my_app
```

---

## 8. rpath 与 `LD_LIBRARY_PATH` 的组合建议

* **开发调试阶段**：
  可以先用 `LD_LIBRARY_PATH`，方便快速试库：

  ```bash
  export LD_LIBRARY_PATH=/path/to/snpe/lib:$LD_LIBRARY_PATH
  ./my_app
  ```

* **准备交付/打包时**：
  固定目录结构（bin + lib），把 `$ORIGIN` 形式的 rpath 写进去，做到 **拎包就跑**、不用改环境变量。

---

如果你愿意，把你现在的工程目录结构（bin、lib/runtime 放在哪里）发给我，我可以照着你实际情况写一份完全贴合的 CMake rpath 配置示例，这样你直接抄上去就能用。
