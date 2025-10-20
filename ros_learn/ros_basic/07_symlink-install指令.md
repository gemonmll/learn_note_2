 --symlink-install 指令
## 🧩 简答：

> ❌ 对 **C++代码（编译产物）** 几乎 **没有帮助**。
> ✅ 对 **Python 节点、脚本、launch 文件、资源文件** 等 **非编译文件** 很有用。

---

## 🧠 深入解释：

当你执行：

```bash
colcon build --symlink-install
```

colcon 会在 `install/` 目录下，为每个包创建符号链接（symlink）指向 `src/` 的源文件。

---

### 📦 举例：

以 `ros2_ws/src/my_pkg/` 为例：

* 不加 `--symlink-install` 时：

  * `colcon` 会把编译后的或复制的文件放到 `install/my_pkg/lib/...`
  * 如果你修改了 Python 脚本或 launch 文件，**必须重新 build** 才能生效。

* 加上 `--symlink-install` 时：

  * `install/my_pkg/lib/...` 下不会复制文件，而是创建到 `src/my_pkg/...` 的符号链接。
  * 所以你只要修改源码（如 `.py` 或 `.launch.xml` 文件），立刻生效，不用重新构建。

---

### 🔧 对比：

| 类型                            | 是否受益 | 原因            |
| ----------------------------- | ---- | ------------- |
| C++ 源码 (.cpp)                 | ❌ 否  | 需要重新编译生成二进制文件 |
| Python 节点 (.py)               | ✅ 是  | 直接解释执行源码      |
| Launch 文件 (.launch.xml / .py) | ✅ 是  | 动态读取文件路径      |
| 资源文件（URDF、YAML 等）             | ✅ 是  | 路径直接链接，无需拷贝   |

---

## 🧱 结论：

* 对 **C++ 项目（ament_cmake）**：
  `--symlink-install` 没什么区别，仍然要重新编译。
  但 **不会造成坏处**，所以一般都会加上。

* 对 **Python 项目（ament_python）**：
  `--symlink-install` **极其有用**，开发效率提升很大。

---

✅ **推荐实践（通用写法）**

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

即使主要是 C++，也建议一直加上这项参数，保证 workspace 内的所有包都统一构建逻辑。

---
