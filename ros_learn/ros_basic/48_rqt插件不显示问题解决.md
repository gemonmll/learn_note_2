非常好，你的操作说明问题已经很明确了：
`rqt_tf_tree` 插件虽然**安装了**，但 **rqt 没有检测到这个插件**。

让我们有条理地修一下（主要是插件路径没被 root 环境正确识别）。

---

## 🔍 问题核心

`rqt --list-plugins | grep tf_tree` 没输出任何东西
→ 表示插件未注册到 Qt 插件系统
→ 常见原因：

1. 插件 XML 没被 `ament_index` 发现；
2. 使用 root 导致环境变量路径不同；
3. ROS 安装环境和运行环境的 Python 环境不一致。

---

## ✅ 解决步骤（建议一步步执行）

### **1️⃣ 清理缓存并强制刷新插件**

执行：

```bash
rm -rf ~/.ros/rqt_gui.ini ~/.ros/plugin_descriptions.xml
rqt --force-discover
```

然后再查：

```bash
rqt --list-plugins | grep tf_tree
```

如果这时能看到类似：

```
rqt_tf_tree.tf_tree.RosTfTree
```

再运行：

```bash
ros2 run rqt_tf_tree rqt_tf_tree
```

---

### **2️⃣ 如果仍无效，检查插件是否存在于系统路径**

查看插件 XML 是否存在：

```bash
find /opt/ros/${ROS_DISTRO} -name "rqt_tf_tree_plugin.xml"
```

你应当能看到类似输出：

```
/opt/ros/humble/share/rqt_tf_tree/plugin.xml
```

如果找不到这个文件，说明安装异常，可尝试**强制重装**：

```bash
sudo apt reinstall ros-${ROS_DISTRO}-rqt-tf-tree
```

---

### **3️⃣ 检查 `AMENT_PREFIX_PATH` 环境变量**

确认 ROS 环境是否正确加载（尤其是 root 下）：

```bash
echo $AMENT_PREFIX_PATH
```

如果输出为空或不包含 `/opt/ros/humble`，你需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
```

再试：

```bash
rqt --list-plugins | grep tf_tree
```

---

### **4️⃣ （推荐）不要用 root 运行 GUI 工具**

`rqt` 在 root 环境中经常找不到 Qt 插件，因为 Qt 的运行时缓存目录不同。
建议切回普通用户运行：

```bash
exit   # 退出root
source /opt/ros/humble/setup.bash
rqt --force-discover
ros2 run rqt_tf_tree rqt_tf_tree
```

如果你必须用 root（例如容器中），请手动设定 runtime 目录：

```bash
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR
```

再执行：

```bash
ros2 run rqt_tf_tree rqt_tf_tree --force-discover
```

---

### **5️⃣ 手动验证 Python 模块**

如果还是失败，检查模块是否能 import：

```bash
python3 -c "import rqt_tf_tree; print(rqt_tf_tree.__file__)"
```

若出现 `ModuleNotFoundError`，说明 Python path 不对，修复方法：

```bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH
```

---

## ✅ 总结推荐做法

在 root 下运行 Qt 程序非常容易出问题。
建议：

```bash
exit
source /opt/ros/humble/setup.bash
rqt --force-discover
ros2 run rqt_tf_tree rqt_tf_tree
```

---

如果你告诉我你是不是在 **Docker 或 root shell** 里跑 ROS（比如 `/` 下的 `root@jzm`），我可以直接给出针对 Docker/容器内的解决方案。
要我帮你看容器内该怎么设环境吗？
