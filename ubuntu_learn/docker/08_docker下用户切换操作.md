## 第一部分 创建用户 
QGroundControl **禁止以 root 身份运行**（从 4.x 版本开始），所以你看到：

```
You are running QGroundControl as root. You should not do this...
QGroundControl will now exit.
```

这是 **硬性限制**，不会让你继续运行。

---

# ✅ **解决方法：在 Docker 容器中创建普通用户再运行 QGroundControl**

你的 Docker run 已经传入了：

```
-e LOCAL_UID=$(id -u)
-e LOCAL_GID=$(id -g)
-e LOCAL_USER=$(id -un)
-e LOCAL_GROUP=$(id -gn)
```

但 **容器里没根据这些变量创建用户**，所以仍然是 root。

下面给你一个完全正确的做法，保证能运行 QGroundControl。

---

# ⭐ 方案 A：在容器里手动创建用户（最简单）

进入容器后执行：

```bash
groupadd -g $LOCAL_GID $LOCAL_GROUP
useradd -m -s /bin/bash -u $LOCAL_UID -g $LOCAL_GID $LOCAL_USER
```

给权限：

```bash
echo "$LOCAL_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
```

切换用户：

```bash
su $LOCAL_USER
```

进入 QGC 目录后运行：

```bash
cd /workspace/squashfs-root
./AppRun
```

这样 QGroundControl 就能正常启动.

---

# ⭐ 方案 B：在 Dockerfile 自动创建用户（推荐）

在 `Dockerfile` 末尾加入：

```dockerfile
ARG LOCAL_UID=1000
ARG LOCAL_GID=1000

RUN groupadd -g ${LOCAL_GID} usergroup \
    && useradd -m -u ${LOCAL_UID} -g ${LOCAL_GID} user \
    && echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER user
WORKDIR /workspace
```

重新 build：

```bash
docker build -t my_ros:ros_humble_px4 .
```

之后运行时：

```bash
docker run --rm -it --gpus all --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    my_ros:ros_humble_px4
```

(你设置的 proxy 变量可继续保留)

进入容器后直接运行：

```bash
cd squashfs-root
./AppRun
```

---

# ⭐ 方案 C：让 QGroundControl 忽略 root 限制（不推荐，但能用）

如果你暂时只想快速试一下：

```bash
export QT_FORCE_STDERR_LOGGING=1
export QGC_DISABLE_ROOT_CHECK=1
./AppRun
```

注意：

* 这是“跳过安全检查”，不是官方推荐
* 某些功能（串口 MAVLink）可能受影响

但在 Docker 中通常是可用的。

---

# 📌 总结

| 问题                                  | 原因              | 解决方法                              |
| ----------------------------------- | --------------- | --------------------------------- |
| QGroundControl 报错 "running as root" | QGC 禁止 root     | 切换到普通用户（推荐）                       |
| Docker 默认用户是 root                   | 镜像没创建普通用户       | 在 Dockerfile 或容器中创建用户             |
| 想快速运行                               | 临时绕过 root check | `export QGC_DISABLE_ROOT_CHECK=1` |

---

如果你愿意，我可以直接帮你生成：

### ✔ 一个完整的 Dockerfile

### ✔ 一个 docker run 脚本

### ✔ 自动创建非 root 用户

### ✔ 自动启动 QGroundControl（已解压）

### ✔ 支持 GPU + X11

只要告诉我：
你想让 QGroundControl **在 Docker 内运行**还是**宿主机运行，只把PX4 SITL放进Docker**？

--

## 第二部分 -e LOCAL_UID=$(id -u) -e LOCAL_GID=$(id -g) -e LOCAL_USER=$(id -un) -e LOCAL_GROUP=$(id -gn)为什么没有配置

你 **传了这些参数**：

```
-e LOCAL_UID=$(id -u)
-e LOCAL_GID=$(id -g)
-e LOCAL_USER=$(id -un)
-e LOCAL_GROUP=$(id -gn)
```

但 **容器里面仍然是 root**，所以 QGroundControl 报：

```
You are running QGroundControl as root...
```

原因只有一个：

# ❗ 这些环境变量 **只是传进去，但没有被使用！！！**

Docker **不会自动根据环境变量创建用户**。
你必须在：

* **Dockerfile** 内
* 或 **容器启动后手动**

根据这些变量 **创建一个普通用户**，否则容器永远是 root。

---

# ⭐ 为什么会这样？

这些变量只是：

```
LOCAL_UID=1000
LOCAL_GID=1000
LOCAL_USER=jzm
LOCAL_GROUP=jzm
```

但容器里：

* 只存在 root 用户
* root UID=0，GID=0
* 没有名为 jzm 的用户
* 没有 GID 1000 的 group
* 没有自动切换用户

所以容器内部默认是：

```
uid=0(root) gid=0(root)
```

即使你传了 UID/GID 环境变量，**容器不会自动帮你创建用户，也不会自动切换用户**。

---

# ⭐ 如何让这些环境变量“生效”？（必须执行以下操作）

## 🔹方案 1：容器内部手动创建用户（最简单）

进入容器后执行：

```bash
groupadd -g $LOCAL_GID $LOCAL_GROUP
useradd -m -s /bin/bash -u $LOCAL_UID -g $LOCAL_GID $LOCAL_USER
```

切换用户：

```bash
su $LOCAL_USER
```

然后运行 QGroundControl：

```bash
cd /workspace/squashfs-root
./AppRun
```

---

## 🔹方案 2：在 Dockerfile 自动创建用户（推荐）

你应该在 Dockerfile 末尾加入：

```dockerfile
ARG LOCAL_UID=1000
ARG LOCAL_GID=1000

RUN groupadd -g ${LOCAL_GID} devgroup \
    && useradd -m -u ${LOCAL_UID} -g ${LOCAL_GID} dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER dev
WORKDIR /workspace
```

然后你 docker run 时传入的环境变量：

```
-e LOCAL_UID=$(id -u)
```

就会作为 build 参数使用，创建一个和宿主机一致的 UID/GID 用户。

---

## 🔹方案 3：临时跳过 root 禁止（不推荐，但能运行）

```
export QGC_DISABLE_ROOT_CHECK=1
./AppRun
```

但可能导致串口等功能异常。

---

# ⭐ 总结

你的问题：

**为什么传了 UID/GID 变量，却没有效果？**

答案：

✔ Docker **不会自动创建用户**
✔ Docker **不会自动切换用户**
✔ 你传的变量只是环境变量，不会被系统使用
✔ QGroundControl **禁止 root 运行**

要解决：

* **要么创建普通用户**（推荐）
* **要么跳过 root 检查**
* **要么修改 Dockerfile**

---

如果你愿意，我可以给你：

👉 **一个完整可运行 QGroundControl + PX4 SITL 的 Dockerfile**
👉 **一个一键 docker run 脚本**
👉 **支持 GPU + X11 + 非 root 用户**

要我帮你生成吗？
