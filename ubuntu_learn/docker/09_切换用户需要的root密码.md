你遇到的问题是：

> **从普通用户切回 root 时，被要求输入密码，但你从没设置过 root 密码。**

这是很多 Docker/Ubuntu 用户第一次遇到都会困惑的问题。
下面来讲清楚 **为什么会这样、该怎么解决、你当前 Docker 环境里应该怎么做**。

---

# ✅ **一、为什么会要求输入 root 密码？**

关键点：

### **在 Linux 中，`su` 命令切换到另一个用户 → 必须输入该用户的密码。**

* `su root` → 需要 root 密码
* `su 用户名` → 需要对应用户密码

这是 Linux 的正常行为，不是你的系统有问题。

---

# ✅ **二、但是 Docker 的 root 用户本来就是无密码的**

在 Docker 容器里：

* root 用户通常 **没有密码（locked password）**
* 表示密码为空？不是！
* 表示密码“禁用”，**不能 su 回 root**

你可以在容器里查看：

```bash
grep root /etc/shadow
```

大概率看到：

```
root:*:...
```

或

```
root:!:...
```

`*` 或 `!` 代表 **密码被禁用**，不能使用 su 切换。

---

# ✅ **三、为什么你现在能从 root 切换到普通用户？**

因为你用的是：

```
su 普通用户
```

普通用户可以切换到自己，不需要密码。

但是从普通用户切回 root：

```
su root
```

需要 root 密码，而你的 Docker 容器里 root 密码是“锁定”状态，所以：

⚠️ **系统必须提示你输入密码，且永远不会成功**。

---

# ⭐ 解决办法（一）：使用 sudo，不用 su（最推荐）

如果你给普通用户加了 sudo 权限（你之前已经加了）：

```
echo "$LOCAL_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
```

那么，你切换到普通用户后，切回 root 不用 su，只要：

```bash
sudo -i
```

✔ 不需要密码
✔ 可以直接进入 root shell

这是 Docker 和 Ubuntu 推荐的正确方式。

---

# ⭐ 解决办法（二）：给 root 设置一个密码（次推荐）

如果你一定要用 `su root`，那你必须先设置 root 密码：

进入容器后（以 root 身份）：

```bash
passwd
```

两次输入你想设置的密码。

设置完成后：

```bash
su root
```

就能用了。

---

# ⭐ 解决办法（三）：恢复 root 无密码登录（不推荐）

可以让 root 不需要密码就能 su，不过不安全：

```bash
passwd -d root
```

但很多系统会拒绝这种操作，依旧不建议。

---

# 🧩 最推荐 workflow（Docker + QGC + PX4）：

1. 进入容器自动登录普通用户
2. 运行 QGroundControl
3. 如果要执行 root 操作，用 sudo（不会要求密码）

举例：

```bash
sudo apt update
sudo modprobe something
```

**不要使用 su**。

---

# 📌 总结一句话

你被要求输入 root 密码，是因为：

* Docker 的 root 密码本来就是锁定状态
* su 切换用户必须输入密码
* 所以你永远无法 su root

✔ 正确切回 root 的方式是：
👉 **sudo -i**

无密码，安全，Docker 官方推荐。

---

如果你愿意，我还可以帮你：

✔ 生成完整的 Docker 运行 QGC 环境
✔ 自动创建非 root 用户
✔ 自动配置 sudo
✔ 支持 X11 和 GPU

需要吗？

---
## 第二部分 删除密码

你现在想要的是：

> **把系统恢复成一开始那种：`su root` 不需要密码、直接进入 root 的状态。**

如果你是在 **Docker 容器里**（你之前的描述就是），那么最初 root 是：

* **没有密码**
* **密码字段为空**
* **可以直接 su root 不需要密码**

要恢复到这个状态非常简单。

下面给你最标准、最安全、不会把 root 锁死的方法。

---

# ✅ 方法 1：删除 root 密码（恢复为“无密码可直接登录”的状态）

在 Docker 容器里，用 root 执行：

```bash
passwd -d root
```

这会把 root 密码删除，让 `/etc/shadow` 变成：

```
root::...
```

这代表：

* root 密码为空
* `su - root` 不需要输入密码
* 行为与你刚进入 Docker 时完全一致

测试：

```bash
su - root
```

✔ 不会再要求密码
✔ 直接进入 root shell

---

# 🚩 如果出现“Su 仍然要求密码”的情况（少数镜像会这样）

某些系统的 root 是被“Lock”状态而不是“Empty”状态，要先解锁：

```bash
passwd -u root
passwd -d root
```

完成后测试 `su - root`。

---

# 🔥 方法 2（推荐）：直接允许 su root 不检查密码

你也可以彻底禁用密码认证：

编辑 PAM 文件：

```bash
nano /etc/pam.d/su
```

找到这一行：

```
auth       sufficient pam_rootok.so
```

确保它存在并未被注释。
这行的意思是：

> 如果切换目标用户是 root → 无条件放行，不检查密码

这个通常默认开启，只要 root 没有被 lock，就可以无密码 su root。

---

# ❗ 注意（非常重要）

以上操作 **只适用于 Docker**。
不要在宿主机（如真实 Ubuntu、工作电脑）上操作，否则会导致 **任何人无需密码即可获得 root**。

---

# 📌 总结

### 想恢复成最初 Docker 容器 root 无密码模式 → 执行：

```bash
passwd -d root
```

如果 root 被锁定：

```bash
passwd -u root
passwd -d root
```

完成后：

```bash
su - root
```

即可无需密码直接进入。

---

如果你愿意，我可以帮你检查：

```
grep root /etc/shadow
```

看看你当前 root 到底是什么状态（锁定、空密码、带密码）。
