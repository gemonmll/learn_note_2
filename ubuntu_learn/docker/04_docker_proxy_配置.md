## 第一部分原始docker proxy配置
你当前遇到的错误是：

```
ERROR: failed to fetch anonymous token:
dial tcp 74.86.3.208:443: i/o timeout
```

🔍 **这说明：Docker build 一开始连不上 registry.docker.io（拉基础镜像 ubunutu:20.04）**

而且你的 `HTTP_PROXY=http://172.17.0.1:7890` **只作用于构建步骤内的 RUN 命令**
⚠️ **不作用于 Docker 自己的“拉基础镜像”阶段**。

所以你必须让 **Docker Daemon 本身也走代理**。

---

# ✅ 解决方案（最关键）：给 Docker Daemon 配置代理

编辑配置：

```
sudo mkdir -p /etc/systemd/system/docker.service.d
sudo nano /etc/systemd/system/docker.service.d/proxy.conf
```

写入：

```
[Service]
Environment="HTTP_PROXY=http://172.17.0.1:7890"
Environment="HTTPS_PROXY=http://172.17.0.1:7890"
Environment="NO_PROXY=localhost,127.0.0.1,172.17.0.1"
```

保存后：

```
sudo systemctl daemon-reload
sudo systemctl restart docker
```

检查是否生效：

```
systemctl show docker | grep -i proxy
```

你应该看到：

```
Environment=HTTP_PROXY=http://172.17.0.1:7890
```

---

# ⚠️ Docker Daemon 代理与 Dockerfile 内的 ENV 不一样！

| 阶段                     | 用到的代理                  |
| ---------------------- | ---------------------- |
| **拉基础镜像（FROM ubuntu）** | Docker Daemon（必须配置）    |
| **RUN apt/pip 等命令**    | Dockerfile 内 ENV / ARG |
| **docker pull / push** | Docker Daemon（可选）      |

所以你现在卡在 **第一步**：Daemon 没走代理。

---

# 🧪 测试代理是否真的可用

你可以进入你的宿主机：

```
curl -x http://172.17.0.1:7890 https://www.google.com -I
```

正常输出说明代理可用。

---

# 🔧（可选）给 Docker pull 单独添加代理

如果你不想修改 systemd，也可以这样构建：

```
HTTP_PROXY=http://172.17.0.1:7890 \
HTTPS_PROXY=http://172.17.0.1:7890 \
docker build -t test .
```

注意：

⚠️ 这种方法 **并不总是有效**，特别是使用 buildkit 时。

---

# 🚀 最稳定方案（强烈推荐）

## 在 `/etc/docker/daemon.json` 添加：

```
{
  "proxies": {
    "default": {
      "httpProxy": "http://172.17.0.1:7890",
      "httpsProxy": "http://172.17.0.1:7890",
      "noProxy": "localhost,127.0.0.1,172.17.0.1"
    }
  }
}
```

然后重启 Docker：

```
sudo systemctl restart docker
```

---

# 📌 最终提醒

你使用的是 **宿主机本地代理 running at 127.0.0.1:7890**
而 Docker 里要访问它 → 必须用 **docker0 网卡地址，比如 172.17.0.1**

但是 Docker Daemon 运行在宿主系统
所以 Docker Daemon 代理配置中 **写 127.0.0.1:7890也可以**：

```
"HTTP_PROXY": "http://127.0.0.1:7890"
```

---

# 🌟 如果你愿意，我可以帮你生成：

* 完整的 Docker 全局代理配置脚本
* 自动检测 docker0 的 IP
* 自动配置 `/etc/docker/daemon.json`
* 自动重启 docker

只需要告诉我你是在：

**Ubuntu / Debian / Arch / CentOS ？**

---

## 第二部分 docker buildkit配置代理

你这四条测试结果 **已经完全锁定问题所在**：

# 🎯 最重要的结论：

## 👉 **Docker Daemon 已成功配置代理**

## 👉 **容器内 `curl` 加代理能访问 Google**

## 👉 **容器内 `curl` 不加代理不能访问外网**

## ❗ **Docker Daemon 在拉镜像时没有使用代理！**

并且你的日志显示：

```
docker run --rm curlimages/curl https://www.google.com -I
```

一直停在那里 → **容器的默认网络不通外网**
但加代理之后：

```
docker run --rm curlimages/curl -x http://172.17.0.1:7890 https://www.google.com -I
```

能正常访问 Google（成功）。

所以你的网络状态是：

| 环境                   | 外网访问 | 备注                |
| -------------------- | ---- | ----------------- |
| Host 用户态 curl        | ✔ 正常 | 走代理               |
| Host root curl       | ✔ 正常 | 走代理               |
| docker run curl 不加代理 | ❌ 不通 | Docker 网络直连无网络    |
| docker run curl 加代理  | ✔ 正常 | Docker 网络只能通过代理访问 |

### 🔥 这说明你的 Docker 网络环境无法直连外网，只能走代理。

但现在真正的问题是：

# ❌ Docker Daemon 在拉镜像（FROM ubuntu:20.04）时没有使用代理

虽然你设了：

```
Environment=HTTP_PROXY=http://172.17.0.1:7890
```

但 **buildkit 不会自动继承 systemd 代理！**

# ✔ 解决方案：禁用 buildkit 或显式让 buildkit 使用代理

---

# ✅ 方案 1：禁用 buildkit（最简单且 100% 成功）

临时关闭：

```
DOCKER_BUILDKIT=0 docker build -t dsm_app:my_dsm .
```

如果成功下载 Ubuntu 20.04 → 问题解决。

永久关闭：

编辑：

```
sudo nano /etc/docker/daemon.json
```

加入：

```
{
  "features": {
    "buildkit": false
  }
}
```

重启：

```
sudo systemctl restart docker
```

### 为什么这有效？

因为传统 builder 会读取 systemd 的 HTTP_PROXY，而 buildkit 不会。

---

# ✅ 方案 2：让 buildkit 使用代理（推荐，但要手动配置）

编辑：

```
mkdir -p ~/.docker
nano ~/.docker/config.json
```

加入：

```
{
  "proxies": {
    "default": {
      "httpProxy": "http://172.17.0.1:7890",
      "httpsProxy": "http://172.17.0.1:7890",
      "noProxy": "localhost,127.0.0.1,172.17.0.1"
    }
  }
}
```

然后再执行：

```
docker build -t dsm_app:my_dsm .
```

---

# 🧪 方案 3：用构建参数传递给 buildkit（也能成功）

```
docker build \
  --build-arg HTTP_PROXY=http://172.17.0.1:7890 \
  --build-arg HTTPS_PROXY=http://172.17.0.1:7890 \
  --progress=plain \
  -t dsm_app:my_dsm .
```

---

# 🚀 最推荐最快的方法（你可以马上试）：

## **先试方案 1（最简单，几乎必成功）**

```
DOCKER_BUILDKIT=0 docker build -t dsm_app:my_dsm .
```

这一步很关键，能迅速验证是否是 buildkit 代理问题。

---

如果你现在执行这一条命令，请把结果贴给我，我能马上确认是否已解决。
