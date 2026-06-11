下面我用 **最清晰、最容易上手的方式** 教你：**Dockerfile 如何构建镜像（从编写到构建完整流程）**

---

# 🧱 一、Dockerfile 是什么？

Dockerfile 就是一个**文本文件**，里面写着构建镜像的步骤，比如：

* 基于哪个系统（FROM）
* 复制哪些文件进去（COPY）
* 安装哪些依赖（RUN）
* 启动时执行什么命令（CMD / ENTRYPOINT）

---

# 📦 二、一个最简单的 Dockerfile 示例

假设你有一个 C++ 或 Python 项目，现在写一个最简单的 Dockerfile：

### 示例：Python 应用

```
FROM python:3.10-slim

WORKDIR /app

COPY . .

RUN pip install -r requirements.txt

CMD ["python", "main.py"]
```

### 示例：C++ 应用（构建 + 运行两阶段）

```
# ===== 构建阶段 =====
FROM ubuntu:22.04 AS builder

RUN apt update && apt install -y build-essential cmake

WORKDIR /src

COPY . .

RUN mkdir build && cd build && cmake .. && make -j4

# ===== 运行阶段 =====
FROM ubuntu:22.04

COPY --from=builder /src/build/my_app /usr/local/bin/my_app

CMD ["my_app"]
```

---

# 🚀 三、构建镜像命令（最重要）

你在 Dockerfile 所在目录执行：

```
docker build -t my_image_name .
```

### 参数解释：

| 参数            | 含义            |
| ------------- | ------------- |
| `-t my_image` | 给镜像起一个名字      |
| `.`           | 使用当前目录作为构建上下文 |

例如：

```
docker build -t dsm_app:latest .
```

---

# 🧪 四、运行你构建的镜像

```
docker run --rm my_image_name
```

如果你的程序要映射端口：

```
docker run -p 8080:8080 my_image_name
```

如果要挂载目录：

```
docker run -v $(pwd):/app my_image_name
```

---

# 📝 五、Dockerfile 常用命令（速查）

| 指令           | 作用                      |
| ------------ | ----------------------- |
| `FROM`       | 指定基础镜像                  |
| `WORKDIR`    | 设置容器内工作目录               |
| `COPY`       | 拷贝文件到镜像                 |
| `ADD`        | COPY + 支持解压（不推荐常用）      |
| `RUN`        | 构建期执行命令（如 apt、pip、make） |
| `CMD`        | 容器启动时执行（可被覆盖）           |
| `ENTRYPOINT` | 启动入口（不易被覆盖）             |
| `EXPOSE`     | 声明开放端口（文档用）             |
| `ENV`        | 设置环境变量                  |

---

# 🔥 六、快速从零示例：构建一个 C++ 图像处理程序镜像（适合你）

假设你有：

```
main.cpp
CMakeLists.txt
```

Dockerfile：

```
FROM ubuntu:22.04 AS builder

RUN apt update && apt install -y g++ cmake make libopencv-dev

WORKDIR /workspace
COPY . .

RUN mkdir build && cd build && cmake .. && make -j4

FROM ubuntu:22.04
RUN apt update && apt install -y libopencv-dev
COPY --from=builder /workspace/build/app /usr/local/bin/app

CMD ["app"]
```

构建：

```
docker build -t cpp_image_app .
```

运行：

```
docker run --rm cpp_image_app
```

---

# 📘 七、你想在 Ubuntu PC → Docker 中构建 → 最终拷贝到 8155 QNX / Android 吗？

如果你后续要：

* 在 Docker 里交叉编译 8155（QNX 或 Android）
* 使用 clang / aarch64-linux-android-gcc
* SNPE / QNN 工具链

我也可以给你专门写一个完整可用的 8155 cross-build Dockerfile。

只要告诉我：

你的目标环境？

* **QNX？**
* **Android 12 (Snapdragon 8155)?**
* **Linux aarch64?**

你的模型运行环境？

* SNPE？
* QNN？
* TensorRT？

我可以直接给你一个能够 *一键构建 + 打包模型* 的 Dockerfile。

---

如果你需要我帮你写一个**你的项目专属 Dockerfile**，把你的项目结构贴一下，我可以直接帮你生成。

--

## 第二部分 build时走代理

要让 **Dockerfile 构建镜像时所有流量都走你的代理 `http://127.0.0.1:7890`**，有 **两种最可靠方式**：

---

# ✅ **方案1：构建命令指定代理（最推荐，简洁可靠）**

直接在 `docker build` 时加环境变量：

```
docker build \
  --build-arg HTTP_PROXY=http://127.0.0.1:7890 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7890 \
  --build-arg http_proxy=http://127.0.0.1:7890 \
  --build-arg https_proxy=http://127.0.0.1:7890 \
  -t my_image .
```

### 优点

✔️ 最简单
✔️ Dockerfile 不用改
✔️ 所有 RUN、apt、pip 都自动走代理

---

# ✅ **方案2：在 Dockerfile 中指定代理（推荐在持续构建时使用）**

在 Dockerfile 顶部加入：

```
ARG HTTP_PROXY=http://127.0.0.1:7890
ARG HTTPS_PROXY=http://127.0.0.1:7890
ARG http_proxy=http://127.0.0.1:7890
ARG https_proxy=http://127.0.0.1:7890

ENV HTTP_PROXY=${HTTP_PROXY}
ENV HTTPS_PROXY=${HTTPS_PROXY}
ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}
```

完整示例：

```
FROM ubuntu:22.04

ARG HTTP_PROXY=http://127.0.0.1:7890
ARG HTTPS_PROXY=http://127.0.0.1:7890
ARG http_proxy=http://127.0.0.1:7890
ARG https_proxy=http://127.0.0.1:7890

ENV HTTP_PROXY=${HTTP_PROXY}
ENV HTTPS_PROXY=${HTTPS_PROXY}
ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}

RUN apt update && apt install -y curl git
```

---

# ⚠️ **关键注意事项**

## 1. 你本机的 `127.0.0.1:7890` **不能直接给 Docker 容器用**

Docker 容器里的 `127.0.0.1` 是容器内部，不是你的宿主机。

所以需要改为：

### → Linux + Docker Desktop：

```
http://host.docker.internal:7890
```

或用 Docker 自带 DNS：

```
http://host.docker.internal:7890
```

### → Linux 原生 Docker（最常见）

Linux 下没有 `host.docker.internal`，你要用：

```
http://172.17.0.1:7890
```

可以这样查：

```
ip addr show docker0
```

找到 docker0 的 IP，一般是：

```
172.17.0.1
```

---

# 🧭 **你在 Linux 下建议使用（最常见好用）**

构建时这样写：

```
docker build \
  --build-arg HTTP_PROXY=http://172.17.0.1:7890 \
  --build-arg HTTPS_PROXY=http://172.17.0.1:7890 \
  -t my_image .
```

Dockerfile 中写：

```
ARG HTTP_PROXY=http://172.17.0.1:7890
ARG HTTPS_PROXY=http://172.17.0.1:7890

ENV HTTP_PROXY=${HTTP_PROXY}
ENV HTTPS_PROXY=${HTTPS_PROXY}
```

---

# 🚀 **如果你想所有 Docker 构建默认走代理（包括 pull/push）**

可以写入 Docker Daemon：

编辑：

```
/etc/docker/daemon.json
```

加入：

```
{
  "proxies": {
    "default": {
      "httpProxy": "http://172.17.0.1:7890",
      "httpsProxy": "http://172.17.0.1:7890"
    }
  }
}
```

重启 Docker：

```
sudo systemctl restart docker
```

---

# ❓需要我帮你写一个「全局自动代理的 Dockerfile 模板」吗？

如果你告诉我：

* 你是 **Ubuntu 还是 Windows？**
* 使用的是 **Clash / V2Ray / sing-box？**
* Docker 是 **Desktop 还是原生？**

我可以帮你生成 **一键代理 + 一键构建** 的完整配置。
