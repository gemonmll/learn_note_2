下面按你的需求给一套可直接用的配置。注意：Docker 构建时参数是 `--network=host`，不是 `--host=net`。

## 1. Docker system/daemon 代理设置

### Linux 推荐方式：`/etc/docker/daemon.json`

用于 Docker daemon 拉取镜像、访问 registry 等场景。

```bash
sudo mkdir -p /etc/docker
sudo tee /etc/docker/daemon.json > /dev/null <<'EOF'
{
  "proxies": {
    "http-proxy": "http://127.0.0.1:7897",
    "https-proxy": "http://127.0.0.1:7897",
    "no-proxy": "localhost,127.0.0.1,::1"
  }
}
EOF

sudo systemctl daemon-reload
sudo systemctl restart docker
```

Docker 官方现在推荐用 `daemon.json` 配置 daemon 代理；Docker Desktop 则需要在 Desktop 的设置里配置，`daemon.json` 的这种方式不适用于 Desktop。([Docker Documentation][1])

检查：

```bash
docker info | grep -i proxy -A 5
```

但这里有个关键点：**daemon 里的 `127.0.0.1` 指的是 Docker daemon 所在环境的本机**。如果你的代理只监听在当前用户桌面的 `127.0.0.1:7897`，而 Docker daemon 是 systemd 服务，它通常也在宿主机上，Linux 下大多可用；Docker Desktop/WSL 场景则可能不通。

---

## 2. Docker CLI/构建阶段代理设置

daemon 代理主要影响 Docker 自己拉镜像；Dockerfile 里的 `RUN apt update`、`pip install`、`curl` 等，还需要构建阶段代理。

可以在用户目录配置 Docker client：

```bash
mkdir -p ~/.docker
cat > ~/.docker/config.json <<'EOF'
{
  "proxies": {
    "default": {
      "httpProxy": "http://127.0.0.1:7897",
      "httpsProxy": "http://127.0.0.1:7897",
      "noProxy": "localhost,127.0.0.1,::1"
    }
  }
}
EOF
```

Docker 官方说明：Docker CLI 的 proxy 配置会自动给容器和 build 预填代理相关 build arguments。([Docker Documentation][2])

也可以每次 build 显式传：

```bash
docker build \
  --build-arg HTTP_PROXY=http://127.0.0.1:7897 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7897 \
  --build-arg http_proxy=http://127.0.0.1:7897 \
  --build-arg https_proxy=http://127.0.0.1:7897 \
  --build-arg NO_PROXY=localhost,127.0.0.1,::1 \
  --build-arg no_proxy=localhost,127.0.0.1,::1 \
  -t myimage:latest .
```

Docker 的 build arguments 是用于 Dockerfile 构建阶段的变量；代理相关变量可以通过 build args 传入。([Docker Documentation][3])

---

## 3. 使用 Dockerfile 构建镜像

示例目录：

```bash
mkdir demo-docker
cd demo-docker
```

创建 `Dockerfile`：

```Dockerfile
FROM ubuntu:24.04

ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG http_proxy
ARG https_proxy
ARG NO_PROXY
ARG no_proxy

RUN apt-get update && \
    apt-get install -y curl ca-certificates && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]
```

构建：

```bash
docker build -t demo:latest .
```

运行：

```bash
docker run --rm -it demo:latest
```

---

## 4. 构建时使用宿主机网络访问本机代理

你要的是：

```bash
docker build --network=host ...
```

完整命令：

```bash
docker build \
  --network=host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7897 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7897 \
  --build-arg http_proxy=http://127.0.0.1:7897 \
  --build-arg https_proxy=http://127.0.0.1:7897 \
  --build-arg NO_PROXY=localhost,127.0.0.1,::1 \
  --build-arg no_proxy=localhost,127.0.0.1,::1 \
  -t demo:latest .
```

`--network=host` 的作用是让 build 容器使用宿主机网络命名空间。这样 Dockerfile 里的 `RUN curl ...`、`RUN apt-get update` 等命令访问 `127.0.0.1:7897` 时，才更可能访问到宿主机上的代理。

---

## 5. 常见坑：代理监听地址

很多代理软件默认只监听：

```text
127.0.0.1:7897
```

这种情况下：

```bash
docker build --network=host ...
```

通常可以访问。

但如果不用 `--network=host`，容器里的 `127.0.0.1` 指的是**容器自己**，不是宿主机。此时应该用宿主机网关地址，例如 Linux 新版 Docker 可用：

```bash
docker build \
  --add-host=host.docker.internal:host-gateway \
  --build-arg HTTP_PROXY=http://host.docker.internal:7897 \
  --build-arg HTTPS_PROXY=http://host.docker.internal:7897 \
  -t demo:latest .
```

不过你明确要构建时使用宿主机网络，那么首选就是：

```bash
docker build --network=host ...
```

---

## 推荐最终命令

```bash
docker build \
  --network=host \
  --build-arg HTTP_PROXY=http://127.0.0.1:7897 \
  --build-arg HTTPS_PROXY=http://127.0.0.1:7897 \
  --build-arg http_proxy=http://127.0.0.1:7897 \
  --build-arg https_proxy=http://127.0.0.1:7897 \
  -t your-image-name:latest .
```

如果还拉不通，优先检查代理是否允许局域网/本机服务访问，以及端口 `7897` 是否实际在监听：

```bash
curl -x http://127.0.0.1:7897 https://www.google.com -I
ss -lntp | grep 7897
```

[1]: https://docs.docker.com/engine/daemon/proxy/?utm_source=chatgpt.com "Daemon proxy configuration"
[2]: https://docs.docker.com/engine/cli/proxy/?utm_source=chatgpt.com "Use a proxy server with the Docker CLI"
[3]: https://docs.docker.com/build/building/variables/?utm_source=chatgpt.com "Build variables"
