你图里 Windows 的：

```bat
netsh -c i i add neighbors <接口索引> <IPv4> <MAC>
```

本质是在 **“邻居表/ARP 表”里加静态绑定**（把某个 IP 固定解析到某个 MAC），避免 ARP 学习/被覆盖。

在 **Ubuntu/Linux** 里等价操作是 `ip neigh`（推荐）或 `arp -s`（老工具）。

---

## 1）先找到要绑到哪块网卡（Linux 不用数字索引，用网卡名）

查看网卡名：

```bash
ip link
```

你会看到类似 `enp0s31f6`、`wlp0s20f3` 这种。

（如果你真的想看类似 Windows 的接口 index：`ip -o link show | awk -F': ' '{print $1,$2}'`，左边数字就是 ifindex。）

---

## 2）添加静态 ARP/neighbor（最常用写法）

把 Windows 里的 `aa-bb-cc-dd-ee-0a` 改成 Linux 用的冒号格式 `aa:bb:cc:dd:ee:0a`

```bash
sudo ip neigh add 172.20.0.11 lladdr aa:bb:cc:dd:ee:0a dev enp0s31f6 nud permanent
sudo ip neigh add 172.20.0.14 lladdr aa:bb:cc:dd:ee:0d dev enp0s31f6 nud permanent
```

如果你担心已存在导致报错，直接用 `replace`（更省心）：

```bash
sudo ip neigh replace 172.20.0.11 lladdr aa:bb:cc:dd:ee:0a dev enp0s31f6 nud permanent
sudo ip neigh replace 172.20.0.14 lladdr aa:bb:cc:dd:ee:0d dev enp0s31f6 nud permanent
```

查看是否生效：

```bash
ip neigh show dev enp0s31f6
```

删除（需要时）：

```bash
sudo ip neigh del 172.20.0.11 dev enp0s31f6
```

> 说明：`nud permanent` 就是“永久静态条目”。

---

## 3）让它“重启后还在”（持久化）

`ip neigh add/replace` 默认**重启会丢**。常见做法之一是写个 systemd 服务：

1）新建脚本：

```bash
sudo tee /usr/local/sbin/add-static-neigh.sh >/dev/null <<'EOF'
#!/usr/bin/env bash
set -e
DEV="enp0s31f6"
ip neigh replace 172.20.0.11 lladdr aa:bb:cc:dd:ee:0a dev "$DEV" nud permanent
ip neigh replace 172.20.0.14 lladdr aa:bb:cc:dd:ee:0d dev "$DEV" nud permanent
EOF
sudo chmod +x /usr/local/sbin/add-static-neigh.sh
```

2）建 systemd 单元：

```bash
sudo tee /etc/systemd/system/static-neigh.service >/dev/null <<'EOF'
[Unit]
Description=Add static ARP/neighbor entries
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/add-static-neigh.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
```

3）启用并立刻执行：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now static-neigh.service
```

---

## 4）注意两点（很容易踩坑）

1. **只能绑定同一二层网络/同网段直连**的 IP（ARP 工作范围决定的）。
2. 有些网络设备/驱动会在链路波动时刷新邻居表，用 `nud permanent` 一般能顶住，但如果有策略冲突（比如 NetworkManager/脚本反复改），要统一管理方式。

---

如果你把你 Ubuntu 上的网卡名（`ip link` 的结果里那一行，比如 `enp0s31f6` / `wlp...`）发我，我可以把上面的命令直接替换成你机器的“可复制粘贴版”。

---- 删除服务
如果你之前是按我给的 **systemd 持久化方式**（`/etc/systemd/system/static-neigh.service` + `/usr/local/sbin/add-static-neigh.sh` + `systemctl daemon-reload`）加的，那“删除/撤销”分三步：

## 1）先停用并停止这个 service

```bash
sudo systemctl disable --now static-neigh.service
```

可选：确认一下状态

```bash
systemctl status static-neigh.service
```

## 2）删除 service 文件和脚本

```bash
sudo rm -f /etc/systemd/system/static-neigh.service
sudo rm -f /usr/local/sbin/add-static-neigh.sh
```

## 3）让 systemd 重新加载配置（这就是你提到的 daemon-reload 对应的“撤销”动作）

```bash
sudo systemctl daemon-reload
sudo systemctl reset-failed
```

---

## 另外：把当前已经加进去的静态 neighbor 也删掉（如果你想立即恢复）

先查看：

```bash
ip neigh show | grep 172.20.0.
```

删除（把网卡名换成你的，比如 `enp0s31f6`）：

```bash
sudo ip neigh del 172.20.0.11 dev enp0s31f6
sudo ip neigh del 172.20.0.14 dev enp0s31f6
```

> 如果当初用的是 `ip neigh replace`，`del` 可能提示不存在也没关系，说明已经不在了。

---

如果你不确定 service 叫啥，可以先列出来：

```bash
systemctl list-unit-files | grep neigh
```
