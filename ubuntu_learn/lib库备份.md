我可以把“删除/禁用旧 PCL 1.13”这件事的**最安全做法**给你一套可直接复制执行的命令（在你容器里跑），但我不能直接替你在容器里执行删除操作。你按下面步骤走就行，**可回滚**，不会像 `rm -rf` 那样不可逆。

> 你当前旧库在 `/usr/lib/libpcl_*.so.1.13*`，而且 **不归 dpkg 管**（`not owned by dpkg`），所以“删除”只能靠手工操作。

---

## 方案：把旧 PCL 1.13 移走（禁用），保留备份（推荐）

### 0）先确认旧库数量（可选）

```bash
ls -l /usr/lib/libpcl_*.so* | head
```

### 1）创建备份目录

```bash
sudo mkdir -p /usr/lib/pcl_legacy_disabled_1.13
```

### 2）把 **1.13 的真实库文件和版本软链接** 移走

```bash
sudo sh -c 'mv -v /usr/lib/libpcl_*.so.1.13* /usr/lib/pcl_legacy_disabled_1.13/ 2>/dev/null || true'
```

### 3）把 **指向 1.13 的通用软链接** 也移走（关键）

很多工程会链接 `-lpcl_common` 这种“无版本名”，会命中 `/usr/lib/libpcl_common.so -> ...1.13`，所以也要挪走：

```bash
sudo sh -c 'for f in /usr/lib/libpcl_*.so; do \
  [ -L "$f" ] && target="$(readlink "$f")" && echo "$target" | grep -q "1.13" && mv -v "$f" /usr/lib/pcl_legacy_disabled_1.13/; \
done'
```

### 4）刷新动态链接器缓存

```bash
sudo ldconfig
```

### 5）验证旧库已禁用

```bash
ls -l /usr/lib/libpcl_common.so* 2>/dev/null || echo "old pcl common link removed"
ldconfig -p | grep -i pcl | head -n 30
```

---

## 回滚（如果发现别的东西坏了）

把文件移回来并刷新缓存：

```bash
sudo sh -c 'mv -v /usr/lib/pcl_legacy_disabled_1.13/libpcl_* /usr/lib/ 2>/dev/null || true'
sudo ldconfig
```

---

## 做完后你下一步应该是

1. **把 `libpcl-dev` 安装完整**（你之前中断了）：

```bash
sudo dpkg --configure -a
sudo apt-get -f install
sudo apt-get update
sudo apt-get install -y --no-install-recommends libpcl-dev
```

2. 找到并确认 `PCLConfig.cmake`：

```bash
sudo find /usr/lib -name PCLConfig.cmake 2>/dev/null | head
```

3. 重新构建：

```bash
rm -rf build
make tgz
```

---

### 一个提醒

你这容器里之前能看到 `/usr/bin/pcl_*` 和 `/usr/include/pcl`，但 `PCLConfig.cmake` 没有，说明以前那套 PCL 是“半装”的。移走后如果你还需要这些工具/头文件，安装 `libpcl-dev` 会一并补齐。

如果你希望我帮你把“只禁用 1.13，但保留别的 pcl 工具/头文件”做得更精细，也可以——但上面这套是最稳、可回滚、对编译干扰最小的做法。
