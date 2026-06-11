非常好 👍 我来一步步教你**如何配置本地 Git 并上传代码到 GitHub 云端**。以下步骤适用于 **Ubuntu / Linux / macOS / Windows（WSL）** 等环境。

---

## 🚀 一、检查是否安装 Git

```bash
git --version
```

如果没有安装，执行：

```bash
sudo apt install git -y
```

---

## 🧩 二、配置用户信息（全局一次性配置）

```bash
git config --global user.name "你的GitHub用户名"
git config --global user.email "你的GitHub邮箱"
```

查看配置是否成功：

```bash
git config --list
```

---

## 🔑 三、生成 SSH 密钥（推荐）

1. 生成密钥（用你在 GitHub 注册时的邮箱）：

   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ```

   一路回车即可。

2. 查看公钥内容：

   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```

3. 复制显示的整行内容。

4. 打开 GitHub → 点击右上角头像 → **Settings → SSH and GPG keys → New SSH key**
   粘贴刚刚复制的公钥。

5. 验证是否连通：

   ```bash
   ssh -T git@github.com
   ```

   若看到：

   ```
   Hi yourname! You've successfully authenticated...
   ```

   表示成功 ✅

---

## 📂 四、在本地创建一个项目仓库

假设你有一个项目文件夹：

```bash
cd ~/workspace/myproject
git init
```

添加文件并提交：

```bash
git add .
git commit -m "Initial commit"
```

---

## ☁️ 五、在 GitHub 创建一个远程仓库

1. 登录 GitHub → 点击右上角 “+” → **New repository**
2. 仓库名例如：`myproject`
3. 可选是否公开（Public / Private）
4. 不要勾选 “Initialize this repository with a README”
5. 创建后，你会看到一个类似的远程地址：

   ```
   git@github.com:yourname/myproject.git
   ```

---

## 🔗 六、将本地仓库与 GitHub 远程仓库关联

```bash
git remote add origin git@github.com:yourname/myproject.git
```

---

## ⬆️ 七、推送到 GitHub

```bash
git branch -M main
git push -u origin main
```

如果是第一次推送，会要求输入 SSH 确认，之后就不用了。

---

## ✅ 八、常见更新操作

日常更新推送命令如下：

```bash
git add .
git commit -m "update something"
git push
```

如果想拉取远端最新代码：

```bash
git pull
```

---

是否希望我帮你**生成一整套命令模板**（包含替换占位符的版本，比如用户名、邮箱、仓库名），你只要复制粘贴就能上传？
