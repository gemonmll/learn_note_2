使用 Git LFS（Git Large File Storage）上传大文件的步骤如下：

### 1. 安装 Git LFS

首先，确保你已经安装了 Git LFS。如果没有安装，可以通过以下命令进行安装：

#### 对于 macOS：

```bash
brew install git-lfs
```

#### 对于 Windows：

下载并运行 Git LFS 安装程序：[Git LFS 下载](https://git-lfs.github.com/)

#### 对于 Linux：

```bash
sudo apt-get install git-lfs
```

安装完成后，运行以下命令初始化 Git LFS：

```bash
git lfs install
```

### 2. 跟踪大文件

通过 `git lfs track` 命令指定你希望 Git LFS 跟踪的大文件类型。例如，如果你想上传 `.zip` 文件：

```bash
git lfs track "*.zip"
```

这会在你的项目中生成一个 `.gitattributes` 文件，里面记录了 Git LFS 应该跟踪的文件类型。你可以打开 `.gitattributes` 文件，检查跟踪规则。

### 3. 添加大文件到 Git 仓库

将大文件添加到 Git 仓库，就像平时操作 Git 一样：

```bash
git add <your-large-file.zip>
```

Git 会自动将符合跟踪规则的文件使用 LFS 存储，而不是直接存储在 Git 中。

### 4. 提交文件

提交你添加的大文件到 Git 仓库：

```bash
git commit -m "Add large file"
```

### 5. 推送到远程仓库

最后，使用 `git push` 将文件推送到远程 Git 仓库：

```bash
git push origin <your-branch-name>
```

Git LFS 会自动将大文件上传到 Git LFS 服务器，并将其对应的指针文件（轻量级的文本文件）推送到 Git 仓库中。

### 6. 验证文件是否正确上传

你可以使用 `git lfs ls-files` 命令来查看哪些文件是由 Git LFS 跟踪和存储的：

```bash
git lfs ls-files
```

这会列出所有被 LFS 跟踪和上传的文件。

### 注意事项

* Git LFS 会替代 Git 用来存储大文件的方式，因此会减轻 Git 仓库的体积。
* 上传的文件会存储在 Git LFS 专用的服务器上，通常这意味着你需要使用 Git LFS 兼容的托管平台（如 GitHub, GitLab, Bitbucket 等）。
* Git LFS 的使用通常有存储配额限制，不同的服务平台可能会有不同的免费/付费配额。

这样，你就可以使用 Git LFS 上传和管理大文件了！


-------------------------

## 第二部分 取消追踪
从 Git LFS 中删除大文件追踪，分两种情况：

## 情况 1：以后不再用 LFS 追踪某类文件

比如你之前执行过：

```bash
git lfs track "*.zip"
```

现在想取消 `.zip` 文件的 LFS 追踪：

```bash
git lfs untrack "*.zip"
```

然后提交 `.gitattributes` 的变化：

```bash
git add .gitattributes
git commit -m "Stop tracking zip files with Git LFS"
```

可以查看当前 LFS 追踪规则：

```bash
git lfs track
```

或者直接看：

```bash
cat .gitattributes
```

---

## 情况 2：某个文件已经被 LFS 管理，想改回普通 Git 文件

假设文件是：

```bash
large_file.zip
```

步骤如下：

```bash
git lfs untrack "large_file.zip"
git rm --cached large_file.zip
git add .gitattributes
git add large_file.zip
git commit -m "Move large_file.zip out of Git LFS"
```

这里重点是：

```bash
git rm --cached large_file.zip
```

它不会删除你本地文件，只是把它从 Git 索引里移除一次。然后重新 `git add`，Git 就会按新的规则把它当普通文件提交。

---

## 情况 3：想从仓库历史中彻底删除 LFS 大文件

如果你已经把大文件提交到了历史里，仅仅 `git lfs untrack` 不会删除历史记录中的 LFS 对象。

这时要重写历史，可以用：

```bash
git lfs migrate export --include="large_file.zip"
```

这会把 LFS 文件转回普通 Git 文件。

如果你想从历史中彻底删除这个文件，一般用：

```bash
git filter-repo --path large_file.zip --invert-paths
```

然后强推：

```bash
git push --force --all
git push --force --tags
```

注意：这会改写历史，团队协作时要非常谨慎，其他人需要重新拉仓库或重新 rebase。

---

## 常用检查命令

查看哪些文件还在 LFS 管理：

```bash
git lfs ls-files
```

查看 LFS 追踪规则：

```bash
git lfs track
```

查看 `.gitattributes`：

```bash
cat .gitattributes
```

---

最常见的流程是：

```bash
git lfs untrack "*.zip"
git rm --cached your_file.zip
git add .gitattributes
git add your_file.zip
git commit -m "Stop tracking zip files with LFS"
git push
```

核心理解是：

```bash
git lfs untrack
```

只是不再追踪新添加的文件；
已经被 LFS 管理的文件，需要 `git rm --cached` 后重新 `git add`，才会按普通 Git 文件重新提交。

