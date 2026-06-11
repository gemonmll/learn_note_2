下面按“**用 Git 备份 AI agent 自动改代码**”这个目标来讲：先把**基础命令**讲清楚，再把你说的 **Code 插件（我按 VS Code 的 Git/插件体系来讲）**展开，最后给一套你可以直接照抄的 **AI 修改代码安全工作流**。

---

## 1) 你要解决的核心问题：AI 改代码怎么“可控 + 可回退 + 可对比”

Git 最适合干三件事：

1. **每次 AI 改动都有记录**（commit）
2. **随时能看差异**（diff）
3. **随时能撤回/回滚到任意版本**（restore/reset/revert）

对 AI agent 来说最关键的是：

> **在运行 agent 前先做一个“干净基线”提交；agent 跑完后只把你认可的变更提交。**
> 这样即使 AI 把项目改炸了，你也能一键回到跑之前。

---

## 2) Git 基础：从 0 到能备份（命令行版）

### 2.1 初始化仓库 & 第一次提交

在项目目录：

```bash
git init
git status
```

创建 `.gitignore`（非常重要，避免把日志/缓存/密钥提交上去），常见例子：

* Python: `__pycache__/`, `.venv/`, `.env`
* Node: `node_modules/`, `dist/`
* 通用: `.DS_Store`, `*.log`

然后首提：

```bash
git add .
git commit -m "chore: initial commit"
```

### 2.2 每天最常用的 6 个命令

* 看状态（有没有改动/哪些未暂存）：

```bash
git status
```

* 看改了啥（非常适合审 AI 改动）：

```bash
git diff
```

* 把改动放进暂存区（准备提交）：

```bash
git add <file>
# 或全部
git add .
```

* 提交（形成一个“可回退点”）：

```bash
git commit -m "feat: ... / fix: ... / chore: ..."
```

* 看历史：

```bash
git log --oneline --graph --decorate
```

* 快速看某个文件的变更历史：

```bash
git blame <file>
```

### 2.3 撤销/回退：AI 改炸了怎么办

这里分三种“后悔”：

**A) 还没 add / commit（只是工作区改了）**

* 丢弃某文件改动，恢复到上次提交：

```bash
git restore <file>
```

* 全丢弃：

```bash
git restore .
```

**B) 已经 add 了，但还没 commit（暂存区后悔）**

* 取消暂存（不丢改动）：

```bash
git restore --staged <file>
```

**C) 已经 commit 了（提交后后悔）**

* 安全做法：`revert`（生成一个“反向提交”，适合已经 push 的情况）：

```bash
git revert <commit_hash>
```

* 危险但有用：`reset`（改历史，适合自己本地、还没 push）

  * 回到某个提交，保留改动在工作区：

  ```bash
  git reset --soft <commit_hash>
  ```

  * 回到某个提交，连改动也不要：

  ```bash
  git reset --hard <commit_hash>
  ```

> 对“AI 自动改代码”而言，最常用的是：
> **跑 agent 前做 commit；跑完不满意就 `git restore .` 或直接 `git reset --hard <基线提交>`**

---

## 3) 分支思维：给 AI 一条“试验田”

强烈建议：**每次让 agent 大改之前，开一个分支**。

```bash
git checkout -b agent/<日期-简述>
# 例：git checkout -b agent/2025-12-25-refactor-auth
```

改完你审核通过后，再合并回主分支：

```bash
git checkout main
git merge agent/2025-12-25-refactor-auth
```

如果 agent 把事情搞复杂了，你直接删分支也行（主分支不受影响）：

```bash
git branch -D agent/2025-12-25-refactor-auth
```

---

## 4) 远程备份：GitHub/GitLab = “异地存档”

本地 Git 只是版本管理；要“备份”，还得 push 到远程。

```bash
git remote add origin <你的仓库地址>
git push -u origin main
```

之后日常就是：

```bash
git push
```

以及从远程更新：

```bash
git pull
```

> 习惯：**每次完成一个可运行的阶段就 push**
> AI agent 的改动更要勤 push，避免本地翻车丢记录。

---

## 5) 专门为“AI agent 自动修改”设计的工作流（照做就稳）

### 工作流 A：最稳（推荐）

1. 确保工作区干净：

```bash
git status
```

2. 拉最新（如果多人协作）：

```bash
git pull
```

3. 开 agent 分支：

```bash
git checkout -b agent/<任务名>
```

4. **做基线提交**（哪怕只是 “chore: baseline before agent”）

```bash
git add .
git commit -m "chore: baseline before agent changes"
```

5. 运行 agent（它会改一堆文件）
6. 审查差异（重点！）：

```bash
git diff
```

7. 只暂存你认可的文件：

```bash
git add <file1> <file2>
```

8. 跑测试/lint（如果有）
9. 提交：

```bash
git commit -m "feat: agent implemented XXX"
```

10. push 备份：

```bash
git push -u origin agent/<任务名>
```

11. 最后合并回 main（或走 PR）

### 工作流 B：AI 改太多，想“分批提交”

* 先 `git add -p` 交互式挑选变更块（超级适合审 AI）：

```bash
git add -p
```

你可以逐段选择 `y/n/s`，把一个大改动拆成多个小提交，更好回溯。

### 工作流 C：AI 经常改配置/生成文件，搞乱 diff

* 用 `.gitignore` 忽略生成物
* 或用 `.gitattributes` 降噪（例如把某些文件标为 linguist-generated 或设置 diff 策略）
* 关键是：**确保“AI 的输出产物”和“你真正维护的源码”分开**（比如 `outputs/`、`artifacts/` 目录）

---

## 6) VS Code（Code 插件）怎么用：从“点点点”到高效审 AI

我按 VS Code 里最常见的 Git 体系讲：**内置 Source Control + 常用扩展**。

### 6.1 VS Code 内置 Git（不用装插件也有）

左侧图标通常有个“分叉”的 **源代码管理(Source Control)**：

你能做的事：

* **查看改动列表**（非常适合 AI 改完一眼扫）
* 点文件直接看 **Diff 对比**
* **Stage/Unstage**（暂存/取消暂存）
* 填写 message 一键 commit
* 右下角切分支、创建分支、切换分支
* 解决冲突：冲突文件会标记，界面给你 “Accept Current / Incoming / Both” 等按钮

> 审 AI 改动的最佳姿势：
> **改完先不急着 commit → 在 Source Control 里逐文件点开 diff → 只 stage 你认可的。**

### 6.2 强烈推荐的 VS Code Git 扩展（“插件”）

（1）**GitLens**（最推荐）

* 每行是谁改的、什么时候改的（blame）
* 文件历史、提交历史、对比任意两个提交
* 很适合追踪 “agent 改了哪几行导致 bug”

（2）**GitHub Pull Requests and Issues**

* 直接在 VS Code 里提 PR、看 Review、处理评论
* 团队协作更顺

（3）**Conventional Commits**

* 帮你写规范提交信息（`feat:` `fix:` `chore:`）
* AI 改动很容易提交信息乱，有这个更统一

> 如果你说的“code插件”是 JetBrains 系列（IDEA/PyCharm）也没问题：逻辑完全一样——核心是 **diff 审查、分批暂存、分支隔离、提交与推送**。

---

## 7) 进阶但很实用：防止 AI 把密钥/垃圾提交上去

### 7.1 本地密钥：永远不要进 Git

* `.env`、`config.local.json` 一律 `.gitignore`
* 用 `.env.example` 提供模板

### 7.2 pre-commit 钩子（可选但强）

你可以加 **pre-commit**（或简单 git hook）在 commit 前自动跑：

* 格式化（black/prettier）
* lint（ruff/eslint）
* 测试（pytest）
* secrets 扫描（例如 gitleaks）

这样 AI 改出不合规代码时，提交会被拦住。

---

## 8) 给你一套“最小可落地清单”（你今天就能开始用）

1. 初始化仓库 + `.gitignore`
2. main 分支保持可运行
3. 每次 agent 大改：

* `git checkout -b agent/xxx`
* `git commit -m "chore: baseline..."`
* agent 跑完 → VS Code diff 审查 → 分批 stage → commit
* `git push` 远程备份

4. 不满意：`git restore .` 或删分支

---

如果你告诉我你用的是：

* VS Code / Cursor / Windsurf / JetBrains / 还是某个叫 “Code” 的特定编辑器插件（以及你的 agent 形态：Claude Code / OpenAI Codex CLI / Aider / 自己写的脚本），
  我可以把上面流程再“贴合你工具的具体按键/命令”，甚至给你一份**适配你项目语言的 `.gitignore + pre-commit 配置模板`**。
