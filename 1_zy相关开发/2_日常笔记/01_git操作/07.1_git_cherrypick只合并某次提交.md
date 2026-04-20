`git cherry-pick` 的作用是：

**把“某一次提交”拣出来，应用到你当前所在的分支上。**

你可以把它理解成：

* `merge` 是把“整个分支的历史”合过来
* `cherry-pick` 是只挑“某几个提交”拿过来

---

## 1. 一个直观例子

假设现在有两个分支：

* `main`
* `dev`

你在 `dev` 上做了 3 次提交：

* A：改日志
* B：修一个 bug
* C：改 README

但你现在只想把 **B 这个 bug 修复** 放到 `main`，不想把 A 和 C 一起带过去。

这时就很适合用：

```bash
git checkout main
git cherry-pick <B的提交id>
```

这样 Git 会把 B 这个提交的改动，重新在 `main` 上生成一条新提交。

---

## 2. 它到底做了什么

注意，`cherry-pick` **不是把原来的提交“搬过去”**，而是：

1. 找到那个提交里的改动
2. 在当前分支上尝试重新应用一遍
3. 生成一个新的提交

所以即使内容一样，**新提交的 commit hash 也会不一样**。

比如：

* `dev` 上原提交：`abc123`
* `main` 上 cherry-pick 后的新提交：`def456`

内容可能一样，但它们不是同一个 commit。

---

## 3. 基本用法

### 挑一个提交

```bash
git cherry-pick <commit-id>
```

例如：

```bash
git cherry-pick a1b2c3d
```

---

### 挑多个不连续提交

```bash
git cherry-pick <id1> <id2> <id3>
```

例如：

```bash
git cherry-pick a1b2c3d e4f5g6h i7j8k9l
```

---

### 挑一段连续提交

```bash
git cherry-pick <start>^..<end>
```

例如：

```bash
git cherry-pick A^..C
```

意思是把 **A、B、C** 都挑过来。

如果写成：

```bash
git cherry-pick A..C
```

通常表示 **不包含 A，只包含 A 后面的到 C**，也就是可能是 **B、C**。

这个地方很多人容易搞混。

---

## 4. 常见使用场景

### 场景 1：把 bugfix 从开发分支拿到发布分支

比如：

* `dev`：日常开发
* `release`：准备上线

你只想把某个修 bug 的提交拿到 `release`：

```bash
git checkout release
git cherry-pick <bugfix-commit>
```

---

### 场景 2：误提交到了错误分支

本来应该提交到 `feature-a`，结果你提交到了 `main`。

你可以：

1. 记下这个提交 id
2. 切到正确分支
3. `cherry-pick` 过去

```bash
git checkout feature-a
git cherry-pick <commit-id>
```

然后再回 `main` 把那次误提交处理掉。

---

### 场景 3：只想复用某次改动，不想合并整个分支

别人一个分支上有很多实验代码，但你只想拿里面一个小修复。

这时 `merge` 不合适，`cherry-pick` 很合适。

---

## 5. 最常见的操作流程

先看历史：

```bash
git log --oneline --graph --all
```

找到你想挑的提交，比如：

```bash
f3a8c21 fix: 修复登录超时
```

然后切到目标分支：

```bash
git checkout main
```

执行：

```bash
git cherry-pick f3a8c21
```

成功后，`main` 上就会多一个新提交。

---

## 6. 如果发生冲突怎么办

`cherry-pick` 本质上也是“把改动应用到当前代码”，所以也可能冲突。

比如执行：

```bash
git cherry-pick f3a8c21
```

Git 提示冲突了。

这时步骤通常是：

### 第一步：看冲突文件

```bash
git status
```

### 第二步：手动改冲突

打开文件，把冲突标记处理掉。

冲突通常长这样：

```txt
<<<<<<< HEAD
当前分支内容
=======
被 cherry-pick 过来的内容
>>>>>>> f3a8c21
```

你需要自己决定保留什么。

### 第三步：标记冲突已解决

```bash
git add <冲突文件>
```

### 第四步：继续 cherry-pick

```bash
git cherry-pick --continue
```

---

## 7. 如果不想继续了

### 放弃这次 cherry-pick

```bash
git cherry-pick --abort
```

这会回到 cherry-pick 之前的状态。

---

### 跳过当前这个提交

当你在一次挑多个提交时，某一个不想要了：

```bash
git cherry-pick --skip
```

---

## 8. 很有用的参数

### `-x`

会在新提交信息里加一句：

```txt
(cherry picked from commit xxxxxx)
```

命令：

```bash
git cherry-pick -x <commit-id>
```

这个在团队协作里很有用，方便追踪“这个提交是从哪里拣过来的”。

---

### `-n` / `--no-commit`

只应用改动，但**先不自动提交**：

```bash
git cherry-pick -n <commit-id>
```

适合你想：

* 先看改动
* 再继续改一改
* 最后手动 `git commit`

---

## 9. 和 merge 的区别

### `merge`

```bash
git merge dev
```

意思是把 `dev` 分支整体并到当前分支。

特点：

* 会带来整个分支的提交历史
* 适合真正分支合流

### `cherry-pick`

```bash
git cherry-pick <commit-id>
```

意思是只拿某一次提交的改动。

特点：

* 精准
* 灵活
* 但如果用太多，历史会变得比较碎，甚至出现“内容相同但 commit 不同”的重复提交

---

## 10. 和 rebase 的区别

### `rebase`

是把“一串提交”换个基底重新排一遍。

### `cherry-pick`

是把“指定提交”单独拿出来应用。

你可以理解为：

* `rebase` 更像整段搬迁
* `cherry-pick` 更像精确摘樱桃

---

## 11. 一个完整示例

假设提交历史如下：

```txt
main:    A---B
dev:          C---D---E
```

你只想把 `D` 拿到 `main`：

```bash
git checkout main
git cherry-pick D
```

结果会变成：

```txt
main:    A---B---D'
dev:          C---D---E
```

这里的 `D'` 表示：

* 内容来自 D
* 但它是一个新的提交，不是原来的 D

---

## 12. 常见坑

### 1）重复 cherry-pick

你可能把已经合过来的提交又 pick 一次，导致：

* 代码重复冲突
* 历史难看

---

### 2）pick 依赖链中的中间提交

有的提交不是独立的，它依赖前面的提交。

比如：

* A：新增一个函数
* B：修改这个函数

如果你只 pick B，不 pick A，就可能冲突，甚至代码坏掉。

所以 cherry-pick 之前要先看清楚这个提交是否能独立存在。

---

### 3）在公共分支上乱操作

如果团队里很多人都在用同一个分支，频繁 cherry-pick 可能让历史不好理解。

---

## 13. 你可以这样记

一句话记忆：

**`git cherry-pick` = 把别处的某个提交，复制一份改动到当前分支。**

---

## 14. 最实用的几个命令

```bash
# 查看提交历史
git log --oneline --graph

# 切到目标分支
git checkout main

# 挑一个提交
git cherry-pick <commit-id>

# 挑多个提交
git cherry-pick <id1> <id2>

# 出冲突后继续
git cherry-pick --continue

# 放弃
git cherry-pick --abort

# 记录原始提交来源
git cherry-pick -x <commit-id>
```

---

## 15. 一个特别实战的例子

比如你在 `dev` 上提交了：

```bash
git log --oneline
```

输出：

```txt
c333333 docs update
b222222 fix login timeout
a111111 refactor auth
```

你现在只想把 `fix login timeout` 拿到 `main`：

```bash
git checkout main
git cherry-pick b222222
```

这样不会把 `docs update` 和 `refactor auth` 一起带过去。

这就是它最经典的用途。

---

如果你愿意，我可以接着给你讲两个很实用的内容之一：
**1）cherry-pick 冲突时怎么处理**，或者 **2）结合分支图给你画一个特别容易懂的示意图**。
