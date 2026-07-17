# Git 使用指南 — blade_measurement 项目

> 面向 Git 新手。按本文操作即可完成日常的「改代码 → 校验 → 提交 → 推送」。
> 记录时间：2026-07-17　目标远程平台：GitHub

---

## 0. 先理解几个概念（一分钟）

| 概念 | 大白话 |
|---|---|
| **仓库 repository** | 一个被 Git 管理的项目文件夹，里面有个隐藏的 `.git` 目录记录所有历史 |
| **工作区 working tree** | 你实际在编辑的文件 |
| **暂存区 staging / index** | 「准备提交」的清单。改了文件不代表会被提交，得先 `git add` 放进暂存区 |
| **提交 commit** | 给当前暂存区拍一张快照，永久存进历史，可随时回退 |
| **分支 branch** | 一条独立的开发线。`main` 是主线，新功能在新分支上做，做完再合并 |
| **远程 remote** | GitHub 上的那份仓库副本，名字通常叫 `origin` |
| **推送 push / 拉取 pull** | 把本地提交上传到 GitHub 叫 push，把 GitHub 的更新下载下来叫 pull |

核心心法：**改动不会自动保存进历史**。永远是三步 —— `add`（选要提交的）→ `commit`（存快照）→ `push`（传上去）。

---

## 1. 一次性准备（只做一次）

### 1.1 配置身份（提交记录里会显示是谁提交的）

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱@example.com"
```

> 当前这台机器还没配置过，第一次提交前**必须**先跑这两行，否则提交会报错。
> 邮箱建议用 GitHub 账号的邮箱。

### 1.2 让中文文件名正常显示（Windows 建议）

```bash
git config --global core.quotepath false
```

### 1.3 换行符处理（Windows 建议）

```bash
git config --global core.autocrlf true
```

---

## 2. 日常操作流程（每天都用）

### 2.1 看看现在改了啥 —— `git status`

```bash
git status
```

输出里：
- 红色 `Changes not staged` = 改了但没 add
- 绿色 `Changes to be committed` = 已 add，等着 commit
- `Untracked files` = 全新的、Git 还没管的文件

**任何操作前先跑一次 `git status`，看清状态再动手。**

### 2.2 看具体改了哪几行 —— `git diff`

```bash
git diff                 # 看还没 add 的改动
git diff --staged        # 看已经 add 的改动
git diff mainwindow.cpp  # 只看某个文件
```

### 2.3 把改动放进暂存区 —— `git add`

```bash
git add mainwindow.cpp mainwindow.h   # 加指定文件（推荐，精确）
git add pointcloud/                    # 加整个文件夹
git add -A                             # 加全部改动（谨慎，可能带上不想提交的）
```

> 推荐用「加指定文件」，避免手滑把临时文件、密钥、构建产物一起提交。

### 2.4 提交 —— `git commit`

```bash
git commit -m "修复：拟合球函数重复实现，合并为 fitSphere3D"
```

提交信息（`-m` 后面那句）写清楚**做了什么**。建议格式：

```
类型：一句话说明

类型可选：
  新增 / feat     新功能
  修复 / fix      修 bug
  重构 / refactor 不改功能只调结构（本次代码整理大多属于这类）
  文档 / docs     只改文档
  清理 / chore    删无用文件、调依赖
```

例：`重构：mainwindow.cpp 按功能分区`、`清理：删除未使用的 QXlsx 副本文件夹`

### 2.5 校验（提交前/后确认没问题）

Git 层面的校验：

```bash
git status          # 确认该提交的都提交了，工作区干净
git log --oneline -10   # 看最近 10 条提交历史对不对
```

代码层面的校验（本项目用 Qt Creator + MSVC 2019 + Ninja）：

1. 在 Qt Creator 里重新构建（Release 目录：`build/Desktop_Qt_5_15_2_MSVC2019_64bit-Release/`）
2. 确认**编译通过、能正常运行**再推送
3. 重构类改动尤其要跑一遍关键功能（点云加载、重建、STEP 导出）

> 原则：**先本地编译通过，再推送**。不要把编译不过的代码推上去。

### 2.6 推送到 GitHub —— `git push`

```bash
git push                       # 已建立跟踪后，直接推
git push -u origin main        # 第一次推某分支时，-u 建立跟踪关系
```

---

## 3. 分支工作流（做较大改动时用，比如这次代码整理）

不要直接在 `main` 上做大改动。正确做法：开新分支 → 做 → 验证 → 合并回 main。

```bash
# 1. 从 main 建并切到新分支
git checkout -b refactor/mainwindow-split

# 2. 在分支上正常 add / commit（可多次提交）
git add mainwindow.cpp
git commit -m "重构：拆分点云相关槽函数到独立分区"

# 3. 推送分支到 GitHub
git push -u origin refactor/mainwindow-split

# 4. 验证 OK 后，切回 main 合并
git checkout main
git merge refactor/mainwindow-split

# 5. 推送 main
git push

# 6.（可选）删掉已合并的分支
git branch -d refactor/mainwindow-split
```

分支命名建议：`refactor/xxx`（重构）、`fix/xxx`（修 bug）、`feat/xxx`（新功能）。

---

## 4. 后悔药（改错了怎么办）

```bash
# 撤销某文件「还没 add」的改动，恢复到上次提交的样子（改动会丢失，慎用）
git checkout -- mainwindow.cpp

# 把已 add 的文件撤出暂存区（改动还在，只是不提交了）
git restore --staged mainwindow.cpp

# 看历史，找到某次提交的 ID（前几位就够）
git log --oneline

# 临时回到某次提交看看（不改变历史）
git checkout <提交ID>
git checkout main     # 看完切回来

# 已经 commit 但还没 push，想改最后一次提交信息
git commit --amend -m "新的提交信息"
```

> ⚠️ 危险命令别乱用：`git reset --hard`、`git push -f`、`git clean -f` 会**永久丢失**改动。
> 拿不准时先 `git status` + `git log`，或者先复制一份文件夹做备份。

---

## 5. 忽略不该提交的文件 —— `.gitignore`

项目根目录的 `.gitignore` 文件列出**不纳入 Git 管理**的东西，比如：
- `build/`（构建产物，98MB，绝不提交）
- IDE 缓存、临时文件
- 密钥、本地配置

本项目的 `.gitignore` 已按下面「附录」配置好。以后想忽略新东西，往里加一行文件/文件夹名即可。

---

## 6. 常见问题

**Q：`git push` 报错要账号密码 / 认证失败？**
GitHub 已不支持密码推送，需要用 **Personal Access Token (PAT)** 或 SSH key：
- PAT：GitHub → Settings → Developer settings → Personal access tokens，生成后当密码用。
- SSH：`ssh-keygen` 生成密钥，把公钥贴到 GitHub → Settings → SSH keys，远程地址用 `git@github.com:用户名/仓库.git`。

**Q：中文提交信息乱码？**
见 1.2 的 `core.quotepath false`。

**Q：`build/` 已经被提交了怎么移除？**
```bash
git rm -r --cached build/    # 从 Git 移除但保留本地文件
git commit -m "清理：移除误提交的 build 目录"
```

**Q：提交历史里想看某文件的演变？**
```bash
git log --oneline -- pointcloud/Point_Algorithm.cpp
```

---

## 附录：本项目初始化步骤（建仓库时用，只做一次）

以下命令在 `blade_measurement_v3.3.4` 目录下执行：

```bash
# 1. 初始化仓库
git init

# 2. 配置身份（若全局没配过）
git config user.name "你的名字"
git config user.email "你的邮箱@example.com"

# 3. .gitignore 已就位（见项目根目录）

# 4. 首次提交
git add -A
git commit -m "chore: 初始化仓库，纳入 v3.3.4 代码"

# 5. 在 GitHub 上建一个空仓库（不要勾 README/.gitignore），拿到地址后：
git remote add origin https://github.com/你的用户名/blade_measurement.git
git branch -M main
git push -u origin main
```

> 之后日常就只用第 2 节的流程。
