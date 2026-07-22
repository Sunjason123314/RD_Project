# Git 使用指南 — blade_measurement 项目

> 面向**完全没用过 Git 的新手**。从「装软件、建仓库」讲起，一直到「日常提交、下载、更新」。
> 更新时间：2026-07-18　远程平台：GitHub（私有仓库）
> 远程地址：`https://github.com/Sunjason123314/blade_measurement.git`

---

## ⭐ 速查表（装好、建好仓库后，日常照抄这几条）

> 前提：终端已在项目目录里（提示符显示 `...blade_measurement_v3.3.4>`）。
> 若不在，先切进去：`cd /d e:\Code\QT\blade_measurement\blade_measurement_v3.3.4`

| 我想做什么 | 命令 |
|---|---|
| 改了几个文件，全部上传 | `git add -A` → `git commit -m "说明"` → `git push` |
| 只上传其中一部分 | `git add 文件名` → `git commit -m "说明"` → `git push` |
| 换电脑第一次下载 | `git clone https://github.com/Sunjason123314/blade_measurement.git` |
| 同步 GitHub 最新版 | `git pull` |
| 看当前状态 | `git status` |
| 看提交历史 | `git log --oneline` |

核心心法：**改动不会自动进历史**。永远是三步 —— `add`（选要提交的）→ `commit`（存快照）→ `push`（传上去）。

---

## 0. 先理解几个概念（一分钟）

| 概念 | 大白话 |
|---|---|
| **仓库 repository** | 一个被 Git 管理的项目文件夹，内含隐藏的 `.git` 目录记录所有历史 |
| **工作区 working tree** | 你实际在编辑的文件 |
| **暂存区 staging** | 「准备提交」的清单。改了文件不代表会提交，得先 `git add` 放进来 |
| **提交 commit** | 给暂存区拍一张快照，永久存进本地历史，可随时回退 |
| **远程 remote** | GitHub 上那份仓库副本，名字通常叫 `origin` |
| **推送 push / 拉取 pull** | 本地提交上传 GitHub 叫 push；把 GitHub 更新下载下来叫 pull |
| **克隆 clone** | 第一次把整个仓库从 GitHub 下载到本地 |

---

## 1. 从零开始：安装 Git 与一次性配置（每台新电脑只做一次）

### 1.1 装 Git

到官网 <https://git-scm.com/download/win> 下载 Windows 版，双击安装，一路默认「Next」即可。
装完验证——打开命令提示符（按 `Win`，输入 `cmd` 回车），敲：
```bash
git --version
```
显示出版本号（如 `git version 2.xx`）就说明装好了。

### 1.2 配置你的身份（提交记录里会显示是谁提交的，必须先做）

```bash
git config --global user.name "你的名字"
git config --global user.email "你的GitHub邮箱@example.com"
```
> `--global` 表示对这台电脑上所有仓库生效，只需配一次。邮箱建议用 GitHub 账号的邮箱。

### 1.3 Windows 建议的两条设置

```bash
git config --global core.quotepath false   # 中文文件名正常显示，不再是数字乱码
git config --global core.autocrlf true      # 自动处理 Windows/Linux 换行符差异
```

### 1.4 验证配置

```bash
git config --global --list
```
能看到刚才填的 `user.name`、`user.email` 就对了。

---

## 2. 创建仓库的两种方式

你会遇到两种起步情况，对号入座：

### 方式一：本地已经有一堆代码，想开始用 Git 管理它（本项目当初就是这么做的）

**第 1 步：把项目文件夹变成 Git 仓库**
用终端进到你的项目文件夹，然后初始化：
```bash
cd /d e:\你的项目路径
git init
```
`git init` 会创建隐藏的 `.git` 目录，从此这个文件夹就被 Git 管理了。

**第 2 步：写 `.gitignore`（告诉 Git 哪些别管）**
在项目根目录新建一个名为 `.gitignore` 的文件，把不该提交的写进去，例如：
```
build/
*.obj
*.exe
*.user
.vscode/
```
> 否则会把上百 MB 的构建产物、临时文件全提交进去，仓库会又大又乱。

**第 3 步：第一次提交（建立第一个 commit）**
```bash
git add -A                          # 把所有该管的文件放进暂存区
git commit -m "chore: 初始化仓库"    # 拍下第一张快照
```
> 这就是你的**第一个 commit**。此时代码还只在本地，GitHub 上还没有。

**第 4 步：在 GitHub 网页上建一个空仓库**
1. 登录 <https://github.com>，点右上角 `+` → `New repository`
2. 填仓库名（如 `blade_measurement`）
3. 选 **Private（私有）**
4. **不要**勾选 "Add a README" / "Add .gitignore" / "license"（保持空仓库，否则会冲突）
5. 点 `Create repository`，页面会给你一个地址 `https://github.com/你的用户名/仓库名.git`

**第 5 步：关联远程并首次推送**
```bash
git remote add origin https://github.com/你的用户名/仓库名.git   # 关联 GitHub
git branch -M main                                              # 主分支命名为 main
git push -u origin main                                         # 首次推送，-u 建立跟踪
```
> 若这一步报 `Connection was reset`，是网络问题，见第 5 节配代理。
> 之后日常推送就只用 `git push`，不用再加 `-u origin main`。

### 方式二：GitHub 上已经有仓库，我要把它下载到本地 —— `git clone`

在你想放代码的**上级文件夹**里执行（不要在已有项目里执行）：
```bash
git clone https://github.com/Sunjason123314/blade_measurement.git
```
它会自动新建 `blade_measurement` 文件夹，把整个项目和全部历史下载下来，且已自动关联好远程。进去就能直接开始 `git pull` / `git push`。

---

## 3. 日常操作详解（每天都用）

### 3.1 先看状态 —— `git status`

```bash
git status
```
- `Changes not staged` = 改了但还没 `add`
- `Changes to be committed` = 已 `add`，等着 `commit`
- `Untracked files` = 全新文件，Git 还没管
- `nothing to commit, working tree clean` = 干净，没有未保存的改动

**任何操作前先跑一次 `git status`，看清状态再动手。**

### 3.2 看具体改了哪几行 —— `git diff`

```bash
git diff                        # 看还没 add 的改动
git diff app/mainwindow.cpp     # 只看某个文件
```

### 3.3 放进暂存区 —— `git add`（这一步决定「这次推哪些」）

```bash
git add app/mainwindow.cpp app/mainwindow.h   # 只挑指定文件（推荐，精确）
git add app/                                   # 加整个文件夹
git add -A                                     # 加全部改动
```

> **这就是「只推一部分」的关键**：你 `add` 了哪些，`commit` 就只包含哪些，`push` 也就只推哪些。
> 没 `add` 的文件改动会留在本地，不受影响，下次想推再单独 `add` + `commit`。

**加错了想撤出暂存区**（文件改动保留，只是这次不提交）：
```bash
git restore --staged app/mainwindow.cpp
```

### 3.4 提交 —— `git commit`（建立一个新 commit）

```bash
git commit -m "修复：点云加载崩溃问题"
```
`-m` 后面写清楚**做了什么**。建议格式 `类型：说明`：

| 类型 | 含义 |
|---|---|
| feat / 新增 | 新功能 |
| fix / 修复 | 修 bug |
| refactor / 重构 | 不改功能只调结构 |
| docs / 文档 | 只改文档 |
| chore / 清理 | 删无用文件、调依赖 |

> 什么时候该 commit？**完成一件有意义的事就 commit 一次**（修好一个 bug、加完一个功能）。不用每改一行都提交。

### 3.5 推送 —— `git push`

```bash
git push                    # 日常就这一个词
git push group main
git push -u origin main     # 仅「第一次推某分支」时用（本项目 main 已建立，不用再加）
```

---

## 4. 更新代码 —— `git pull`

已有本地代码，想同步 GitHub 上的最新版（别人推的、或你在另一台电脑推的）：
```bash
git pull
```
> **好习惯**：每次开始工作前先 `git pull`，拿到最新代码再改，能减少冲突。
> 若提示冲突（conflict），先别硬来，把提示贴出来找人帮忙。
>
> （第一次下载用 `git clone`，见第 2 节方式二。）

---

## 5. 网络问题（国内访问 GitHub 常见）

`git push` / `git clone` 报 `Connection was reset` 或卡住，几乎都是网络到不了 GitHub。
注意：`ping github.com` 能通**不代表** git 能推（走的不是同一回事）。

**解决：开代理软件 → 让 git 走代理**（本项目就是这么推送成功的）
```bash
# 端口换成你代理软件里显示的实际端口（常见 7890 / 10809 / 1080）
git config --global http.proxy http://127.0.0.1:7890
git config --global https.proxy http://127.0.0.1:7890
```
配一次即可，以后自动生效（前提是代理软件开着）。

**想取消代理**（换了网络环境）：
```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

---

## 6. 后悔药（改错了怎么办）

```bash
# 某文件「还没 add」，想丢弃改动、恢复到上次提交（改动会丢失，慎用）
git checkout -- app/mainwindow.cpp

# 已 add，想撤出暂存区（改动还在，只是不提交）
git restore --staged app/mainwindow.cpp

# 看历史提交列表
git log --oneline

# 已 commit 但还没 push，想改最后一次提交信息
git commit --amend -m "新的提交信息"
```

> ⚠️ 危险命令别乱用：`git reset --hard`、`git push -f`、`git clean -f` 会**永久丢失**改动。
> 拿不准时先 `git status` + `git log`，或先把文件夹复制一份备份。

---

## 7. `.gitignore` —— 忽略不该提交的文件

项目根目录的 `.gitignore` 列出**不纳入 Git 管理**的东西：`build/`（构建产物，100+MB）、IDE 缓存、`*.user`、`*.obj/.exe` 等。想忽略新东西，往里加一行文件名/文件夹名即可。

> 这份文件**跟着仓库走**（会被提交、团队共享），不是本地专属。
> 只想「自己本地忽略、不影响别人」的，写进 `.git/info/exclude`。

---

## 8. 常见问题

**Q：`git push` 要账号密码 / 认证失败？**
GitHub 不支持密码推送，需 **Personal Access Token (PAT)**：
GitHub → Settings → Developer settings → Personal access tokens → 勾 `repo` 权限，生成后**当密码用**（用户名填 GitHub 用户名）。

**Q：`fatal: not a git repository`？**
终端不在项目目录。先 `cd /d e:\Code\QT\blade_measurement\blade_measurement_v3.3.4` 切进去。

**Q：CMD 里切不到 E 盘？**
先敲 `e:` 回车切到 E 盘，再 `cd 路径`；或一步到位 `cd /d e:\完整路径`。

**Q：中文文件名显示成一串数字乱码？**
`git config --global core.quotepath false`

**Q：`build/` 被误提交了怎么移除？**
```bash
git rm -r --cached build/
git commit -m "清理：移除误提交的 build 目录"
```


