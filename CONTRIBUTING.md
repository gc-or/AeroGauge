# AeroGauge 协作指南

## 邀请同伴

由仓库所有者登录 GitHub 后操作：

1. 打开 `https://github.com/gc-or/AeroGauge`。
2. 点击 **Settings**，在左侧 **Access** 下进入 **Collaborators**。
3. 点击 **Add people**，输入同伴的 GitHub 用户名或邮箱，核对账号后发送邀请。
4. 同伴在 GitHub 通知或邮件中接受邀请，即可向此仓库推送代码和发起 PR。

公开仓库允许其他人浏览，但向原仓库写入代码仍需要协作者权限。个人仓库的协作者权限与组织仓库的角色菜单不同，不必寻找组织的 Write 角色选择器。

官方说明：https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/inviting-collaborators-to-a-personal-repository

## 在部署电脑第一次获取代码

```bash
git clone https://github.com/gc-or/AeroGauge.git
cd AeroGauge
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m unittest discover -s tests -v
python -m aerogauge --task hover --out outputs/first_hover
```

不要直接在原 Isaac 训练环境安装这些依赖；模型推理环境的安排见 `docs/VERSIONS.md`。在有权限推送代码的电脑上，通过 GitHub 的正常浏览器登录或 SSH 完成 Git 身份认证；无需把密码、访问令牌或私钥放进仓库或聊天。

## 每项任务的日常流程

先确认当前工作已提交或妥善保存，再切换分支：

```bash
git switch main
git pull --ff-only origin main
git switch -c feat/your-task
```

完成一项可检查的修改，运行相关测试，按实际修改文件提交：

```bash
python -m unittest discover -s tests -v
git status
git add <本次修改的文件>
git commit -m "Describe the completed change"
git push -u origin feat/your-task
```

在 GitHub 上点击 **Compare & pull request**，目标为 `main`。写清问题、修改内容、测试命令和结果，请另一人审阅后合并。合并后双方更新本地 main。若 pull 报分歧，先检查双方修改并处理冲突，不要 force push main 或丢弃本地修改。

按任务建短期分支，例如 `feat/parameter-sweep`、`feat/observation-noise`。不要两个人长期各自维护一个大分支。修改坐标、状态、动作或时间语义前，先共同确认接口。

## 提交边界

- 跟踪源码、配置、必要资产、文档及小型验证记录。
- `.venv`、`.cache`、参考压缩包、训练权重及 `outputs/` 已忽略，不提交。
- 不把本机路径写成运行必需条件；使用项目相对路径。
- CI 会运行无界面基础测试；没有 PyTorch 时，可选策略测试会跳过，不能把这解释为真实模型验证通过。
- GitHub 上公开可读不等于已授予任意用途的开源许可证；授权范围由项目所有者另行确定。
