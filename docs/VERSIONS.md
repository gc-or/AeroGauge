# 版本决策与训练/评估边界

核对日期：2026-10-29。

## 首版决定

MuJoCo **3.15.0 固定为可复现基线**，并非宣称它长期优于其他版本。它是原 WRJ 资产验证记录和训练参考包独立资产验证环境共同使用的版本。当前 AeroGauge 已在该版本实际运行；3.16/3.17 尚未在本项目执行比较。

官方当前稳定更新日志列出 3.17.0（2026-10-22）；该版增加批量 API，也包含求解器与模型常量重算行为、ABI/API 变化。新功能要按本项目需要评估，不能只按版本号选型。来源：[MuJoCo 官方更新日志](https://mujoco.readthedocs.io/en/stable/changelog.html)。

MuJoCo 是物理引擎；官方机器人模型集合 [Menagerie](https://github.com/google-deepmind/mujoco_menagerie) 是另一个项目。升级引擎不等于自动获得适合 WRJ 的评估场景，也不能直接运行 Isaac Lab 任务。引入新模型或组件时应核对其要求和许可证。

## 与主训练环境的关系

用户说明：Isaac Sim 5.1.0.0、Isaac Lab 2.3.2、PyTorch 2.7.0、cu128。参考包 `constraints.txt` 和环境记录进一步显示，其当时实际训练分发包为 `isaaclab==2.3.2.post1`，参考源码 tag 为 v2.3.2。这里是包内记录，不等于本轮登录训练电脑再次核验。

| 环境 | 建议保留的内容 |
|---|---|
| Isaac 训练 | 保持已有环境；不通过本项目升级、替换其依赖 |
| MuJoCo 评估 | 独立 Python 3.11 环境，固定 MuJoCo 3.15.0；核心仅依赖 NumPy/SciPy |
| 可选策略推理 | 用与导出模型匹配的 PyTorch；首次迁移优先保持 2.7.0，CPU/GPU 按实际需求选择 |

官方 Isaac Lab 安装文档要求 Isaac Sim 5.x 使用 Python 3.11，并列出 x86_64 的 PyTorch 2.7.0/cu128 安装组合：[官方安装说明](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/setup/installation/pip_installation.html)。本轮本地基础仿真实际用 Python 3.12.15，不把它称为已验证的 Isaac 训练环境。

普通 MuJoCo 刚体计算不依赖 PyTorch/cu128；可视化需要相应图形环境。未引入 MJX、MuJoCo Warp、JAX 或双引擎同步，避免首版扩张。Isaac 使用 PhysX，策略迁移到 MuJoCo 属于 Sim-to-Sim，需要核对坐标、惯量、步长、执行器、动作、观测和接触；能导入权重不代表行为一致。

## 后续升级方法

在独立分支/环境中改变 `pyproject.toml` 的 MuJoCo 固定版本，保留旧环境和测试结果。依次检查资产编译、质量/惯量、单电机力矩符号、悬停平衡、被动接触、固定种子任务、轨迹指标与渲染。比较误差和接触变化，再决定是否调整基线；不要默认跨版本结果逐位相同。

升级前固定同一资产、控制器、参数和种子，输出新 manifest。若只是为了新渲染效果，不能据此宣称物理精度提升。任何尚未执行的平台或版本仅记录为计划支持。
