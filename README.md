# AeroGauge

针对自有 WRJ 四旋翼的 MuJoCo 评估原型。先构建通用飞行、参数偏差和扰动测试，再扩展专项环境。它是评估工程，不承担 Isaac Lab 训练。

## 当前可运行范围

- 原 WRJ 单自由刚体、72 个显示实例、36 个碰撞实例，约 0.797 kg；原始资产保持不变。
- 显式处理资产 S 系与控制 FLU 系、电机新旧编号及不对称力臂配平。
- 传统位置/姿态 PD 基线，悬停、0.5 m 位置阶跃、圆形跟踪。
- 固定随机种子，质量/惯量缩放、重心偏移、分电机推力偏差、反扭矩比、电机一阶滞后和延迟。
- 恒定外力、外力脉冲、可选简化机体风阻。
- 每回合轨迹 CSV、成功/失败、误差、初始状态、完整配置、代码/资产校验值及运行版本。
- 可选的无状态 TorchScript 悬停 actor 适配器：16 维观测、四路配平推力修正。

**未实现：**地面起飞与受控降落、速度阶跃/八字任务、观测噪声/偏置/丢帧、复杂风场、海浪/运动平台、视觉、原始训练 checkpoint 自动导入、44 维降落策略的配套控制器迁移。这些不作为当前已完成能力。默认从 1 m 离地开始，不把悬停测试称为起飞测试。

## 安装与运行

推荐 Ubuntu 独立 Python 3.11 环境；当前本地实测为 Windows / Python 3.12.15 / MuJoCo 3.15.0。两者与现有 Isaac 环境分别安装。不要将本项目依赖覆盖安装进正在训练的环境。

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m aerogauge --task hover --seeds 0 1 2 --out outputs/hover_001
python -m aerogauge --task step --out outputs/step_001
python -m aerogauge --task circle --out outputs/circle_001
python -m unittest discover -s tests -v
```

Windows 可用 `py -3.11 -m venv .venv`，或已有的 Python 3.12；激活命令为 `.venv\Scripts\Activate.ps1`。本次创建的 `.venv` 可直接用 `.venv\Scripts\python.exe`。`--viewer` 打开实时窗口，默认不渲染以便批量运行。输出目录必须是新目录，避免覆盖已有结果。

参数文件是对默认配置的部分覆盖，未知字段直接报错：

```bash
python -m aerogauge --config configs/mass_110.json --out outputs/mass_001
python -m aerogauge --config configs/motor_lag.json --out outputs/lag_001
python -m aerogauge --config configs/force_pulse.json --out outputs/pulse_001
python -m aerogauge --config configs/wind_demo.json --out outputs/wind_001
```

`mass_110` 只改变真实仿真体，基线控制器继续使用标称质量，故可能产生稳态误差。这是模型失配测试，不是自动重新整定控制器。质量与惯量独立缩放、重心偏移均为合成敏感性实验，不能冒充某个已实测载荷的完整惯量更新。

`wind_demo` 的有效阻力面积是未标定演示假设。默认风阻面积为零；开启非零风速时也必须明确配置阻力面积。当前按机体系三轴相对风速计算二次阻力，只计机体平动阻力，不模拟旋翼来流、地效、建筑/船体绕流或完整 CFD。没有同时开启 MuJoCo 内置流体阻力，避免重复受力。

## 结果口径

默认回合 10 s，物理步长 2 ms、控制周期 20 ms；初始位置和姿态按种子扰动。所有回合都保存，包括提前失败的回合。

成功要求：完整运行，未接触地面/超限，最后 2 s **每个控制采样点**同时满足位置误差 <0.10 m、速度跟踪误差 <0.10 m/s、倾角 <10°、角速度 <0.30 rad/s。此为原型门槛；阶跃的暂态质量仍应结合整段 RMSE 和最大误差看，不能仅看通过率。

高度 <0.2 m 或 >5 m、目标距离 >3 m、倾角 >60°、地面接触均终止。接触在每个物理步检查。时间到期而未达标记为 `tracking_threshold`；时间到期不会自动算成功。关闭窗口产生独立终止原因。发生程序异常时保留已输出文件并使命令失败，不把不完整批次当完整评估。

每个新输出目录包含 `config.json`、`manifest.json`、`initial_*.json`、`trajectory_*.csv`、`episodes.json`、`summary.json`。误差是已执行时间内的统计；提前失败与完整回合的 RMSE 不能单独直接排名，必须一起看成功率、终止原因和时长。未给出综合分数、统计显著性或实机可靠性结论。

## 项目分工

| 负责人 | 主要文件 | 首阶段任务 |
|---|---|---|
| 飞行底座负责人 | `vehicle.py`、`simulation.py`、`policies.py` | 物理参数、控制、模型观测/动作适配及校准 |
| 评估负责人 | `tasks.py`、`evaluation.py`、`configs/` | 测试任务、条件组合、指标、重复试验与报告 |
| 共同维护 | 接口文档、测试、版本决策 | 修改坐标、时间、状态或动作语义时同步确认 |

## 阅读入口

- [参考案例分析](docs/REFERENCE_REVIEW.md)
- [版本与训练环境关系](docs/VERSIONS.md)
- [接口和模型接入](docs/INTERFACES.md)
- [本轮验证记录](docs/VALIDATION.md)
- [待办与 GitHub 协作](docs/ROADMAP.md)
- [邀请同伴与日常提交](CONTRIBUTING.md)
- [跨电脑任务承接与需求记录](readme/TASK_HANDOFF.md)

原压缩包、虚拟环境、缓存、运行输出和权重已列入忽略规则。运行必需资产约 10 MB，随代码管理，脱离原主目录也可以加载。项目代码未指定开源许可证；保留引用和资产来源。协作流程见 CONTRIBUTING.md。
