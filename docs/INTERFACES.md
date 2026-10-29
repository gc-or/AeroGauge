# 公共接口 v0.1

## 状态、坐标与时间

- 世界坐标 W：右手、Z 向上。暂不指定地理东/北含义。
- 控制机体系 B：FLU，前/左/上。源资产 S 到 B 为 diag(-1,-1,1)，不改变原网格。
- `position`：真实重心世界位置 m；`velocity`：真实重心世界速度 m/s。
- `rotation`：R_WB，B 分量映射到 W；`omega`：B 系角速度 rad/s；`time_s`：仿真秒。
- 外露状态用旋转矩阵。MuJoCo 根四元数使用 wxyz；SciPy 内部 xyzw 仅在转换点使用。
- 电机顺序：前左、前右、后左、后右，对应原 motor_3、motor_1、motor_4、motor_2。
- 控制器每 20 ms 运行一次；仿真每 2 ms 更新执行器和外力，再推进物理。延迟是物理步的整数倍。

## 模块边界

- `Simulation.reset(seed)` 返回真值状态；清空电机延迟队列/接触状态，初始执行器取标称配平推力。
- `Simulation.step(thrust_N[4])` 保持控制命令一个控制周期，逐物理步更新执行器，返回新状态。非有限动作报错。
- `policy.reset()` 清空上次动作；`policy.act(state,target_position,target_velocity)` 返回四路 N。
- `tasks.reference(task,t,config)` 返回世界系参考位置、速度；当前 step 触发时间/位移和 circle 半径/速度固定在代码。
- 评估器读取真值，不将真实扰动后的质量、惯量和风自动传给控制器。传统基线固定使用原标称参数。
- 合力与合矩在实际 COM 只施加一次；力臂由分配矩阵计算，不额外逐电机重复施力。

当前基线直接使用真值。未来观测噪声应添加在策略观测层，评估真值保持独立。不要通过污染真值模拟传感器噪声。

## 悬停模型接入

在原训练环境导出**无状态、确定性 actor**，接口必须是 float32 `(batch,16)` → `(batch,4)` 的单张量输出。归一化如需使用，应封装在导出模块内。原始 RSL-RL `.pt` checkpoint 不直接读取；循环策略、tuple/dict 输出和 44 维降落策略需要专用适配器，当前不支持。

核对并复制示例 contract 为模型自己的元数据。其字段定义输入尺度、配平动作缩放和控制周期；示例不自动代表任意已训练权重。

```bash
python -m aerogauge --task hover --policy /path/to/exported_actor.pt \
  --contract configs/hover16_contract.example.json --out outputs/policy_001
```

PyTorch 是可选推理依赖，不由基础安装自动安装。优先在单独推理环境配置与训练导出相同的版本。首次接入应先对同一组离线状态，在 Isaac 与 MuJoCo 适配层比较观测和动作，然后短时比较标称轨迹。没有这一步，就不能把行为差异都归因于引擎。

不把噪声下的失败、未完成的任务或异常批次静默当作有效样本。任何需要新观测/动作语义的任务都添加独立命名契约。
