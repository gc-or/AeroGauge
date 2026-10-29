# WRJ 无人机仿真资产包 v1

这是从 SolidWorks「装配体1」独立导出的仿真包，文件名、网格名、刚体名和电机标记均为英文/数字。原 `models/3D models` 中的装配体与零件未另存覆盖、未改名。包内资源使用相对路径，可以把整个 `wrj_sim_v1` 文件夹搬到另一台电脑。

Ubuntu 部署端请先读 [UBUNTU_HANDOFF.md](UBUNTU_HANDOFF.md)；可转交的承接任务文本见 [HANDOFF_PROMPT.md](HANDOFF_PROMPT.md)，结果填写 [UBUNTU_TEST_REPORT_TEMPLATE.md](UBUNTU_TEST_REPORT_TEMPLATE.md)。

包内包含 **38 种零件网格、72 个装配实例、36 个简化碰撞体**。质量与惯量继承最近一次标定，整机质量为 **0.797048708 kg**。这是可加载、可显示、带接触几何的刚体资产，尚不是完整的飞行控制或强化学习任务。

## 打开哪个文件

| 用途 | 文件 |
| --- | --- |
| Isaac Sim / Isaac Lab 资产入口 | `usd/drone.usda` |
| USD 显示网格资源，不要漏拷 | `usd/visuals.usdc` |
| Isaac Lab 刚体配置 | `config/isaaclab_asset.py` |
| MuJoCo 完整落地预览场景 | `mujoco/scene.xml` |
| MuJoCo 无地面的机体模型 | `mujoco/drone.xml` |
| 通用质量、惯量、电机位置与待定参数 | `config/vehicle.json` |
| 英文零件名与原始名称对照 | `config/name_map.csv`、`config/name_map.json` |
| 独立零件显示网格（米制 OBJ） | `meshes/visual/` |
| 简化碰撞网格（米制 OBJ） | `meshes/collision/` |
| 验证记录和实际 MuJoCo 预览 | `validation/` |

例如，上板为 `plate_top`，转向柱为 `landing_pivot`，7 cm 起落架管为 `landing_strut_70mm`，电机为 `motor_v2306_v3`。重复实例带 `_01`、`_02` 等后缀。中文名称只在溯源对照表和说明中保留，模拟器不需要访问原 CAD 路径。

“深度相机卡件”以 `depth_camera_clip.obj` 随包提供，但原装配体没有引用它，所以它没有被放入整机，也没有增加整机总质量。开发板的内部连接器被保留为显示网格，质量仍按整个开发板的 50 g 只计算一次，散热器单独计入。

## 坐标和物理模型

- 长度单位为 **m**，质量 **kg**，惯量 **kg·m²**，配置中的角度 **rad**，四元数为 **w,x,y,z**。
- 导出机体原点位于已标定重心；坐标轴仍平行于 SolidWorks 装配体轴，**+Z 向上，尚未指定机头为 +X 或 +Y**。
- CAD 中坐标转到包内机体坐标：`p_body = p_CAD - (-0.003312721692, -0.058741881148, 0.087952148811)`，单位 m。没有改变零件间的相对位置，也没有旋转惯量坐标轴。
- 整机采用 **一个自由刚体**。全部质量与惯量显式写入根刚体，显示网格和碰撞体不会再叠加一次质量。
- `motor_1` 至 `motor_4` 对应四个推力作用位置，其原装配实例映射在配置中。编号不等于飞控电机顺序；正反转、最大推力、反扭矩比和响应时间尚未确定。
- 没有假定真实桨叶转速或添加假的推力控制器。桨叶保留显示和其对整机质量/惯量的贡献，暂不参与碰撞。后续如拆出转动刚体，需要重新分配其质量与惯量。

## 显示与碰撞

显示网格来自 SolidWorks 现有图形三角化，保持模型外形；它是仿真显示资源，不是用于加工的精确曲面。颜色按部件类别设置，未携带 SolidWorks 贴图、商标或完整外观材质。

碰撞使用逐零件凸包，保留起落架各部件的分布，避免把整机下方空隙封成一个大凸包。部分微小内部件、垫片、铜柱以及桨叶不参与碰撞。凸包会填平单个零件上的孔和凹槽，暂不适合精细插接或孔槽配合仿真。

摩擦系数暂取 0.6，USD 恢复系数暂取 0；MuJoCo 接触求解使用引擎默认柔度设置。这些是可调整的仿真初值，**不是材料实测数据，也不代表两个引擎具有相同接触响应**。减震块暂视为刚体外形，未标定真实变形/阻尼。

## 使用方式

在 MuJoCo 环境中，从包目录执行：

```bash
python tools/preview_mujoco.py
```

它显示被动落地场景，没有电机推力。`tools/load_mujoco.py` 使用内存资源映射加载模型，可避免部分 Windows MuJoCo 版本直接读取中文上级路径失败的问题。在自己的训练代码中也可以导入该文件的 `load_model(with_ground=False)`。

在已安装的 Isaac Lab 环境中，从包目录执行：

```bash
python tools/preview_isaaclab.py
```

也可以用目标 Isaac Lab 的 `isaaclab.sh -p` / `isaaclab.bat -p` 启动该脚本。该入口先启动 AppLauncher，再导入资产配置并加入地面、灯光。自己的场景可复用 `DRONE_CFG`，并按环境路径替换 `prim_path`。在 Isaac Sim 中可将 `usd/drone.usda` 作为资产引用到已有物理场景。

Isaac 脚本采用官方 `RigidObjectCfg` / `UsdFileCfg` 接口，但本机未运行 Isaac Sim/Lab；迁移时应结合目标版本复核。不要在现有 Isaac 环境中直接安装本包的独立验证依赖，避免替换它配套的库。

独立验证脚本：

```bash
python tools/validate_package.py --render
```

它所需的包列在 `requirements-validation.txt` 中，可在单独的 Python 环境内配置。`tools/build_package.py` 是维护用生成器，额外需要 SolidWorks 只读三角化导出和原质量快照；使用现成仿真包无需运行它。

## 已验证与未完成项目

已用 MuJoCo 3.15.0 和 OpenUSD 26.8 验证网格引用、单位、整机尺寸、72 个显示实例、质量/重心/惯量和四个电机位置。MuJoCo 完成 1500 步（3 s）被动落地检查并稳定，记录在 `validation/package_validation.json`。`preview.png` 与 `settled.png` 是实际引擎渲染。

未运行 Isaac Sim/Lab 实机引擎，未验证旋翼气动、受控飞行或强化学习。下一步还需要确定机头方向与电机序号、正反转和推力/反扭矩曲线，接入控制器、传感器光学位姿/内参、观测/动作/奖励与任务场景。铜柱和减震块质量仍未称重，摄像头线继续按用户要求排除，打印件与 PVC 管的密度来源保留在分项质量状态中。

格式参考：[NVIDIA 质量与惯量属性](https://docs.omniverse.nvidia.com/kit/docs/asset-requirements/1.11.2/capabilities/physics_bodies/physics_rigid_bodies/requirements/rigid-body-mass.html)、[Isaac Lab 刚体用法](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/01_assets/run_rigid_object.html)、[MuJoCo XML 格式](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。
