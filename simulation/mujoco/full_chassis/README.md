# Full-chassis MuJoCo models

本目录固定采用三层模型边界：

| 文件 | 职责 | 修改规则 |
| --- | --- | --- |
| `robot_source.xml` | CAD 原始导出档案，保留原始命名、执行器和 `home` | 不可修改；SHA-256 为 `9387924d3dad5ea2097e8953228b8de0cfd61661b36b10f039596c25b4e14d55` |
| `robot_geometry.xml` | 权威机构定义，包含闭链、坐标、限位、传感器、模态 tendon、调试滑块和 11 个姿态 | 机构语义只在此维护；SHA-256 为 `e7e1f21c22bbcc8846be8548e071819e72ccfad49df203704b56205417b800ea` |
| `robot_dynamics.xml` | 从 geometry 和 `params/` 确定性生成的动力学模型 | 不手工编辑；只包含动力学质量属性、被动参数和 6 个直接力矩 motor |

`robot_geometry.xml` 的显式惯量仅用于独立编译和 Viewer 调试，不是动力学权威数据。它使用零重力，不包含地面、真实电机、气弹簧力、joint damping、frictionloss 或 armature。真实质量惯量和所有动力学参数只能存在于派生的 dynamics 模型中。

## Geometry interface

四个主动关节采用“正方向伸腿”，并保持连续：

```text
rotation = (qA - qB) / 2
shape    = (qA + qB) / 2
qA       = shape + rotation
qB       = shape - rotation
```

每条腿有一个 `+0.5, -0.5` 的 rotation tendon 和一个 `+0.5, +0.5` 的 shape tendon。shape 机械范围为 `[-0.351, 0.3505] rad`，调试滑块的软件范围为 `[-0.281, 0.2805] rad`；rotation 滑块观察范围为 `[-pi, pi]`。原始 A/B 标定值保存在 `params/joint_limits.yaml`，不再作为独立关节机械范围解释。

Viewer Control 面板提供：

```text
left_leg_rotation_position
left_leg_shape_position
right_leg_rotation_position
right_leg_shape_position
```

气弹簧 slide 采用 `EXTEND = 0`，压缩为负，范围为 `[-0.0231, 0] m`。被动关节范围、6 个闭链 connect、轮轴传感器及 `+X` 前、`+Y` 左、`+Z` 上的维护坐标系也属于 geometry 权威接口。

## Viewer

```sh
/Applications/MuJoCo.app/Contents/MacOS/simulate \
  "$(pwd)/simulation/mujoco/full_chassis/robot_geometry.xml"
```

Simulation 面板中的 11 个 keyframe 为：

```text
0  CALIB_MID
1  CROUCH
2  EXTEND
3  INTERMEDIATE_N025
4  INTERMEDIATE_N0125
5  INTERMEDIATE_P0125
6  INTERMEDIATE_P025
7  ROTATION_N180
8  ROTATION_N090
9  ROTATION_P090
10 ROTATION_P180
```

后四个姿态保持中等腿长，用于查看整腿旋转。机构设计支持连续无限
360 度旋转，该能力已经由用户在实物机构上验证；MuJoCo 的全旋转姿态和扫描
进一步验证了数字模型的运动学、闭链约束与模态接口。模型关闭相关接触是几何
调试配置，不改变上述实物验证结论。

## Validation

```sh
python3 simulation/mujoco/full_chassis/validate_model.py
python3 simulation/mujoco/full_chassis/scripts/test_joint_limits.py
python3 simulation/mujoco/full_chassis/scripts/test_mass_properties.py
```

`validate_model.py` 同时锁定 source 与 geometry 哈希，并用 MuJoCo 3.9 检查 `25 body / 24 joint / 6 connect / 4 tendon / 4 modal actuator / 11 keyframe`、禁止的 joint 动力学属性、气弹簧坐标、闭链残差、轮轴半径和四个模态滑块的左右隔离。

`scripts/sync_inertials.py` 直接读取权威 geometry，先校验精确哈希，再覆盖 CSV 中的质量属性、应用被动参数并将四个 geometry 调试位置执行器替换为六个 dynamics 力矩 motor。它只校验 Phase 3 接口，不会重新创建 tendon、keyframe 或归一化 slide。

```sh
python3 simulation/mujoco/full_chassis/scripts/sync_inertials.py
python3 simulation/mujoco/full_chassis/scripts/sync_inertials.py --check
```

六个动力学执行器接口为：

```text
left_joint_a_motor   -> left_joint_a
left_joint_b_motor   -> left_joint_b
right_joint_a_motor  -> right_joint_a
right_joint_b_motor  -> right_joint_b
left_wheel_motor     -> left_wheel_joint
right_wheel_motor    -> right_wheel_joint
```

全部使用 `gear=1` 和直接力矩输入。用户提供的厂家 V1.2 资料给出：DM-J4310-2EC 关节电机额定/峰值力矩 `3.5/12.5 Nm`、额定/空载最大转速 `120/200 rpm`；DM-H6215 轮电机为 `1/2 Nm`、`120/320 rpm`。dynamics 因此分别使用 `±12.5 Nm` 和 `±2 Nm` 的 `ctrlrange/forcerange`。旧截图中的关节电机 `3/7 Nm` 已作为被 V1.2 资料取代的历史证据保留在 `params/actuator_params.yaml`。

额定力矩不被解释为可无限时间持续的力矩：厂家温升图显示 J4310 在 `120 rpm / 3.5 Nm` 工况下约 `320 s` 由约 `32 C` 升至 `99 C`，该数字是读图近似值。当前 MJCF 仍是带峰值饱和的理想 MIT `t_ff` 力矩源，不包含电流环、速度相关力矩曲线、温度状态或热降额；厂家资料不足以支持不失真地构造完整力矩-转速包络。关节电机内部 `10:1` 减速器已经反映在厂家输出端参数中，外部同步带仍为 `1:1`，所以 MJCF `gear` 保持 `1`。整车前方定义为世界 `+X`；已验证左轮负力矩、右轮正力矩的组合使底盘向 `+X` 运动，反号组合使底盘向 `-X` 运动。

Phase 0 的旧冻结记录保留在 `results/geometry_baseline.json` 作为历史证据。当前权威记录为 `results/geometry_authority.json`，迁移报告为 `results/dynamics_migration_report.json`。迁移前 dynamics 哈希只作为历史对照，不参与当前生成。

## Mass-property snapshot

Phase 1-3 的质量属性仍保存在 `params/body_mass_map.csv` 和生成的 `params/inertials.yaml`。当前 dynamics 包含 25 个显式惯性元素，并以 `balanceinertia=false` 编译；主要机身质量采用 SolidWorks 指定值，网格推导的质心和惯性仍是较低置信度证据，两侧气弹簧质量也仍是临时估计。

`scripts/test_mass_properties.py` 测试质量表的缩放、镜像和惯性合法性；`scripts/audit_mass_properties.py` 验证 dynamics 与 CSV 一致，并输出质量和 COM 报告。

## Scan snapshot

`params/joint_limits.yaml` 保留模态定义、原始 A/B 标定证据、被动范围和气弹簧 slide 约定。模态位置执行器只存在于 geometry，因此 `scripts/scan_passive_joints.py` 明确扫描权威 geometry；dynamics 不再通过位置执行器复刻该接口。扫描报告属于数字模型证据；连续无限 360 度旋转能力另有用户实物验证。

## Dynamics scenes and parameters

```sh
python3 simulation/mujoco/full_chassis/scripts/generate_scenes.py
python3 simulation/mujoco/full_chassis/scripts/generate_scenes.py --check
```

生成的 `scenes/fixed_base.xml`、`scenes/free_ground.xml` 和 `scenes/drop_test.xml` 使用重力和 `0.001 s` 步长。visual mesh 不参与碰撞；左右轮使用 CAD 外包络推导的 cylinder collision，用户已确认模型轮胎半径和宽度与实物一致。底盘使用 CAD 外包络 box，其他连杆及气弹簧外形使用与 visual 同位姿的独立凸包 mesh collision，避免中心线 capsule 包不住板件外形而出现视觉穿地。气弹簧仍不施加弹力，这里只保留对地碰撞外形。碰撞掩码只允许机器人与地面接触，暂不启用机器人自碰撞。自由场景的 keyframe 保留 geometry 中的底盘初始旋转，与 `fixed_base.xml` 的轮轴传感器姿态一致。地面与摩擦参数位于 `params/contact_params.yaml`，摩擦仍是 `PROVISIONAL`。全部 11 个 keyframe 已通过 `5 s` 数值稳定性、小于 `0.1 mm` 的动态闭链误差和对地接触检查。用户于 2026-08-16 重新加载姿态修正后的 `free_ground.xml` 和 `drop_test.xml`，并确认全部 key 下均未出现穿地、异常抖动、数值爆炸或可见闭链分离。实物摩擦和真实沉降时间尚未验证。

`params/passive_params.yaml` 提供 `ideal` 与 `nominal` 两套阻尼、干摩擦和 armature。`ideal` 用于机构调试；`nominal` 作为第一版强化学习基线。两套 profile 已通过零控制、四关节电机脉冲、左右对称、脉冲撤除和全部 11 个 keyframe 的 `5 s` 接地回归。标准脉冲撤除 `0.5 s` 后，`ideal` 速度范数保留比为 `1.0016`，`nominal` 为 `0.000258`。这只验证数值行为；nominal 仍是迁移前占位值，未做实物辨识。气弹簧的 `23 mm` 行程和 `60 N` 厂家标称力已经记录，但按用户决定，当前基础动力学和第一版强化学习环境都不施加气弹簧力；真实公差、刚度、阻尼和力—行程曲线留待该子系统作为独立增量重新启动时处理。

完整迁移回归：

```sh
python3 simulation/mujoco/full_chassis/scripts/validate_dynamics.py
python3 simulation/mujoco/full_chassis/scripts/test_actuator_signs.py
python3 simulation/mujoco/full_chassis/scripts/validate_passive_profiles.py
```

## Sites and frames

在桌面 Viewer 中启用 site 分组 3，并将 `Rendering > Label` 设为 `Site`。蓝色 site 是闭环连接锚点，红色 site 是左右基座参考点，绿色 site 是轮轴中心；Sensor 面板显示轮轴中心相对于对应基座参考点的位置。`Frame` 可切换为 `World` 或 `Site`，`F6` 可循环坐标系显示。

## Evidence limits

已验证：模型结构、文件哈希、确定性 geometry-to-dynamics 生成、11 个姿态的闭链残差、模态解耦、shape tendon 范围、固定 shape 的轮轴半径、气弹簧 slide 坐标、六个 motor 的目标与正坐标方向，以及 MuJoCo 3.9 编译。用户已在实物机构上验证连续无限 360 度旋转能力。

未验证：逐连杆实物质量分配、质心和惯量、厂家电机极限在当前实机供电/驱动配置下的复现、速度上限和热降额、被动参数、气弹簧真实力学、动态地面接触和站立控制。当前 scene 只能作为后续接触辨识和 Phase 9 动力学闭链验收的起点。
