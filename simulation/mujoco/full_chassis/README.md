# Full-chassis MuJoCo models

本目录固定采用三层模型边界：

| 文件 | 职责 | 修改规则 |
| --- | --- | --- |
| `robot_source.xml` | CAD 原始导出档案，保留原始命名、执行器和 `home` | 不可修改；SHA-256 为 `9387924d3dad5ea2097e8953228b8de0cfd61661b36b10f039596c25b4e14d55` |
| `robot_geometry.xml` | 权威机构定义，包含闭链、坐标、限位、传感器、模态 tendon、调试滑块和 11 个姿态 | 机构语义只在此维护；SHA-256 为 `e7e1f21c22bbcc8846be8548e071819e72ccfad49df203704b56205417b800ea` |
| `robot_dynamics.xml` | 从 geometry 派生的动力学模型 | 当前保留为 Phase 1-3 迁移前快照，等待基础动力学任务迁移生成器 |

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

旧的 `scripts/sync_inertials.py` 暂时被迁移保护锁住。无论普通运行还是 `--check`，它都会在写文件前以 `MIGRATION REQUIRED` 停止。基础动力学任务应先让生成器直接读取新的权威 geometry，并删除重复应用 tendon、模态 keyframe 和 slide 归一化的旧逻辑，然后再恢复可重复的 geometry-to-dynamics 同步检查。

Phase 0 的旧冻结记录保留在 `results/geometry_baseline.json` 作为历史证据。当前权威记录为 `results/geometry_authority.json`。当前 dynamics 快照仍可独立加载，但不代表新的生成链路已经迁移完成。

## Mass-property snapshot

Phase 1-3 的质量属性仍保存在 `params/body_mass_map.csv` 和 `params/inertials.yaml`。当前 dynamics 快照包含 25 个显式惯性元素，并以 `balanceinertia=false` 编译；主要机身质量采用 SolidWorks 指定值，网格推导的质心和惯性仍是较低置信度证据，两侧气弹簧质量也仍是临时估计。

在生成器迁移完成前，不要运行同步命令更新 dynamics。`scripts/test_mass_properties.py` 仍可测试质量表的缩放、镜像和惯性合法性，但 `scripts/audit_mass_properties.py` 与同步流程应在基础动力学任务恢复 geometry-to-dynamics 链路后再运行。

## Scan snapshot

`params/joint_limits.yaml` 保留模态定义、原始 A/B 标定证据、被动范围和气弹簧 slide 约定。`results/modal_scan.csv` 与 `results/modal_scan_report.json` 是迁移前 dynamics 的 `25 x 21` rotation/shape 扫描证据。扫描器应随动力学生成器一起改为直接继承新 geometry 接口，再生成新的 dynamics 报告。扫描报告属于数字模型证据；连续无限 360 度旋转能力另有用户实物验证。

## Sites and frames

在桌面 Viewer 中启用 site 分组 3，并将 `Rendering > Label` 设为 `Site`。蓝色 site 是闭环连接锚点，红色 site 是左右基座参考点，绿色 site 是轮轴中心；Sensor 面板显示轮轴中心相对于对应基座参考点的位置。`Frame` 可切换为 `World` 或 `Site`，`F6` 可循环坐标系显示。

## Evidence limits

已验证：模型结构、文件哈希、11 个姿态的闭链残差、模态解耦、shape tendon 范围、固定 shape 的轮轴半径、气弹簧 slide 坐标和 MuJoCo 3.9 编译。用户已在实物机构上验证连续无限 360 度旋转能力。

未验证：逐连杆实物质量分配、质心和惯量、气弹簧真实力学、地面接触和站立控制。
