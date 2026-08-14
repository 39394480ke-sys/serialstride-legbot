# SerialStride Legbot

> 从单电机台架开始，逐步完成一台桌面级串联轮腿机器人。
>
> An open development journal for a desktop-scale serial wheel-legged robot,
> progressing from motor bring-up to standing, driving, and jumping.

## 当前阶段

**当前状态：MuJoCo source / geometry / dynamics 三层模型职责已固化。**

在单腿 FK 和整车闭链模型基础上，仓库现已明确区分不可修改的 CAD source、
权威 geometry 与派生 dynamics。机构设计和实物均支持连续无限 360 度旋转，
MuJoCo 模型也完成了全旋转运动学、闭链约束和模态控制验证。

- MC02 已验证编译、SWD 下载、1 ms 主循环和 CAN1 1 Mbps 通信。
- CAN1 上的两台 DM4310 关节电机和一台 H6215 轮毂电机可同时在线。
- 已验证逐台受限正反转、三电机并行运动、非目标电机失能和 `STOP ALL`。
- 已建立统一 `MotorState`、`motor_manager` 和 `safety_manager` 架构。
- 已修复 USB 断连 HardFault、旧 CAN 反馈时间戳、急停队列和 Disable 确认问题。
- 已确认机械伸腿正方向：JOINT_A `+1`，JOINT_B `-1`。
- 已建立独立标定参数、软件限位和悬空安全目标。
- 完整双关节协同序列已通过，WHEEL 全程保持 Disabled。
- 24 V 上下电两次读数一致，但尚未形成夹具化重复性验收。
- 单腿模型采用 `l1=l4=0.110 m`、`l2=l3=0.132 m`、`l5=0`，FK 与角度转换测试通过。
- 软限位内 32,761 个采样状态全部有效，工作空间为 `Hx=-37.33..37.07 mm`、`Hz=-176.16..-93.07 mm`。
- `robot_source.xml` 保持 CAD 原始导出不变，并由 SHA-256 锁定。
- 权威 `robot_geometry.xml` 包含 25 个 body、24 个 joint、6 个 connect、4 个 tendon、4 个模态 actuator 和 11 个 keyframe。
- 11 个 keyframe 的最大闭链残差约为 `1.43e-9 m`，四个 rotation/shape 滑块的目标跟踪与左右腿隔离通过。
- `robot_dynamics.xml` 保留为 Phase 1-3 快照；旧生成器已加迁移保护，写入前会停止。

尚未验证：

- 三电机长时间 Powered + Disabled 稳定性；
- 单设备实机掉线和危险保护故障注入；
- 标准 CAN 终端阻值正式验收；
- 承重、稳定性、动力学控制和长期耐久；
- 临时 `STAND` 是否适合实际承重；
- 低增益下约 `0.04..0.07 rad` 的最终跟踪残差；
- 一次 Disable 瞬间 JOINT_B `+0.212 rad/s` 单帧读数的原因。
- 单腿 FK 的独立多姿态 SolidWorks/实物坐标精度；
- 各 link 的精确实物质量分配、质心和惯量，以及气弹簧动力学、地面接触和承重控制。

下一阶段：

1. 迁移 geometry-to-dynamics 生成器，避免重复创建 tendon、keyframe 或 slide 归一化。
2. 在 dynamics 中恢复 6 个力矩电机、阻尼、摩擦、armature、气弹簧力、重力和地面场景。
3. 使用多个 SolidWorks/实物姿态验证 FK 坐标，并在支架和低力矩限制下开始受力验证。

## 项目目标

SerialStride Legbot 的长期目标是一台主要由 3D 打印件和低成本机加工件
构成的桌面级串联轮腿平台，用于学习和验证：

- 嵌入式实时控制与 CAN 总线；
- 串联腿运动学、雅可比和 VMC；
- 两轮自平衡、LQR 和状态估计；
- 变高度、行驶、转向及低高度跳跃。

## 当前硬件

| 模块 | 当前配置 |
|---|---|
| 当前轮毂电机 | 达妙 DM-H6215 × 1，`CAN_ID=1`, `MST_ID=0`, Velocity |
| 早期台架主控 | STM32F103C8T6 Blue Pill |
| 当前主控 | 达妙 DM-MC02 |
| 关节电机 A | 达妙 DM4310，`CAN_ID=6`, `MST_ID=3`, MIT |
| 关节电机 B | 达妙 DM4310，`CAN_ID=8`, `MST_ID=4`, MIT |
| 早期台架 CAN 收发器 | TJA1050 模块 |
| 台架供电 | 24 V 直流电源 |
| 早期台架调试接口 | CH340 USB-TTL, 115200 8N1 |

## 仓库结构

```text
firmware/   嵌入式固件和控制器代码
hardware/   CAD、电气设计、BOM 和制造资料（有内容时创建）
simulation/ 单腿运动学与整车 MuJoCo 数字模型
tools/      自编写的日志、CAN 和绘图工具（有内容时创建）
docs/       bring-up 教程、进展记录和设计笔记
assets/     README 使用的压缩图片
```

H6215 台架入口：

- [固件说明](firmware/stm32f103-h6215-bench/README.md)
- [接线与已确认参数](docs/bring-up/h6215-single-motor/硬件接线与参数.md)
- [首次调通记录](docs/progress/2026-07-29_H6215单电机调通.md)

MC02 单侧三电机入口：

- [三电机固件说明](firmware/dm-mc02-can1-three-motor/README.md)
- [MC02 Bring-up 与验收记录](docs/bring-up/dm-mc02/README.md)
- [2026-08-09 阶段进展](docs/progress/2026-08-09_DM-MC02_CAN1三电机Bring-up.md)
- [2026-08-10 单腿机械 Bring-up](docs/progress/2026-08-10_单腿机械Bring-up.md)

运动学与 MuJoCo 模型入口：

- [单腿 FK 与工作空间](simulation/kinematics/single_leg/README.md)
- [整车 MuJoCo 闭链几何模型](simulation/mujoco/full_chassis/README.md)
- [2026-08-13 模型阶段记录](docs/progress/2026-08-13_单腿FK与整车MuJoCo模型.md)
- [2026-08-14 三层模型职责重构](docs/progress/2026-08-14_MuJoCo三层模型职责重构.md)

验证命令：

```sh
(cd simulation/kinematics/single_leg && python3 -m unittest -v test_forward_kinematics.py)
python3 simulation/mujoco/full_chassis/validate_model.py
```

## 进度概览

| 阶段 | 状态 | 结果 |
|---|---|---|
| STM32F103 + H6215 单电机台架 | 已完成 | 双向速度控制、反馈解析、看门狗与自动失能 |
| MC02 本体与 CAN1 | 已完成 | SWD、1 ms 主循环、1 Mbps CAN1 |
| MC02 + 2×DM4310 + 1×H6215 | 核心完成 | 逐台运动、并行运动、`STOP ALL`、统一安全架构 |
| 单腿悬空调试 | 已完成 | 方向、机械极限、软件限位与低速双关节协同 |
| 单腿 FK 与工作空间 | 已完成 | 数字模型与软限位工作空间通过测试，实物精度验证豁免 |
| 整车 MuJoCo 几何模型 | 已完成 | 闭链、keyframe 与阶跃稳定性验证通过 |
| MuJoCo 三层模型职责 | 已完成 | CAD source、权威 geometry 与派生 dynamics 边界已固化 |
| 单腿承重与动力学 | 未开始 | 独立实物坐标、质量参数、接触与受力验证 |
| 整车站立、行驶与跳跃 | 未开始 | 需先完成单腿和双侧电控验证 |

## 版本路线

| 版本 | 里程碑 | 状态 |
|---|---|---|
| `v0.1.0` | H6215 单电机台架 | 已发布 |
| `v0.2.0` | 单腿机械 Bring-up | 代码已合并，尚未发布 Tag/Release |
| `v0.3.0` | 首次站立 | 未开始 |
| `v0.4.0` | 首次行驶 | 未开始 |
| `v0.5.0` | 首次跳跃 | 未开始 |
| `v1.0.0` | 可运行原型 | 未开始 |

日常修改通过 Git commit 保存，实验使用 `feat/`、`fix/`、
`experiment/`、`mechanical/` 或 `docs/` 分支。工作目录不使用
`_v1`、`最终版`等文件名保存历史。

## 安全

轮毂电机和关节电机可能在瞬间产生危险运动。台架测试时必须固定电机定子、
保持转子悬空、设置软件限幅，并将 24 V 物理断电开关放在触手可及的位置。
软件失能不能代替物理断电。

## 授权

本仓库不是整体采用同一种许可证：

- 原创代码使用 [MIT License](LICENSES/MIT.txt)。
- CAD、STEP、图纸及其他硬件资料当前为 **All Rights Reserved**。
- 文档、日志和媒体资料默认保留版权。

仓库公开可见不代表硬件设计已获得开源、复制、制造或商用授权。完整范围见
[LICENSE](LICENSE)。
