# SerialStride Legbot

> 从单电机台架开始，逐步完成一台桌面级串联轮腿机器人。
>
> An open development journal for a desktop-scale serial wheel-legged robot,
> progressing from motor bring-up to standing, driving, and jumping.

## 当前阶段

**当前状态：DM-MC02 CAN1 单侧三电机核心 Bring-up 已完成。**

当前处于从“单电机台架”走向“单腿实机”的过渡阶段。本阶段结论为
**Partial**：核心 Bring-up 已通过，长时间稳定性和故障注入仍需补验。

- MC02 已验证编译、SWD 下载、1 ms 主循环和 CAN1 1 Mbps 通信。
- CAN1 上的两台 DM4310 关节电机和一台 H6215 轮毂电机可同时在线。
- 已验证逐台受限正反转、三电机并行运动、非目标电机失能和 `STOP ALL`。
- 已建立统一 `MotorState`、`motor_manager` 和 `safety_manager` 架构。
- 已修复 USB 断连 HardFault、旧 CAN 反馈时间戳、急停队列和 Disable 确认问题。

尚未验证：

- 三电机长时间 Powered + Disabled 稳定性；
- 单设备实机掉线和危险保护故障注入；
- 标准 CAN 终端阻值正式验收；
- 机构装配后的负载、承重与长期运动。

下一阶段：

1. 补齐长时间带电静置、实机掉线和危险保护故障注入验证。
2. 将单侧三电机控制网络装入单腿机构，先完成悬空低力矩调试。
3. 接入第二台 H6215 和另一侧腿，开始左右同步与底盘级验证。

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
simulation/ 动力学与控制仿真（有内容时创建）
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

## 进度概览

| 阶段 | 状态 | 结果 |
|---|---|---|
| STM32F103 + H6215 单电机台架 | 已完成 | 双向速度控制、反馈解析、看门狗与自动失能 |
| MC02 本体与 CAN1 | 已完成 | SWD、1 ms 主循环、1 Mbps CAN1 |
| MC02 + 2×DM4310 + 1×H6215 | 核心完成 | 逐台运动、并行运动、`STOP ALL`、统一安全架构 |
| 单腿悬空调试 | 待开始 | 关节方向、零点、传动比与低力矩轨迹 |
| 整车站立、行驶与跳跃 | 未开始 | 需先完成单腿和双侧电控验证 |

## 版本路线

| 版本 | 里程碑 | 状态 |
|---|---|---|
| `v0.1.0` | H6215 单电机台架 | 已发布 |
| `v0.2.0` | 单腿运动 | 准备中 |
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
