# DM-MC02 CAN1 三电机 Bring-up

日期：2026-08-09
状态：**Partial，核心 Bring-up 完成，带两项已接受验收例外关闭**

## 阶段目标

在 DM-MC02 的 CAN1 上建立安全、可复用的单侧三电机控制网络：
两台 DM4310 关节电机加一台 H6215 轮毂电机。

## 已验证结论

- MC02 支持干净编译、SWD 下载、启动、1 ms 主循环及 CAN1 1 Mbps 通信。
- H6215 已从 STM32F103 台架迁移至 MC02，使用 `CAN_ID=1` / `MST_ID=0`。
- `JOINT_A` 使用 `ID=6` / `MST_ID=3`，`JOINT_B` 使用 `ID=8` / `MST_ID=4`，两者均为 MIT 模式。
- 三台电机可同时在线，参数和反馈正确，ID 无冲突。
- 已实机验证逐台安全正反向运动、非目标电机失能、三电机并行运动和 `STOP ALL`。
- 已实现统一 `MotorState`、`motor_manager` 和 `safety_manager`。
- USB 断连 HardFault、旧 CAN 反馈时间戳、急停队列和 Disable 确认问题已修复。

## 固件架构

当前三电机固件在
[`firmware/dm-mc02-can1-three-motor/`](../../firmware/dm-mc02-can1-three-motor/)，
主要责任分层为：

```text
drivers/   CAN 传输、接收时间与 USB 命令/日志
motors/    DM4310、H6215 协议与统一 motor_manager
safety/    单电机及并行控制安全快照和全局保护
app/       三电机 Bring-up 命令编排与遥测
```

最终固件 SHA-256：

```text
0a009a84c3dc6d595c3caabee45a400a13fde38ce9e523e6a68b53de9f1d9021
```

该二进制已完成烧录、校验和 MCU 复位。仓库不在本阶段重复归档构建产物。

## 验收证据

- 完整 Host tests 通过。
- 干净 ARM 构建与 `git diff --check` 通过。
- 最终固件烧录、校验及 MCU 复位成功。
- 用户实机确认逐台正反转、三电机并行运动及 `STOP ALL` 正常。
- 原始串口日志证明三台电机均在线、Disabled、参数正确，且 CAN 无主动错误。

原始日志：
[`records-2026-08-09-22-20-44.json`](../bring-up/dm-mc02/logs/records-2026-08-09-22-20-44.json)

注意：该原始日志本身不证明运动和 `STOP ALL`；这两项来自用户实机观察。

## 已接受例外

1. 三电机长时间 Powered + Disabled 稳定性测试未执行。
2. 单设备实机掉线测试未执行；掉线逻辑只有主机测试证据。

## 其他未验证项

- 危险保护尚未进行实机故障注入。
- 标准 CAN 终端阻值尚未形成正式测量验收。
- 负载、机构装配、承重和长期运动尚未验证。

## 下一步

先补齐两项已接受例外与终端阻值正式验收，再将控制网络装入机构，
以严格限矩、可靠支撑和物理急停开始单腿负载调试。
