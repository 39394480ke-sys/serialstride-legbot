# DM-MC02 本体 Bring-up

这里保存 DM-MC02 CAN1 三电机 Stage 的 Phase 1 实机记录。本阶段没有接入
电机，也没有发送 CAN 帧。

## 已验证事实

- 主控由 ST-Link 识别为 STM32H72x/H73x，目标电压为 3.31 V，片上 Flash 为
  1 MiB；板卡资料标注的具体型号为 STM32H723VGT6。
- 工程可以干净编译，固件已通过 ST-Link 写入 `0x08000000` 并完成下载后校验。
- USB CDC 可以枚举并输出全部启动标记和约 100 ms 一条的健康日志。
- HAL 初始化成功：24 MHz HSE 配置为 480 MHz SYSCLK、240 MHz HCLK，HAL
  SysTick 为 1 kHz。
- FDCAN1 使用 PD0/PD1、Classic CAN normal mode、1 Mbps，应用层保持
  `TX=DISABLED`，没有发送调用。因此无设备、无 ACK 的情况下不会形成发送
  重试阻塞。
- ST-Link 硬件复位后会自动重新启动，启动标记完整，主循环继续运行。
- 同时移除 MC02 USB-C 和 ST-Link 供电后重新连接，首次读取到
  `uptime_ms=47600`，证明 MCU 已真正复位并自动进入主循环；此时 `missed=0`，
  CAN TEC、REC、warning、passive、bus-off 仍全部为 0。
- 30 分钟连续采集得到 18,000 条健康记录。`missed=0`，周期最小值和最大值
  均为 1 ms；CAN TEC、REC、warning、passive、bus-off 全程为 0。

30 分钟测试摘要见 [`phase1-soak-2026-08-08.json`](logs/phase1-soak-2026-08-08.json)，
完整原始串口记录见
[`phase1-soak-2026-08-08.log.gz`](logs/phase1-soak-2026-08-08.log.gz)。

## 明确观察

- 采集开始前主机尚未持续读取 CDC，固件的非阻塞日志发送累计丢弃了 208 条；
  正式连续采集期间该数值保持不变。这说明日志路径不会反向阻塞 1 kHz 主循环。
- `can1_lec=7` 全程不变。按照 STM32 FDCAN 协议状态定义，该值表示自上次读取
  后没有新的总线错误，不是第 7 类故障。
- 物理重新上电时电脑尚未打开 CDC，因此没有捕获启动瞬间的五条标记；随后出现
  的低 uptime 健康记录直接确认了重新上电和主循环自动启动。五条启动标记已在
  ST-Link 硬件复位测试中完整捕获。

## 尚未验证

- 时钟频率是由 RCC 配置和运行日志确认的，尚未用示波器独立测量时钟输出。
- CAN1 初始化和控制器错误状态已验证，但尚未连接收发节点，因此没有验证物理
  总线波形、终端匹配、收发或 ACK。

## 固件

源码与构建说明位于
[`firmware/dm-mc02-can1-three-motor/`](../../../firmware/dm-mc02-can1-three-motor/)。
烧录前原固件只保存在本机临时备份中，没有纳入仓库。

## Phase 2.1：H6215 只读与安全反馈探测

2026-08-08 在 CAN1 上只连接一台 H6215，并给电机提供 24 V。已验证：

- 1 Mbps CAN 物理链路可以持续收发，最终 TEC、REC、warning、passive、
  bus-off 全部为 0。
- 电机为 `CAN_ID=1`、`MST_ID=0`，软件版本 `5406`，控制模式 `3`。
- 参数为 `P_MAX=12.500`、`V_MAX=45.000`、`T_MAX=10.000`，五项参数有效位
  `PARAM_MASK=0x1F`。
- 当前固件不包含 Enable、速度、位置或力矩命令。五项参数全部有效后只发送一次
  明确的 Disable 帧，以强制失能并请求状态反馈。
- Disable 返回的实机反馈为 `STATE=DISABLED`、`P=0.000 rad`、
  `V=-0.011 rad/s`、`T=0.002 N.m`、`TMOS=27 C`、`TROTOR=25 C`。
- 最终连续记录中 `TX_OK=122`、`RX=122`、`TX_FAIL=0`、`missed=0`。

H6215 `5406` 固件没有响应单独测试的 `0x7FF/0xCC` 状态读取请求，因此最终固件
不再周期发送该请求；反馈解析由启动时的一次 Disable 安全探测触发。

摘要见 [`phase2.1-h6215-safe-probe-2026-08-08.json`](logs/phase2.1-h6215-safe-probe-2026-08-08.json)，
完整串口记录见
[`phase2.1-h6215-safe-probe-2026-08-08.log.gz`](logs/phase2.1-h6215-safe-probe-2026-08-08.log.gz)。

尚未验证任何运动、使能、速度斜坡、看门狗或运动保护；这些属于 Phase 2.2 和
Phase 2.3。
