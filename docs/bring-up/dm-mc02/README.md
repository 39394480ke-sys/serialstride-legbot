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

## Phase 2.2a：H6215 单次正方向低速测试

2026-08-08 已完成一次受限的 `A` 后 `G` 正方向测试。浏览器导出中记录到一次
`a`（20:36:28.855）和一次 `g`（20:36:34.166）；固件依次记录
`MOTION_RUNNING`、约 1001 ms 后的 `ZERO_SPEED_HOLD`，以及约 199 ms 后的
`TEST_COMPLETE MOTOR_DISABLED`。运行阶段的十条反馈均为 `STATE=ENABLED`、
`V=+0.186 rad/s`；采样位置从 `-0.164` 到 `+0.014 rad`，采样增量为
`+0.178 rad`。零速保持后，最终三条采样均为 `STATE=DISABLED`、
`V=-0.011 rad/s`、`TMOS=33 C`、`TROTOR=31 C`，最终 CAN 状态为
`TEC=REC=0`、`warning=passive=bus_off=0`、`missed=0`、`TX_FAIL=0`。

用户明确观察到“现在正常了 我看到电机逆时针转了”。这是一条物理观察；由于未
指定观察视角或机构参考面，本记录**不**将它解释为协议正速度与机构逆时针方向的
已验证映射。负方向和连续运动均未实现、未验证。

完整浏览器导出在 `TEST_COMPLETE` 后只保留约 3.002 秒的窗口，未达到规定的
5 秒；这项浏览器记录限制仍然保留，未被补写。随后控制器在不发送任何串口命令
字节、也不发送外部 CAN/运动命令的只读采样中，取得八组 `[WHEEL]`/`HEALTH`
配对记录，控制器 `uptime_ms=2813552..2820552`，首末跨度 7000 ms。八条
`[WHEEL]` 均为 `STATE=DISABLED`、`V=-0.033..-0.011 rad/s`、`TMOS=33 C`、
`TROTOR=31 C`、`TX_FAIL=0`；八条 `HEALTH` 均为 `missed=0`、1 ms 周期、
`TEC=REC=0`、`warning=passive=bus_off=0`。这满足独立的控制器端后续失能/CAN
健康观察，但不声称从 `TEST_COMPLETE` 起连续未中断。`dropped_logs=240` 在浏览器
记录中运动前已存在、整段保持恒定，符合无监听器时 CDC 严格丢弃日志的行为，
不作为本次运动故障报告。

本浏览器导出没有规定的初始 `S` 写入记录，也没有补发该命令。控制器将此判定为
非承载性流程偏差：同一导出在 `A`/`G` 前已经反复给出安全的失能、近零速度、CAN
健康遥测，且 Task 5 已单独验证精确 `S` 路径；它不使已接受的受限运动结果失效，
但本导出中的 `S` 仍明确标注为缺失/未验证。

可复现摘要及分类证据见
[`phase2.2a-h6215-positive-2026-08-08.json`](logs/phase2.2a-h6215-positive-2026-08-08.json)，
由浏览器导出按原顺序拼接所有 `read` 项 `data` 字段得到的原始读取流见
[`phase2.2a-h6215-positive-2026-08-08.log.gz`](logs/phase2.2a-h6215-positive-2026-08-08.log.gz)。

### 2026-08-09 安全加固回归

针对最终检查发现的三个安全问题，固件提交 `75aadc1` 增加了 CDC 未配置/复位
保护、基于 FDCAN 接收时间与主循环周期的保守反馈时间，以及独立于普通队列容量的
`X` 紧急停止通道。主机测试、干净 ARM 构建和 STM32CubeProgrammer 下载/校验均
通过，固件 SHA-256 为
`687cc89d3c9549527fe1565f8e1c51b6f0e3161acc9f89e87a4cd37db682d5af`。

实机验证包括：Disabled 状态下物理拔插 USB 后 MCU uptime 连续增长；一次 33 字符
满队列写入仍立即执行 `X`、零速和 Disable；随后单次 `A`/`G` 回归保持
`V=+0.186 rad/s` 约 1 秒，零速保持约 200 ms 后自动 Disable，并继续记录超过
5 秒的 Disabled/CAN 健康状态。用户再次明确观察到电机逆时针转动；由于观察
视角未定义，仍不将其解释为协议正速度与机构方向的正式映射。

最小回归证据见
[`phase2.2a-h6215-safety-hardening-2026-08-09.json`](logs/phase2.2a-h6215-safety-hardening-2026-08-09.json)。
