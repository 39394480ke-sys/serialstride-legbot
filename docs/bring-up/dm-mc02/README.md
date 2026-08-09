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

## Phase 2 完整 H6215 驱动迁移

2026-08-09 将 F103 台架已验证的反向脉冲、连续速度、斜坡、主机看门狗与
显式停止迁移到 MC02。提交 `3dc2464` 的主机测试和干净 ARM 构建通过，
二进制 SHA-256 为
`514106d11e02f69f05824df70d8851fdb008896c98d1c2373a08ad97bef206d9`，
并由 STM32CubeProgrammer 写入 `0x08000000`、校验和复位成功。

实机上，`A`/`B` 反向脉冲三次均返回 `V=-0.209 rad/s`，运行 1 秒、
零速保持 200 ms 后自动 Disable。连续模式按 100 ms 速度步进从零到
`+0.500 rad/s`，实际反馈 `+0.494 rad/s`；第六个 `+` 仍限制在
`+0.500 rad/s`。随后斜坡反转至 `-0.500 rad/s`，实际反馈
`-0.517 rad/s`。`0` 会斜坡回零但保持 Enabled；再到 `+0.200 rad/s`
后停止控制字符，5 秒看门狗按预期斜坡回零、保持 200 ms 并 Disable。
另一次连续测试以 `X` 结束，立即零速并 Disable。所有运动段均无
`SAFETY_TRIP` 和新的 TX failure，CAN warning/passive/bus-off 均为零。

用户在同一观察位置明确确认正速度先逆时针、负速度后顺时针。
该方向映射仅对此台架当前观察面成立。

最后的 30 分钟 24 V 带电静置采集到 1800 组 `[WHEEL]` 和 1800 组
`HEALTH`；uptime 从 645406 ms 增至 2444406 ms，无复位，1800 组均为
ONLINE 且 Disabled，无 CAN 异常，TX failure 增量为零，最高温度为
`TMOS=30 C` / `TROTOR=28 C`。危险故障注入（超温、超速和运动中 USB
断开）仅由主机测试覆盖，未对实机人为制造故障。

最小证据摘要见
[`phase2-h6215-complete-2026-08-09.json`](logs/phase2-h6215-complete-2026-08-09.json)。

## Phase 3：CAN1 单侧三电机核心 Bring-up

2026-08-09 在 MC02 CAN1 1 Mbps 总线上同时接入两台 DM4310
关节电机和一台 H6215 轮毂电机。本阶段完成了核心 Bring-up，
但不将尚未执行的长时间稳定性、实机掉线和危险故障注入写成已验证。

### CAN1 逻辑拓扑与设备参数

三台电机共享同一对 CAN1 `CAN_H`/`CAN_L` 线，通信速率为
1 Mbps。具体电源端子和电机接口必须按 MC02 与电机官方说明书接线；
本记录不根据尚未归档的实物接线推测端子定义。

| 角色 | 电机 | CAN ID | MST ID | 控制模式 | 已读参数 |
|---|---|---:|---:|---|---|
| `JOINT_A` | DM4310 | 6 | 3 | MIT, `MODE=1` | `P_MAX=12.5`, `V_MAX=30`, `T_MAX=10` |
| `JOINT_B` | DM4310 | 8 | 4 | MIT, `MODE=1` | `P_MAX=12.5`, `V_MAX=30`, `T_MAX=10` |
| `WHEEL` | H6215 | 1 | 0 | Velocity, `MODE=3` | `P_MAX=12.5`, `V_MAX=45`, `T_MAX=10` |

设备 ID 和反馈 ID 无冲突。最终归档日志中三台电机均在线、参数有效、
状态为 `Disabled`，CAN 保持 error-active，`TEC=0` 且 `REC=0`。
终端电阻尚未形成本阶段的正式测量验收记录。

### 安全操作与已验证动作

固件默认不自动运动。在电机电源断开时先使用 `A`、`P` 完成
VCC_OUT1 受限上电流程；上电后使用 `R` 完成三设备探测，再用
`1`/`2`/`3` 选择单台电机，或用 `4` 选择三电机并行受限测试。
选择完成后，每次运动前都需要再发送 `A` 进入 10 秒一次性运动预备，
然后发送 `G` 或 `B`。
`X` 是独立优先级急停通道，会清零并 Disable 全部电机。

已验证：

- 三台电机可同时在线，参数与反馈正确；
- 逐台受限正反向运动，同时保持非目标电机失能；
- 三电机并行受限运动；
- `STOP ALL`/`X` 可正常中止并失能全部电机；
- USB 断连 HardFault、旧 CAN 反馈时间戳、急停队列和 Disable 确认问题已修复；
- 统一 `MotorState`、`motor_manager` 和 `safety_manager` 已成为当前架构。

动作结果由用户的实机观察确认；归档的原始串口日志只直接证明三台在线、
Disabled、参数正确与 CAN 无主动错误，不单独证明运动或 `STOP ALL`。

### 已接受例外与未验证边界

1. 未执行三电机长时间 Powered + Disabled 稳定性测试。
2. 未执行单设备实机掉线测试；掉线逻辑只有主机测试证据。
3. 未对实机人为注入危险保护故障。
4. 未完成标准 CAN 终端阻值的正式测量验收。
5. 未验证负载、机构装配、承重和长期运动。

归档原始日志见
[`records-2026-08-09-22-20-44.json`](logs/records-2026-08-09-22-20-44.json)。
详细阶段摘要见
[`2026-08-09_DM-MC02_CAN1三电机Bring-up.md`](../../progress/2026-08-09_DM-MC02_CAN1三电机Bring-up.md)。
