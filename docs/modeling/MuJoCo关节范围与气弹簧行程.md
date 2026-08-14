# MuJoCo 关节范围与气弹簧行程

## 主动关节与模态坐标

`robot_geometry.xml` 中的四个主动 A/B 关节均为连续关节，不再把早期实机
标定端点解释为每个主动关节的独立机械范围。权威调试接口采用：

```text
rotation = (qA - qB) / 2
shape    = (qA + qB) / 2
```

`shape` 的机械范围为 `[-0.351, +0.3505] rad`，软件调试范围为
`[-0.281, +0.2805] rad`。`rotation` 是连续坐标，Viewer 使用 `[-pi, +pi]`
作为一圈调试区间，越过边界后按周期继续。机构设计支持连续无限 360 度旋转，
该能力已经由用户在实物机构上验证。

早期 A/B 端点数据继续保存在 `params/joint_limits.yaml`，作为 shape 范围与
实机标定来源证据，不再用于限制 geometry 中的单个 A/B hinge。正方向继续
定义为伸腿。

`CROUCH` 和 `EXTEND` 是 CAD/视觉参考姿态。它们不构成承重或站立验证。

## 气弹簧 slide

动力学模型采用：

```text
q = 0       EXTEND，最大伸展参考
q < 0       气弹簧压缩
range       [-0.0231, 0] m
```

转换只改变 slide 的坐标原点和 keyframe 数值，不改变七个姿态的几何位置。
模型得到 `CROUCH -> EXTEND` 行程 `23.008515 mm`；用户提供的实物行程约
`23 mm`，证据状态为 `USER_ESTIMATED / PROVISIONAL`，第二版结构完成后重测。

## 被动关节扫描

历史 joint 扫描在每条腿的 A/B 软件安全范围内执行 `21 x 21` 网格；当前
modal 扫描则对每条腿执行 `25 x 21` 的 rotation/shape 网格。扫描记录轮轴
`Hx/Hz`、相关被动关节、气弹簧 slide、闭链残差和稳态速度。

已验证结果：

- 历史 882 个 joint 状态全部收敛，无 NaN；
- 当前 1,050 个 modal 状态全部收敛，无 NaN；
- modal 扫描最大闭链残差约 `1.51e-8 m`；
- rotation/shape 最大跟踪误差分别约 `1.29e-7 rad` 和 `5.90e-8 rad`；
- 软件范围内左右 slide 约为 `-21.174 .. -1.713 mm`，没有碰到行程限位；
- 所有被动 hinge 在现有 `[-1.57, +1.57] rad` 范围内具有至少 15% 的扫描
  行程裕量，因此暂不人为收紧；
- 非对称 A/B 命令产生非零 `Hx`，轮轴没有被约束在腿的中心线上。

上述扫描验证的是 MuJoCo 软件工作域，不是被动关节的实物机械限位测量。
详细范围保存在 `params/joint_limits.yaml`，历史证据位于
`results/joint_scan_*`，当前模态证据位于 `results/modal_scan_*`。
