# QJ2M 遥操作已知问题

供后续 agent / 开发者修改时参考。按 ISSUE 追加，解决后把状态改为 `resolved` 并注明 PR / commit。

---

## ISSUE-001: 加长臂肘外翻导致左右失稳

- **状态:** in_progress（已落地 soft `joints_task`；待仿真目视 + 真机验收）
- **记录日期:** 2026-08-12
- **优先级:** high（影响真机站立遥操作稳定性）

### 现象

手臂加长后，结构仍按先前版本设计。当双手肘部关节向外翻转（左肘偏向身体左侧、右肘偏向身体右侧）时，机器人很容易在左右方向上失去平衡。

### 运控侧结论

较稳定的手臂布局：**左右肘内收、向后**，以减轻左右偏转力矩。

约束：肘过分内收会碰机器人本体。规划目标应是：

- 肘尽量向后
- 尽量不向外（避免外翻 / 外展）
- 不要过度贴胸内收（自碰）

### 根因理解

1. 加长臂增大了肘在外侧时的左右（滚转）力臂，WBC 更难稳住。
2. 7-DOF 冗余：腕部位姿确定后，肘仍可在“外翻 / 后收”等构型间选择。
3. 原 Placo IK：EE `frame_task` + 弱 `manipulability`，无肘后收零空间偏好。
4. `HOME_Q_*` 只在一键回零生效，不约束连续遥操作。
5. `manipulability` 常偏好外展，与稳定布局冲突。

### 已实施修改（2026-08-12）

| 项 | 取值 / 位置 |
|----|-------------|
| `PREF_Q`（左右相同） | `[0.52, 0, 0, -2.09, 0, 0, 0]` |
| `HOME_Q_*` | **未改**（回零仍用原非对称测量姿） |
| `JOINTS_TASK_WEIGHT` | `5e-4`（soft；EE=1.0，manip≈1e-2） |
| 常量与 helper | `xrobotoolkit_teleop/utils/qj2m_homing.py`（`pref_joints_dict`） |
| 硬件 | `qj2m_teleop_controller.py` `_placo_setup` 增加 `joints_task` |
| 仿真 | `scripts/simulation/teleop_qj2m_placo.py` 同上 |

未改：`base_teleop_controller` 全局、关节限位、`manipulability` 权重、肘 link 空间约束。

**注意：** 理想姿 J2/J3=0 是否真为「肘后收」需 Placo 目视确认；若偏外/偏前，优先改 `PREF_Q` 的 J2/J3，再调权重。

### 相关代码

| 文件 | 说明 |
|------|------|
| `xrobotoolkit_teleop/utils/qj2m_homing.py` | `HOME_Q_*`、`PREF_Q_*`、`JOINTS_TASK_WEIGHT` |
| `xrobotoolkit_teleop/hardware/qj2m_teleop_controller.py` | 真机 soft joints 正则 |
| `scripts/simulation/teleop_qj2m_placo.py` | 仿真 soft joints 正则 |
| `xrobotoolkit_teleop/common/base_teleop_controller.py` | EE + manipulability（未改） |

参考：`scripts/simulation/teleop_unitree_g1_placo.py`（`add_joints_task`）。

### 后续可选

1. 肘仍外翻 → 权重提到 `1e-3`，或略降 `manipulability`（`1e-2` → `1e-3`）
2. 跟手发粘 → 权重降到 `1e-4` / `2e-4`
3. 肘 link soft position 约束（侧向/后方）
4. 与运控再标定 `PREF` 的 J2/J3

### 验收

- [ ] Placo 可视化：理想 `PREF_Q` 肘在身后、略内收、不蹭躯干
- [ ] Placo：同腕部目标下肘不再持续外翻
- [ ] 真机：肘外展倾向动作时左右晃动明显下降
- [ ] 内收/后收不导致持续蹭躯干
- [ ] 一键回零仍走到原 `HOME_Q_*`（与 `PREF` 分离）

### 备注

调参或改 `PREF_Q` 后同步更新本 ISSUE。
