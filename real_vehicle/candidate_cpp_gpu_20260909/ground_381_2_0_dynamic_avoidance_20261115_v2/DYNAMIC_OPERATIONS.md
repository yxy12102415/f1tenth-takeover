# 381 v2 操作

软件与实车分开记录。旧成功入口保留不变；新入口固定选择本目录版本。

在工作区根目录：

```bash
cd ~/f1tenth_real_migration_ws_20260829
./381_2_0_dynamic_avoidance.sh pid status
./381_2_0_dynamic_avoidance.sh dynamic_mpc status
```

查看软件验收或重跑合成测试（不启动电机）：

```bash
./381_2_0_dynamic_avoidance_prepare.sh status
./381_2_0_dynamic_avoidance_prepare.sh static pid
./381_2_0_dynamic_avoidance_prepare.sh moving dynamic_mpc
```

实车检查/试车入口（本次未执行；软件验收通过，实车行驶开关仍关闭）：

```bash
./381_2_0_dynamic_avoidance.sh pid check
./381_2_0_dynamic_avoidance.sh pid run
./381_2_0_dynamic_avoidance.sh dynamic_mpc run
```

`run` 沿用现有会话的定位、手柄 L1、数据时效和零输出检查；匹配的已完成证据按原机制复用。不要在车辆尚未复位时执行。第一次验证静止前车，再验证同向慢速前车。原地图、原起点与原开放路线不变；指令最高 2.0 m/s，持续行驶最低 0.6 m/s，正加速度上限 1.0 m/s²，终点锁定零输出。

空间不足以完成超过、回归、停车时进入跟随。381 跑完跟随测试不等于完成完整超车。完整动态超车是否通过，单独读取 `reports/dynamic_preparation/acceptance.json` 的 `full_dynamic_overtake_passed`；实车结果另行验收。

## Tight profile follow-up

Preferred swept-body gap is now read from `adaptive_corridor_v2.desired_body_gap_m` by the actual C++ planner: 0.10 m. Envelope seed reserve is 0.02 m. Controller predicted-body hard gap remains 0.08 m. Map/coast, curvature and 0.5 s source freshness checks remain active. Maximum speed remains 2.0 m/s, running command floor 0.6 m/s, positive acceleration 1.0 m/s².

Captured-scene replay and PID/dMPC offset closed-loop software cases passed for these exact profile settings. The new tight profile has not yet been physically qualified. Previous profile and source snapshots are in `review/pre_tight_profile_20261115`. Read-only handset monitoring recorded native button 4 held for 3.809 s; no remapping was required.

Before arm, the disabled/stationary planner keeps a validated path while the pose is stable; initialization or a translation above 3 cm / heading change above 0.05 rad refreshes the anchor. Every new scan still revalidates the path, and a newly blocked path triggers replanning. Once enabled/moving, the selected path remains continuous unless fresh occupancy invalidates it. Envelope seeds now include time-aligned predicted bounds and choose an approach length from available collision distance. The reference map speed preview includes the recent steering range to anticipate servo recovery; the independent current-state coast/body check is unchanged.

## Control timing update: 200 ms

PID、dMPC 及最终输出前的完整控制回调硬门限为 200 ms。dMPC 求解迭代预算仍为 85 ms，残差容差仍为 0.001；连续三次超过 45 ms 仍触发持续过载停车。传感器数据门限仍为 0.5 s。

参考地图限速表由规划器预计算，与 plan_id、完整路径、地图数值版本、速度下限和墙面余量绑定。控制器收到新路径时一次校验，PID 预测和 MPC 各预测步复用表格。裁剪或更换路径必须生成匹配的新表；参数或地图不匹配会拒绝。当前状态停车距离、障碍预测和输出前最新观测复核仍实时执行。

`mpc_reference_speed_ms`、`mpc_static_corridor_ms`、`mpc_parameter_update_ms` 分别记录参考速度、静态走廊和求解器参数更新；动态走廊为 `mpc_tube_setup_ms`。它们均属于参考准备耗时，不能当成纯求解耗时。完整回调另记录 `control_to_emit_ms` 与 `runtime_timing.control_to_drive_publish_ms`，后者仍不是执行器收到命令的延迟。

本次更改只做软件验证。不得以软件通过宣称实车 READY；实车行驶后必须更新复位确认。新失败应先报告首个拒绝原因，与用户讨论后再进行新的架构或约束变更。

## Low-inflation controlled-track policy

静态目标的运动不确定性与额外测量膨胀为 0；紧凑物体已有的隐藏实体外廓补全、F1 实体外廓和 8 cm 碰撞间距仍参与检查。当前受控赛道内，无运动历史且速度估计不超过 0.08 m/s 的新鲜目标按静态观测处理；一旦观测到运动则切换预测模型。

已确认、观测新鲜且运动方向沿参考线的目标仍预测前进位置，预测不确定性上限为纵向 2 cm、横向 1 cm，不继续随预测时长放大。横穿、方向不符、长期遮挡和未确认运动继续原兜底规则。

扩展静态点簇记录测量凸包及 0.6 s 支持历史，碰撞和预测走廊使用同一凸包，防止斜墙的轴对齐大矩形填满空白角。凸包内部仍算占用；没有改成只检查离散点。

用户选择：所有至少 4 次观测且无运动证据、速度不超过 0.03 m/s 的确认静态目标（含非地图物体与静止前车），被遮挡时保持原位置、原轮廓和原时间戳，不额外加 12 cm 或未知运动增长。曾移动的目标保留运动证据，不因失去可见性误认成墙。

## Reachable speed-limit transitions

停车距离预览的新上限不再瞬间施加到 MPC 第一个未来状态。绝对指令上限仍为 2 m/s；新的预览上限通过受减速度 -0.8 m/s²、首步加速度变化率 4 m/s³、后续模型步变化量 0.15 m/s² 约束的阶段上界收敛。接近 0.6 m/s 底速时同时保留释放减速度所需余量，防止到达底速仍带着不可消除的负加速度。

上限下降时使用可行的制动序列初始化 MPC，独立核对未来阶段速度界。提前将停车距离余量限速加入参考速度。完整地图／车身预测、最新观测复核、当前真实速度的停车轨迹检查继续在输出之前执行；预览减速过渡不是碰撞检查豁免。PID 控制策略沿用前版，并做回归检查。

本轮旧日志未保存失败瞬间的最新障碍快照，因此状态回放使用确切状态和路径、可用的规划观测及冷启动求解种子；不能视为完整等输入实车重演。新日志会在求解之前保存实际使用的观测，避免求解失败时丢失它。

## Prepared observation and prearm-search optimization

每个不可变观测／时间上下文只解析一次障碍，地图静态先验和不随预测时间变化的静态边界预先计算。动态与未知运动仍按原预测时刻逐点计算。主预测检查和最新观测复核分别准备各自的新观测，不跨消息复用旧数据，不刷新旧时间戳。

有效路径仍检查原来的全部扫掠采样点、地图、实体间距、地图速度可行性与运动预测；同一点重复的地图检查及相同静态检查合并。先做便宜的静态几何排除，再为有希望通过的候选计算速度可行性。纯静态场景在生成候选种子时无需预测到达时间，但最终候选仍执行原地图速度可行性检查。

准备阶段取消每秒清空有效路径。起点姿态明显改变时重新锚定；每个新扫描仍独立复核路径，真实新障碍可以立即使旧路径失效。行驶时保持原来的连续路径策略。

本轮固定输入测试：19 组预测拒绝／通过结果与旧版一致；同一条保留路径复核中位约 82 ms → 15 ms，预测检查约 16 ms → 4 ms；3 次 200 ms 预算的冷启动搜索均找到有效路线，耗时约 166–196 ms。冷启动可因截止时间选择不同的有效候选；相同保留路径的对比才是配对耗时证据。实车是否通过另行记录。

## Current clearance adjustment after the 2026-11-16 contact run

User requested a modest increase in passing offset. Desired body gap is now 0.15 m and envelope seed reserve 0.07 m (previously 0.10 / 0.02 m); independent hard gap remains 0.08 m and wall margin 0.03 m. Both tracker profiles derive from the same config. Offset is still computed from obstacles and map, not forced to a fixed d. Preferred-gap search may still fall back to the hard gap; inspect the selected candidate's actual `generic_required_body_gap_m`.

Recorded prearm-scene fresh searches at 200 ms selected d=-0.17863 m three times with the 0.15 m gap. Previous physical run held d=-0.13673 m; this comparison is indicative, not a guarantee on a new scan. The failed physical run reported rear contact during post-stop coasting. This configuration adjustment does not qualify a fix for `joint_prediction_corridor_unavailable`, nor does it change stopping behavior. Real retest remains pending.

## Motion-confirmation fix after the 2026-11-17 static-opponent rejection

DynamicMemory now separates internal filtered velocity from published velocity for a confirmed static component. A quiet shared-support baseline of at least 0.30 s can latch static; fresh measured geometry is still updated. Ordinary movement requires coherent translation of both support edges by more than 4.5 cm over at least 0.30 s, filtered speed above 0.08 m/s, and three consecutive supporting scans. Strong coherent translation above 10 cm / 0.18 s at above 0.35 m/s can revoke the latch earlier. Settling uses the longer low-speed support baseline; the static latch can be revoked on later motion. Explicit unknown/occluded-moving prediction remains unchanged.

The native test covers bounded 1 cm jitter, 0.15 / 0.5 / 0.75 m/s starts, lateral motion, stop and restart. Recorded-box regression uses 676 published support observations to infer temporal state, then preserves the recorded failure geometry and replaces only its motion classification for a cold-state MPC replay. This is not a raw LiDAR or full warm-solver replay. The reproduced corridor failure clears and independent predicted-body guard passes in that diagnostic. Physical qualification remains pending.

## Controlled-scene lead identity and route prior (2026-11-17)

User declares all background objects stationary; only the lead may move and it mainly follows the same reference route. Reported lead outer tyre dimensions: length 0.32 m, width 0.26 m (corrected by user; supersedes 0.23 x 0.30 m). These dimensions gate association; measured obstacle geometry and the existing physical hidden-extent completion remain intact.

`lead_identity.hpp` is called once per new aligned scan in the production C++ observation producer. Initial acquisition uses route-relative position (0.6–3.3 m ahead, |d|<=0.65 m), component size and a trimmed symmetric point-distance score against eight newly captured shell-view templates. Largest local support is used only for appearance matching. Every raw obstacle remains in the collision stream, including isolated points and unmatched components. Adjacent components can be jointly associated without replacing their separate physical geometry.

Acquisition needs three consistent observations and >=0.15 s; disjoint equally plausible candidates stay ambiguous. After acquisition, position continuity dominates, permitting component-ID changes. Measured d is retained. Associated forward motion receives the route prior; evident lateral/reversing motion revokes it. Only an explicitly identified route-aligned lead gets the 0.65 m route-prior band; unlabelled legacy inputs retain the original 0.35 m band. Semantic role is part of the projection-cache key. All prediction source timestamps remain unchanged.

Unassociated background is annotated fixed-world under the declared scene assumption, not deleted. Near the last lead, unresolved support retains its original conservative motion model. Identity loss does not itself command a stop, renew old observation stamps or permit arbitrary distant reacquisition. Normal collision, sensing, freshness and command deadlines still apply. `lead_identity`, `semantic_role` and `lead_identity_ms` are recorded in observations/plans.

Validation scope: eight-view same-capture scan holdouts (not independent recognition accuracy), synthetic ID changes/ambiguity/motion and recorded observation reclassification. In the previous terminal failure, original occupancy rejects the held path, while controlled-scene classification preserves and admits that same path. Closed-loop software and physical results must be qualified separately; full moving overtaking remains unqualified. The Python reference under `../../lead_route_identity_20261117` is a separate read-only prototype; the actual controller integration is C++.

Dimension correction: the operator subsequently corrected the outer tyre dimensions to length 0.32 m / width 0.26 m. Both generated tracker profiles use these values. No C++ binary rebuild is needed for this runtime configuration change. The gap regression previously required every 200 ms search to attain a soft 15 cm preference, contradicting the documented 8 cm fallback. Its assertions now require a valid hard-checked path and the analytically expected additional offset from the seed-reserve change, and explicitly report how many trials attained 15 cm. The initial strict-assertion failure is retained in `reports/compact_fix/lead_dimensions_32x26_gap_retry.log`; runtime gap/fallback policy was not changed.
