# Fitzpatrick 现场操作单（2026-09-27 核对）

现有地图名为 `Fitzpatrick`，区分大小写。建议先沿用旧地图，重新定位并录制新路线。只有墙体、场地布局与地图明显不符，或定位无法对齐时，才重新建图。

2026-09-27 用户已在新场地对齐并指定固定起点：地图坐标 x=0.6138574243 m、y=-0.3471186155 m、yaw=0 rad（朝地图 +X）。已保存 `config/Fitzpatrick_fixed_start.json`，包含地图 SHA256、采样及确认记录。以后沿用该实际位置和车头朝向，不自动换成其他路线的首点；新录路线/平滑路线要检查首点是否匹配此锚点。当前录线文件为 `routes/manual_route_20260927_204931.csv`。起点文件只是持久记录，不会自动完成每次开机后的定位；若重新建图导致坐标系改变，不能直接套用旧坐标。

本次录线已结束并完成闭合平滑：`routes/Fitzpatrick_fixed_start_closed_v1.csv`，全长约 41.27 m，起点位置/朝向不变，前 6 m 为半径 5.60 m 的恒曲率左弯，闭合接缝位置、朝向、曲率连续。保守参考线静态净距至少约 0.712 m，所需最大转角 0.258 rad，无自交。对应 `Fitzpatrick_fixed_start_closed_v1_speed_profile.csv` 的曲率巡航配置为 0.70–2.00 m/s、加速度 1.00、减速度 0.80，起终点参考速度 0。地图图示为 `routes/Fitzpatrick_fixed_start_closed_v1_map.png`。生成工具为 `prepare_fitzpatrick_closed_route.py`，报告与导出文件独立校验结果均在 CSV 同目录。

**后续按用户要求重新规划，当前选定方案为 v2：** `routes/Fitzpatrick_fixed_start_arc_straight_v2.csv`，保留上述固定起终点和朝向，长度 44.73 m；起步先走半径 7.80 m、长 11.95 m 的恒曲率圆弧，随后三段直线分别为 4.80 / 3.38 / 11.05 m，其余三段定半径圆弧为 2.80 / 2.80 / 1.536 m。约 90.6% 路程是严格直线或定半径圆弧，连接处使用 7 段各 0.60 m 的线性曲率缓和段，保持包括首尾接缝在内的曲率连续。地图保守净距 0.793 m，最大转角 0.161 rad，按 2 m/s 计算的几何转角变化率上界 0.543 rad/s；无自交，独立积分、固定起终点、直线共线性和圆弧半径检查通过。旧 v1 保留，不再作为当前首选。速度文件为同名 `_speed_profile.csv`，配置仍为 0.70–2.00 m/s、+1.00/-0.80 m/s²、起终点 0；图为同名 `_map.png`。选择记录为 `config/Fitzpatrick_route_plan.json`。

**新固定起点路线已接入独立版本 `ground_fitzpatrick_joint_20260927_v2`。** PID / 动态 MPC 整圈模型和 ROS 终点测试通过；模式为单圈回到起点停车，90 秒行驶上限。实车验收另行记录。原 381 成功版本保留。

最新启动入口是 `./fitzpatrick_avoidance_pid.sh` 和 `./fitzpatrick_avoidance_dynamic_mpc.sh`（完整步骤见第 6 步）；启动器要求匹配的离线证据及实车零输出定位检查。旧 `fitzpatrick_pid.sh` / `fitzpatrick_dynamic_mpc.sh` / `new_site_*.sh` 仍为历史跟踪版，`381_2_0_frenet_*.sh` 仍使用 381 地图和路线。

1. 打开 Ubuntu 图形桌面、连接 PS4，进入工作目录。

   ```bash
   cd ~/f1tenth_real_migration_ws_20260829
   ./ensure_ps4_controller.sh
   MAP=Fitzpatrick
   ```

   以下命令在这个终端依次执行；另开终端需要重新 `cd`，并重新设置 `MAP`。当前建图/录线包装脚本固定使用 `DISPLAY=:1`；若 RViz 没打开，先检查桌面显示设置，不要按 L1 试探。

2. 可选：重新建图。沿用旧地图则完全跳过此步。

   终端 A：

   ```bash
   ./new_site_slam.sh
   ```

   按住 L1，用右摇杆控制前后、左摇杆转向。松开 L1 停车。建图时先移走测试纸盒，避免把它录入静态地图。

   扫描完成后保持建图程序运行、松开 L1。在终端 B 保存为新名字，保留原地图：

   ```bash
   cd ~/f1tenth_real_migration_ws_20260829
   ./save_current_slam_map.sh Fitzpatrick_v2
   ```

   等待打印 `Saved map and pose graph`，再回终端 A 按一次 Ctrl+C，等待 `Stopped`。不能先退出 SLAM 再保存。若 `Fitzpatrick_v2` 已存在，请另选未使用的名字，不要删除原地图来绕过检查。

   在终端 A 改用刚保存的地图：

   ```bash
   MAP=Fitzpatrick_v2
   ```

3. 重新定位，并录制新起点的参考路线。

   将车放在准备长期使用的起点，标记车身位置与朝向。这个起点不需要与当初 SLAM 起点相同。

   ```bash
   ./new_site_record_route.sh "$MAP"
   ```

   在 RViz 点击 `2D Pose Estimate`，在地图上的车辆实际位置按下鼠标，沿车头方向拖出箭头。确认激光点与墙面对齐、定位稳定，然后在启动程序的终端输入：

   ```text
   ALIGNED
   ```

   等待录制就绪，按住 L1 手动开完整条路线。尽量选择宽敞直道、缓弯，预留一段能放纸盒且侧面有绕行空间的直道；先不放纸盒。

   到终点后松开 L1，再按一次 Ctrl+C，等待停止。**不用回到起点闭合路线**；脚本有历史提示要求返回附近，但当前路线预处理输出的是 `open_stop_at_end`。停止录制前不要搬动车辆。

   终端打印的 `Route will be saved to: ...csv` 就是本次原始路线。保存此路径，并记录/拍照标记实际起点与朝向。当前手动录线速度配置为 0.70–1.50 m/s，加速度 1.00 m/s²。

4. 平滑路线并检查地图净距、曲率。

   在同一终端输入下面两行；第一行会提示粘贴上一步打印的 CSV 完整路径，粘贴时不加引号：

   ```bash
   read -r -p '粘贴本次原始路线 CSV 路径: ' RAW
   ./new_site_prepare_route.sh "$RAW" "$MAP"
   ```

   必须看到 `CONTROLLER READY` / `controller_ready: true` 才继续。`clearance_ok` 或 `steering_ok` 为 false 时，先调整路线，不能仅凭生成了 CSV 就开始跑。

   不使用只传地图名的自动选线方式：它会在所有地图的原始路线中找最新文件，可能拿错路线。平滑只检查参考路线的地图净距与转向可行性，并不保证任意纸盒位置都有可行绕行路径。

5. 生成供最新版接入使用的速度参考。

   按用户最新要求，Fitzpatrick 新避障候选使用 0.70–2.00 m/s 巡航目标，加速度保持 1.00 m/s²。第 4 步旧脚本自动生成的曲线加速度为 1.50，因此另生成明确命名的参考文件：

   ```bash
   ROUTE="${RAW%.csv}_controller_ready.csv"
   PROFILE="${RAW%.csv}_controller_ready_avoidance_speed_profile.csv"
   python3 generate_speed_profile.py "$ROUTE" "$PROFILE" \
     --minimum-speed 0.70 --maximum-speed 2.00 \
     --acceleration 1.00 --deceleration 0.80
   ```

   起点、终点参考速度为 0；参考曲线加减速过渡点可能低于 0.50，但它不是直接的执行器命令。实际 PID / 联合动态 MPC 保留软启动、达到 0.50 后的非零命令下限和终点切零锁止；独立输出窗口也检查下限。软启动和切零后的实际轮速不可避免经过 0–0.50，不能承诺实际轮速瞬间跳变。仅生成此文件不会更新任何控制器绑定。输出已存在时脚本拒绝覆盖。

6. 最新避障版测试（本次固定起点及新规划路线）：

   ```bash
   ./fitzpatrick_avoidance_pid.sh
   # PID 结束、车辆复位并恢复朝向、L1 松开后，再执行：
   ./fitzpatrick_avoidance_dynamic_mpc.sh
   ```

   入口绑定下述各自版本、Fitzpatrick 地图和 `Fitzpatrick_fixed_start_arc_straight_v2.csv`；不自动选最新录制 CSV。更换路线需重新接入验证。

   动态 MPC 入口现指向 `ground_fitzpatrick_joint_20260927_v5`，包含航向跨 ±180° 修复，并补齐缓存路径以覆盖预测窗口，避免巡航反复加减速；v5 用轮速和带响应延迟的转角估计初始化角速度，修复入弯状态不一致，无临时障碍整圈实测通过，用户确认跑完且稳定；另有一次偶发计算超时，调度稳定性仍待验证；PID 入口仍指向 v2。

   两个控制器目标范围 0.70–2.00 m/s、加速度 1.00 m/s²，起终点为 0；软启动达到 0.50 后，非零命令保持至少 0.50，终点切零锁止。单圈最长 90 秒，松开 L1 立即撤销驱动。每轮移动后必须复位。

   启动器复用同一指纹的离线集成证据 24 小时、同一控制器的零输出/定位证据 15 分钟；缺少时按需检查，实车零输出检查持续 5 秒。无需每次重复整套模型测试。

   先不放纸盒验证完整路线及终点停车，再在宽敞直道放纸盒验证避障。车辆须在固定起点并恢复朝向，L1 松开；等待 READY 后按住 L1。录线/SLAM 进程应先完全退出。每次结束检查轨迹、是否碰撞、摆动及停车表现。

### 2026-09-27 双障碍动态 MPC 测试

当前动态 MPC 入口为 `ground_fitzpatrick_joint_20260927_v6`；v5 无障碍成功日志保留。v6 候选路径纳入整个约 3.2 m 局部窗口内的障碍，防止 2 m/s 时控制预测先看到障碍而规划尚未绕行。速度及碰撞门限保持。

- A：首段长弯道，(6.022, 1.832) m，燕麦桶约直径 10 cm、高 20 cm。
- B：最后长直线，(-1.172, 7.276) m，瓶外套约直径 10 cm 不透明筒，覆盖雷达扫描高度。
- 图：`maps/Fitzpatrick_two_obstacles_A_B.png`。两物体中心放在图中叉号。
- 双障碍感知/整圈模型及实际 ROS 检查通过；现场双障碍实测尚未完成。
- 放好、车辆回固定起点并恢复朝向、L1 松开、人员离开后运行 `./fitzpatrick_avoidance_dynamic_mpc.sh`，实车 READY 后按住 L1。

### v7 输入检查时序修复

动态 MPC 入口现为 `ground_fitzpatrick_joint_20260927_v7`：先获取不可变输入快照，再取检查时间，避免刚到的数据被旧检查时间误判；新增故障快照日志。门限仍为 500 ms，路线、速度与 v6 相同。新版本 ROS 检查已通过，实车待复测。v6 的终点提前停车问题仍保留。


2026-09-27 v8 更新：当前动态 MPC 一键入口指向 `ground_fitzpatrick_joint_20260927_v8`。修正连续动态状态估计，预测横向加速度限值 1.65 m/s²（原 1.50），弯道目标仍按 1.20。离线/ROS 检查通过，实车结果另记；PID 入口不变。

2026-09-27 23:20 v8 实车更新：A/B 双障碍整圈 27.98 s 通过，终点误差 7.45 cm；用户确认两处均无碰撞、基本无摆动、自然停稳。当前命令仍为 `./fitzpatrick_avoidance_dynamic_mpc.sh`。速度配置 0.70–2.00 m/s；起步软启动，起终点 0，正常行驶后正速度不低于 0.50 m/s。仅完成一轮该版本实测，后续再确认重复性。
