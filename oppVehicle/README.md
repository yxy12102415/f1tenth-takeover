# HOSIM OPP vehicle

ESP32 小车项目，作为 `f1tenth-takeover/oppVehicle/` 保存。当前实现 PS4 手柄直接通过蓝牙遥控 HOSIM；尚未实现 F1→OPP 通信、定位、Waypoint Following 或闭环车速控制。

## 项目与实车状态

- `ps4_control/`：当前小车固件。GPIO18 控制 ESC，GPIO19 控制转向；50Hz PWM。
- `motor_test/`：保留原串口试验项目，未修改。它的油门上限和舵机中位不同于当前 PS4 固件，不应当作当前行驶配置。
- 已实车确认 PS4 配对、前进及转向；1600µs 中位基本回正。左右转向指令为 1400–1800µs，中心附近使用柔和曲线；实际轮角尚未测量。
- 当前源码的 PS4 油门范围 1000–1180µs（本次由1200µs下调，降速版尚未烧录或实车验证）。松开 L1、停止键或 250ms 无新数据均恢复 1000µs；1000µs 是撤油门，不是主动刹车。无倒车。
- 没有确认存在编码器或速度传感器，因此 PWM 不能直接换算成 m/s。

## 在 Mac 编译

从仓库根目录执行 `cd oppVehicle`。需要 Git、rsync、PlatformIO CLI；本机 CLI 位于 `$HOME/.platformio/penv/bin/pio`。

```sh
./ps4_control/tools/pio.sh run
clang++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  ps4_control/checks/control_test.cpp -o /tmp/hosim-ps4-control-test
/tmp/hosim-ps4-control-test
```

第一次编译会从官方模板下载依赖，固定模板提交 `fa03ce64f9aea9d750601aa92ce9893f72066b0a` 和 Arduino 子模块 `5e19e086c43d0fa5e5a596497ff8f11a0a43f6c2`。上游组件未修改，通过 `tools/bootstrap.sh` 恢复，不重复上传大型依赖和构建缓存。许可证信息见 `ps4_control/LICENSE` 及下载组件内的许可证文件。

包含空格的源码路径由脚本镜像到 `$HOME/.cache/hosim-ps4-build` 编译；请勿同时编译不同副本。已有完整组件目录会保留，不由脚本覆盖。

上传前：关闭 ESC 动力电源，断开 BEC→ESP32 VIN，USB 接 Mac，关闭占用串口的 monitor。

```sh
./ps4_control/tools/pio.sh run -t upload --upload-port /dev/cu.usbserial-0001
"$HOME/.platformio/penv/bin/pio" device monitor \
  --port /dev/cu.usbserial-0001 --baud 115200 --rts 0 --dtr 0
```

完整接线和操作见 [PS4 控制说明](ps4_control/README.md)。仅编译或推送 GitHub 不会更新 ESP32；必须单独上传固件。

## 本次降速

最大油门指令由1200µs降为1180µs，有效跨度减少10%，不是实测车速减少10%。有效摇杆行程一半对应1090µs，推满对应1180µs。转向中位1600µs、左右端点1400/1800µs及停止逻辑保持不变。

验证：原工作区与通过固定版本依赖恢复的导出副本均编译成功；控制逻辑测试（含1,050,625组摇杆输入、停止/超时及转向对称性）通过。尚未烧录降速版或实车测量。

此前1200µs能起转，1180µs可能无法可靠起步，尤其落地带载时。新版本须先架空验证；若不转或抖动，应松开L1停止，不通过持续堵转或自动加油门补偿。若需要可靠的低速巡航，下一步应测量起转/维持转动阈值与轮速；现有软件不能保证指定m/s。

## 一个 PS4 控制两辆车：建议方案（尚未实现）

```text
PS4 ──蓝牙──> F1 Jetson / ROS 2
                  ├──现有 F1 许可、控制与停车链路──> F1 驱动
                  └──新增 OPP bridge ──无线链路──> ESP32 ──> OPP ESC/转向
```

手柄只配对F1。F1保留现有L1运行许可，并向OPP发送运行许可、OPP专用油门/转向指令和心跳。第一版可由PS4摇杆遥控OPP，F1继续执行它自己的跟车/规划控制；同一个L1协调启停。不要直接把F1的转向角复制给前车，两车路径位置及转向机构不同。

### 蓝牙还是 Wi-Fi

F1→ESP32蓝牙可以设计为BLE GATT或经典蓝牙SPP，但需确认Jetson蓝牙适配器、BlueZ环境及与PS4同时连接的表现；当前代码没有这种接收协议。

建议第一版使用本地Wi-Fi上的UDP（不依赖互联网）：ESP32已有Wi-Fi，便于ROS节点发送心跳和记录日志。仍需确认F1现有无线接口/热点条件；无法提供本地Wi-Fi时再采用BLE。Wi-Fi并不天然保证时延或停车可靠性，必须做链路中断验证。

链路接收固件应作为独立模式，只有一个控制源拥有PWM输出。当前Bluepad32使用BTstack，不能直接与Arduino `BluetoothSerial` 所用的蓝牙栈拼接；见[Bluepad32开发说明](https://bluepad32.readthedocs.io/en/stable/developer_notes/)。当前PS4直连固件保留作回退版本。

### 与现有 F1 停车链路的接口

已核对仓库中的[v8监督程序](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/stand_follow.py)及[运行窗口](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/stand_window.py)：

- PS4输入为`/baseline/joy`，L1是`buttons[4]`。
- `/stand_pf/permit`是监督许可；`/baseline/final`还会经过运行窗口，输出`/stand_pf/actuator_input`，之后才进入执行器保护。
- 运行窗口还检查健康许可、输入新鲜度、90秒时限等；L1释放及故障会锁定停车，不能仅靠重新按L1自动恢复该F1会话。

新增bridge必须综合**新鲜的监督许可、L1、运行窗口输出/状态及执行器状态**，任何源失效或停车锁定都撤销OPP运行许可。单独订阅原始摇杆、名义控制或`/baseline/final`不足以覆盖所有下游停车条件。部署前应核对F1车上实际启动版本及topic，仓库导出不是可直接替代车上工作区的部署包。

### 接收端与联动规则

建议50Hz发送，OPP本地250ms没有收到有效新包就恢复1000µs并锁定停止；这个数是接收超时预算，不是从故障发生到车辆静止的保证。指令包含会话标识、递增序号、运行许可和限幅后的控制量；配对/鉴权并拒绝旧会话、重放、重复包和越界值，重复旧包不能续期。超时后的恢复必须重新握手并释放L1，不自动续跑。

F1应监测OPP的应答和状态：OPP掉线也撤销两车公共运行许可。此项需要与F1监督状态机明确集成，不能由bridge绕过或代发原有安全许可。两车均就绪后才允许共同启动，停止优先于油门渐变。

“F1动，OPP才动”建议定义为：**F1获准运行且下游有有效前进指令时，OPP才获准执行自己的前进指令**。若确实要求F1已经发生物理移动，需额外使用新鲜里程计并设迟滞；会产生起步延迟。对于前车实验，等后车先移动也可能和避障停车形成互锁，不能直接以轮速大于零作为唯一启动逻辑。

PS4可以继续充当无线人工停止输入，但不等同于独立硬件急停。当前OPP的1000µs只是撤油门，车辆会滑行，不能保证与F1同时静止；保留独立断电手段，需要短制动距离时应选择经确认支持制动的车用ESC。

实现顺序：先确认F1车上的接口和无线条件 → 做只输出日志的bridge → ESC断电时验证PWM与断连锁停 → 架空低油门联动 → 测量实际停车距离后再低速落地。这里仅记录设计，没有新增运动节点、修改F1安全代码或部署到两辆车。
