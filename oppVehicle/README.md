# HOSIM OPP vehicle

ESP32 小车项目，作为 `f1tenth-takeover/oppVehicle/` 保存。当前实现 PS4 手柄直接通过蓝牙遥控 HOSIM；尚未实现 F1→OPP 通信、定位、Waypoint Following 或闭环车速控制。

## 项目与实车状态

- `ps4_control/`：当前小车固件。GPIO18 控制 ESC，GPIO19 控制转向；50Hz PWM。
- `motor_test/`：保留原串口试验项目，未修改。它的油门上限和舵机中位不同于当前 PS4 固件，不应当作当前行驶配置。
- 已实车确认 PS4 配对、前进及转向；1600µs 中位基本回正。左右转向指令为 1400–1800µs，中心附近使用柔和曲线；实际轮角尚未测量。
- 当前 PS4 油门范围 1000–1200µs。松开 L1、停止键或 250ms 无新数据均恢复 1000µs；1000µs 是撤油门，不是主动刹车。无倒车。
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
