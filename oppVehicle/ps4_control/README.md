# 前车 PS4：L1 固定低油门直行

当前版本 `fixed-straight-v3-1150-calibration`。1150 µs 固件已重新烧录、Flash 校验和启动检查通过，默认停止指令为 1000 µs；实体 L1 输出复核已完成；一次地面测速约 0.483 m/s（0.5 秒雷达回归窗口）。已参与两车试验，最新无接触，但 F1 MPC 中途停车，整段超车尚未完成。使用前车自己的 PS4 连接 ESP32；F1 手柄不控制前车。

| 操作 | 前车输出 |
|---|---|
| 上电后等待 5 秒，并松开 L1 | 允许启动 |
| 按住 L1 | 固定 1150 µs 油门，转向保持 1600 µs |
| 松开 L1 | 油门回到 1000 µs，转向保持回中 |
| × 或 L2 | 收油；松开停止键和 L1 后才能再启动 |
| 断连或 250 ms 没有新数据 | 收油；恢复后必须先松开 L1 |
| 左右摇杆 | 不改变油门或方向 |

1140 µs 已验证 L1 指令，但用户报告地面测试电机无声、不起步，蓝灯及 ESC 提示音正常。1150 µs 一次地面观察前移约 1.43 m、横移约 −0.02 m（用户按住约 3 秒，位移包含滑行）。这不是跨电量、负载的恒速标定。此前 1160 µs 雷达测速约 0.64 m/s，并有直行偏差。固定 PWM 不是闭环恒速；1000 µs 是收油滑停，不是主动刹车。转向保留 1600 µs，待实测确认。

参数位于 `main/control.h`：`kFixedThrottleUs`、`kCenter`。启动互锁、输入超时、任务看门狗与 PWM 故障处理保留。

## 当前车载机编译

```bash
cd /home/f1tenth/f1tenth-takeover-381-overtaking/oppVehicle/ps4_control
./tools/bootstrap.sh
/home/f1tenth/.local/share/opp-link-tools/bin/pio run
```

上传前保持 ESC 动力关闭，USB 供电时 BEC→ESP32 VIN 断开。目标是 CP2102 的 ESP32 串口，不是 VESC。

```bash
/home/f1tenth/.local/share/opp-link-tools/bin/pio run -t upload
```

配对用 SHARE＋PS；已有配对通常按 PS 即可。先在 ESC 断电时确认串口日志：L1 按下为 `FIXED_STRAIGHT motor=1150us steer=1600us`，松开为 `READY_PRESS_L1 motor=1000us`。之后架空短测，再测地面速度和松开后的滑行距离。

接线保持 GPIO18→ESC、GPIO19→转向，50 Hz / 16 bit LEDC。基础为固定版本 Bluepad32 4.1.0 模板及 ESP-IDF 4.4.7；版本由 bootstrap 脚本固定。

## 本地逻辑验证

```bash
g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined checks/control_test.cpp -o /tmp/opp-fixed-straight-test
/tmp/opp-fixed-straight-test
```

原始完整 Flash 已校验备份在 `../../deployment/381_2_0/opp_link/private/esp32_before_opp_link_20261107.bin`。备份含配对信息，仅保存在本机，禁止提交到 Git。
