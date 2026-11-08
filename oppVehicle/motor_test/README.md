# ESP32 + HobbyPower SimonK 30A 电机控制

这是一个独立的最小项目，控制 GPIO18 上的 ESC 和 GPIO19 上的转向舵机。它使用开源库 [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)，不包含 PID、IMU、倒车、刹车或油门校准。

## 当前配置

- 开发板：ESP32-D0WD-V3 revision 3.1，4MB Flash
- 串口：`/dev/cu.usbserial-0001`
- ESC 信号：GPIO18，50Hz
- 停止：1000µs
- 速度命令：`speed 0..100`；0 为停止，1–100 映射到 1200–1500µs
- 转向舵机：GPIO19；中位 1500µs，初始限幅 1300–1700µs
- 命令超过 5 秒没有刷新时自动回到 1000µs
- 命令：`speed N`、`steer N`、`center`、`start`、`stop`、`status`
- 开源依赖：`madhephaestus/ESP32Servo@1.2.1`，固定旧版是为了兼容当前 Arduino-ESP32 2.0.17

程序上电只输出 1000µs，不会自动启动。复位后先保持最低油门 5 秒。`speed 0` 或 `stop` 停止；`start` 等同于 `speed 1`。

这里的百分比是 1200–1500µs 受限油门范围内的百分比，不是实测 RPM，也不是 ESC 的完整 1000–2000µs 范围。要保持运行，控制程序需在 5 秒内再次发送速度命令；这样电脑或控制程序失联时会自动停止。

## 接线

- ESC 白色信号线 → ESP32 GPIO18
- ESC 黑色地线 → ESP32 GND
- 舵机黄色信号线 → ESP32 GPIO19
- 舵机红线 → ESC 红色 BEC 线（5V）
- 舵机棕线 → ESP32 GND，与 ESC 黑线共地
- ESP32 由 USB 供电
- ESC 动力端由 7.4V 外部电源供电
- 三相 A/B/C 保持连接电机

最低油门不是主动刹车。ESP32 复位、掉电或信号线脱落时不能继续输出 1000µs；保留人工切断 ESC 电源的手段。

## 编译和上传

上传前关闭 ESC 动力电源：

```sh
cd "/Users/milesye/Desktop/Indep Study/oppVehicle/motor_test"
export PATH="$HOME/.platformio/penv/bin:$PATH"
pio run
pio run -e esp32dev -t upload --upload-port /dev/cu.usbserial-0001
```

打开串口：

```sh
pio device monitor -e esp32dev \
  --port /dev/cu.usbserial-0001 --baud 115200 \
  --rts 0 --dtr 0 --eol LF --echo
```

串口命令：

```text
speed 1    # 已确认可以起转的最低档，1200µs
speed 25   # 约1272µs；5秒内需再次发送以续期
speed 50   # 约1348µs
speed 100  # 1500µs软件上限，不是ESC满油门
speed 0    # 停止
start      # 等同于speed 1
stop       # 立即回到1000µs
steer -25  # 小幅向一侧转向
steer 25   # 小幅向另一侧转向
center     # 回到1500µs中位
status     # 查看电机和转向输出状态
```

## 已验证

- 开源库版本编译、上传成功。
- ESC 断电时执行 `start`，日志显示 1200µs 持续 5 秒，随后自动恢复 1000µs。
- 1200µs 开源库版本已带电运行 5 秒，电机正常转动。

SimonK 上游的默认范围约为停止 1060µs、满油门 1860µs，但克隆固件和历史校准会改变实际范围。[SimonK 源码与说明](https://github.com/sim-/tgy)

四轮必须架空。若电机抖动、ESC 反复重启、电源进入限流或出现异常声音，立即断开 ESC 动力电源。
