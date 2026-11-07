# HOSIM — PS4 蓝牙直接控制

本次降速：油门上限从1200µs降到1180µs，停止仍为1000µs；只修改油门限幅，转向保持不变。降速版已通过ESP32编译及控制逻辑测试，尚未上传或实车验证。1180µs低于此前记录的1200µs起转点，可能无法可靠起步；不自动提高油门补偿。

此前状态：用户确认1600µs时前轮基本回正。按要求将转向指令改为左右对称的1400–1800µs（中位1600µs），保留30%线性+70%三次曲线。右侧端点从1200µs收至1400µs。新版已通过编译、控制逻辑与全行程PWM对称性测试，上传Flash校验通过；实际轮角对称性仍需测量。

此前验证：两只DualShock 4均已成功配对，右摇杆油门已完成实机试验，电机相线及转向方向已调整。当前转向范围1400–1800µs，机械限位尚待确认。按用户要求取消右摇杆回中才能重新使能的条件；本次修改已通过编译与控制逻辑测试，已上传且Flash校验通过；取消回中条件后的实机操作待验证。
适用当前 ESP32-D0WD-V3（经典蓝牙），原装 DualShock 4。兼容手柄须以识别结果为准。
基于 [Bluepad32 官方模板 4.1.0](https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template/tree/4.1.0)，固定提交 `fa03ce64f9aea9d750601aa92ce9893f72066b0a`，Arduino 子模块固定 `5e19e086c43d0fa5e5a596497ff8f11a0a43f6c2`。
使用该版本配套 ESP-IDF 4.4.7 和 PlatformIO espressif32 6.6.0；保留原 motor_test。
PWM 使用 ESP-IDF LEDC，50 Hz、16 bit；日志中的微秒为指令值，未经示波器实测。

## 操作

| 操作 | 效果 |
| --- | --- |
| 连接/重连后松开 L1 | 允许再次按L1使能；复位后先保持最低油门5秒 |
| 按住 L1 + 左摇杆左右 | 转向，1400–1800µs，中间死区，中心附近降低灵敏度 |
| 按住 L1 + 右摇杆向上 | 前进，1000–1180µs，回中/向下为1000µs |
| 松开 L1 | 电机最低油门、转向回中；再次按L1立即按当前摇杆位置输出 |
| ×（Cross）或 L2 | 停止、回中；重新行驶前松开L1和停止键 |
| 断连 / ≥250ms没有新数据 | 停止、回中，恢复后必须先松开L1 |

右摇杆无需回中即可再次按L1启动；如果仍向上推着，再次按L1会立即输出对应油门。启动等待、重连或超时后仍需先松开L1一次。
持续按住L1并向上推右摇杆且数据持续更新时，持续运行，不再使用motor_test的5秒单次测试期限。
R2不再控制油门；右摇杆向下不倒车、不主动刹车。
当前最高脉宽1180µs；此前成功起转记录为1200µs。有效油门跨度由200µs缩为180µs（减少10%），不代表实际车速降低10%。
右摇杆向上大部分行程可能落在ESC启动死区，不是按比例的m/s；不要用先前估算的车速表作为实测值。
根据实车反馈设置main/control.h中kSteerSign=-1：左摇杆向左输出较大PWM（最高1800µs），向右输出较小PWM（最低1400µs），回中1600µs。
转向曲线：死区外归一化行程u映射到0.3u+0.7u³，再按该侧PWM范围缩放。有效行程一半时输出约该侧23.75%的PWM偏移；实际轮角尚未标定。左右PWM幅度均为200µs，同等摇杆行程产生对称指令；实际轮角是否对称仍需实车测量。
2秒任务看门狗用于任务卡死后复位；它不替代250ms输入超时，也不保证掉电后仍输出1000µs。
最低油门和断电均不等于主动刹车。本程序没有倒车、主动制动或油门校准。

## 接线与供电

- ESC白线 → GPIO18；舵机黄线 → GPIO19。
- ESC BEC红线5V → 舵机红线。
- 舵机棕线与ESC黑线接公共地，再接ESP32 GND；舵机回流尽量直接回BEC，避免通过细杜邦线/开发板绕行。
- 当前调试：ESP32由Mac USB供电；外部7.4V电源接ESC动力输入。
- 脱离Mac：可用USB充电宝给ESP32供电；ESC/舵机仍需适配电池供电，并共地。
- 不要将BEC红线与Mac USB 5V在ESP32 VIN上并联。
- 上电时舵机可能移动到1600µs目标位置；它不一定对应前轮机械正中。

## 编译与上传

工作区路径含空格；ESP-IDF不支持，因此脚本将本项目镜像到专用缓存 `$HOME/.cache/hosim-ps4-build` 并在那里编译。

```sh
cd "/Users/milesye/Desktop/Indep Study/oppVehicle"
./ps4_control/tools/pio.sh run
```

上传前关闭ESC外部电源，断开BEC→ESP32 VIN，再接Mac USB；关闭占用串口的monitor。

```sh
./ps4_control/tools/pio.sh run -t upload --upload-port /dev/cu.usbserial-0001
"$HOME/.platformio/penv/bin/pio" device monitor \
  --port /dev/cu.usbserial-0001 --baud 115200 --rts 0 --dtr 0
```

只在ESC断电时进行首次配对：
1. PS4手柄拔掉USB，不要在Mac蓝牙设置里把手柄连接到Mac。
2. 同时按住SHARE + PS，直到灯条快速闪烁。
3. ESP32日志应显示 `PS4 CONNECTED`；手柄灯条变蓝。
4. 松开L1、右摇杆回中，等待 `READY_PRESS_L1`。
5. ESC仍断电，操作L1、左右摇杆，观察PWM；松开L1确认motor=1000。
6. 确认后保持四轮架空，松开全部按键，再打开ESC电源；先验证转向，再L1+右摇杆向上短暂试转。
7. 电机抖动、舵机顶住限位/持续嗡鸣、电源限流时立即断电。

配对信息保留，不在每次启动清除；之后通常按PS重连。如果此前连接Mac/PS4，先断开它们的手柄连接。

## 验证控制逻辑

```sh
clang++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  ps4_control/checks/control_test.cpp -o /tmp/hosim-ps4-control-test
/tmp/hosim-ps4-control-test
```

覆盖启动互锁、停止键、摇杆未回中时松开/再次按L1、重连互锁、250ms新数据超时、计时溢出及全摇杆范围。
此测试不代替蓝牙配对或物理PWM测量。

## 回退

关闭ESC电源后，仍可上传原测试项目：

```sh
"$HOME/.platformio/penv/bin/pio" run -d motor_test -t upload
```
