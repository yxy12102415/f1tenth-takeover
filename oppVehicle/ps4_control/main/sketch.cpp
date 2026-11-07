// HOSIM PS4 controller, based on Bluepad32's Apache-2.0 Arduino template.
#include <Arduino.h>
#include <Bluepad32.h>
#include <driver/ledc.h>
#include <esp_task_wdt.h>
#include "control.h"

namespace {
constexpr int kEscPin = 18;
constexpr int kSteeringPin = 19;
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
ControllerPtr pad = nullptr;
hosim::Control control;
uint32_t bootAt = 0;
uint32_t lastLog = 0;
bool outputsOk = false;
int writtenMotor = -1;
int writtenSteering = -1;

uint32_t duty(int us) { return (uint32_t(us) * 65536u + 10000u) / 20000u; }

void pwmFailure() {
    outputsOk = false;
    control.halt("PWM_ERROR");
    // Best effort safe pulses, followed by low outputs if the driver fails.
    if (ledc_set_duty(kMode, LEDC_CHANNEL_0, duty(hosim::kStop)) != ESP_OK ||
        ledc_update_duty(kMode, LEDC_CHANNEL_0) != ESP_OK)
        ledc_stop(kMode, LEDC_CHANNEL_0, 0);
    if (ledc_set_duty(kMode, LEDC_CHANNEL_1, duty(hosim::kCenter)) != ESP_OK ||
        ledc_update_duty(kMode, LEDC_CHANNEL_1) != ESP_OK)
        ledc_stop(kMode, LEDC_CHANNEL_1, 0);
    Serial.println("ERROR PWM failure; cut ESC power. Reset required.");
}

void applyOutputs() {
    if (!outputsOk) return;
    if (control.motorUs != writtenMotor) {
        if (ledc_set_duty(kMode, LEDC_CHANNEL_0, duty(control.motorUs)) != ESP_OK ||
            ledc_update_duty(kMode, LEDC_CHANNEL_0) != ESP_OK) {
            pwmFailure();
            return;
        }
        writtenMotor = control.motorUs;
    }
    if (control.steeringUs != writtenSteering) {
        if (ledc_set_duty(kMode, LEDC_CHANNEL_1, duty(control.steeringUs)) != ESP_OK ||
            ledc_update_duty(kMode, LEDC_CHANNEL_1) != ESP_OK) {
            pwmFailure();
            return;
        }
        writtenSteering = control.steeringUs;
    }
}

void connected(ControllerPtr ctl) {
    // Device properties are available here; the report class arrives later.
    if (!pad && ctl->getModel() == Controller::CONTROLLER_TYPE_PS4Controller) {
        pad = ctl;
        control.disconnect();
        Serial.printf("PS4 CONNECTED model=%s; release L1 first\n", ctl->getModelName().c_str());
        ctl->setColorLED(0, 0, 80);
    } else {
        Serial.printf("IGNORED controller model=%s (PS4 only, one driver)\n", ctl->getModelName().c_str());
    }
}
void disconnected(ControllerPtr ctl) {
    if (ctl == pad) {
        pad = nullptr;
        control.disconnect();
        applyOutputs();
        Serial.println("STOP controller disconnected");
    }
}
}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(kEscPin, OUTPUT);
    pinMode(kSteeringPin, OUTPUT);
    digitalWrite(kEscPin, LOW);
    digitalWrite(kSteeringPin, LOW);
    ledc_timer_config_t timer = {};
    timer.speed_mode = kMode;
    timer.duty_resolution = LEDC_TIMER_16_BIT;
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = 50;
    timer.clk_cfg = LEDC_AUTO_CLK;
    bool ok = ledc_timer_config(&timer) == ESP_OK;
    for (int i = 0; i < 2; ++i) {
        ledc_channel_config_t channel = {};
        channel.gpio_num = i == 0 ? kEscPin : kSteeringPin;
        channel.speed_mode = kMode;
        channel.channel = i == 0 ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
        channel.timer_sel = LEDC_TIMER_0;
        // Set the safe duty at channel creation, avoiding an initial 1500us ESC pulse.
        channel.duty = duty(i == 0 ? hosim::kStop : hosim::kCenter);
        ok = (ledc_channel_config(&channel) == ESP_OK) && ok;
    }
    outputsOk = ok;
    if (!ok) pwmFailure();
    bootAt = millis();
    // Separate from input freshness protection: reset if the Arduino task stalls.
    if (esp_task_wdt_init(2, true) != ESP_OK || esp_task_wdt_add(nullptr) != ESP_OK) {
        outputsOk = false;
        control.halt("WATCHDOG_ERROR");
        Serial.println("ERROR task watchdog setup; drive disabled");
    }
    BP32.setup(&connected, &disconnected);
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);
    // Keep bonding keys across reboots. Pair using SHARE + PS.
    Serial.printf("HOSIM PS4 / %s\n", BP32.firmwareVersion());
    Serial.printf("GPIO18 ESC=%d..%dus; GPIO19 steer=%d..%dus center=%dus; 50Hz\n",
        hosim::kStop, hosim::kMaxThrottle,
        hosim::kSteerMin, hosim::kSteerMax, hosim::kCenter);
    Serial.println("Hold L1: left stick=steer, right stick UP=forward; center/down=stop; CROSS or L2=stop");
    Serial.println("Steering curve: 30% linear + 70% cubic; endpoints unchanged; wheel angle uncalibrated.");
    Serial.println("5s boot hold; stale input >=250ms stops and requires L1 release");
    Serial.println("No stick-center interlock: pressing L1 applies current stick throttle immediately.");
    Serial.println("Motor starts only on deliberate L1+right stick UP. Servo may move to center at power-on.");
}

void loop() {
    esp_task_wdt_reset();
    // Freshness checked on every iteration, including before new packets.
    control.tick(millis());
    applyOutputs();
    BP32.update();
    const uint32_t now = millis();
    if (pad && pad->isConnected() && pad->hasData() && pad->isGamepad() && outputsOk) {
        hosim::Input input;
        input.x = pad->axisX();
        input.driveY = pad->axisRY();
        input.enable = pad->l1();
        input.stop = pad->a() || pad->l2() || pad->brake() > hosim::kTriggerDeadzone;
        control.packet(now, uint32_t(now - bootAt), input);
    }
    applyOutputs();
    if (uint32_t(now - lastLog) >= 500) {
        lastLog = now;
        Serial.printf("STATUS state=%s motor=%dus steer=%dus pad=%d age=%lums\n",
            control.state, control.motorUs, control.steeringUs, pad != nullptr,
            static_cast<unsigned long>(now - control.lastPacket));
    }
    delay(1);
}
