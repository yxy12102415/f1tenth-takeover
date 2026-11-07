#include <Arduino.h>
#include <ESP32Servo.h>

// Simple SimonK ESC controller based on the open-source ESP32Servo library.
constexpr int kEscPin = 18;
constexpr int kSteeringPin = 19;
constexpr int kStopUs = 1000;
constexpr int kMinRunUs = 1200;    // Confirmed starting point on this car.
constexpr int kMaxRunUs = 1500;    // Software cap; below nominal SimonK full range.
constexpr int kSteeringCenterUs = 1500;
constexpr int kSteeringMinUs = 1300;  // Conservative initial mechanical limits.
constexpr int kSteeringMaxUs = 1700;
constexpr uint32_t kArmMs = 5000;  // Hold minimum throttle after every reset.
constexpr uint32_t kWatchdogMs = 5000;

Servo esc;
Servo steering;
char command[16];
size_t commandLength = 0;
uint32_t bootAt = 0;
uint32_t runAt = 0;
uint32_t lastStatusAt = 0;
int currentUs = kStopUs;
int speedPercent = 0;
int steeringPercent = 0;
int steeringUs = kSteeringCenterUs;
bool running = false;

void setThrottle(int microseconds) {
  esc.writeMicroseconds(microseconds);
  currentUs = microseconds;
}

void stopMotor(const char *reason) {
  setThrottle(kStopUs);
  speedPercent = 0;
  running = false;
  Serial.printf("STOP pwm=%dus reason=%s\n", currentUs, reason);
}

void setSpeed(int percent) {
  if (percent == 0) {
    stopMotor("speed 0");
    return;
  }
  const int pulse = map(percent, 1, 100, kMinRunUs, kMaxRunUs);
  setThrottle(pulse);
  speedPercent = percent;
  runAt = millis();
  running = true;
  Serial.printf("SPEED percent=%d pwm=%dus watchdog=%lums\n", percent,
                currentUs, static_cast<unsigned long>(kWatchdogMs));
}

void setSteering(int percent) {
  steeringPercent = percent;
  steeringUs = map(percent, -100, 100, kSteeringMinUs, kSteeringMaxUs);
  steering.writeMicroseconds(steeringUs);
  Serial.printf("STEER percent=%d pwm=%dus\n", steeringPercent, steeringUs);
}

void handleCommand(const char *text) {
  if (!strcmp(text, "start")) {
    if (millis() - bootAt < kArmMs) {
      Serial.println("ERROR wait for 5-second minimum-throttle arming hold");
      return;
    }
    setSpeed(1);
  } else if (!strcmp(text, "stop")) {
    stopMotor("command");
  } else if (!strcmp(text, "status")) {
    Serial.printf("STATUS speed=%d%% motor_pwm=%dus state=%s steer=%d%% steering_pwm=%dus\n",
                  speedPercent, currentUs, running ? "RUNNING" : "STOPPED",
                  steeringPercent, steeringUs);
  } else if (!strcmp(text, "center")) {
    setSteering(0);
  } else if (!strncmp(text, "steer ", 6)) {
    char *end = nullptr;
    const long value = strtol(text + 6, &end, 10);
    if (*end != '\0' || value < -100 || value > 100) {
      Serial.println("ERROR use: steer -100..100");
    } else {
      setSteering(static_cast<int>(value));
    }
  } else if (!strncmp(text, "speed ", 6)) {
    char *end = nullptr;
    const long value = strtol(text + 6, &end, 10);
    if (*end != '\0' || value < 0 || value > 100) {
      stopMotor("invalid speed");
      Serial.println("ERROR use: speed 0..100");
    } else if (value > 0 && millis() - bootAt < kArmMs) {
      Serial.println("ERROR wait for 5-second minimum-throttle arming hold");
    } else {
      setSpeed(static_cast<int>(value));
    }
  } else {
    stopMotor("invalid command");
    Serial.println("Commands: speed 0..100 | steer -100..100 | center | start | stop | status");
  }
}

void setup() {
  // Hold the line low before attaching PWM. ESP32Servo then generates 50Hz.
  pinMode(kEscPin, OUTPUT);
  digitalWrite(kEscPin, LOW);
  pinMode(kSteeringPin, OUTPUT);
  digitalWrite(kSteeringPin, LOW);

  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  esc.setPeriodHertz(50);
  steering.setPeriodHertz(50);
  esc.attach(kEscPin, 1000, 2000);
  steering.attach(kSteeringPin, 1000, 2000);
  setThrottle(kStopUs);  // First commanded pulse width is minimum throttle.
  steering.writeMicroseconds(kSteeringCenterUs);
  bootAt = millis();

  Serial.println("ESP32Servo motor and steering controller");
  Serial.println("Motor GPIO18: 50Hz, stop=1000us, run=1200..1500us");
  Serial.println("Steering GPIO19: center=1500us, limits=1300..1700us");
  Serial.println("Commands: speed 0..100 | steer -100..100 | center | start | stop | status");
  Serial.println("Motor stopped and steering centered; neither moves automatically");
}

void loop() {
  if (running && millis() - runAt >= kWatchdogMs)
    stopMotor("5-second command watchdog");

  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (commandLength) {
        command[commandLength] = '\0';
        handleCommand(command);
        commandLength = 0;
      }
    } else if (c >= 32 && c <= 126 && commandLength < sizeof(command) - 1) {
      command[commandLength++] = c;
    } else {
      commandLength = 0;
      stopMotor("input error");
    }
  }

  if (millis() - lastStatusAt >= 1000) {
    lastStatusAt = millis();
    Serial.printf("STATUS speed=%d%% motor_pwm=%dus state=%s steer=%d%% steering_pwm=%dus\n",
                  speedPercent, currentUs, running ? "RUNNING" : "STOPPED",
                  steeringPercent, steeringUs);
  }
  delay(1);
}
