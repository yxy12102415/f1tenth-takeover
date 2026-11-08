#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/ledc.h>
#include <esp_task_wdt.h>
#include <mbedtls/md.h>
#include "link_core.h"
#include "link_private.h"

static_assert(!opp::kActuationEnabled, "This release must remain motor-inhibited");
namespace {
WiFiUDP udp;
opp::Core* core;
IPAddress peer;
uint16_t peerPort = 0;
bool pwmOk = false, listening = false;
uint32_t lastPrint = 0, lastStatus = 0;
uint32_t rejected = 0, authFailed = 0, txFailed = 0;
const size_t kPacketSize = sizeof(opp::Frame) + 32;

bool mac(const uint8_t* data, uint8_t* out) {
  return mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), LINK_KEY,
                         sizeof(LINK_KEY), data, sizeof(opp::Frame), out) == 0;
}
bool authenticate(const uint8_t* data) {
  uint8_t expected[32];
  if (!mac(data, expected)) return false;
  uint8_t different = 0;
  for (size_t i=0;i<32;++i) different |= expected[i] ^ data[sizeof(opp::Frame)+i];
  return different == 0;
}
void reply(IPAddress address, uint16_t port, uint64_t session) {
  opp::Frame frame;
  frame.type = opp::Status; frame.flags = 0x80; // hard motor inhibition
  frame.boot = core->boot; frame.session = session; frame.sequence = core->sequence;
  frame.motor = opp::kStop; frame.steering = opp::kCenter;
  frame.state = core->state; frame.tick = millis();
  uint8_t packet[kPacketSize];
  memcpy(packet, &frame, sizeof(frame));
  if (!mac(packet, packet+sizeof(frame))) return;
  if (udp.beginPacket(address, port)) {
    udp.write(packet, sizeof(packet)); if (!udp.endPacket()) ++txFailed;
  } else ++txFailed;
}
bool configureFixedStopPWM() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE; timer.duty_resolution = LEDC_TIMER_16_BIT;
  timer.timer_num = LEDC_TIMER_0; timer.freq_hz = 50; timer.clk_cfg = LEDC_AUTO_CLK;
  if (ledc_timer_config(&timer) != ESP_OK) return false;
  for (int i=0;i<2;++i) {
    ledc_channel_config_t ch = {};
    ch.gpio_num = i==0 ? 18 : 19; ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ch.channel = i==0 ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1; ch.timer_sel = LEDC_TIMER_0;
    ch.duty = (uint32_t(i==0 ? opp::kStop : opp::kCenter)*65536u+10000u)/20000u;
    if (ledc_channel_config(&ch) != ESP_OK) return false;
  }
  return true; // there are intentionally no runtime PWM writes anywhere
}
}

void setup() {
  pinMode(18, OUTPUT); pinMode(19, OUTPUT); digitalWrite(18, LOW); digitalWrite(19, LOW);
  Serial.begin(115200);
  uint64_t boot = (uint64_t(esp_random())<<32) | esp_random();
  static opp::Core state(boot ? boot : 1); core = &state;
  pwmOk = configureFixedStopPWM();
  if (!pwmOk) {
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    core->stop();
  }
  esp_task_wdt_init(2, true); esp_task_wdt_add(NULL);
  WiFi.persistent(false); WiFi.mode(WIFI_STA); WiFi.setHostname("opp-link-bench");
  WiFi.setSleep(false); WiFi.begin(WIFI_SSID, WIFI_PSK);
  Serial.println("OPP_LINK_BENCH_V1 MOTOR_INHIBITED stop=1000 center=1600");
}

void loop() {
  esp_task_wdt_reset();
  const uint32_t now = millis(); core->tick(now);
  if (WiFi.status() == WL_CONNECTED && !listening) listening = udp.begin(42132);
  if (WiFi.status() != WL_CONNECTED && listening) {
    core->stop(); udp.stop(); listening = false;
  }
  if (listening) {
    // Bound work so a packet flood cannot prevent timeout/watchdog evaluation.
    for (int i=0;i<4;++i) {
      int size = udp.parsePacket(); if (size <= 0) break;
      if (size != int(kPacketSize)) { udp.flush(); continue; }
      const IPAddress address = udp.remoteIP(); const uint16_t port = udp.remotePort();
      uint8_t packet[kPacketSize];
      if (udp.read(packet, sizeof(packet)) != int(sizeof(packet))) continue;
      if (!authenticate(packet)) { ++authFailed; continue; }
      opp::Frame frame; memcpy(&frame, packet, sizeof(frame));
      if (frame.type != opp::Hello && peerPort && (peer != address || peerPort != port)) continue;
      if (!pwmOk || !core->accept(frame, millis())) {
        ++rejected;
        if (rejected<=4) Serial.printf("RX_REJECT type=%u age_ms=%ld boot_match=%u seq=%lu motor=%u steer=%u state=%u\n",
           unsigned(frame.type),long(int32_t(millis()-frame.tick)),unsigned(frame.boot==core->boot),
           (unsigned long)frame.sequence,unsigned(frame.motor),unsigned(frame.steering),unsigned(core->state));
        continue;
      }
      if (frame.type == opp::Command && !peerPort) { peer = address; peerPort = port; }
      if (frame.type == opp::Hello) reply(address, port, frame.session);
    }
    if (peerPort && uint32_t(now-lastStatus) >= 20) {
      reply(peer, peerPort, core->session); lastStatus = now;
    }
  }
  if (uint32_t(now-lastPrint) >= 1000) {
    lastPrint = now;
    Serial.printf("OPP_BENCH ip=%s state=%u requested_us=%u applied_us=1000 inhibited=1 pwm_ok=%u reject=%lu auth_fail=%lu rssi=%ld sleep=%u heap=%lu tx_fail=%lu\n",
                  WiFi.localIP().toString().c_str(), unsigned(core->state), unsigned(core->requested), unsigned(pwmOk),
                  (unsigned long)rejected,(unsigned long)authFailed,long(WiFi.RSSI()),unsigned(WiFi.getSleep()),
                  (unsigned long)ESP.getFreeHeap(),(unsigned long)txFailed);
  }
  delay(1);
}
