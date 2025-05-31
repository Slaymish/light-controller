/*────────────────────────────────────────────────────────
  ESP32 → Rotary-only Tapo Dimmer
  Hamish Burke – May 2025
────────────────────────────────────────────────────────*/
#include <WiFi.h>
#include "tapo_device.h"
#include <AiEsp32RotaryEncoder.h>
#include "env.h"


/* Bulbs */
struct Bulb { String ip; TapoDevice dev; };
Bulb bulbs[] = {
  { "192.168.1.144", TapoDevice() },
  { "192.168.1.225", TapoDevice() }
};
constexpr uint8_t NUM_BULBS = sizeof(bulbs) / sizeof(bulbs[0]);

/* Pins */
#define PIN_A  18
#define PIN_B  19
#define PIN_SW 23

AiEsp32RotaryEncoder enc(PIN_A, PIN_B, PIN_SW, -1);

/* State */
bool allOn = false;
uint8_t brightness = 50;
volatile long prevVal = -1;

/* Helpers */
void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(200);
}

void loginAll() {
  for (auto& b : bulbs) { b.dev.begin(b.ip, TAPO_USER, TAPO_PASS); }
}

void setAllBrightness(uint8_t level) {
  for (auto& b : bulbs) b.dev.set_brightness(level);
}

void toggleAll() {
  allOn = !allOn;
  for (auto& b : bulbs) allOn ? b.dev.on() : b.dev.off();
}

/* ISR */
void IRAM_ATTR readISR() { enc.readEncoder_ISR(); }

void setup() {
  Serial.begin(115200);
  connectWiFi();
  loginAll();

  pinMode(PIN_A,  INPUT_PULLUP);
  pinMode(PIN_B,  INPUT_PULLUP);
  pinMode(PIN_SW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_A), readISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), readISR, CHANGE);

  enc.setBoundaries(1, 100, false);
  enc.setEncoderValue(brightness);
}

void loop() {
  long v = enc.readEncoder();
  if (v != prevVal) {
    prevVal = v;
    brightness = static_cast<uint8_t>(v);
    if (!allOn) toggleAll();
    setAllBrightness(brightness);
  }
  if (enc.isEncoderButtonClicked()) toggleAll();
}
