/*────────────────────────────────────────────────────────
  ESP32 → Rotary-only Tapo Dimmer
  Hamish Burke – May 2025
────────────────────────────────────────────────────────*/
#include <WiFi.h>
// Include the Asynchronous TCP library, required by ESPAsyncWebServer
#include <AsyncTCP.h>
// Include the ESPAsyncWebServer library for creating a web server
#include <ESPAsyncWebServer.h>
// Include the ElegantOTA library for over-the-air updates
#include <ElegantOTA.h>
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

// Create an Asynchronous Web Server object on port 80
AsyncWebServer server(80);

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
  if (WiFi.status() == WL_CONNECTED) {
    return; // Already connected, no action needed
  }
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  // Try to connect for a limited time (approx. 10 seconds)
  while (WiFi.status() != WL_CONNECTED && attempts < 50) { // 50 attempts * 200ms delay = 10 seconds
    delay(200);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed after several attempts.");
  }
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

  // Start the ElegantOTA service, passing the web server object
  ElegantOTA.begin(&server);
  // Start the HTTP server
  server.begin();
  Serial.println("HTTP server started for OTA updates."); // Optional: for debugging

  pinMode(PIN_A,  INPUT_PULLUP);
  pinMode(PIN_B,  INPUT_PULLUP);
  pinMode(PIN_SW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_A), readISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), readISR, CHANGE);

  enc.setBoundaries(1, 100, false);
  enc.setEncoderValue(brightness);
}

void loop() {
  // Handle any pending OTA update tasks
  ElegantOTA.loop();

  // Check Wi-Fi connection and attempt to reconnect if necessary
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(); // Attempt to reconnect to Wi-Fi
    if (WiFi.status() == WL_CONNECTED) {
      loginAll(); // Re-login to bulbs after WiFi reconnection
      // ESPAsyncWebServer and ElegantOTA should handle WiFi reconnections gracefully by themselves.
      // No explicit server restart is usually needed.
    }
  }
  long v = enc.readEncoder();
  if (v != prevVal) {
    prevVal = v;
    brightness = static_cast<uint8_t>(v);
    if (!allOn) toggleAll();
    setAllBrightness(brightness);
  }
  if (enc.isEncoderButtonClicked()) toggleAll();
}
