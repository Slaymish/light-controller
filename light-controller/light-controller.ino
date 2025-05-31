/*────────────────────────────────────────────────────────
  ESP32 → Rotary-only Tapo Dimmer
  Hamish Burke – May 2025
────────────────────────────────────────────────────────*/
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFi.h>

// Include the Asynchronous TCP library, required by ESPAsyncWebServer
#include <AsyncTCP.h>
// Include the ESPAsyncWebServer library for creating a web server
#include <ESPAsyncWebServer.h>
// Include the ElegantOTA library for over-the-air updates
#include <ElegantOTA.h>

#include <SPIFFS.h>
#include "tapo_device.h"
#include <AiEsp32RotaryEncoder.h>
#include "env.h"
#include <EEPROM.h>

// EEPROM Settings
#define EEPROM_SIZE 64 // Allocate enough space for settings
#define ADDR_TIMEOUT 0
#define ADDR_TIMER_STEPS (ADDR_TIMEOUT + sizeof(unsigned long))
#define ADDR_LED_BRIGHTNESS (ADDR_TIMER_STEPS + sizeof(int))
#define ADDR_MAGIC_NUMBER (ADDR_LED_BRIGHTNESS + sizeof(uint8_t))
#define CURRENT_MAGIC_NUMBER 0xAB

/* Bulbs */
struct Bulb { String ip; TapoDevice dev; };
Bulb bulbs[] = {
  { "192.168.1.144", TapoDevice() },
  { "192.168.1.225", TapoDevice() }
};
constexpr uint8_t NUM_BULBS = sizeof(bulbs) / sizeof(bulbs[0]);

// Create an Asynchronous Web Server object on port 80

/* Web UI Settings */
unsigned long inactivityTimeout = 5 * 60 * 1000; // 5 minutes in milliseconds
int timerSteps = 5; // Brightness adjustment step for timer-based changes
uint8_t ledRingBrightness = 100; // 0-255

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
unsigned long lastActivityTime = 0;

/* EEPROM Functions */
void saveSettings() {
    Serial.println("Saving settings to EEPROM...");
    EEPROM.put(ADDR_TIMEOUT, inactivityTimeout);
    EEPROM.put(ADDR_TIMER_STEPS, timerSteps);
    EEPROM.put(ADDR_LED_BRIGHTNESS, ledRingBrightness);
    EEPROM.put(ADDR_MAGIC_NUMBER, (uint8_t)CURRENT_MAGIC_NUMBER); // Write magic number
    EEPROM.commit();
    Serial.println("Settings saved.");
}

void loadSettings() {
    Serial.println("Loading settings from EEPROM...");
    uint8_t magic;
    EEPROM.get(ADDR_MAGIC_NUMBER, magic);

    if (magic == CURRENT_MAGIC_NUMBER) { // Check if EEPROM data is valid
        EEPROM.get(ADDR_TIMEOUT, inactivityTimeout);
        EEPROM.get(ADDR_TIMER_STEPS, timerSteps);
        EEPROM.get(ADDR_LED_BRIGHTNESS, ledRingBrightness);
        Serial.println("Settings loaded successfully.");
    } else {
        Serial.println("No valid settings found in EEPROM, using default values.");
        // Optional: save default values to EEPROM now
        // saveSettings();
    }
    // Ensure loaded values are applied
    enc.setStepValue(timerSteps);
}

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
  if (allOn) {
    lastActivityTime = millis(); // Reset inactivity timer when lights are turned on
  }
}

/* ISR */
void IRAM_ATTR readISR() { enc.readEncoder_ISR(); }

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  loadSettings(); // Load settings from EEPROM

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

  SPIFFS.begin(true);
  server.begin();

  // Serve settings.html
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/settings.html", String(), false);
  });

  // Handle settings update
  server.on("/settings", HTTP_POST, [](AsyncWebServerRequest *request){
    String message = "Settings updated successfully!";
    bool error = false;

    if (request->hasParam("timeout", true)) {
        inactivityTimeout = request->getParam("timeout", true)->value().toInt() * 1000; // Convert s to ms
    } else {
        message = "Error: Timeout parameter missing.";
        error = true;
    }

    if (request->hasParam("timer_steps", true)) {
        timerSteps = request->getParam("timer_steps", true)->value().toInt();
    } else {
        message = "Error: Timer Steps parameter missing.";
        error = true;
    }

    if (request->hasParam("led_brightness", true)) {
        ledRingBrightness = request->getParam("led_brightness", true)->value().toInt();
    } else {
        message = "Error: LED Brightness parameter missing.";
        error = true;
    }

    if (error) {
        request->send(400, "text/plain", message);
    } else {
        enc.setStepValue(timerSteps); // Apply timer steps setting before saving
        saveSettings(); // Save the updated settings
        request->send(200, "text/plain", message + "\nRedirecting back in 2 seconds...");
        // It would be better to redirect properly, but for simplicity:
        // request->redirect("/"); // This might not work as expected without more complex JS on client
    }
  });

  enc.setBoundaries(1, 100, false);
  enc.setEncoderValue(brightness); // Brightness is not saved/loaded yet, uses default
  // enc.setStepValue(timerSteps); // This is now called within loadSettings()
  lastActivityTime = millis(); // Initialize activity time after settings are potentially loaded
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
  // Inactivity check
  if (allOn && (millis() - lastActivityTime > inactivityTimeout)) {
    toggleAll(); // Turn off lights
    Serial.println("Inactivity timeout reached. Turning off lights.");
  }

  // TODO: Implement LED ring brightness control using ledRingBrightness variable

  long v = enc.readEncoder();
  if (v != prevVal) {
    lastActivityTime = millis(); // Update activity time on encoder change
    prevVal = v;
    brightness = static_cast<uint8_t>(v);
    if (!allOn) {
      toggleAll();
    }
    setAllBrightness(brightness);
  }

  if (enc.isEncoderButtonClicked()) {
    // lastActivityTime is updated by toggleAll() if it turns lights on,
    // or we can set it explicitly if lights were already on.
    // For simplicity, toggleAll() handles it if lights turn on.
    // If lights are turned off, lastActivityTime is not critical for the *next* timeout period.
    // If lights were already on and are toggled (which is rare for a simple click),
    // toggleAll will update it.
    // Explicitly setting it here ensures it's always updated on a click.
    lastActivityTime = millis();
    toggleAll();
  }
}
