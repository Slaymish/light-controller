/*────────────────────────────────────────────────────────
  ESP32 → Rotary-only Tapo Dimmer
  Hamish Burke – May 2025
────────────────────────────────────────────────────────*/
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFi.h>

// Include the ElegantOTA library for over-the-air updates
#include <ElegantOTA.h>

// WiFi connection constants
static constexpr int WIFI_CONNECT_MAX_ATTEMPTS = 50; ///< Maximum number of attempts to connect to WiFi.
static constexpr int WIFI_CONNECT_DELAY_MS = 200;     ///< Delay in milliseconds between WiFi connection attempts.

#include <SPIFFS.h>
#include "tapo_device.h"
#include <AiEsp32RotaryEncoder.h>
#include "env.h"
#include <EEPROM.h>

// EEPROM Settings
static constexpr size_t EEPROM_SIZE = 64;            ///< Total size allocated for EEPROM storage.
static constexpr int ADDR_TIMEOUT = 0;               ///< EEPROM address for storing inactivityTimeout.
static constexpr int ADDR_TIMER_STEPS = ADDR_TIMEOUT + sizeof(unsigned long); ///< EEPROM address for storing timerSteps.
static constexpr int ADDR_LED_BRIGHTNESS = ADDR_TIMER_STEPS + sizeof(int);      ///< EEPROM address for storing ledRingBrightness.
static constexpr int ADDR_MAGIC_NUMBER = ADDR_LED_BRIGHTNESS + sizeof(uint8_t); ///< EEPROM address for storing the magic number (to check if EEPROM is initialized).
static constexpr uint8_t CURRENT_MAGIC_NUMBER = 0xAB; ///< Current magic number to validate EEPROM data.

/* Bulbs Configuration */
/** @struct Bulb
 *  @brief Structure to hold IP address and TapoDevice object for each bulb. */
struct Bulb { String ip; TapoDevice dev; };
Bulb bulbs[] = {                                     ///< Array of bulbs to control. Add your bulb IPs here.
  { "192.168.1.144", TapoDevice() },
  { "192.168.1.225", TapoDevice() }
  // Add more bulbs as needed: { "IP_ADDRESS", TapoDevice() }
};
constexpr uint8_t NUM_BULBS = sizeof(bulbs) / sizeof(bulbs[0]); ///< Total number of configured bulbs.

// Web Server
AsyncWebServer server(80);                           ///< Asynchronous web server object on port 80.

/* Web UI Settings - These are the default values if not loaded from EEPROM. */
unsigned long inactivityTimeout = 5 * 60 * 1000; ///< Default inactivity timeout: 5 minutes in milliseconds. Controls auto-shutoff.
int timerSteps = 5;                              ///< Default brightness adjustment step for rotary encoder.
uint8_t ledRingBrightness = 100;                 ///< Default brightness for the LED ring (0-255). Placeholder, actual hardware control TBD.

/* Pins */
// Rotary Encoder Pins
static constexpr int PIN_A = 18;  ///< Rotary encoder DT pin (Output A).
static constexpr int PIN_B = 19;  ///< Rotary encoder CLK pin (Output B).
static constexpr int PIN_SW = 23; ///< Rotary encoder SW pin (Switch).

// Rotary Encoder Object
AiEsp32RotaryEncoder enc(PIN_A, PIN_B, PIN_SW, -1); ///< Rotary encoder instance. The last parameter (-1) means no VCC pin is used by the library.

/* Global State Variables */
bool allOn = false;                            ///< Tracks if all bulbs are currently considered ON or OFF.
uint8_t brightness = 50;                       ///< Current brightness level of the bulbs (1-100). Default 50.
volatile long prevVal = -1;                    ///< Previous rotary encoder value to detect changes. Marked volatile as it's modified in ISR.
unsigned long lastActivityTime = 0;            ///< Timestamp of the last interaction (encoder turn/click), used for inactivity timeout.

/* EEPROM Functions */
/**
 * @brief Saves current settings (inactivityTimeout, timerSteps, ledRingBrightness) to EEPROM.
 * Also writes a magic number to indicate that EEPROM has been initialized with valid data.
 */
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
        updateLedRingBrightness(ledRingBrightness); // Call placeholder
    } else {
        Serial.println("No valid settings found in EEPROM, using default values.");
        // Optional: save default values to EEPROM now
        // saveSettings();
    }
    // Ensure loaded values are applied
    enc.setStepValue(timerSteps);
    // Note: updateLedRingBrightness is called from within loadSettings if data is valid.
}

/* Helper Functions */
/**
 * @brief Connects to WiFi using credentials from env.h.
 * Retries connection up to WIFI_CONNECT_MAX_ATTEMPTS times with WIFI_CONNECT_DELAY_MS delay.
 * Prints connection status to Serial.
 */
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return; // Already connected, no action needed
  }
  Serial.println("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  // Try to connect for a limited time (approx. 10 seconds)
  while (WiFi.status() != WL_CONNECTED && attempts < WIFI_CONNECT_MAX_ATTEMPTS) { // 50 attempts * 200ms delay = 10 seconds
    delay(WIFI_CONNECT_DELAY_MS);
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

/**
 * @brief Logs in to all configured Tapo bulbs.
 * Iterates through the `bulbs` array and calls the `begin` method on each device.
 * Assumes `dev.begin()` returns a boolean status (true for success, false for failure).
 * This assumption is made as the `tapo_device.h` header is not available for direct inspection.
 * @note Side effect: Prints login status to Serial for each bulb.
 */
void loginAll() {
  Serial.println("Attempting to login to all bulbs...");
  for (auto& b : bulbs) {
    if (b.dev.begin(b.ip, TAPO_USER, TAPO_PASS)) {
      Serial.println("Successfully logged in to bulb: " + b.ip);
    } else {
      Serial.println("Failed to login to bulb: " + b.ip);
    }
  }
}

/**
 * @brief Sets the brightness for all configured Tapo bulbs.
 * @param level The brightness level to set (typically 0-100).
 * Iterates through the `bulbs` array and calls `set_brightness` on each device.
 * Assumes `dev.set_brightness()` returns a boolean status (true for success, false for failure).
 * This assumption is made as the `tapo_device.h` header is not available for direct inspection.
 * @note Side effect: Prints failure status to Serial if setting brightness fails for a bulb.
 */
void setAllBrightness(uint8_t level) {
  // Serial.print("Setting brightness for all bulbs to: "); // Optional: Could be noisy
  // Serial.println(level);
  for (auto& b : bulbs) {
    if (!b.dev.set_brightness(level)) {
      Serial.println("Failed to set brightness for bulb: " + b.ip);
    }
  }
}

/**
 * @brief Toggles the power state (on/off) for all configured Tapo bulbs.
 * Updates the global `allOn` state, then iterates through the `bulbs` array,
 * calling `on()` or `off()` on each device based on the new state.
 * If lights are turned on, it resets the `lastActivityTime`.
 * Assumes `dev.on()` and `dev.off()` return boolean statuses (true for success, false for failure).
 * This assumption is made as the `tapo_device.h` header is not available for direct inspection.
 * @note Side effect: Prints failure status to Serial if toggling power fails for a bulb.
 * @note Side effect: Modifies global `allOn` state.
 * @note Side effect: Modifies global `lastActivityTime` if lights are turned on.
 */
void toggleAll() {
  allOn = !allOn;
  // Serial.print("Toggling all bulbs to: "); // Optional: Could be noisy
  // Serial.println(allOn ? "ON" : "OFF");

  for (auto& b : bulbs) {
    bool success = false;
    if (allOn) {
      success = b.dev.on();
    } else {
      success = b.dev.off();
    }

    if (!success) {
      Serial.println("Failed to toggle power for bulb: " + b.ip);
    }
  }

  if (allOn) {
    lastActivityTime = millis(); // Reset inactivity timer when lights are turned on
  }
}

/* ISR (Interrupt Service Routines) */
/**
 * @brief Interrupt Service Routine for rotary encoder pin changes.
 * Calls AiEsp32RotaryEncoder's ISR handler. Must be IRAM_ATTR.
 */
void IRAM_ATTR readISR() { enc.readEncoder_ISR(); }

// Web server request handlers
/**
 * @brief Handles incoming POST requests to /settings to update device configuration.
 * Parses parameters for timeout, timer_steps, and led_brightness.
 * Saves valid settings to EEPROM and applies them.
 * @param request Pointer to the AsyncWebServerRequest object.
 */
void handleSettingsUpdate(AsyncWebServerRequest *request) {
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
        updateLedRingBrightness(ledRingBrightness); // Call placeholder
        saveSettings(); // Save the updated settings
        request->send(200, "text/plain", message + "\nRedirecting back in 2 seconds...");
        // It would be better to redirect properly, but for simplicity:
        // request->redirect("/"); // This might not work as expected without more complex JS on client
    }
}

void setupWebServerRoutes() {
  // Serve settings.html
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/settings.html", String(), false);
  });

  // Handle settings update
  server.on("/settings", HTTP_POST, [](AsyncWebServerRequest *request){
    handleSettingsUpdate(request);
  });
}

/**
 * @brief Initializes EEPROM and loads saved settings.
 * Calls EEPROM.begin() and then loadSettings().
 */
void initializeEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  loadSettings(); // Load settings from EEPROM
}

/**
 * @brief Main setup function. Initializes Serial, EEPROM, WiFi, OTA, pins, and web server routes.
 * Also sets initial encoder values and last activity time.
 */
void setup() {
  Serial.begin(115200);
  initializeEEPROM(); // Call the new function

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

  setupWebServerRoutes(); // Call the new function

  enc.setBoundaries(1, 100, false);
  enc.setEncoderValue(brightness); // Brightness is not saved/loaded yet, uses default
  // enc.setStepValue(timerSteps); // This is now called within loadSettings()
  lastActivityTime = millis(); // Initialize activity time after settings are potentially loaded
}

/**
 * @brief Handles rotary encoder interactions (rotation and button clicks).
 * On rotation, updates brightness and potentially toggles lights if they were off.
 * On button click, toggles all lights.
 * Updates lastActivityTime on any interaction.
 */
void handleRotaryEncoder() {
  long v = enc.readEncoder();
  if (v != prevVal) {
    lastActivityTime = millis(); // Update activity time on encoder change
    prevVal = v;
    brightness = static_cast<uint8_t>(v);
    if (!allOn) { // If lights were off, turn them on when encoder is first moved
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

/**
 * @brief Checks Wi-Fi connection status and attempts to reconnect if disconnected.
 * If reconnection is successful, logs in to all Tapo bulbs again.
 */
void checkWiFiConnection() {
  // Check Wi-Fi connection and attempt to reconnect if necessary
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi(); // Attempt to reconnect to Wi-Fi
    if (WiFi.status() == WL_CONNECTED) {
      loginAll(); // Re-login to bulbs after WiFi reconnection
      // ESPAsyncWebServer and ElegantOTA should handle WiFi reconnections gracefully by themselves.
      // No explicit server restart is usually needed.
    }
  }
}

void updateLedRingBrightness(uint8_t brightnessValue) {
  Serial.print("LED Ring Brightness Placeholder: Would set to ");
  Serial.println(brightnessValue);
  // TODO: Add actual hardware code to control LED ring brightness here.
}

/**
 * @brief Checks if the inactivity timeout has been reached.
 * If lights are on and no activity has been detected for the duration of `inactivityTimeout`,
 * this function toggles all lights off.
 */
void checkInactivityTimeout() {
  // Inactivity check
  if (allOn && (millis() - lastActivityTime > inactivityTimeout)) {
    toggleAll(); // Turn off lights
    Serial.println("Inactivity timeout reached. Turning off lights.");
  }
}

/**
 * @brief Main loop function.
 * Handles OTA updates, Wi-Fi connection, inactivity timeout, LED ring brightness, and rotary encoder.
 */
void loop() {
  // Handle any pending OTA update tasks
  ElegantOTA.loop();

  checkWiFiConnection();
  checkInactivityTimeout(); // Call the new function
  updateLedRingBrightness(ledRingBrightness); // Call placeholder in loop

  handleRotaryEncoder();
}
