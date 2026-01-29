#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_DRV2605.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Heltec Wireless Tracker V1.1 TFT pins (from datasheet)
// TFT: ST7735 160x80, SPI interface
static const int TFT_CS   = 38;
static const int TFT_RST  = 39;
static const int TFT_DC   = 40;  // TFT_RS
static const int TFT_SCLK = 41;  // TFT_SCLK
static const int TFT_MOSI = 42;  // TFT_SDIN

// Power control
static const int VEXT_CTRL = 3;  // Vext Ctrl (powers TFT + GNSS)
static const int TFT_LED_K = 21; // Backlight cathode control

// Vext control polarity (some Heltec boards use HIGH to enable)
static const int VEXT_ACTIVE = HIGH;

// DRV2605L haptic driver (I2C) - fixed pins
static const int I2C_SDA = 4;
static const int I2C_SCL = 5;
static const uint16_t MOTOR_PULSE_MS = 200;

// User button (GPIO0) cycles vibration intensity
static const int USER_BTN_PIN = 0;
static const bool USER_BTN_ACTIVE_LOW = true;
static const uint32_t BTN_DEBOUNCE_MS = 200;
static const uint32_t BTN_HOLD_MS = 700;

enum UiSetting : uint8_t {
  UI_VIBE = 0,
  UI_PING = 1,
  UI_FEEDBACK = 2
};

// Status LED
static const int STATUS_LED_PIN = 18;
static const bool LED_ENABLED = false;
static const uint32_t LED_PULSE_MS = 40;

// Use explicit MOSI/SCLK pins as in known working examples
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_DRV2605 drv;

// LoRa pins (Heltec Wireless Tracker V1.x)
static const int LORA_NSS  = 8;
static const int LORA_SCK  = 9;
static const int LORA_MOSI = 10;
static const int LORA_MISO = 11;
static const int LORA_RST  = 12;
static const int LORA_BUSY = 13;
static const int LORA_DIO1 = 14;

// LoRa settings (US915 defaults)
static const float LORA_FREQ_MHZ = 915.0;
static const float LORA_BW_KHZ = 500.0;
static const uint8_t LORA_SF = 6;
static const uint8_t LORA_CR = 5;
static const uint8_t LORA_SYNC_WORD = 0x12;
static const int8_t LORA_TX_POWER = 22;

SPIClass spiLoRa(FSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spiLoRa);

volatile bool loraRxFlag = false;
volatile bool loraInterruptEnabled = true;

static uint32_t nextPingMs = 0;
static uint32_t pingCounter = 0;
static uint32_t lastPingSentMs = 0;
static uint32_t pingIntervalLogEvery = 10;
static int16_t lastRssi = 0;
static float lastSnr = 0.0f;
static uint32_t motorStopMs = 0;
static bool drvOk = false;
static uint8_t intensityMode = 0; // 0=soft,1=medium,2=hard,3=max
static uint32_t lastBtnMs = 0;
static bool lastBtnState = false;
static uint32_t btnPressStartMs = 0;
static bool btnHoldHandled = false;
static uint32_t ledOffMs = 0;
static bool ledOn = false;
static uint8_t uiSetting = UI_VIBE;
static bool feedbackConstant = true;
static TaskHandle_t vibeTaskHandle = nullptr;
static volatile bool requestHapticPulse = false;
static volatile int requestHapticStep = 0;
static uint32_t lastRxMs = 0;
static const uint32_t PING_RATES_MS[] = {1000, 800, 600, 400, 300, 200, 100, 50};
static uint8_t pingRateIndex = 0;
static uint32_t pingBaseMs = 300;
static uint32_t pingJitterMs = 100;
static const uint32_t PING_LATE_WARN_MS = 50;

static const uint32_t TX_WARN_MS = 80;
static const uint32_t PEER_TIMEOUT_MS = 5000;

enum Role : uint8_t { ROLE_SEARCH = 0, ROLE_PINGER = 1, ROLE_RESPONDER = 2 };
static Role role = ROLE_SEARCH;
static uint32_t deviceId = 0;
static uint32_t peerId = 0;
static uint32_t lastPeerSeenMs = 0;
static const uint32_t VIBE_TIMEOUT_MS = 5000;
static uint32_t nextVibeToggleMs = 0;
static int lastStep = 19;
static float smoothedRssi = -100.0f;
static const float RSSI_SMOOTH_ALPHA = 0.10f;
static bool vibeOn = false;
static float smoothedIntervalMs = 600.0f;
static const float INTERVAL_SMOOTH_ALPHA = 0.20f;

// Watchdog: re-enter RX mode if we go quiet
static const uint32_t RX_WATCHDOG_MS = 3000;

void setLoraFlag() {
  if (!loraInterruptEnabled) {
    return;
  }
  loraRxFlag = true;
}

int estimateDistanceStep(int16_t rssiDbm) {
  // Map RSSI into 20 coarse "distance" steps (0 = very close, 19 = very far)
  // Adjust rssiMin/rssiMax if you want a different range.
  const int16_t rssiMin = -120;
  const int16_t rssiMax = -10;
  int16_t rssi = rssiDbm;

  if (rssi > rssiMax) rssi = rssiMax;
  if (rssi < rssiMin) rssi = rssiMin;

  float normalized = (float)(rssiMax - rssi) / (float)(rssiMax - rssiMin);
  int step = (int)round(normalized * 19.0f);
  if (step < 0) step = 0;
  if (step > 19) step = 19;
  return step;
}

const char* distanceLabel(int step) {
  if (step <= 2) return "VERY CLOSE";
  if (step <= 6) return "CLOSE";
  if (step <= 12) return "MID";
  if (step <= 16) return "FAR";
  return "VERY FAR";
}

const char* intensityLabel(uint8_t mode) {
  switch (mode) {
    case 0: return "soft";
    case 1: return "medium";
    case 2: return "hard";
    default: return "max";
  }
}

void updateIntensityDisplay() {
  tft.fillRect(0, 12, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 12);
  tft.print("Vibe: ");
  tft.print(intensityLabel(intensityMode));
}

void updatePingRateDisplay() {
  tft.fillRect(0, 24, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 24);
  tft.print("Ping: ");
  tft.print(pingBaseMs);
  tft.print("ms");
}

void updateFeedbackDisplay() {
  tft.fillRect(0, 36, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 36);
  tft.print("Mode: ");
  tft.print(feedbackConstant ? "constant" : "pulse");
}

void updateUiSelectionDisplay() {
  tft.fillRect(0, 48, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 48);
  tft.print("Edit: ");
  if (uiSetting == UI_VIBE) tft.print("vibe");
  else if (uiSetting == UI_PING) tft.print("ping");
  else tft.print("feedback");
}

void flashStatusLed() {
  if (!LED_ENABLED) {
    return;
  }
  digitalWrite(STATUS_LED_PIN, HIGH);
  ledOn = true;
  ledOffMs = millis() + LED_PULSE_MS;
}

String buildPing() {
  char buf[10];
  snprintf(buf, sizeof(buf), "P%08lX", (unsigned long)deviceId);
  return String(buf);
}

String buildReply() {
  char buf[10];
  snprintf(buf, sizeof(buf), "R%08lX", (unsigned long)deviceId);
  return String(buf);
}

bool parsePeerId(const String& payload, uint32_t& outId) {
  if (payload.length() < 9) return false;
  char c0 = payload[0];
  if (c0 != 'P' && c0 != 'R') return false;
  String hex = payload.substring(1, 9);
  outId = (uint32_t)strtoul(hex.c_str(), nullptr, 16);
  return true;
}

void updateRoleWithPeer(uint32_t otherId) {
  if (otherId == 0 || otherId == deviceId) return;
  peerId = otherId;
  lastPeerSeenMs = millis();
  Role newRole = (deviceId < otherId) ? ROLE_PINGER : ROLE_RESPONDER;
  if (role != newRole) {
    role = newRole;
    Serial.print("Role: ");
    Serial.println(role == ROLE_PINGER ? "PINGER" : "RESPONDER");
  }
}

uint8_t motorIntensityForStep(int step) {
  // Map step (0=close .. 19=far) to DRV2605L realtime value (0..127)
  int intensity = (int)round(127.0f - (step / 19.0f) * 100.0f); // 127..27
  if (intensity < 20) intensity = 20;
  if (intensity > 127) intensity = 127;
  return (uint8_t)intensity;
}

uint8_t motorIntensityWithMode(int step) {
  uint8_t intensity = motorIntensityForStep(step);
  // Apply user-selected intensity scaling
  switch (intensityMode) {
    case 0: intensity = (uint8_t)round(intensity * 0.40f); break; // soft
    case 1: intensity = (uint8_t)round(intensity * 0.65f); break; // medium
    case 2: intensity = (uint8_t)round(intensity * 0.85f); break; // hard
    case 3: default: /* max */ break;
  }
  if (intensity < 10) intensity = 10;
  if (intensity > 127) intensity = 127;
  return intensity;
}

void motorPulseForStep(int step) {
  if (!drvOk) {
    return;
  }
  uint8_t intensity = motorIntensityWithMode(step);
  drv.setRealtimeValue(intensity);
  motorStopMs = millis() + MOTOR_PULSE_MS;
}

uint32_t vibrationIntervalForStep(int step) {
  // Closer -> faster pulses
  // step 0 => 120ms, step 19 => 800ms
  const uint32_t minMs = 120;
  const uint32_t maxMs = 800;
  float t = (float)step / 19.0f;
  uint32_t interval = (uint32_t)round(minMs + (maxMs - minMs) * t);
  if (interval < minMs) interval = minMs;
  if (interval > maxMs) interval = maxMs;
  return interval;
}

float vibrationOnRatioForStep(int step) {
  // Closer -> longer on time
  // step 0 => 0.80, step 19 => 0.30
  float t = (float)step / 19.0f;
  float ratio = 0.80f - 0.50f * t;
  if (ratio < 0.30f) ratio = 0.30f;
  if (ratio > 0.80f) ratio = 0.80f;
  return ratio;
}

void vibrationTask(void* parameter) {
  const TickType_t delayTicks = pdMS_TO_TICKS(10);
  for (;;) {
    if (drvOk) {
      uint32_t now = millis();

      // Handle short haptic pulse requests (e.g. button feedback)
      if (requestHapticPulse) {
        requestHapticPulse = false;
        motorPulseForStep(requestHapticStep);
      }

      // Stop motor after pulse duration
      if (motorStopMs != 0 && now > motorStopMs) {
        drv.setRealtimeValue(0);
        motorStopMs = 0;
      }

      // Vibration rate control: only vibrate if we received recently
      if (now - lastRxMs > VIBE_TIMEOUT_MS) {
        if (motorStopMs != 0 || vibeOn) {
          drv.setRealtimeValue(0);
          motorStopMs = 0;
          vibeOn = false;
        }
        nextVibeToggleMs = now + 200;
      } else if (feedbackConstant) {
        // Smooth interval to reduce jitter
        float targetInterval = (float)vibrationIntervalForStep(lastStep);
        smoothedIntervalMs = INTERVAL_SMOOTH_ALPHA * targetInterval
                           + (1.0f - INTERVAL_SMOOTH_ALPHA) * smoothedIntervalMs;

        float onRatio = vibrationOnRatioForStep(lastStep);
        uint32_t onMs = (uint32_t)round(smoothedIntervalMs * onRatio);
        uint32_t offMs = (uint32_t)round(smoothedIntervalMs - onMs);
        if (onMs < 60) onMs = 60;
        if (offMs < 40) offMs = 40;

        if (now >= nextVibeToggleMs) {
          if (vibeOn) {
            drv.setRealtimeValue(0);
            vibeOn = false;
            nextVibeToggleMs = now + offMs;
          } else {
            uint8_t intensity = motorIntensityWithMode(lastStep);
            drv.setRealtimeValue(intensity);
            vibeOn = true;
            nextVibeToggleMs = now + onMs;
          }
        }
      }
    }

    vTaskDelay(delayTicks);
  }
}

void setup() {
  // Initialize Serial - using USB CDC on ESP32-S3
  Serial.begin(115200);
  
  // Wait for USB CDC to be ready
  while(!Serial && millis() < 5000) {
    delay(10);
  }
  
  // Longer delay for serial monitor connection
  delay(5000);
  Serial.println("\nStarting in 3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("  Wireless Tracker - TFT + LoRa");
  Serial.println("========================================\n");

  // Seed RNG for ping jitter
  randomSeed((uint32_t)ESP.getEfuseMac());
  uint64_t mac = ESP.getEfuseMac();
  deviceId = (uint32_t)(mac ^ (mac >> 32));
  Serial.print("Device ID: ");
  Serial.println(deviceId, HEX);

  // Enable Vext (powers TFT + GNSS)
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, VEXT_ACTIVE);
  delay(100);

  // Enable TFT backlight (polarity may vary by board)
  pinMode(TFT_LED_K, OUTPUT);
  digitalWrite(TFT_LED_K, HIGH);
  delay(50);

  // Setup user button
  pinMode(USER_BTN_PIN, INPUT_PULLUP);

  // Setup status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Initialize TFT
  tft.initR(INITR_MINI160x80_PLUGIN);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // Draw test text
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.println("Wireless Tracker");
  tft.println("TFT OK!");
  tft.println("LoRa init...");

  tft.setTextSize(2);
  tft.setCursor(0, 40);
  tft.print("HELLO");

  Serial.println("TFT init complete.");

  // Initialize LoRa radio
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = lora.begin(LORA_FREQ_MHZ);
  lora.setDio2AsRfSwitch();
  lora.setTCXO(1.6);
  lora.setBandwidth(LORA_BW_KHZ);
  lora.setSpreadingFactor(LORA_SF);
  lora.setCodingRate(LORA_CR);
  lora.setOutputPower(LORA_TX_POWER);
  lora.setSyncWord(LORA_SYNC_WORD);
  lora.setCRC(true);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init OK.");
    tft.fillRect(0, 24, 160, 12, ST77XX_BLACK);
    tft.setCursor(0, 24);
    tft.println("LoRa OK");
  } else {
    Serial.print("LoRa init failed: ");
    Serial.println(state);
    tft.fillRect(0, 24, 160, 12, ST77XX_BLACK);
    tft.setCursor(0, 24);
    tft.println("LoRa FAIL");
  }

  lora.setDio1Action(setLoraFlag);
  lora.startReceive();

  // Setup DRV2605L (I2C) - fixed SDA/SCL pins
  Wire.begin(I2C_SDA, I2C_SCL);
  drvOk = drv.begin();

  if (drvOk) {
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_REALTIME);
    drv.setRealtimeValue(0);
    Serial.println("DRV2605L init OK.");
    Serial.print("DRV2605L I2C pins: SDA=");
    Serial.print(I2C_SDA);
    Serial.print(" SCL=");
    Serial.println(I2C_SCL);
    Serial.println("Button cycles intensity: soft/medium/hard/max");

    // Startup vibration sweep (soft -> max)
    for (uint8_t i = 0; i < 4; i++) {
      intensityMode = i;
      updateIntensityDisplay();
      drv.setRealtimeValue(motorIntensityForStep(0));
      delay(150);
      drv.setRealtimeValue(0);
      delay(100);
    }
    intensityMode = 0;
    updateIntensityDisplay();
  } else {
    Serial.println("DRV2605L init FAILED.");
    Serial.println("Check wiring + power. DRV2605L addr is 0x5A or 0x5B.");
  }

  updateIntensityDisplay();
  updatePingRateDisplay();
  updateFeedbackDisplay();
  updateUiSelectionDisplay();

  if (drvOk && vibeTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(
      vibrationTask,
      "vibrationTask",
      4096,
      nullptr,
      2,
      &vibeTaskHandle,
      0
    );
  }
}

void loop() {
  // Handle received packets
  if (loraRxFlag) {
    loraInterruptEnabled = false;
    loraRxFlag = false;

    String payload;
    int state = lora.readData(payload);
    if (state == RADIOLIB_ERR_NONE) {
      lastRxMs = millis();
      lastRssi = (int16_t)lora.getRSSI();
      lastSnr = lora.getSNR();

      Serial.print("RSSI=");
      Serial.print(lastRssi);
      Serial.print(" dBm SNR=");
      Serial.print(lastSnr);
      Serial.print(" dB");

      // Smooth RSSI to reduce jitter
      smoothedRssi = RSSI_SMOOTH_ALPHA * lastRssi + (1.0f - RSSI_SMOOTH_ALPHA) * smoothedRssi;
      int step = estimateDistanceStep((int16_t)round(smoothedRssi));
      Serial.print(" | Step ");
      Serial.print(step);
      Serial.print("/19 (");
      Serial.print(distanceLabel(step));
      Serial.println(")");

      // Track latest step for vibration rate control
      lastStep = step;

      // No TFT updates on every RX to reduce jitter

      uint32_t otherId = 0;
      if (parsePeerId(payload, otherId)) {
        updateRoleWithPeer(otherId);
      }

      if (payload.startsWith("P")) {
        // Respond to ping
        String reply = buildReply();
        lora.transmit(reply);
      }

      flashStatusLed();

      // Pulse mode: short haptic on any reception
      if (!feedbackConstant) {
        requestHapticStep = step;
        requestHapticPulse = true;
      }
    }

    lora.startReceive();
    loraInterruptEnabled = true;
  }

  // Vibration is handled by a dedicated task

  // Handle user button
  bool btnPressed = digitalRead(USER_BTN_PIN) == (USER_BTN_ACTIVE_LOW ? LOW : HIGH);

  if (btnPressed && !lastBtnState && (millis() - lastBtnMs > BTN_DEBOUNCE_MS)) {
    // Button pressed
    lastBtnMs = millis();
    btnPressStartMs = millis();
    btnHoldHandled = false;
  }

  if (btnPressed && !btnHoldHandled && (millis() - btnPressStartMs > BTN_HOLD_MS)) {
    // Long-press: cycle which setting is being edited
    btnHoldHandled = true;
    uiSetting = (uiSetting + 1) % 3;

    Serial.print("Edit setting: ");
    if (uiSetting == UI_VIBE) Serial.println("vibe");
    else if (uiSetting == UI_PING) Serial.println("ping");
    else Serial.println("feedback");
    updateUiSelectionDisplay();
    requestHapticStep = 0;
    requestHapticPulse = true;
  }

  if (!btnPressed && lastBtnState && !btnHoldHandled) {
    // Short press: adjust the active setting
    if (uiSetting == UI_VIBE) {
      intensityMode = (intensityMode + 1) % 4;
      const char* label = (intensityMode == 0) ? "soft" :
                          (intensityMode == 1) ? "medium" :
                          (intensityMode == 2) ? "hard" : "max";
      Serial.print("Intensity mode: ");
      Serial.println(label);
      updateIntensityDisplay();
    } else if (uiSetting == UI_PING) {
      pingRateIndex = (pingRateIndex + 1) % (sizeof(PING_RATES_MS) / sizeof(PING_RATES_MS[0]));
      pingBaseMs = PING_RATES_MS[pingRateIndex];
      pingJitterMs = max<uint32_t>(10, pingBaseMs / 4);
      Serial.print("Ping rate: ");
      Serial.print(pingBaseMs);
      Serial.println(" ms");
      updatePingRateDisplay();
    } else {
      feedbackConstant = !feedbackConstant;
      Serial.print("Feedback mode: ");
      Serial.println(feedbackConstant ? "constant" : "pulse");
      updateFeedbackDisplay();
    }

    // Haptic feedback on mode change
    requestHapticStep = 0;
    requestHapticPulse = true;
  }

  if (!btnPressed) {
    btnPressStartMs = 0;
  }
  lastBtnState = btnPressed;

  // Send ping with jitter to reduce collisions
  uint32_t now = millis();

  // Warn if we're late sending a ping (only when we are supposed to ping)
  if (role != ROLE_RESPONDER && nextPingMs != 0 && now > nextPingMs + PING_LATE_WARN_MS) {
    Serial.print("Ping late by ");
    Serial.print(now - nextPingMs);
    Serial.println(" ms");
    // avoid spamming: move the schedule forward
    nextPingMs = now + pingBaseMs + random(0, pingJitterMs);
  }

  // Role timeout: return to search if peer disappears
  if (role != ROLE_SEARCH && (now - lastPeerSeenMs > PEER_TIMEOUT_MS)) {
    role = ROLE_SEARCH;
    peerId = 0;
    Serial.println("Role: SEARCH");
  }

  if (now >= nextPingMs) {
    // In SEARCH mode, avoid pinging right after RX to reduce collisions
    if (role == ROLE_SEARCH && now - lastRxMs < 400) {
      nextPingMs = now + pingBaseMs + random(0, pingJitterMs);
    } else if (role == ROLE_SEARCH || role == ROLE_PINGER) {
      pingCounter++;

      String msg = buildPing();

      loraInterruptEnabled = false;
      uint32_t txStartMs = millis();
      lora.transmit(msg);
      uint32_t txDurMs = millis() - txStartMs;
      lora.startReceive();
      loraInterruptEnabled = true;

      // No TFT updates on every TX to reduce jitter
      flashStatusLed();

      if (lastPingSentMs != 0 && (pingCounter % pingIntervalLogEvery == 0)) {
        Serial.print("Ping interval: ");
        Serial.print(now - lastPingSentMs);
        Serial.println(" ms");
      }
      if (txDurMs > TX_WARN_MS) {
        Serial.print("TX duration: ");
        Serial.print(txDurMs);
        Serial.println(" ms");
      }
      lastPingSentMs = now;
      nextPingMs = now + pingBaseMs + random(0, pingJitterMs);
    }
  }

  // RX watchdog: recover if RX stalls
  if (millis() - lastRxMs > RX_WATCHDOG_MS) {
    lora.startReceive();
    lastRxMs = millis();
  }

  // LED pulse off

  if (LED_ENABLED && ledOn && millis() > ledOffMs) {
    digitalWrite(STATUS_LED_PIN, LOW);
    ledOn = false;
  }
}