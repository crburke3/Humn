#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
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
static const uint32_t DISPLAY_IDLE_MS = 15000;

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
static const uint32_t BTN_POWER_HOLD_MS = 3500;

enum UiSetting : uint8_t {
  UI_VIBE = 0,
  UI_PING = 1,
  UI_FEEDBACK = 2,
  UI_SIM = 3,
  UI_SCALE = 4
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
static uint32_t rxLogEvery = 10;
static uint32_t rxCounter = 0;
static int16_t lastRssi = 0;
static float lastSnr = 0.0f;
static uint32_t motorStopMs = 0;
static uint32_t hapticPreviewUntilMs = 0;
static bool drvOk = false;
static uint8_t intensityMode = 1; // 0=soft,1=medium,2=hard,3=max
static uint32_t lastBtnMs = 0;
static bool lastBtnState = false;
static uint32_t btnPressStartMs = 0;
static bool btnHoldHandled = false;
static bool btnPowerHandled = false;
static uint32_t ledOffMs = 0;
static bool ledOn = false;
static uint8_t uiSetting = UI_VIBE;
enum FeedbackMode : uint8_t { FEEDBACK_PULSED_RATE = 0, FEEDBACK_PULSE_ON_RX = 1, FEEDBACK_CONSTANT = 2 };
static FeedbackMode feedbackMode = FEEDBACK_PULSED_RATE;
static TaskHandle_t vibeTaskHandle = nullptr;
static volatile bool requestHapticPulse = false;
static volatile int requestHapticStep = 0;
static uint32_t lastRxMs = 0;
static uint32_t lastSignalMs = 0;
static const uint32_t PING_RATES_MS[] = {1000, 800, 600, 400, 300, 200, 100, 50};
static uint8_t pingRateIndex = 0;
static uint32_t pingBaseMs = 100;
static uint32_t pingJitterMs = 100;
static const uint32_t PING_LATE_WARN_MS = 50;

static const uint32_t TX_WARN_MS = 80;
static const uint32_t PEER_TIMEOUT_MS = 3000;

enum Role : uint8_t { ROLE_SEARCH = 0, ROLE_PINGER = 1, ROLE_RESPONDER = 2 };
static Role role = ROLE_SEARCH;
static uint32_t deviceId = 0;
static uint32_t peerId = 0;
static uint32_t lastPeerSeenMs = 0;
static Preferences prefs;
static bool simulateDistance = false;
static uint8_t distanceScaleIndex = 10; // 10 = 1.0x

void saveSettings() {
  prefs.putUChar("intensity", intensityMode);
  prefs.putUChar("pingIdx", pingRateIndex);
  prefs.putUChar("feedback", (uint8_t)feedbackMode);
  prefs.putUChar("ui", uiSetting);
  prefs.putBool("sim", simulateDistance);
  prefs.putUChar("scale", distanceScaleIndex);
}

void loadSettings() {
  intensityMode = prefs.getUChar("intensity", 1);
  pingRateIndex = prefs.getUChar("pingIdx", 6);
  feedbackMode = (FeedbackMode)prefs.getUChar("feedback", (uint8_t)FEEDBACK_PULSED_RATE);
  uiSetting = prefs.getUChar("ui", (uint8_t)UI_VIBE);
  simulateDistance = prefs.getBool("sim", false);
  distanceScaleIndex = prefs.getUChar("scale", 10);
  if (distanceScaleIndex < 10) distanceScaleIndex = 10;
  if (distanceScaleIndex > 30) distanceScaleIndex = 30;

  if (pingRateIndex >= (sizeof(PING_RATES_MS) / sizeof(PING_RATES_MS[0]))) {
    pingRateIndex = 0;
  }
  pingBaseMs = PING_RATES_MS[pingRateIndex];
  pingJitterMs = max<uint32_t>(10, pingBaseMs / 4);
}
static const uint32_t VIBE_TIMEOUT_MS = 3000;
static uint32_t nextVibeToggleMs = 0;
static int lastStep = 19;
static float smoothedRssi = -100.0f;
static const float RSSI_SMOOTH_ALPHA = 0.10f;
static bool vibeOn = false;
static float smoothedIntervalMs = 600.0f;
static const float INTERVAL_SMOOTH_ALPHA = 0.20f;

// Watchdog: re-enter RX mode if we go quiet
static const uint32_t RX_WATCHDOG_MS = 3000;

static int lastDisplayStep = -1;
static uint32_t lastSimUpdateMs = 0;
static const uint32_t SIM_UPDATE_MS = 500;
static bool displayOn = true;
static uint32_t lastUserInteractionMs = 0;
static bool deviceOn = true;
static const uint16_t HUMN_LOGO_W = 120;
static const uint16_t HUMN_LOGO_H = 40;
static uint8_t humnLogoBitmap[HUMN_LOGO_W * HUMN_LOGO_H / 8];
static bool humnLogoBuilt = false;


// Simulation mode: cycles distance without RF (user-toggle)
static const uint32_t SIM_PHASE_MS = 10000;
static const uint32_t SIM_CYCLE_MS = SIM_PHASE_MS * 3;

void setLoraFlag() {
  if (!loraInterruptEnabled) {
    return;
  }
  loraRxFlag = true;
}

int applyDistanceScale(int step) {
  float scale = distanceScaleIndex / 10.0f;
  int scaled = (int)round(step * scale);
  if (scaled < 0) scaled = 0;
  if (scaled > 19) scaled = 19;
  return scaled;
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
  return applyDistanceScale(step);
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
  tft.fillRect(0, 0, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 0);
  tft.print("Vibe: ");
  tft.print(intensityLabel(intensityMode));
}

void updatePingRateDisplay() {
  tft.fillRect(0, 12, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 12);
  tft.print("Ping: ");
  tft.print(pingBaseMs);
  tft.print("ms");
}

void updateFeedbackDisplay() {
  tft.fillRect(0, 24, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 24);
  tft.print("Mode: ");
  if (feedbackMode == FEEDBACK_PULSED_RATE) {
    tft.print("pulsed");
  } else if (feedbackMode == FEEDBACK_PULSE_ON_RX) {
    tft.print("rx-pulse");
  } else {
    tft.print("constant");
  }
}

void updateUiSelectionDisplay() {
  tft.fillRect(0, 72, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 72);
  tft.print("Edit: ");
  if (uiSetting == UI_VIBE) tft.print("vibe");
  else if (uiSetting == UI_PING) tft.print("ping");
  else if (uiSetting == UI_FEEDBACK) tft.print("feedback");
  else if (uiSetting == UI_SIM) tft.print("sim");
  else tft.print("scale");
}

void updateSimDisplay() {
  tft.fillRect(0, 48, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 48);
  tft.print("Sim: ");
  tft.print(simulateDistance ? "on" : "off");
}

void updateScaleDisplay() {
  tft.fillRect(0, 60, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 60);
  tft.print("Scale: ");
  tft.print(distanceScaleIndex / 10.0f, 1);
  tft.print("x");
}

enum DistanceBand : uint8_t {
  BAND_NONE = 0,
  BAND_FAR = 1,
  BAND_MED = 2,
  BAND_CLOSE = 3
};

static DistanceBand lastBand = BAND_NONE;

const char* distanceCategoryLabel(DistanceBand band) {
  switch (band) {
    case BAND_NONE: return "none";
    case BAND_FAR: return "far";
    case BAND_MED: return "medium";
    default: return "close";
  }
}

DistanceBand distanceBandFromStep(int step) {
  if (step >= 18) return BAND_NONE;
  if (step >= 13) return BAND_FAR;
  if (step >= 7) return BAND_MED;
  return BAND_CLOSE;
}

void updateDistanceDisplay(int step) {
  // Add hysteresis by only updating when the band changes
  DistanceBand band = distanceBandFromStep(step);
  if (band == lastBand) {
    return;
  }
  lastBand = band;
  tft.fillRect(0, 36, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 36);
  tft.print("Dist: ");
  tft.print(distanceCategoryLabel(band));
}

void setBitmapPixel(uint8_t* bitmap, uint16_t w, uint16_t x, uint16_t y) {
  uint32_t index = x + (uint32_t)y * w;
  uint32_t byteIndex = index >> 3;
  uint8_t bit = 7 - (index & 0x7);
  bitmap[byteIndex] |= (1 << bit);
}

void buildHumnLogoBitmap() {
  if (humnLogoBuilt) {
    return;
  }
  memset(humnLogoBitmap, 0, sizeof(humnLogoBitmap));

  const int letterW = 22;
  const int letterH = 24;
  const int gap = 6;
  const int thick = 3;
  const int startX = (HUMN_LOGO_W - (letterW * 4 + gap * 3)) / 2;
  const int startY = (HUMN_LOGO_H - letterH) / 2;

  auto plot = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= (int)HUMN_LOGO_W || y >= (int)HUMN_LOGO_H) return;
    setBitmapPixel(humnLogoBitmap, HUMN_LOGO_W, (uint16_t)x, (uint16_t)y);
  };

  auto drawRect = [&](int x, int y, int w, int h) {
    for (int yy = y; yy < y + h; yy++) {
      for (int xx = x; xx < x + w; xx++) {
        plot(xx, yy);
      }
    }
  };

  auto drawH = [&](int x) {
    drawRect(x, startY, thick, letterH);
    drawRect(x + letterW - thick, startY, thick, letterH);
    drawRect(x, startY + letterH / 2 - 1, letterW, thick);
  };

  auto drawU = [&](int x) {
    drawRect(x, startY, thick, letterH - thick);
    drawRect(x + letterW - thick, startY, thick, letterH - thick);
    drawRect(x, startY + letterH - thick, letterW, thick);
  };

  auto drawM = [&](int x) {
    drawRect(x, startY, thick, letterH);
    drawRect(x + letterW - thick, startY, thick, letterH);
    for (int y = 0; y < letterH; y++) {
      int dx = (letterW - 2 * thick - 2) * y / (letterH - 1);
      for (int t = 0; t < thick; t++) {
        plot(x + thick + dx + t, startY + y);
        plot(x + letterW - thick - 1 - dx - t, startY + y);
      }
    }
  };

  auto drawN = [&](int x) {
    drawRect(x, startY, thick, letterH);
    drawRect(x + letterW - thick, startY, thick, letterH);
    for (int y = 0; y < letterH; y++) {
      int dx = (letterW - thick - 1) * y / (letterH - 1);
      for (int t = 0; t < thick; t++) {
        plot(x + t + dx, startY + y);
      }
    }
  };

  int x0 = startX;
  drawH(x0);
  x0 += letterW + gap;
  drawU(x0);
  x0 += letterW + gap;
  drawM(x0);
  x0 += letterW + gap;
  drawN(x0);

  humnLogoBuilt = true;
}

void showHumnLogo() {
  buildHumnLogoBitmap();
  tft.fillScreen(ST77XX_BLACK);
  int x = (160 - HUMN_LOGO_W) / 2;
  int y = (80 - HUMN_LOGO_H) / 2;
  tft.drawBitmap(x, y, humnLogoBitmap, HUMN_LOGO_W, HUMN_LOGO_H, ST77XX_WHITE);
}

void showSleepCat() {
  tft.fillScreen(ST77XX_BLACK);
  // Bigger sleeping cat using simple shapes
  const int cx = 60;
  const int cy = 44;
  const int headR = 14;

  // Body
  tft.fillRoundRect(cx - 18, cy + 6, 60, 22, 10, ST77XX_WHITE);
  tft.fillCircle(cx + 40, cy + 16, 10, ST77XX_WHITE);

  // Head
  tft.fillCircle(cx, cy, headR, ST77XX_WHITE);
  tft.fillTriangle(cx - 12, cy - 8, cx - 2, cy - 22, cx - 2, cy - 6, ST77XX_WHITE);
  tft.fillTriangle(cx + 12, cy - 8, cx + 2, cy - 22, cx + 2, cy - 6, ST77XX_WHITE);

  // Face details (eyes closed + mouth)
  tft.drawLine(cx - 6, cy - 2, cx - 1, cy - 2, ST77XX_BLACK);
  tft.drawLine(cx + 1, cy - 2, cx + 6, cy - 2, ST77XX_BLACK);
  tft.drawLine(cx - 1, cy + 2, cx + 1, cy + 2, ST77XX_BLACK);

  // Tail curl
  tft.drawCircle(cx + 50, cy + 18, 10, ST77XX_BLACK);
  tft.drawCircle(cx + 50, cy + 18, 8, ST77XX_BLACK);

  // "Zz"
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(10, 8);
  tft.print("Z");
  tft.setCursor(20, 4);
  tft.print("z");
}

void refreshDisplay() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);

  updateIntensityDisplay();
  updatePingRateDisplay();
  updateFeedbackDisplay();
  updateUiSelectionDisplay();
  updateSimDisplay();
  updateScaleDisplay();

  lastBand = (DistanceBand)255;
  updateDistanceDisplay(lastStep);
}

void setDisplayOn(bool on) {
  if (displayOn == on) {
    return;
  }
  displayOn = on;
  if (on) {
    // Clear display before backlight to avoid stale flash
    tft.fillScreen(ST77XX_BLACK);
    digitalWrite(TFT_LED_K, HIGH);
    refreshDisplay();
  } else {
    digitalWrite(TFT_LED_K, LOW);
  }
}
void flashStatusLed() {
  if (!LED_ENABLED) {
    return;
  }
  digitalWrite(STATUS_LED_PIN, HIGH);
  ledOn = true;
  ledOffMs = millis() + LED_PULSE_MS;
}

void updateSimulatedDistance() {
  uint32_t now = millis();
  if (now - lastSimUpdateMs < SIM_UPDATE_MS) {
    return;
  }
  lastSimUpdateMs = now;
  uint32_t t = now % SIM_CYCLE_MS;

  if (t < SIM_PHASE_MS) {
    // Phase 1: no node nearby
    lastSignalMs = now - (VIBE_TIMEOUT_MS + 1000);
  } else if (t < SIM_PHASE_MS * 2) {
    // Phase 2: getting closer (far -> close)
    uint32_t p = t - SIM_PHASE_MS;
    float frac = (float)p / (float)SIM_PHASE_MS;
    int step = (int)round(19.0f * (1.0f - frac));
    if (step < 0) step = 0;
    if (step > 19) step = 19;
    lastStep = applyDistanceScale(step);
    lastSignalMs = now;
  } else {
    // Phase 3: getting farther (close -> far)
    uint32_t p = t - (SIM_PHASE_MS * 2);
    float frac = (float)p / (float)SIM_PHASE_MS;
    int step = (int)round(19.0f * frac);
    if (step < 0) step = 0;
    if (step > 19) step = 19;
    lastStep = applyDistanceScale(step);
    lastSignalMs = now;
  }
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

      if (!deviceOn) {
        if (motorStopMs != 0 || vibeOn) {
          drv.setRealtimeValue(0);
          motorStopMs = 0;
          vibeOn = false;
        }
        vTaskDelay(delayTicks);
        continue;
      }

      // Handle short haptic pulse requests (e.g. button feedback)
      if (requestHapticPulse) {
        requestHapticPulse = false;
        motorPulseForStep(requestHapticStep);
        hapticPreviewUntilMs = now + MOTOR_PULSE_MS + 20;
      }

      // Stop motor after pulse duration
      if (motorStopMs != 0 && now > motorStopMs) {
        drv.setRealtimeValue(0);
        motorStopMs = 0;
      }

      // During preview pulse, skip other vibration modes
      if (hapticPreviewUntilMs != 0) {
        if (now < hapticPreviewUntilMs) {
          vTaskDelay(delayTicks);
          continue;
        }
        hapticPreviewUntilMs = 0;
      }

      // Vibration rate control: only vibrate if we received recently
      if (now - lastSignalMs > VIBE_TIMEOUT_MS) {
        if (motorStopMs != 0 || vibeOn) {
          drv.setRealtimeValue(0);
          motorStopMs = 0;
          vibeOn = false;
        }
        nextVibeToggleMs = now + 200;
      } else if (feedbackMode == FEEDBACK_PULSED_RATE) {
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
      } else if (feedbackMode == FEEDBACK_CONSTANT) {
        // Constant vibration intensity (no pulsing)
        uint8_t intensity = motorIntensityWithMode(lastStep);
        drv.setRealtimeValue(intensity);
        vibeOn = true;
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
  
  // Short delay for serial monitor connection
  delay(300);

  Serial.println("\n\n========================================");
  Serial.println("  Wireless Tracker - TFT + LoRa");
  Serial.println("========================================\n");

  // Seed RNG for ping jitter
  randomSeed((uint32_t)ESP.getEfuseMac());
  uint64_t mac = ESP.getEfuseMac();
  deviceId = (uint32_t)(mac ^ (mac >> 32));
  Serial.print("Device ID: ");
  Serial.println(deviceId, HEX);

  prefs.begin("humn", false);
  loadSettings();

  // Enable Vext (powers TFT + GNSS)
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, VEXT_ACTIVE);
  delay(100);

  // Enable TFT backlight (polarity may vary by board)
  pinMode(TFT_LED_K, OUTPUT);
  digitalWrite(TFT_LED_K, LOW);
  displayOn = false;
  lastUserInteractionMs = millis();
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

  // Draw boot logo
  setDisplayOn(true);
  showHumnLogo();
  delay(1000);
  refreshDisplay();

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
  } else {
    Serial.print("LoRa init failed: ");
    Serial.println(state);
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

    // Startup vibration sweep (low -> high) ~0.5s total
    const uint32_t sweepMs = 500;
    const uint8_t steps = 20;
    for (uint8_t i = 0; i < steps; i++) {
      float t = (steps <= 1) ? 1.0f : (float)i / (float)(steps - 1);
      uint8_t intensity = (uint8_t)round(20.0f + t * (127.0f - 20.0f));
      drv.setRealtimeValue(intensity);
      delay(sweepMs / steps);
    }
    drv.setRealtimeValue(0);
  } else {
    Serial.println("DRV2605L init FAILED.");
    Serial.println("Check wiring + power. DRV2605L addr is 0x5A or 0x5B.");
  }

  updateSimDisplay();

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
  // Optional distance simulation (disables RF-based distance)
  if (simulateDistance) {
    updateSimulatedDistance();
    updateDistanceDisplay(lastStep);
  }

  // Handle received packets
  if (!simulateDistance && loraRxFlag) {
    loraInterruptEnabled = false;
    loraRxFlag = false;

    String payload;
    int state = lora.readData(payload);
    if (state == RADIOLIB_ERR_NONE) {
      lastRxMs = millis();
      lastSignalMs = lastRxMs;
      lastRssi = (int16_t)lora.getRSSI();
      lastSnr = lora.getSNR();

      // Smooth RSSI to reduce jitter
      smoothedRssi = RSSI_SMOOTH_ALPHA * lastRssi + (1.0f - RSSI_SMOOTH_ALPHA) * smoothedRssi;
      int step = estimateDistanceStep((int16_t)round(smoothedRssi));
      rxCounter++;
      if (rxCounter % rxLogEvery == 0) {
        Serial.print("RSSI=");
        Serial.print(lastRssi);
        Serial.print(" dBm SNR=");
        Serial.print(lastSnr);
        Serial.print(" dB | Step ");
        Serial.print(step);
        Serial.print("/19 (");
        Serial.print(distanceLabel(step));
        Serial.println(")");
      }

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

      // RX-pulse mode: short haptic on any reception
      if (feedbackMode == FEEDBACK_PULSE_ON_RX) {
        requestHapticStep = step;
        requestHapticPulse = true;
      }

      updateDistanceDisplay(step);
    }

    lora.startReceive();
    loraInterruptEnabled = true;
  }

  // If simulating, ignore RF updates (clear flag if set)
  if (simulateDistance && loraRxFlag) {
    loraRxFlag = false;
    lora.startReceive();
  }

  // Vibration is handled by a dedicated task

  // Handle user button
  bool btnPressed = digitalRead(USER_BTN_PIN) == (USER_BTN_ACTIVE_LOW ? LOW : HIGH);

  uint32_t nowMs = millis();
  bool pressedEdge = btnPressed && !lastBtnState && (nowMs - lastBtnMs > BTN_DEBOUNCE_MS);
  if (pressedEdge) {
    lastBtnMs = nowMs;
    btnPressStartMs = nowMs;
    btnHoldHandled = false;
    btnPowerHandled = false;
  }

  if (btnPressed && btnPressStartMs != 0) {
    uint32_t heldMs = nowMs - btnPressStartMs;

    if (heldMs >= BTN_POWER_HOLD_MS && !btnPowerHandled) {
      deviceOn = !deviceOn;
      btnPowerHandled = true;
      if (deviceOn) {
        setDisplayOn(true);
        showHumnLogo();
        delay(1000);
        refreshDisplay();
      } else {
        if (!displayOn) {
          setDisplayOn(true);
        }
        showSleepCat();
        delay(1000);
        setDisplayOn(false);
      }
      lastUserInteractionMs = nowMs;
      if (!deviceOn && drvOk) {
        drv.setRealtimeValue(0);
        motorStopMs = 0;
        vibeOn = false;
      }
      // Power action triggers immediately; ignore release for this hold.
      btnPressStartMs = 0;
    } else if (deviceOn && displayOn && heldMs >= BTN_HOLD_MS && !btnHoldHandled) {
      // Long-press: cycle which setting is being edited
      btnHoldHandled = true;
      uiSetting = (uiSetting + 1) % 5;

      Serial.print("Edit setting: ");
      if (uiSetting == UI_VIBE) Serial.println("vibe");
      else if (uiSetting == UI_PING) Serial.println("ping");
      else if (uiSetting == UI_FEEDBACK) Serial.println("feedback");
      else if (uiSetting == UI_SIM) Serial.println("sim");
      else Serial.println("scale");
      updateUiSelectionDisplay();
      saveSettings();
      requestHapticStep = 0;
      requestHapticPulse = true;
      lastUserInteractionMs = nowMs;
    }
  }

  bool releasedEdge = !btnPressed && lastBtnState;
  if (releasedEdge && btnPressStartMs != 0) {
    uint32_t heldMs = nowMs - btnPressStartMs;
    btnPressStartMs = 0;

    if (!deviceOn) {
      // Ignore on release if device is off
    } else if (!displayOn) {
      setDisplayOn(true);
      lastUserInteractionMs = nowMs;
    } else if (heldMs < BTN_HOLD_MS && !btnPowerHandled) {
      // Short press: adjust the active setting
      if (uiSetting == UI_VIBE) {
        intensityMode = (intensityMode + 1) % 4;
        const char* label = (intensityMode == 0) ? "soft" :
                            (intensityMode == 1) ? "medium" :
                            (intensityMode == 2) ? "hard" : "max";
        Serial.print("Intensity mode: ");
        Serial.println(label);
        updateIntensityDisplay();
        saveSettings();
      } else if (uiSetting == UI_PING) {
        pingRateIndex = (pingRateIndex + 1) % (sizeof(PING_RATES_MS) / sizeof(PING_RATES_MS[0]));
        pingBaseMs = PING_RATES_MS[pingRateIndex];
        pingJitterMs = max<uint32_t>(10, pingBaseMs / 4);
        Serial.print("Ping rate: ");
        Serial.print(pingBaseMs);
        Serial.println(" ms");
        updatePingRateDisplay();
        saveSettings();
      } else if (uiSetting == UI_FEEDBACK) {
        feedbackMode = (FeedbackMode)((feedbackMode + 1) % 3);
        Serial.print("Feedback mode: ");
        if (feedbackMode == FEEDBACK_PULSED_RATE) Serial.println("pulsed");
        else if (feedbackMode == FEEDBACK_PULSE_ON_RX) Serial.println("rx-pulse");
        else Serial.println("constant");
        updateFeedbackDisplay();
        saveSettings();
      } else if (uiSetting == UI_SIM) {
        simulateDistance = !simulateDistance;
        Serial.print("Sim mode: ");
        Serial.println(simulateDistance ? "on" : "off");

        updateSimDisplay();
        saveSettings();
      } else {
        // Distance scale: 1.0x .. 3.0x in 0.1 steps
        distanceScaleIndex++;
        if (distanceScaleIndex > 30) distanceScaleIndex = 10;
        Serial.print("Scale: ");
        Serial.print(distanceScaleIndex / 10.0f, 1);
        Serial.println("x");
        updateScaleDisplay();
        saveSettings();
      }

      // Haptic feedback on mode change
      requestHapticStep = 0;
      requestHapticPulse = true;
      lastUserInteractionMs = nowMs;
    }
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

  // Signal timeout: return to search + clear distance display
  if (!simulateDistance && (now - lastSignalMs > PEER_TIMEOUT_MS)) {
    if (role != ROLE_SEARCH) {
      role = ROLE_SEARCH;
      peerId = 0;
      Serial.println("Role: SEARCH");
    }
    lastStep = 19;
    updateDistanceDisplay(lastStep);
  }

  if (now >= nextPingMs) {
    // In SEARCH mode, avoid pinging right after RX to reduce collisions
    if (role == ROLE_SEARCH && now - lastSignalMs < 400) {
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

  // Display idle timeout
  if (displayOn && (millis() - lastUserInteractionMs > DISPLAY_IDLE_MS)) {
    setDisplayOn(false);
  }

  // LED pulse off

  if (LED_ENABLED && ledOn && millis() > ledOffMs) {
    digitalWrite(STATUS_LED_PIN, LOW);
    ledOn = false;
  }
}