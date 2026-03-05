#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_DRV2605.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_log.h>
#include <WiFi.h>

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

// DRV2605L haptic driver (I2C) - try primary first, then alternate if pins 4/5 are damaged
static const int I2C_SDA_PRIMARY   = 4;
static const int I2C_SCL_PRIMARY   = 5;
static const int I2C_SDA_ALTERNATE = 16;
static const int I2C_SCL_ALTERNATE = 17;
static const uint16_t MOTOR_PULSE_MS = 200;
static const int GNSS_RST_PIN = 35; // GPIO35 = GNSS_RST

// User button (GPIO0) cycles vibration intensity
static const int USER_BTN_PIN = 0;
static const bool USER_BTN_ACTIVE_LOW = true;
static const uint32_t BTN_DEBOUNCE_MS = 200;
static const uint32_t BTN_HOLD_MS = 700;
static const uint32_t BTN_POWER_HOLD_MS = 5000;
static const int ADC_BAT_PIN = 1;   // GPIO1 = Vbat_Read
static const int ADC_CTRL_PIN = 2;  // GPIO2 = ADC_Ctrl
static const float VBAT_DIVIDER = 4.9f;

enum UiSetting : uint8_t {
  UI_VIBE = 0,
  UI_PING = 1,
  UI_FEEDBACK = 2,
  UI_SIM = 3,
  UI_SCALE = 4
};

void updateBatteryDisplay(bool force = false);
void setDisplayOn(bool on, bool refresh = true);

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

// Power saving with CAD (Channel Activity Detection)
static const uint32_t CAD_CHECK_INTERVAL_MS = 100;  // How often to check for signals in light sleep
static const uint32_t IDLE_LIGHT_SLEEP_MS = 30000;  // Enter light sleep after this idle time
static const uint32_t IDLE_DEEP_SLEEP_MS = 300000;  // Enter deep sleep after 5 min idle
static bool powerSaveMode = true;  // Enable CAD-based power saving

// Compile-time switches:
// - Auto sleep controls: display idle-off + ESP32 light sleep/CAD while idle
// - Manual deep sleep: 5s button hold triggers a true deep sleep reboot
#if defined(HUMN_DISABLE_AUTO_SLEEP) && (HUMN_DISABLE_AUTO_SLEEP)
static const bool autoSleepEnabled = false;
#else
static const bool autoSleepEnabled = true;
#endif

#if defined(HUMN_DISABLE_MANUAL_DEEP_SLEEP) && (HUMN_DISABLE_MANUAL_DEEP_SLEEP)
static const bool manualDeepSleepEnabled = false;
#else
static const bool manualDeepSleepEnabled = true;
#endif

SPIClass spiLoRa(FSPI);
SX1262 lora = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY, spiLoRa);

volatile bool loraRxFlag = false;
volatile bool loraInterruptEnabled = true;
volatile bool cadDetectedFlag = false;

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
static volatile bool hapticsEnabled = true;
static uint32_t lastRxMs = 0;
static uint32_t lastSignalMs = 0;
static const uint32_t PING_RATES_MS[] = {1000, 800, 600, 400, 300, 200, 100, 50};
static uint8_t pingRateIndex = 0;
static uint32_t pingBaseMs = 100;
static uint32_t pingJitterMs = 100;
static const uint32_t PING_LATE_WARN_MS = 50;
static uint32_t lastPingLateLogMs = 0;
static const uint32_t PING_LATE_LOG_THROTTLE_MS = 2000;
static const uint32_t SEARCH_SLEEP_PING_MS = 2000;
// Serial logging can trip the INT_WDT on ESP32-S3 with USB CDC under load.
// Keep these off unless actively debugging.
static bool verboseTimingLogs = false;
static bool verboseUiLogs = false;

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
  // Ping fixed at 50ms (setting temporarily disabled)
  pingRateIndex = 7;  // 50ms in PING_RATES_MS
  pingBaseMs = 50;
  pingJitterMs = max<uint32_t>(10, pingBaseMs / 4);

  feedbackMode = (FeedbackMode)prefs.getUChar("feedback", (uint8_t)FEEDBACK_PULSED_RATE);
  uiSetting = prefs.getUChar("ui", (uint8_t)UI_VIBE);
  if (uiSetting == UI_PING) uiSetting = UI_VIBE;  // Ping hidden; fixed at 50ms
  simulateDistance = prefs.getBool("sim", false);
  distanceScaleIndex = prefs.getUChar("scale", 10);
  if (distanceScaleIndex < 10) distanceScaleIndex = 10;
  if (distanceScaleIndex > 30) distanceScaleIndex = 30;
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
// Light sleep mode = display turned off by the idle timeout (not by other causes).
// In this mode we may enter ESP32 light sleep with LoRa CAD, and waking should NOT
// show the boot logo or boot vibration (because we didn't reboot).
static bool lightSleepMode = false;
static uint32_t lastUserInteractionMs = 0;
static bool deviceOn = true;
static uint32_t lastBatteryReadMs = 0;
static int lastBatteryPct = -1;

void enterDeepSleep();
void playBootVibration();
void requireWakeHoldIfNeeded();
static const uint16_t HUMN_LOGO_W = 120;
static const uint16_t HUMN_LOGO_H = 40;
static uint8_t humnLogoBitmap[HUMN_LOGO_W * HUMN_LOGO_H / 8];
static bool humnLogoBuilt = false;


// Simulation mode: cycles distance without RF (user-toggle)
static const uint32_t SIM_PHASE_MS = 10000;
static const uint32_t SIM_CYCLE_MS = SIM_PHASE_MS * 3;

void IRAM_ATTR setLoraFlag() {
  if (!loraInterruptEnabled) {
    return;
  }
  loraRxFlag = true;
}

void IRAM_ATTR setCADFlag() {
  cadDetectedFlag = true;
}

static void printBootReason() {
  esp_reset_reason_t rr = esp_reset_reason();
  Serial.print("Reset reason: ");
  switch (rr) {
    case ESP_RST_POWERON: Serial.println("POWERON"); break;
    case ESP_RST_EXT: Serial.println("EXT"); break;
    case ESP_RST_SW: Serial.println("SW"); break;
    case ESP_RST_PANIC: Serial.println("PANIC"); break;
    case ESP_RST_INT_WDT: Serial.println("INT_WDT"); break;
    case ESP_RST_TASK_WDT: Serial.println("TASK_WDT"); break;
    case ESP_RST_WDT: Serial.println("WDT"); break;
    case ESP_RST_BROWNOUT: Serial.println("BROWNOUT"); break;
    case ESP_RST_SDIO: Serial.println("SDIO"); break;
    default: Serial.println((int)rr); break;
  }

  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  Serial.print("Wakeup cause: ");
  Serial.println((int)wc);
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
  updateBatteryDisplay(true);
}

void updateFeedbackDisplay() {
  tft.fillRect(0, 12, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 12);
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
  tft.fillRect(0, 60, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 60);
  tft.print("Edit: ");
  if (uiSetting == UI_VIBE) tft.print("vibe");
  else if (uiSetting == UI_FEEDBACK) tft.print("feedback");
  else if (uiSetting == UI_SIM) tft.print("sim");
  else tft.print("scale");
}

void updateSimDisplay() {
  tft.fillRect(0, 36, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 36);
  tft.print("Sim: ");
  tft.print(simulateDistance ? "on" : "off");
}

void updateScaleDisplay() {
  tft.fillRect(0, 48, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 48);
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
  tft.fillRect(0, 24, 160, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(0, 24);
  tft.print("Dist: ");
  tft.print(distanceCategoryLabel(band));
}

float readBatteryVoltage() {
  digitalWrite(ADC_CTRL_PIN, HIGH);
  delay(2);
  int mv = analogReadMilliVolts(ADC_BAT_PIN);
  digitalWrite(ADC_CTRL_PIN, LOW);
  if (mv <= 0) {
    return 0.0f;
  }
  return (mv / 1000.0f) * VBAT_DIVIDER;
}

int batteryPercentFromVoltage(float vbat) {
  const float vMin = 3.3f;
  const float vMax = 4.2f;
  if (vbat <= vMin) return 0;
  if (vbat >= vMax) return 100;
  float pct = (vbat - vMin) / (vMax - vMin) * 100.0f;
  return (int)round(pct);
}

void updateBatteryDisplay(bool force) {
  if (!displayOn) {
    return;
  }
  float vbat = readBatteryVoltage();
  if (vbat <= 0.1f) {
    return;
  }
  int pct = batteryPercentFromVoltage(vbat);
  if (!force && pct == lastBatteryPct) {
    return;
  }
  lastBatteryPct = pct;

  char buf[6];
  snprintf(buf, sizeof(buf), "%d%%", pct);
  int len = (int)strlen(buf);
  int textWidth = len * 6; // 6px per char at text size 1
  int clearX = 160 - 40;
  int x = 160 - textWidth;
  if (x < clearX) {
    x = clearX;
  }

  tft.fillRect(clearX, 0, 40, 12, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(x, 0);
  tft.print(buf);
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

void playBootVibration() {
  if (!drvOk) {
    return;
  }
  const uint32_t sweepMs = 500;
  const uint8_t steps = 20;
  for (uint8_t i = 0; i < steps; i++) {
    float t = (steps <= 1) ? 1.0f : (float)i / (float)(steps - 1);
    uint8_t intensity = (uint8_t)round(20.0f + t * (127.0f - 20.0f));
    drv.setRealtimeValue(intensity);
    delay(sweepMs / steps);
  }
  drv.setRealtimeValue(0);
}

void enterDeepSleep() {
  if (!manualDeepSleepEnabled) {
    return;
  }
  // Stop radio activity ASAP so we don't process new events during shutdown.
  loraInterruptEnabled = false;
  lora.sleep();

  // Ensure haptics are fully off before deep sleep. Without this, the vibration
  // task can keep reasserting motor drive during the sleep transition window,
  // and the DRV2605 can continue vibrating into deep sleep.
  hapticsEnabled = false;
  if (vibeTaskHandle != nullptr) {
    vTaskSuspend(vibeTaskHandle);
  }
  requestHapticPulse = false;
  hapticPreviewUntilMs = 0;
  motorStopMs = 0;
  vibeOn = false;
  if (drvOk) {
    drv.setRealtimeValue(0);
    delay(20);
    drv.setRealtimeValue(0);
    // Force DRV2605(L) into standby (bit 6 of MODE register 0x01).
    // Adafruit library doesn't expose a STANDBY constant, so do it directly.
    // Safe to attempt both common addresses (0x5A / 0x5B).
    for (uint8_t addr : { (uint8_t)0x5A, (uint8_t)0x5B }) {
      Wire.beginTransmission(addr);
      Wire.write((uint8_t)0x01);
      Wire.write((uint8_t)0x40);
      Wire.endTransmission();
    }
  }
  showSleepCat();
  delay(1000);
  // Deep sleep is not light sleep mode (display timeout); clear the flag.
  lightSleepMode = false;
  setDisplayOn(false);
  digitalWrite(VEXT_CTRL, !VEXT_ACTIVE);
  // Best-effort radio shutdown for maximum power savings.
  // (WiFi/BLE are not used in normal operation, but explicitly disable anyway.)
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  btStop();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)USER_BTN_PIN, USER_BTN_ACTIVE_LOW ? 0 : 1);
  esp_deep_sleep_start();
}

// Enter light sleep with CAD monitoring for power savings
void enterLightSleepWithCAD(uint32_t durationMs) {
  if (!autoSleepEnabled) {
    delay(durationMs);
    return;
  }
  if (!powerSaveMode) {
    delay(durationMs);
    return;
  }
  // Only allow CAD during light sleep mode (display-off due to timeout).
  if (!lightSleepMode) {
    delay(durationMs);
    return;
  }
  // Development safety: entering light sleep while USB CDC serial is connected
  // can trip the interrupt watchdog on ESP32-S3 (seen as INT_WDT in logs).
  // Skip light sleep when a serial monitor is attached; still allows CAD sleep
  // in untethered/battery use.
  if (Serial) {
    delay(durationMs);
    return;
  }
  
  // Prepare for light sleep
  loraInterruptEnabled = false;
  cadDetectedFlag = false;
  
  // Configure LoRa for CAD mode (low power signal detection)
  // Set up interrupt handler for CAD detection
  lora.setDio1Action(setCADFlag);
  
  // Start interrupt-driven channel scan
  // This will trigger DIO1 when LoRa preamble is detected
  int state = lora.startChannelScan();
  if (state != RADIOLIB_ERR_NONE) {
    // Avoid heavy serial logging right before light sleep; it can trip WDT on some setups.
    if (displayOn) {
      Serial.print("CAD start failed: ");
      Serial.println(state);
    }
    lora.setDio1Action(setLoraFlag);
    lora.startReceive();
    loraInterruptEnabled = true;
    delay(durationMs);
    return;
  }
  
  // Configure ESP32 wake sources.
  // Note: ESP32-S3 GPIO wake from light sleep is LEVEL-only (no edges).
  //
  // Guard against "stuck asserted" lines (which would cause immediate wake loops):
  // if DIO1 or the button is already active, don't enter light sleep.
  if (digitalRead(LORA_DIO1) == HIGH ||
      digitalRead(USER_BTN_PIN) == (USER_BTN_ACTIVE_LOW ? LOW : HIGH)) {
    lora.setDio1Action(setLoraFlag);
    lora.startReceive();
    loraInterruptEnabled = true;
    delay(durationMs);
    return;
  }

  gpio_wakeup_enable((gpio_num_t)LORA_DIO1, GPIO_INTR_HIGH_LEVEL);
  gpio_wakeup_enable((gpio_num_t)USER_BTN_PIN,
                     USER_BTN_ACTIVE_LOW ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  esp_sleep_enable_timer_wakeup(durationMs * 1000); // microseconds
  
  // Enter light sleep (ESP32 ~800µA, SX1262 CAD ~2-3mA = ~4mA total)
  esp_light_sleep_start();
  
  // Check wake reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  bool shouldWakeScreen = false;
  
  if (cadDetectedFlag || wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
    // Check if CAD actually detected something
    int scanResult = lora.getChannelScanResult();
    if (scanResult == RADIOLIB_PREAMBLE_DETECTED) {
      if (displayOn) {
        Serial.println("CAD: Signal detected!");
      }
      lastUserInteractionMs = millis();  // Reset idle timer
      shouldWakeScreen = true;           // "turn on" from light sleep on CAD
    }
  }
  cadDetectedFlag = false;
  
  // Always return to full RX mode after wake
  lora.setDio1Action(setLoraFlag);
  lora.startReceive();
  loraInterruptEnabled = true;

  // If the wake was from a button click (or CAD) during light sleep, turn the display back on.
  // (This does NOT show boot logo/vibe; it's not a reboot.)
  if (!displayOn && wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
    if (digitalRead(USER_BTN_PIN) == (USER_BTN_ACTIVE_LOW ? LOW : HIGH)) {
      shouldWakeScreen = true;
    }
  }
  if (shouldWakeScreen && !displayOn) {
    setDisplayOn(true);
    lastUserInteractionMs = millis();
  }
}

void refreshDisplay() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);

  updateIntensityDisplay();
  updateFeedbackDisplay();
  updateSimDisplay();
  updateScaleDisplay();
  updateUiSelectionDisplay();

  lastBand = (DistanceBand)255;
  updateDistanceDisplay(lastStep);
}

void setDisplayOn(bool on, bool refresh) {
  if (displayOn == on) {
    return;
  }
  displayOn = on;
  if (on) {
    // Exiting light sleep mode.
    lightSleepMode = false;
    // Clear display before backlight to avoid stale flash
    digitalWrite(VEXT_CTRL, VEXT_ACTIVE);
    delay(50);
    tft.initR(INITR_MINI160x80_PLUGIN);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextWrap(false);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(1);
    digitalWrite(TFT_LED_K, HIGH);
    if (refresh) {
      refreshDisplay();
    }
  } else {
    digitalWrite(TFT_LED_K, LOW);
    digitalWrite(VEXT_CTRL, !VEXT_ACTIVE);
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

      if (!hapticsEnabled) {
        if (motorStopMs != 0 || vibeOn) {
          drv.setRealtimeValue(0);
          motorStopMs = 0;
          vibeOn = false;
        }
        vTaskDelay(delayTicks);
        continue;
      }

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
  // Bring up the display ASAP so the HUMN logo appears immediately on boot.
  // Keep backlight off while drawing to avoid a "flash" of uninitialized pixels.
  pinMode(VEXT_CTRL, OUTPUT);
  digitalWrite(VEXT_CTRL, VEXT_ACTIVE);  // Powers TFT + GNSS rail
  pinMode(TFT_LED_K, OUTPUT);
  digitalWrite(TFT_LED_K, LOW);          // Backlight off while initializing
  delay(50);

  tft.initR(INITR_MINI160x80_PLUGIN);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  showHumnLogo();
  digitalWrite(TFT_LED_K, HIGH);         // Backlight on once logo is drawn
  displayOn = true;
  lightSleepMode = false;
  lastUserInteractionMs = millis();

  // Initialize Serial - using USB CDC on ESP32-S3
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  delay(300);

  Serial.println("\n\n========================================");
  Serial.println("  Wireless Tracker - TFT + LoRa");
  Serial.println("========================================\n");
  printBootReason();

  // Reduce ESP-IDF/Arduino core log noise (prevents USB-serial ISR overload -> INT_WDT).
  esp_log_level_set("*", ESP_LOG_ERROR);

  const esp_reset_reason_t resetReason = esp_reset_reason();
  const bool coldBoot = (resetReason == ESP_RST_POWERON);

  // Seed RNG for ping jitter
  randomSeed((uint32_t)ESP.getEfuseMac());
  uint64_t mac = ESP.getEfuseMac();
  deviceId = (uint32_t)(mac ^ (mac >> 32));
  Serial.print("Device ID: ");
  Serial.println(deviceId, HEX);

  prefs.begin("humn", false);
  loadSettings();

  // Hold GNSS in reset (we don't use it)
  pinMode(GNSS_RST_PIN, OUTPUT);
  digitalWrite(GNSS_RST_PIN, LOW);

  // Setup user button
  pinMode(USER_BTN_PIN, INPUT_PULLUP);

  // Setup battery ADC
  pinMode(ADC_CTRL_PIN, OUTPUT);
  digitalWrite(ADC_CTRL_PIN, LOW);
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_BAT_PIN, ADC_11db);

  // Setup status LED
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

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

  // Setup DRV2605L (I2C) - try primary bus (4/5), then alternate (16/17) if needed
  struct { int sda; int scl; } i2cBuses[] = {
    { I2C_SDA_PRIMARY,   I2C_SCL_PRIMARY   },
    { I2C_SDA_ALTERNATE, I2C_SCL_ALTERNATE }
  };
  for (size_t b = 0; b < sizeof(i2cBuses) / sizeof(i2cBuses[0]) && !drvOk; b++) {
    Wire.end();
    Wire.begin(i2cBuses[b].sda, i2cBuses[b].scl);
    drvOk = drv.begin();
    if (drvOk) {
      drv.selectLibrary(1);
      drv.setMode(DRV2605_MODE_REALTIME);
      drv.setRealtimeValue(0);
      Serial.println("DRV2605L init OK.");
      Serial.print("DRV2605L I2C pins: SDA=");
      Serial.print(i2cBuses[b].sda);
      Serial.print(" SCL=");
      Serial.println(i2cBuses[b].scl);
      Serial.println("Button cycles intensity: soft/medium/hard/max");
    } else {
      Serial.print("DRV2605L not found on SDA=");
      Serial.print(i2cBuses[b].sda);
      Serial.print(" SCL=");
      Serial.println(i2cBuses[b].scl);
    }
  }
  if (!drvOk) {
    Serial.println("DRV2605L init FAILED on both I2C buses.");
    Serial.println("Check wiring + power. DRV2605L addr is 0x5A or 0x5B.");
  }

  // Boot vibration (cold boot only). Deep sleep wake is a reboot too, but keep
  // the haptic quieter unless it's a true power-on.
  if (coldBoot) {
    playBootVibration();
    delay(500);
  }
  refreshDisplay();

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
    // Treat press as user activity immediately so the display timeout can't
    // turn the screen off mid-press (which can make the release look like a "wake").
    lastUserInteractionMs = nowMs;
  }

  if (btnPressed && btnPressStartMs != 0) {
    uint32_t heldMs = nowMs - btnPressStartMs;

    if (heldMs >= BTN_POWER_HOLD_MS && !btnPowerHandled) {
      btnPowerHandled = true;
      lastUserInteractionMs = nowMs;
      // Power action triggers immediately; ignore release for this hold.
      btnPressStartMs = 0;
      enterDeepSleep();
    } else if (deviceOn && displayOn && heldMs >= BTN_HOLD_MS && !btnHoldHandled) {
      // Long-press: cycle which setting is being edited (skip ping — fixed at 50ms)
      btnHoldHandled = true;
      do {
        uiSetting = (uiSetting + 1) % 5;
      } while (uiSetting == UI_PING);

      if (verboseUiLogs && Serial) {
        Serial.print("Edit setting: ");
        if (uiSetting == UI_VIBE) Serial.println("vibe");
        else if (uiSetting == UI_FEEDBACK) Serial.println("feedback");
        else if (uiSetting == UI_SIM) Serial.println("sim");
        else Serial.println("scale");
      }
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
        if (verboseUiLogs && Serial) {
          Serial.print("Intensity mode: ");
          Serial.println(label);
        }
        updateIntensityDisplay();
        saveSettings();
      } else if (uiSetting == UI_FEEDBACK) {
        feedbackMode = (FeedbackMode)((feedbackMode + 1) % 3);
        if (verboseUiLogs && Serial) {
          Serial.print("Feedback mode: ");
          if (feedbackMode == FEEDBACK_PULSED_RATE) Serial.println("pulsed");
          else if (feedbackMode == FEEDBACK_PULSE_ON_RX) Serial.println("rx-pulse");
          else Serial.println("constant");
        }
        updateFeedbackDisplay();
        saveSettings();
      } else if (uiSetting == UI_SIM) {
        simulateDistance = !simulateDistance;
        if (verboseUiLogs && Serial) {
          Serial.print("Sim mode: ");
          Serial.println(simulateDistance ? "on" : "off");
        }

        updateSimDisplay();
        saveSettings();
      } else {
        // Distance scale: 1.0x .. 3.0x in 0.1 steps
        distanceScaleIndex++;
        if (distanceScaleIndex > 30) distanceScaleIndex = 10;
        if (verboseUiLogs && Serial) {
          Serial.print("Scale: ");
          Serial.print(distanceScaleIndex / 10.0f, 1);
          Serial.println("x");
        }
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
  uint32_t effectivePingBaseMs = pingBaseMs;
  uint32_t effectivePingJitterMs = pingJitterMs;
  if (!displayOn && role == ROLE_SEARCH) {
    effectivePingBaseMs = SEARCH_SLEEP_PING_MS;
    effectivePingJitterMs = max<uint32_t>(10, effectivePingBaseMs / 4);
  }

  if (role == ROLE_SEARCH && !displayOn && (nextPingMs == 0 || nextPingMs > now + effectivePingBaseMs)) {
    nextPingMs = now + effectivePingBaseMs + random(0, effectivePingJitterMs);
  }

  // Warn if we're late sending a ping (only when we are supposed to ping)
  if (role != ROLE_RESPONDER && nextPingMs != 0 && now > nextPingMs + PING_LATE_WARN_MS) {
    // Throttle this log heavily; at fast ping rates it can create a feedback loop
    // (printing makes loop slower => always "late" => prints constantly => WDT).
    if (verboseTimingLogs && displayOn && Serial &&
        (now - lastPingLateLogMs > PING_LATE_LOG_THROTTLE_MS)) {
      lastPingLateLogMs = now;
      Serial.print("Ping late by ");
      Serial.print(now - nextPingMs);
      Serial.println(" ms");
    }
    // avoid spamming: move the schedule forward
    nextPingMs = now + effectivePingBaseMs + random(0, effectivePingJitterMs);
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
      nextPingMs = now + effectivePingBaseMs + random(0, effectivePingJitterMs);
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

      if (verboseTimingLogs && displayOn && Serial &&
          lastPingSentMs != 0 && (pingCounter % pingIntervalLogEvery == 0)) {
        Serial.print("Ping interval: ");
        Serial.print(now - lastPingSentMs);
        Serial.println(" ms");
      }
      if (verboseTimingLogs && displayOn && Serial && txDurMs > TX_WARN_MS) {
        Serial.print("TX duration: ");
        Serial.print(txDurMs);
        Serial.println(" ms");
      }
      lastPingSentMs = now;
      nextPingMs = now + effectivePingBaseMs + random(0, effectivePingJitterMs);
    }
  }

  // RX watchdog: recover if RX stalls
  if (millis() - lastRxMs > RX_WATCHDOG_MS) {
    lora.startReceive();
    lastRxMs = millis();
  }

  // Battery display update
  if (deviceOn && displayOn && (now - lastBatteryReadMs > 1000)) {
    lastBatteryReadMs = now;
    updateBatteryDisplay(false);
  }

  // Display idle timeout
  // Don't auto-sleep the display while the button is held.
  if (autoSleepEnabled && displayOn && !btnPressed && (millis() - lastUserInteractionMs > DISPLAY_IDLE_MS)) {
    setDisplayOn(false);
    // Light sleep mode begins only when the display times out.
    lightSleepMode = true;
  }

  // LED pulse off

  if (LED_ENABLED && ledOn && millis() > ledOffMs) {
    digitalWrite(STATUS_LED_PIN, LOW);
    ledOn = false;
  }

  // Power saving: use light sleep with CAD when idle
  if (autoSleepEnabled && powerSaveMode && lightSleepMode && !displayOn && deviceOn) {
    uint32_t idleTime = millis() - lastUserInteractionMs;
    
    // If we've been idle a while and no recent signal, enter light sleep
    // This saves power while still monitoring for incoming signals
    if (idleTime > 5000 && (millis() - lastSignalMs) > 1000) {
      // Enter light sleep with CAD monitoring
      // The LoRa module will wake us if it detects a signal
      enterLightSleepWithCAD(CAD_CHECK_INTERVAL_MS);
    }
  }
}