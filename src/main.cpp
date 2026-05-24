#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_DRV2605.h>
#include <RadioLib.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_log.h>
#include <math.h>

// Humn custom PCB: ESP32-C6-WROOM-1-N8 + SX1262 + DRV2605L.
// There is no TFT/display rail on this board; status goes to USB serial.
static const uint32_t DISPLAY_IDLE_MS = 15000;

// DRV2605L haptic driver.
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const int HAPTIC_EN_PIN = 23;
static const int HAPTIC_TRIG_PIN = 15;
static const uint16_t MOTOR_PULSE_MS = 200;

// SW_BOOT is GPIO9 on ESP32-C6. It can be used as a user button after boot,
// but holding it while resetting will enter the ROM bootloader.
static const int USER_BTN_PIN = 9;
static const bool USER_BTN_ACTIVE_LOW = true;
static const uint32_t BTN_DEBOUNCE_MS = 200;
static const uint32_t BTN_HOLD_MS = 700;
static const uint32_t BTN_POWER_HOLD_MS = 5000;
static const int MODE_SWITCH_ADC_PIN = 3;
static const int ADC_BAT_PIN = 2;
static const float VBAT_DIVIDER = 2.0f;

enum UiSetting : uint8_t {
  UI_VIBE = 0,
  UI_PING = 1,
  UI_FEEDBACK = 2,
  UI_SIM = 3,
  UI_SCALE = 4
};

void updateBatteryDisplay(bool force = false);
void setDisplayOn(bool on, bool refresh = true);
int estimateDistanceStep(int16_t rssiDbm);
void configureLoRaRadio();
void recoverLoRaRadio(const char* reason, int state);
void applyModeSwitchScale(bool force = false);

// Status LED
static const int STATUS_LED_PIN = -1;
static const bool LED_ENABLED = false;
static const uint32_t LED_PULSE_MS = 40;

Adafruit_DRV2605 drv;

// LoRa pins from the Humn schematic.
static const int LORA_NSS  = 7;
static const int LORA_SCK  = 1;
static const int LORA_MOSI = 10;
static const int LORA_MISO = 8;
static const int LORA_RST  = 5;
static const int LORA_BUSY = 4;
static const int LORA_DIO1 = 6;
static const int LORA_DIO2 = 18;
static const int LORA_DIO3 = 19;

// LoRa settings (US915 defaults)
static const float LORA_FREQ_MHZ = 915.0;
static const float LORA_BW_KHZ = 500.0;
static const uint8_t LORA_SF = 6;
static const uint8_t LORA_CR = 5;
static const uint8_t LORA_SYNC_WORD = 0x12;
static const int8_t LORA_TX_POWER = 10;

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
static uint32_t rxLogEvery = 5;
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
static uint32_t pingBaseMs = 700;
static uint32_t pingJitterMs = 175;
static const uint32_t PING_LATE_WARN_MS = 50;
static uint32_t lastPingLateLogMs = 0;
static const uint32_t PING_LATE_LOG_THROTTLE_MS = 2000;
static const uint32_t SEARCH_SLEEP_PING_MS = 2000;
// Serial logging can trip the INT_WDT on ESP32-S3 with USB CDC under load.
// Keep these off unless actively debugging.
static bool verboseTimingLogs = false;
static bool verboseUiLogs = false;

static const uint32_t TX_WARN_MS = 80;
static const uint32_t PEER_TIMEOUT_MS = 5000;

enum Role : uint8_t { ROLE_SEARCH = 0, ROLE_PINGER = 1, ROLE_RESPONDER = 2 };
static Role role = ROLE_SEARCH;
static uint32_t deviceId = 0;
static uint32_t peerId = 0;
static uint32_t lastPeerSeenMs = 0;
static Preferences prefs;
static bool simulateDistance = false;
static const bool SIMULATION_MODE_ENABLED = false;
static uint8_t distanceScaleIndex = 10; // 10 = 1.0x
static uint8_t lastModeSwitchBucket = 255;
static int lastModeSwitchStableValue = 0;
static int lastModeSwitchLoggedValue = -10000;
static uint8_t pendingModeSwitchBucket = 255;
static uint8_t pendingModeSwitchCount = 0;
static int pendingModeSwitchValue = 0;
static uint32_t lastModeSwitchReadMs = 0;
static uint32_t lastInvalidModeSwitchLogMs = 0;
static const uint32_t MODE_SWITCH_READ_MS = 75;
static const uint8_t MODE_SWITCH_STABLE_READS = 2;
static const bool MODE_SWITCH_ANALOG_ENABLED = true;
static const uint8_t MODE_SWITCH_INVALID_BUCKET = 255;

void saveSettings() {
  prefs.putUChar("intensity", intensityMode);
  prefs.putUChar("pingIdx", pingRateIndex);
  prefs.putUChar("feedback", (uint8_t)feedbackMode);
  prefs.putUChar("ui", uiSetting);
  if (SIMULATION_MODE_ENABLED) {
    prefs.putBool("sim", simulateDistance);
  }
}

void loadSettings() {
  intensityMode = 3;
  // Beacon at a modest rate for the custom board. Both peers transmit their ID,
  // and distance is estimated from any received beacon.
  pingRateIndex = 4;
  pingBaseMs = 700;
  pingJitterMs = max<uint32_t>(25, pingBaseMs / 4);

  feedbackMode = FEEDBACK_PULSED_RATE;
  uiSetting = prefs.getUChar("ui", (uint8_t)UI_VIBE);
  if (uiSetting == UI_PING || uiSetting == UI_SCALE ||
      (!SIMULATION_MODE_ENABLED && uiSetting == UI_SIM)) {
    uiSetting = UI_VIBE;
  }
  simulateDistance = SIMULATION_MODE_ENABLED ? prefs.getBool("sim", false) : false;
  distanceScaleIndex = 10;
}
static const uint32_t VIBE_TIMEOUT_MS = 3000;
static uint32_t nextVibeToggleMs = 0;
static int lastStep = 19;
static float smoothedRssi = -100.0f;
static const float RSSI_SMOOTH_ALPHA = 0.10f;
static float smoothedDistanceFt = 2000.0f;
static const float DIST_SMOOTH_ALPHA = 0.15f;
static bool vibeOn = false;
static float smoothedIntervalMs = 600.0f;
static const float INTERVAL_SMOOTH_ALPHA = 0.20f;

// Distance estimate model (log-distance path loss).
// Tune these two constants if you want the feet ranges to align better in your environment.
static const float RSSI_AT_1M_DBM = -48.0f;
static const float PATH_LOSS_EXPONENT = 2.4f; // indoor-ish default; higher => distance grows faster for same RSSI

static inline float feetFromMeters(float m) { return m * 3.28084f; }
static inline float metersFromFeet(float ft) { return ft / 3.28084f; }

static float estimateDistanceFeetFromRssi(int16_t rssiDbm) {
  float n = PATH_LOSS_EXPONENT;
  if (n < 1.0f) n = 1.0f;
  // d(m) = 10 ^ ((RSSI@1m - RSSI) / (10*n))
  float expv = (RSSI_AT_1M_DBM - (float)rssiDbm) / (10.0f * n);
  float dMeters = powf(10.0f, expv);
  if (!isfinite(dMeters) || dMeters < 0.01f) dMeters = 0.01f;
  float dFt = feetFromMeters(dMeters);
  if (!isfinite(dFt) || dFt < 0.0f) dFt = 0.0f;
  // Prevent absurd growth if RSSI is very low/noisy
  if (dFt > 50000.0f) dFt = 50000.0f;
  return dFt;
}

static int16_t estimateRssiFromDistanceFeet(float distFt) {
  float dMeters = metersFromFeet(distFt);
  if (dMeters < 0.01f) dMeters = 0.01f;
  float n = PATH_LOSS_EXPONENT;
  if (n < 1.0f) n = 1.0f;
  float rssi = RSSI_AT_1M_DBM - 10.0f * n * log10f(dMeters);
  if (!isfinite(rssi)) rssi = -120.0f;
  if (rssi > -5.0f) rssi = -5.0f;
  if (rssi < -130.0f) rssi = -130.0f;
  return (int16_t)round(rssi);
}

// "Windowed" haptics: treat each distance window as its own scaling range.
// As you move closer within a window, intensity ramps up and interval ramps down.
// When you cross into a new window, scaling resets and we play a short ramp
// (up when moving closer, down when moving farther).
enum HapticWindow : uint8_t {
  WIN_CLOSE = 0,
  WIN_MED = 1,
  WIN_FAR = 2,
  WIN_VERY_FAR = 3
};

struct HapticWindowSpec {
  float distMinFt; // closest distance in this window
  float distMaxFt; // farthest distance in this window
  uint8_t intensityMin; // base realtime intensity (0..127) at far edge
  uint8_t intensityMax; // base realtime intensity (0..127) at near edge
  uint32_t intervalMaxMs; // pulse interval at far edge (slower)
  uint32_t intervalMinMs; // pulse interval at near edge (faster)
  float onRatioMin; // duty at far edge
  float onRatioMax; // duty at near edge
};

static const HapticWindowSpec HAPTIC_WINDOWS[] = {
  // NOTE: Per your request, EVERY window spans the full intensity range.
  // The "feel" of each window is differentiated mostly by interval + duty.
  // WIN_CLOSE (0..20 ft): fastest
  { 1.0f, 20.0f, 0, 127, 600, 150, 0.40f, 0.85f },
  // WIN_MED (20..100 ft)
  { 20.0f, 100.0f, 0, 127, 800, 220, 0.32f, 0.70f },
  // WIN_FAR (100..400 ft)
  { 100.0f, 400.0f, 0, 127, 900, 320, 0.25f, 0.55f },
  // WIN_VERY_FAR (400+ ft): slowest
  { 400.0f, 2000.0f, 0, 127, 1100, 700, 0.20f, 0.30f },
};

static HapticWindow currentHapticWindow = WIN_VERY_FAR;
static uint32_t lastHapticWindowChangeMs = 0;
static float lastWindowDistanceFt = 2000.0f;

static const uint32_t WINDOW_RSSI_AVG_MS = 2000;
static const uint8_t RSSI_HISTORY_MAX = 64; // enough for ~2s at >=30ms updates
struct RssiSample {
  int16_t rssiDbm;
  uint32_t tMs;
};
static RssiSample rssiHistory[RSSI_HISTORY_MAX];
static uint8_t rssiHistoryHead = 0;
static uint8_t rssiHistoryCount = 0;

struct HapticTransitionRequest {
  volatile bool pending;
  volatile uint8_t fromIntensity;
  volatile uint8_t toIntensity;
  volatile uint8_t pattern; // 0 = distance up, 1 = distance down, 2 = scale change
  volatile uint16_t durationMs;
};
static HapticTransitionRequest hapticTransitionReq = { false, 0, 0, 0, 1400 };

static inline float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

static inline float lerpFloat(float a, float b, float t) {
  return a + (b - a) * t;
}

static inline uint8_t lerpU8(uint8_t a, uint8_t b, float t) {
  float v = lerpFloat((float)a, (float)b, t);
  if (v < 0.0f) v = 0.0f;
  if (v > 127.0f) v = 127.0f;
  return (uint8_t)round(v);
}

static HapticWindow hapticWindowFromDistanceFeet(float distFt) {
  if (distFt <= 20.0f) return WIN_CLOSE;
  if (distFt <= 100.0f) return WIN_MED;
  if (distFt <= 400.0f) return WIN_FAR;
  return WIN_VERY_FAR;
}

const HapticWindowSpec& hapticWindowSpec(HapticWindow w) {
  return HAPTIC_WINDOWS[(uint8_t)w];
}

static float hapticWindowProgressDistance(float distFt, const HapticWindowSpec& spec) {
  // 0.0 at far edge (distMax), 1.0 at near edge (distMin)
  float minFt = spec.distMinFt;
  float maxFt = spec.distMaxFt;
  if (maxFt <= minFt) {
    return 1.0f;
  }
  if (distFt < minFt) distFt = minFt;
  if (distFt > maxFt) distFt = maxFt;
  float t = (maxFt - distFt) / (maxFt - minFt);
  return clamp01(t);
}

uint8_t applyUserIntensityMode(uint8_t intensity) {
  // Preserve the full 0..127 range, but change the response curve.
  // soft = less sensitive (more compression at low end)
  // max = linear
  float x = (float)intensity / 127.0f;
  x = clamp01(x);
  float gamma = 1.0f;
  switch (intensityMode) {
    case 0: gamma = 2.4f; break;  // soft
    case 1: gamma = 1.7f; break;  // medium
    case 2: gamma = 1.25f; break; // hard
    case 3: default: gamma = 1.0f; break; // max
  }
  float y = (gamma == 1.0f) ? x : powf(x, gamma);
  y = clamp01(y);
  return (uint8_t)round(y * 127.0f);
}

uint8_t motorIntensityForDistanceInWindow(float distFt, HapticWindow w) {
  const auto& spec = hapticWindowSpec(w);
  float t = hapticWindowProgressDistance(distFt, spec);
  uint8_t base = lerpU8(spec.intensityMin, spec.intensityMax, t);
  return applyUserIntensityMode(base);
}

uint8_t motorIntensityForStepWindowed(int step) {
  (void)step;
  return motorIntensityForDistanceInWindow(smoothedDistanceFt, currentHapticWindow);
}

uint32_t vibrationIntervalForDistanceInWindow(float distFt, HapticWindow w) {
  const auto& spec = hapticWindowSpec(w);
  float t = hapticWindowProgressDistance(distFt, spec);
  float interval = lerpFloat((float)spec.intervalMaxMs, (float)spec.intervalMinMs, t);
  if (interval < 40.0f) interval = 40.0f;
  return (uint32_t)round(interval);
}

uint32_t vibrationIntervalForStepWindowed(int step) {
  (void)step;
  return vibrationIntervalForDistanceInWindow(smoothedDistanceFt, currentHapticWindow);
}

float vibrationOnRatioForDistanceInWindow(float distFt, HapticWindow w) {
  const auto& spec = hapticWindowSpec(w);
  float t = hapticWindowProgressDistance(distFt, spec);
  float ratio = lerpFloat(spec.onRatioMin, spec.onRatioMax, t);
  if (ratio < 0.10f) ratio = 0.10f;
  if (ratio > 0.95f) ratio = 0.95f;
  return ratio;
}

float vibrationOnRatioForStepWindowed(int step) {
  (void)step;
  return vibrationOnRatioForDistanceInWindow(smoothedDistanceFt, currentHapticWindow);
}

uint8_t windowFloorIntensity(HapticWindow w) {
  return applyUserIntensityMode(hapticWindowSpec(w).intensityMin);
}

uint8_t windowPeakIntensity(HapticWindow w) {
  return applyUserIntensityMode(hapticWindowSpec(w).intensityMax);
}

static void recordRssiSample(int16_t rssiDbm, uint32_t nowMs) {
  rssiHistory[rssiHistoryHead] = { rssiDbm, nowMs };
  rssiHistoryHead = (uint8_t)((rssiHistoryHead + 1) % RSSI_HISTORY_MAX);
  if (rssiHistoryCount < RSSI_HISTORY_MAX) {
    rssiHistoryCount++;
  }
}

static int16_t averageRssiLastMs(uint32_t nowMs, uint32_t windowMs, int16_t fallback) {
  if (rssiHistoryCount == 0) {
    return fallback;
  }
  int32_t sum = 0;
  int32_t n = 0;
  for (uint8_t i = 0; i < rssiHistoryCount; i++) {
    // Walk backwards from newest
    int idx = (int)rssiHistoryHead - 1 - (int)i;
    if (idx < 0) idx += RSSI_HISTORY_MAX;
    const RssiSample& s = rssiHistory[idx];
    if (nowMs - s.tMs <= windowMs) {
      sum += (int32_t)s.rssiDbm;
      n++;
    } else {
      // older samples will only be older; stop early
      break;
    }
  }
  if (n <= 0) {
    return fallback;
  }
  return (int16_t)round((float)sum / (float)n);
}

static void resetRssiAverager() {
  rssiHistoryCount = 0;
  rssiHistoryHead = 0;
}

static void onHapticWindowChanged(HapticWindow oldW, HapticWindow newW) {
  uint32_t now = millis();
  // Debounce window changes to avoid jitter-triggered ramps at thresholds.
  if (now - lastHapticWindowChangeMs < 300) return;
  lastHapticWindowChangeMs = now;

  bool movingCloser = ((uint8_t)newW < (uint8_t)oldW);
  uint8_t fromI = motorIntensityForDistanceInWindow(smoothedDistanceFt, oldW);
  uint8_t toI = movingCloser ? windowPeakIntensity(newW) : windowFloorIntensity(newW);
  // Start transitions from silence if we'd otherwise be off (better perceptual cue).
  if (!vibeOn) {
    fromI = 0;
  }

  hapticTransitionReq.fromIntensity = fromI;
  hapticTransitionReq.toIntensity = toI;
  hapticTransitionReq.pattern = movingCloser ? 0 : 1;
  hapticTransitionReq.durationMs = movingCloser ? 1400 : 1600;
  hapticTransitionReq.pending = true;
}

static void updateHapticWindowFromAvgRssi(uint32_t nowMs) {
  int16_t avgRssi = averageRssiLastMs(nowMs, WINDOW_RSSI_AVG_MS, lastRssi);
  float avgDistFt = estimateDistanceFeetFromRssi(avgRssi);
  lastWindowDistanceFt = avgDistFt;
  HapticWindow newW = hapticWindowFromDistanceFeet(avgDistFt);
  if (newW != currentHapticWindow) {
    HapticWindow oldW = currentHapticWindow;
    // Trigger transition before switching the active window.
    onHapticWindowChanged(oldW, newW);
    currentHapticWindow = newW;
  }
}

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
static float lastBatteryVoltage = 0.0f;

void enterDeepSleep();
void playBootVibration();
void requireWakeHoldIfNeeded();


// Simulation mode: cycles distance without RF (user-toggle)
static const uint32_t SIM_HOLD_OUTER_MS = 10000; // "None"
static const uint32_t SIM_RAMP_MS = 30000;       // None -> Very Close (and reverse)
static const uint32_t SIM_HOLD_INNER_MS = 10000; // Very Close hold
static const uint32_t SIM_CYCLE_MS = SIM_HOLD_OUTER_MS + SIM_RAMP_MS + SIM_HOLD_INNER_MS + SIM_RAMP_MS;
static const float SIM_NEAR_FT = 2.0f;
static const float SIM_FAR_FT = 800.0f; // put it well into the 400+ ft bucket

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

const char* feedbackLabel() {
  if (feedbackMode == FEEDBACK_PULSED_RATE) return "pulsed";
  if (feedbackMode == FEEDBACK_PULSE_ON_RX) return "rx-pulse";
  return "constant";
}

void updateIntensityDisplay() {
  if (Serial) {
    Serial.print("Vibe: ");
    Serial.println(intensityLabel(intensityMode));
  }
  updateBatteryDisplay(true);
}

void updateFeedbackDisplay() {
  if (!Serial) {
    return;
  }
  Serial.print("Mode: ");
  Serial.println(feedbackLabel());
}

void updateUiSelectionDisplay() {
  if (!Serial) {
    return;
  }
  Serial.print("Edit: ");
  if (uiSetting == UI_VIBE) Serial.println("vibe");
  else if (uiSetting == UI_FEEDBACK) Serial.println("feedback");
  else if (uiSetting == UI_SIM) Serial.println("sim");
  else Serial.println("scale");
}

void updateSimDisplay() {
  if (Serial) {
    Serial.print("Sim: ");
    Serial.println(simulateDistance ? "on" : "off");
  }
}

void updateScaleDisplay() {
  if (Serial) {
    Serial.print("Scale: ");
    Serial.print(distanceScaleIndex / 10.0f, 1);
    Serial.println("x");
  }
}

const char* modeSwitchReadingUnit() {
  return MODE_SWITCH_ANALOG_ENABLED ? "mV" : "gpio";
}

int readModeSwitchDigitalValue() {
  return digitalRead(MODE_SWITCH_ADC_PIN) == HIGH ? 1 : 0;
}

int readModeSwitchValue() {
  if (!MODE_SWITCH_ANALOG_ENABLED) {
    return readModeSwitchDigitalValue();
  }

  const int samples = 8;
  int total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogReadMilliVolts(MODE_SWITCH_ADC_PIN);
    delay(1);
  }
  return total / samples;
}

uint8_t modeSwitchBucketFromResistanceOhms(int ohms) {
  // Measured on the assembled board at MODE_SWITCH_VAL to GND:
  // position 0 ~= 10k, position 1 ~= 34k, positions 2/3 ~= 20k.
  // Positions 2 and 3 are electrically identical from this one sense pin.
  if (ohms < 15000) return 0;
  if (ohms < 27000) return 2;
  return 1;
}

uint8_t modeSwitchBucketFromMilliVolts(int mv) {
  // Revision 1 switch salvage mode:
  // positions 0/1 can short regulator-enable related nets during travel and
  // are not reliable as live settings. Treat their low-voltage range as invalid
  // and only use the two stable positions that measure around 2.2V and 3.1V.
  if (mv < 1850) return MODE_SWITCH_INVALID_BUCKET;
  if (mv < 2550) return 2;
  return 1;
}

uint8_t modeSwitchBucketFromReading(int switchValue) {
  if (MODE_SWITCH_ANALOG_ENABLED) {
    return modeSwitchBucketFromMilliVolts(switchValue);
  }
  return switchValue ? 1 : 0;
}

uint8_t scaleIndexForModeSwitchBucket(uint8_t bucket) {
  static const uint8_t scales[] = { 10, 10, 30 };
  if (bucket >= sizeof(scales)) {
    bucket = sizeof(scales) - 1;
  }
  return scales[bucket];
}

void requestScaleChangeHapticCue(uint8_t bucket) {
  static const uint8_t cueIntensity[] = { 0, 95, 127 };
  if (bucket >= sizeof(cueIntensity)) {
    bucket = sizeof(cueIntensity) - 1;
  }

  hapticTransitionReq.fromIntensity = 0;
  hapticTransitionReq.toIntensity = cueIntensity[bucket];
  hapticTransitionReq.pattern = 2;
  hapticTransitionReq.durationMs = 300;
  hapticTransitionReq.pending = true;

  if (Serial) {
    Serial.print("Scale haptic cue requested: bucket=");
    Serial.print(bucket);
    Serial.print(" intensity=");
    Serial.println(cueIntensity[bucket]);
  }
}

void printModeSwitchScale(int switchValue, uint8_t bucket, bool boot) {
  if (!Serial) {
    return;
  }
  Serial.print(boot ? "Mode switch boot: " : "Mode switch changed: ");
  Serial.print(switchValue);
  Serial.print(" ");
  Serial.print(modeSwitchReadingUnit());
  Serial.print(" bucket=");
  Serial.print(bucket);
  Serial.print(" scale=");
  Serial.print(distanceScaleIndex / 10.0f, 1);
  Serial.println("x");
}

void printInvalidModeSwitchValue(int switchValue, bool boot) {
  if (!Serial) {
    return;
  }
  Serial.print(boot ? "Mode switch boot ignored: " : "Mode switch ignored: ");
  Serial.print(switchValue);
  Serial.print(" ");
  Serial.print(modeSwitchReadingUnit());
  Serial.println(" invalid Rev1 switch range");
}

void applyModeSwitchScale(bool force) {
  int switchValue = readModeSwitchValue();
  uint8_t bucket = modeSwitchBucketFromReading(switchValue);
  if (bucket == MODE_SWITCH_INVALID_BUCKET) {
    uint32_t now = millis();
    if (force) {
      printInvalidModeSwitchValue(switchValue, true);
      return;
    }
    if (now - lastInvalidModeSwitchLogMs >= 1000) {
      printInvalidModeSwitchValue(switchValue, false);
      lastInvalidModeSwitchLogMs = now;
    }
    pendingModeSwitchBucket = MODE_SWITCH_INVALID_BUCKET;
    pendingModeSwitchCount = 0;
    return;
  }

  if (!force && bucket == lastModeSwitchBucket) {
    if (abs(switchValue - lastModeSwitchLoggedValue) >= 250) {
      printModeSwitchScale(switchValue, bucket, false);
      lastModeSwitchLoggedValue = switchValue;
    }
    lastModeSwitchStableValue = switchValue;
    return;
  }

  if (!force) {
    if (bucket != pendingModeSwitchBucket) {
      pendingModeSwitchBucket = bucket;
      pendingModeSwitchValue = switchValue;
      pendingModeSwitchCount = 1;
      return;
    }

    if (pendingModeSwitchCount < MODE_SWITCH_STABLE_READS) {
      pendingModeSwitchCount++;
      return;
    }
  }

  pendingModeSwitchBucket = bucket;
  pendingModeSwitchCount = MODE_SWITCH_STABLE_READS;
  pendingModeSwitchValue = switchValue;
  lastModeSwitchBucket = bucket;
  lastModeSwitchStableValue = pendingModeSwitchValue;
  distanceScaleIndex = scaleIndexForModeSwitchBucket(bucket);
  printModeSwitchScale(pendingModeSwitchValue, bucket, force);
  lastModeSwitchLoggedValue = pendingModeSwitchValue;
  if (!force) {
    requestScaleChangeHapticCue(bucket);
  }
}

enum DistanceBand : uint8_t {
  BAND_NONE = 0,
  BAND_FAR = 1,
  BAND_MED = 2,
  BAND_CLOSE = 3
};

static DistanceBand lastBand = BAND_NONE;
static uint32_t lastDistanceNumericUpdateMs = 0;

const char* distanceCategoryLabel(DistanceBand band) {
  switch (band) {
    case BAND_NONE: return "none";
    case BAND_FAR: return "far";
    case BAND_MED: return "medium";
    default: return "close";
  }
}

DistanceBand distanceBandFromStep(int step) {
  (void)step;
  // "None" should mean "no recent signal", not "very far".
  if (millis() - lastSignalMs > VIBE_TIMEOUT_MS) {
    return BAND_NONE;
  }
  if (currentHapticWindow == WIN_CLOSE) return BAND_CLOSE;
  if (currentHapticWindow == WIN_MED) return BAND_MED;
  // FAR + VERY_FAR both show as "far" in the UI (limited space).
  return BAND_FAR;
}

void updateDistanceDisplay(int step) {
  // Add hysteresis by only updating when the band changes.
  // Also refresh periodically so serial diagnostics stay useful.
  DistanceBand band = distanceBandFromStep(step);
  uint32_t now = millis();
  bool periodicUpdate = displayOn && (now - lastDistanceNumericUpdateMs >= 1000);
  if (band == lastBand && !periodicUpdate) {
    return;
  }
  lastBand = band;
  lastDistanceNumericUpdateMs = now;
  if (!Serial) {
    return;
  }
  Serial.print("Dist: ");
  uint32_t linkAgeMs = (lastSignalMs == 0) ? UINT32_MAX : now - lastSignalMs;
  if (band == BAND_NONE) {
    Serial.print("no-signal");
  } else {
    Serial.print(distanceCategoryLabel(band));

    float ft = smoothedDistanceFt;
    Serial.print(" ");
    if (!isfinite(ft) || ft < 0.0f) {
      Serial.print("--");
    } else {
      int shownFt = (ft < 100.0f) ? (int)round(ft)
                   : (ft < 1000.0f) ? (int)round(ft / 10.0f) * 10
                   : (int)round(ft / 50.0f) * 50;
      Serial.print(shownFt);
      Serial.print("ft");
    }
  }
  Serial.print(" | rssi=");
  Serial.print(lastRssi);
  Serial.print("dBm snr=");
  Serial.print(lastSnr, 1);
  Serial.print("dB age=");
  if (linkAgeMs == UINT32_MAX) {
    Serial.print("never");
  } else {
    Serial.print(linkAgeMs);
    Serial.print("ms");
  }
  Serial.print(" | scale=");
  Serial.print(distanceScaleIndex / 10.0f, 1);
  Serial.print("x");
  Serial.print(" vibe=");
  Serial.print(intensityLabel(intensityMode));
  Serial.print(" mode=");
  Serial.print(feedbackLabel());
  if (lastBatteryPct >= 0) {
    Serial.print(" batt=");
    Serial.print(lastBatteryVoltage, 2);
    Serial.print("V/");
    Serial.print(lastBatteryPct);
    Serial.print("%");
  }
  Serial.println();
}

float readBatteryVoltage() {
  int mv = analogReadMilliVolts(ADC_BAT_PIN);
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
  float vbat = readBatteryVoltage();
  if (vbat <= 0.1f) {
    return;
  }
  int pct = batteryPercentFromVoltage(vbat);
  lastBatteryVoltage = vbat;
  if (!force && pct == lastBatteryPct) {
    return;
  }
  lastBatteryPct = pct;
}

void showHumnLogo() {
  if (Serial) {
    Serial.println("HUMN");
  }
}

void showSleepCat() {
  if (Serial) {
    Serial.println("Entering deep sleep.");
  }
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
  // Deep sleep is not light sleep mode; clear the flag.
  lightSleepMode = false;
  setDisplayOn(false);
  Serial.println("Wake with reset or power cycle.");
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
  if (Serial) {
    Serial.println("Status refresh:");
  }
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
    lightSleepMode = false;
    if (Serial) {
      Serial.println("Headless UI active.");
    }
    if (refresh) {
      refreshDisplay();
    }
  } else {
    if (Serial) {
      Serial.println("Headless UI idle.");
    }
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

  enum SimPhase : uint8_t { SIM_NONE_HOLD = 0, SIM_RAMP_IN = 1, SIM_INNER_HOLD = 2, SIM_RAMP_OUT = 3 };
  static uint8_t lastPhase = 255;

  SimPhase phase = SIM_NONE_HOLD;
  uint32_t phaseT = t;

  if (t < SIM_HOLD_OUTER_MS) {
    phase = SIM_NONE_HOLD;
    phaseT = t;
  } else if (t < SIM_HOLD_OUTER_MS + SIM_RAMP_MS) {
    phase = SIM_RAMP_IN;
    phaseT = t - SIM_HOLD_OUTER_MS;
  } else if (t < SIM_HOLD_OUTER_MS + SIM_RAMP_MS + SIM_HOLD_INNER_MS) {
    phase = SIM_INNER_HOLD;
    phaseT = t - (SIM_HOLD_OUTER_MS + SIM_RAMP_MS);
  } else {
    phase = SIM_RAMP_OUT;
    phaseT = t - (SIM_HOLD_OUTER_MS + SIM_RAMP_MS + SIM_HOLD_INNER_MS);
  }

  if ((uint8_t)phase != lastPhase) {
    lastPhase = (uint8_t)phase;
    if (phase == SIM_NONE_HOLD) {
      // Entering "None" state: clear averager and reset to far window.
      resetRssiAverager();
      currentHapticWindow = WIN_VERY_FAR;
      lastWindowDistanceFt = SIM_FAR_FT;
      if (hapticTransitionReq.pattern != 2) {
        hapticTransitionReq.pending = false;
      }
      lastStep = 19;
      smoothedDistanceFt = SIM_FAR_FT;
    }
  }

  if (phase == SIM_NONE_HOLD) {
    // No node nearby: suppress signal so vibration stops.
    lastSignalMs = now - (VIBE_TIMEOUT_MS + 1000);
    return;
  }

  float distFt = SIM_FAR_FT;
  if (phase == SIM_RAMP_IN) {
    float frac = (float)phaseT / (float)SIM_RAMP_MS;
    distFt = lerpFloat(SIM_FAR_FT, SIM_NEAR_FT, clamp01(frac));
  } else if (phase == SIM_INNER_HOLD) {
    distFt = SIM_NEAR_FT;
  } else { // SIM_RAMP_OUT
    float frac = (float)phaseT / (float)SIM_RAMP_MS;
    distFt = lerpFloat(SIM_NEAR_FT, SIM_FAR_FT, clamp01(frac));
  }

  if (distFt < 0.1f) distFt = 0.1f;
  // Provide a synthetic RSSI history so windowing can be tested in sim mode.
  int16_t pseudoRssi = estimateRssiFromDistanceFeet(distFt);
  lastRssi = pseudoRssi;
  smoothedRssi = (float)pseudoRssi;
  // Step is still used for the on-screen "distance" display and pulse smoothing.
  lastStep = estimateDistanceStep(pseudoRssi);
  lastSignalMs = now;

  smoothedDistanceFt = DIST_SMOOTH_ALPHA * distFt + (1.0f - DIST_SMOOTH_ALPHA) * smoothedDistanceFt;
  recordRssiSample(pseudoRssi, now);
  updateHapticWindowFromAvgRssi(now);
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

void runLoRaSelfTest() {
  Serial.println("LoRa self-test:");
  Serial.print("  Freq=");
  Serial.print(LORA_FREQ_MHZ, 1);
  Serial.print("MHz BW=");
  Serial.print(LORA_BW_KHZ, 1);
  Serial.print("kHz SF=");
  Serial.print(LORA_SF);
  Serial.print(" CR=4/");
  Serial.print(LORA_CR);
  Serial.print(" TX=");
  Serial.print(LORA_TX_POWER);
  Serial.println("dBm");

  String packet = "T";
  packet += String(deviceId, HEX);
  packet.toUpperCase();

  lora.setOutputPower(LORA_TX_POWER);
  lora.standby();
  uint32_t txStartMs = millis();
  int txState = lora.transmit(packet);
  uint32_t txMs = millis() - txStartMs;
  Serial.print("  TX blocking @ ");
  Serial.print(LORA_TX_POWER);
  Serial.print("dBm: ");
  if (txState == RADIOLIB_ERR_NONE) {
    Serial.print("OK, ");
    Serial.print(txMs);
    Serial.println("ms");
  } else {
    Serial.print("failed, RadioLib code ");
    Serial.println(txState);
  }
}

void configureLoRaRadio() {
  lora.setDio2AsRfSwitch();
  lora.setBandwidth(LORA_BW_KHZ);
  lora.setSpreadingFactor(LORA_SF);
  lora.setCodingRate(LORA_CR);
  lora.setOutputPower(LORA_TX_POWER);
  lora.setSyncWord(LORA_SYNC_WORD);
  lora.setCRC(true);
}

void recoverLoRaRadio(const char* reason, int state) {
  if (Serial) {
    Serial.print("LoRa recover after ");
    Serial.print(reason);
    Serial.print(": ");
    Serial.println(state);
  }
  loraInterruptEnabled = false;
  lora.reset();
  delay(10);
  int beginState = lora.begin(LORA_FREQ_MHZ);
  if (beginState != RADIOLIB_ERR_NONE && Serial) {
    Serial.print("LoRa recover begin failed: ");
    Serial.println(beginState);
  }
  configureLoRaRadio();
  lora.setDio1Action(setLoraFlag);
  lora.startReceive();
  loraRxFlag = false;
  loraInterruptEnabled = true;
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
    if (role == ROLE_PINGER) {
      nextPingMs = millis() + random(20, 80);
    }
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
  // Windowed scaling (resets within each distance window)
  uint8_t intensity = motorIntensityForStepWindowed(step);
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
  return vibrationIntervalForStepWindowed(step);
}

float vibrationOnRatioForStep(int step) {
  return vibrationOnRatioForStepWindowed(step);
}

void vibrationTask(void* parameter) {
  const TickType_t delayTicks = pdMS_TO_TICKS(10);
  bool transitionActive = false;
  uint32_t transitionStartMs = 0;
  uint32_t transitionEndMs = 0;
  uint8_t transitionFrom = 0;
  uint8_t transitionTo = 0;
  uint8_t transitionPattern = 0; // 0 = up, 1 = down
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

      // Window transition cue overrides normal vibration briefly.
      if (transitionActive) {
        if (now >= transitionEndMs) {
          drv.setRealtimeValue(0);
          vibeOn = false;
          transitionActive = false;
          nextVibeToggleMs = now + 20;
        } else {
          uint32_t elapsed = now - transitionStartMs;
          uint32_t dur = transitionEndMs - transitionStartMs;
          float t = (dur <= 1) ? 1.0f : (float)elapsed / (float)dur;
          t = clamp01(t);

          // Signature patterns:
          // - UP: 3 ascending chirps, then pulsed ramp-up (increasing frequency/duty)
          // - DOWN: 1 strong buzz, then sparse pulsed fade (decreasing frequency/duty)
          // - SCALE: obvious confirmation buzzes for the salvaged Rev1 switch
          uint8_t intensity = 0;

          if (transitionPattern == 0) {
            // Ramp-up "chirps" in first ~300ms
            if (elapsed < 320) {
              // 3 x (on 60ms, off 40ms)
              uint32_t slot = elapsed / 100;
              uint32_t inSlot = elapsed % 100;
              bool on = (inSlot < 60) && (slot < 3);
              if (on) {
                float p = (slot + 1) / 3.0f; // 0.33, 0.66, 1.0
                intensity = (uint8_t)round(lerpFloat(10.0f, (float)transitionTo, p * 0.65f));
              } else {
                intensity = 0;
              }
            } else {
              // After chirps: ease-in ramp with pulsed duty cycle
              float tt = (t - 0.20f) / 0.80f;
              tt = clamp01(tt);
              float ease = tt * tt * tt; // cubic ease-in
              uint8_t base = lerpU8(transitionFrom, transitionTo, ease);

              float freqHz = lerpFloat(8.0f, 18.0f, tt);     // faster as it ramps
              float duty = lerpFloat(0.25f, 0.60f, tt);      // more "present" as it ramps
              float timeSec = (float)elapsed / 1000.0f;
              float cycle = fmodf(timeSec * freqHz, 1.0f);
              bool gateOn = (cycle < duty);
              intensity = gateOn ? base : 0;
            }
          } else if (transitionPattern == 1) {
            // Ramp-down: strong buzz then sparse fade
            if (elapsed < 220) {
              // strong buzz start: 140ms on, 80ms off
              intensity = (elapsed < 140) ? transitionFrom : 0;
            } else {
              float ease = 1.0f - (1.0f - t) * (1.0f - t); // ease-out for the drop
              uint8_t base = lerpU8(transitionFrom, transitionTo, ease);

              float freqHz = lerpFloat(16.0f, 6.0f, t);    // slows down as it fades
              float duty = lerpFloat(0.35f, 0.12f, t);      // gets sparser
              float timeSec = (float)elapsed / 1000.0f;
              float cycle = fmodf(timeSec * freqHz, 1.0f);
              bool gateOn = (cycle < duty);
              intensity = gateOn ? base : 0;
            }
          } else {
            // Scale-change cue: tactile "ping" -- sharp attack, quick decay.
            float decay = (1.0f - t);
            float shaped = decay * decay;
            intensity = lerpU8(0, transitionTo, shaped);
          }

          drv.setRealtimeValue(intensity);
          vibeOn = (intensity != 0);
        }
        vTaskDelay(delayTicks);
        continue;
      }

      if (hapticTransitionReq.pending) {
        // Latch request to local state and clear pending quickly.
        transitionFrom = hapticTransitionReq.fromIntensity;
        transitionTo = hapticTransitionReq.toIntensity;
        transitionPattern = hapticTransitionReq.pattern;
        uint16_t dur = hapticTransitionReq.durationMs;
        hapticTransitionReq.pending = false;
        transitionStartMs = now;
        transitionEndMs = now + (dur == 0 ? 1 : dur);
        transitionActive = true;
        if (Serial && transitionPattern == 2) {
          Serial.print("Scale haptic cue started: intensity=");
          Serial.println(transitionTo);
        }
        // Cancel any scheduled pulsing edge so the transition is crisp.
        nextVibeToggleMs = now;
        motorStopMs = 0;
        vTaskDelay(delayTicks);
        continue;
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
  displayOn = true;
  lightSleepMode = false;
  lastUserInteractionMs = millis();

  // Initialize Serial over native USB CDC on the ESP32-C6.
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  while (!Serial && millis() < 5000) {
    delay(10);
  }
  delay(300);

  Serial.println("\n\n========================================");
  Serial.println("  Humn custom PCB - ESP32-C6 + SX1262");
  Serial.println("========================================\n");
  showHumnLogo();
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

  // Setup user button
  pinMode(USER_BTN_PIN, INPUT_PULLUP);
  pinMode(MODE_SWITCH_ADC_PIN, INPUT);

  // Setup battery ADC
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_BAT_PIN, ADC_11db);
  if (MODE_SWITCH_ANALOG_ENABLED) {
    analogSetPinAttenuation(MODE_SWITCH_ADC_PIN, ADC_11db);
  }
  applyModeSwitchScale(true);

  // Setup status LED
  if (LED_ENABLED && STATUS_LED_PIN >= 0) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  // Enable the DRV2605L and keep its external trigger input inactive.
  pinMode(HAPTIC_EN_PIN, OUTPUT);
  digitalWrite(HAPTIC_EN_PIN, HIGH);
  pinMode(HAPTIC_TRIG_PIN, OUTPUT);
  digitalWrite(HAPTIC_TRIG_PIN, LOW);
  delay(10);

  // Initialize LoRa radio
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  int state = lora.begin(LORA_FREQ_MHZ);
  configureLoRaRadio();

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("LoRa init OK.");
    runLoRaSelfTest();
  } else {
    Serial.print("LoRa init failed: ");
    Serial.println(state);
  }

  lora.setDio1Action(setLoraFlag);
  lora.startReceive();

  // Setup DRV2605L (I2C)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  drvOk = drv.begin();
  if (drvOk) {
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_REALTIME);
    drv.setRealtimeValue(0);
    Serial.println("DRV2605L init OK.");
    Serial.print("DRV2605L I2C pins: SDA=");
    Serial.print(I2C_SDA_PIN);
    Serial.print(" SCL=");
    Serial.println(I2C_SCL_PIN);
    Serial.println("Button cycles intensity: soft/medium/hard/max");
  }
  if (!drvOk) {
    Serial.println("DRV2605L init FAILED.");
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
      float distFt = estimateDistanceFeetFromRssi((int16_t)round(smoothedRssi));
      smoothedDistanceFt = DIST_SMOOTH_ALPHA * distFt + (1.0f - DIST_SMOOTH_ALPHA) * smoothedDistanceFt;
      int step = estimateDistanceStep((int16_t)round(smoothedRssi));
      rxCounter++;
      if (verboseTimingLogs && rxCounter % rxLogEvery == 0) {
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
      recordRssiSample(lastRssi, lastRxMs);
      updateHapticWindowFromAvgRssi(lastRxMs);

      // Avoid chatty serial updates on every RX to reduce jitter.

      uint32_t otherId = 0;
      if (parsePeerId(payload, otherId)) {
        updateRoleWithPeer(otherId);
      }

      // No immediate reply here. The custom boards use periodic beacons from
      // both sides to avoid SX1262 half-duplex TX/RX contention.

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
  if (nowMs - lastModeSwitchReadMs >= MODE_SWITCH_READ_MS) {
    lastModeSwitchReadMs = nowMs;
    applyModeSwitchScale(false);
  }

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
      } while (uiSetting == UI_PING || uiSetting == UI_SCALE ||
               (!SIMULATION_MODE_ENABLED && uiSetting == UI_SIM));

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
        if (!SIMULATION_MODE_ENABLED) {
          simulateDistance = false;
          updateSimDisplay();
          saveSettings();
          return;
        }
        simulateDistance = !simulateDistance;
        if (verboseUiLogs && Serial) {
          Serial.print("Sim mode: ");
          Serial.println(simulateDistance ? "on" : "off");
        }

        updateSimDisplay();
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

  // Warn if we're late sending a beacon.
  if (nextPingMs != 0 && now > nextPingMs + PING_LATE_WARN_MS) {
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
    resetRssiAverager();
    currentHapticWindow = WIN_VERY_FAR;
    lastWindowDistanceFt = SIM_FAR_FT;
    smoothedDistanceFt = SIM_FAR_FT;
    if (hapticTransitionReq.pattern != 2) {
      hapticTransitionReq.pending = false;
    }
    updateDistanceDisplay(lastStep);
  }

  if (now >= nextPingMs) {
    // In SEARCH mode, avoid pinging right after RX to reduce collisions
    if (role == ROLE_SEARCH && now - lastSignalMs < 400) {
      nextPingMs = now + effectivePingBaseMs + random(0, effectivePingJitterMs);
    } else {
      pingCounter++;

      String msg = buildPing();

      loraInterruptEnabled = false;
      uint32_t txStartMs = millis();
      int txState = lora.transmit(msg);
      uint32_t txDurMs = millis() - txStartMs;
      lora.startReceive();
      loraInterruptEnabled = true;

      // Avoid chatty serial updates on every TX to reduce jitter.
      flashStatusLed();

      if (txState != RADIOLIB_ERR_NONE) {
        if (Serial) {
          Serial.print("Ping TX failed: ");
          Serial.println(txState);
        }
        recoverLoRaRadio("beacon TX", txState);
      }

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
