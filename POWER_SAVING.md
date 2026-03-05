# Power Saving with CAD (Channel Activity Detection)

## Overview

This implementation uses a highly efficient power-saving technique that combines:

1. **ESP32 Light Sleep** (~800µA) - Quick wake capability
2. **SX1262 CAD Mode** (~2-3mA) - Monitors for LoRa signals without full RX
3. **DIO1 Interrupt** - Wakes ESP32 when signal detected

**Total idle power: ~4mA** (compared to ~55mA in active RX mode)

This is approximately **93% power reduction** while still being able to detect peers!

---

## How It Works

### Normal Operation (Active)
```
ESP32: Active (40mA)
SX1262: RX Mode (13mA)
Display: On (varies)
Total: ~55-80mA
```

### CAD Mode (Idle)
```
ESP32: Light Sleep (0.8mA)
SX1262: CAD Mode (2-3mA)
Display: Off
Total: ~4mA
```

### The Magic: Channel Activity Detection

The SX1262 has a special **CAD mode** where it:
- Listens for LoRa preambles using minimal power
- Takes only ~5ms to detect a signal
- Triggers DIO1 interrupt when preamble detected
- ESP32 wakes up and switches to full RX mode

This means you're **always listening** but using 93% less power!

---

## When Power Saving Activates

The system automatically enters CAD mode when:

1. Display is off (after 15s of inactivity)
2. Device has been idle for >5 seconds
3. No signals received in last 1 second
4. `powerSaveMode` is enabled (default: true)

The system wakes instantly when:
- LoRa signal detected (via CAD)
- Button pressed
- Timer expires (checks every 100ms)

---

## Configuration Parameters

### In `main.cpp`:

```cpp
// How often to check for signals in light sleep (100ms = good balance)
static const uint32_t CAD_CHECK_INTERVAL_MS = 100;

// Enter light sleep after 30s idle
static const uint32_t IDLE_LIGHT_SLEEP_MS = 30000;

// Enter deep sleep after 5 min idle (optional, not currently used)
static const uint32_t IDLE_DEEP_SLEEP_MS = 300000;

// Enable/disable power saving
static bool powerSaveMode = true;
```

### Disable all sleep behaviors (development / always-on)

You can disable *automatic* sleep behaviors (display idle-off + ESP32 light sleep/CAD) while keeping
manual deep sleep (5s hold) enabled by building with:

```ini
build_flags =
  -D HUMN_DISABLE_AUTO_SLEEP=1
```

If you also want to disable the manual 5-second hold deep sleep:

```ini
build_flags =
  -D HUMN_DISABLE_MANUAL_DEEP_SLEEP=1
```

### CAD Parameters (in `enterLightSleepWithCAD`):

RadioLib's `startChannelScan()` uses default CAD parameters recommended by Semtech (AN1200.48).

For custom CAD configuration, you can create a `ChannelScanConfig_t` structure:

```cpp
ChannelScanConfig_t cadConfig;
cadConfig.symbolNum = 2;   // 1-8 symbols to detect
cadConfig.detPeak = 24;    // Detection sensitivity peak (0-255)
cadConfig.detMin = 10;     // Minimum threshold (0-255)

lora.startChannelScan(cadConfig);
```

**Note:** The current implementation uses default parameters which work well for most cases.

---

## Tuning CAD Sensitivity

RadioLib uses default CAD parameters from Semtech AN1200.48 which work well for most cases.

### For custom tuning:

If you need to adjust sensitivity, create a custom CAD configuration:

```cpp
ChannelScanConfig_t cadConfig;
cadConfig.symbolNum = 2;   // Number of symbols (1-8)
cadConfig.detPeak = 24;    // Sensitivity (0-255)
cadConfig.detMin = 10;     // Min threshold (0-255)
```

### symbolNum (1-8)
- **Lower (1-2)**: Faster detection, less reliable
- **Higher (4-8)**: Slower detection, more reliable
- **Recommended: 2** - Good balance for SF6

### detPeak (0-255)
- **Lower (<20)**: Less sensitive, may miss weak signals
- **Higher (>30)**: More sensitive, may false-trigger on noise
- **Recommended: 24** - Good for your setup

### detMin (0-255)
- **Lower**: More sensitive
- **Higher**: Less sensitive to noise
- **Recommended: 10** - Conservative setting

### If you're missing packets:
- Increase `detPeak` to 28-32
- Increase `symbolNum` to 3 or 4
- Note: This uses slightly more power

### If you're getting false wakes:
- Decrease `detPeak` to 18-22
- Increase `detMin` to 12-15

---

## Battery Life Estimates

Assuming 1000mAh battery:

### Without CAD (always active RX):
- Average current: ~55mA
- Battery life: **~18 hours**

### With CAD (idle most of time):
- Active (10% time): 55mA × 0.1 = 5.5mA
- CAD (90% time): 4mA × 0.9 = 3.6mA
- Average: ~9mA
- Battery life: **~111 hours (~4.6 days)**

### If mostly idle (95% CAD):
- Active (5% time): 55mA × 0.05 = 2.75mA
- CAD (95% time): 4mA × 0.95 = 3.8mA
- Average: ~6.5mA
- Battery life: **~154 hours (~6.4 days)**

---

## Implementation Details

### Light Sleep vs Deep Sleep

**Light Sleep** (currently used):
- Wake time: ~1-3ms
- RAM preserved
- Peripherals can wake CPU
- Current: ~800µA (ESP32-S3)

**Deep Sleep** (available via button):
- Wake time: ~100-300ms
- RAM lost (full reboot)
- Only RTC wake sources
- Current: ~10-150µA

**Why Light Sleep for CAD?**
- Fast wake for packet reception
- Can resume immediately
- Good power savings with quick response

### How CAD Detection Works

1. **Idle Detection**: After 5s idle + 1s since last signal
2. **CAD Setup**: Configure SX1262 to monitor for preambles
3. **Sleep**: ESP32 enters light sleep
4. **Monitor**: SX1262 checks for signals (~2-3mA)
5. **Wake**: DIO1 interrupt fires on signal detection
6. **Resume**: Switch to full RX mode immediately

### Code Flow

```
loop() 
  └─> Check if idle + display off
      └─> enterLightSleepWithCAD(100ms)
          ├─> Configure CAD parameters
          ├─> lora.startChannelScan()
          ├─> esp_light_sleep_start()
          └─> Wake on:
              ├─> CAD detection (DIO1)
              ├─> Button press
              └─> Timer (100ms)
          └─> lora.startReceive() - back to normal RX
```

---

## Debugging CAD Mode

### Monitor Serial Output

You'll see:
```
CAD: Signal detected!  // When CAD triggers
```

### Check Wake Behavior

The device should:
1. Enter light sleep when idle
2. Wake instantly on button press
3. Wake and receive packets from peers
4. Resume normal operation immediately

### Common Issues

**Packets are missed:**
- CAD sensitivity too low
- Increase `detPeak` or `symbolNum`
- Reduce `CAD_CHECK_INTERVAL_MS` to 50ms

**False wakes (no packets):**
- CAD too sensitive or RF noise
- Decrease `detPeak`
- Increase `detMin`
- Check environment for RF interference

**Not entering sleep:**
- Display might still be on
- Check `displayOn` flag
- Verify `powerSaveMode = true`
- Check idle timers

---

## Advanced: Deep Sleep with CAD

For extremely long idle periods, you could:

1. Enter deep sleep after 5+ minutes
2. Wake periodically (every 1-5 seconds)
3. Run CAD check
4. If signal found, stay awake
5. If not, return to deep sleep

This would reduce average to ~0.5-1mA but adds latency.

**Not currently implemented** as light sleep gives excellent battery life with instant response.

---

## Testing Power Consumption

### Measure Current Draw

Use a multimeter in series with battery:

1. **Active RX** (display off): Should see ~40-50mA
2. **CAD mode** (idle): Should see ~3-5mA
3. **Deep sleep** (button press): Should see <1mA

### Test CAD Detection

1. Upload code to two devices
2. Let one go idle (wait 5s)
3. Send ping from other device
4. Idle device should wake and respond
5. Check serial: "CAD: Signal detected!"

---

## Recommendations

### For Maximum Battery Life:
```cpp
static const uint32_t CAD_CHECK_INTERVAL_MS = 200;  // Check every 200ms
```

### For Fastest Response:
```cpp
static const uint32_t CAD_CHECK_INTERVAL_MS = 50;   // Check every 50ms
```

### For Noisy RF Environment:
```cpp
lora.setCadParams(3, 20, 15);  // Less sensitive, more reliable
```

### For Weak Signal Detection:
```cpp
lora.setCadParams(4, 30, 8);   // More sensitive, may false-trigger
```

---

## Summary

This CAD implementation gives you:

- ✅ **93% power savings** when idle
- ✅ **Instant wake** on signal detection
- ✅ **No missed packets** (if tuned correctly)
- ✅ **Automatic operation** - no user intervention
- ✅ **~6 days battery life** on 1000mAh (vs ~18 hours without)

The key is the SX1262's hardware CAD capability - it's specifically designed for this use case!
