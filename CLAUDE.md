# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino firmware project for the **QT Py ESP32-S3** board that implements a WiFi-enabled audio DSP console. The device acts as a web server providing a REST API for controlling audio processing parameters including EQ, crossovers, limiters, generators, and sequencer functionality.

**Hardware:** Adafruit QT Py ESP32-S3 with onboard NeoPixel LED

**Target Board:** QT Py ESP32-S3 (board package must define `NEOPIXEL_PIN`)

## Development Commands

### Building and Uploading

This is an Arduino sketch (`.ino` file). Use Arduino IDE or Arduino CLI:

```bash
# Arduino CLI - compile
arduino-cli compile --fqbn esp32:esp32:adafruit_qtpy_esp32s3 console-esp32.ino

# Arduino CLI - upload
arduino-cli upload -p COM3 --fqbn esp32:esp32:adafruit_qtpy_esp32s3 console-esp32.ino

# PlatformIO (if using)
pio run --target upload
```

### Serial Monitor

```bash
# Arduino CLI
arduino-cli monitor -p COM3 -c baudrate=115200

# PlatformIO
pio device monitor --baud 115200
```

## Architecture

### WiFi Connection Strategy

The device implements **smart AP selection** (lines 35-118):
- Scans for all APs with matching SSID
- Selects the strongest RSSI (signal strength)
- Connects to specific BSSID + channel for fastest roaming
- Uses WiFi STA mode so clients can reach it over LAN

Hardcoded credentials are at lines 14-15. These should be modified for deployment.

### State Management

**Core State Structure** (lines 124-194):
- `Device` struct contains all audio processing parameters
- 4-channel output system (each with independent routing, filtering, limiting)
- Input path: 15-band GEQ + parametric EQ
- Tools: sine/sweep/pink generators + 3-channel sequencer

**Persistence** via ESP32 NVS (Non-Volatile Storage):
- Namespace: "whir" (line 22)
- Working state auto-saved on parameter changes
- 16 preset slots (preset_0 through preset_15)
- Lock code stored persistently

**Master Volume Ramping** (lines 196-199, 738-748):
- Smooth interpolation prevents pops/clicks
- `masterTarget` set by API, `dev.master` ramps toward it
- Rate: 0→100% in ~1 second (configurable via `MASTER_RATE_PER_SEC`)

### JSON Serialization

**State → JSON** (line 213): `stateToJson()` generates full device state
**JSON → State** (lines 285-366): `loadWorking()` deserializes with clamping/validation

All numeric values are clamped to safe ranges during deserialization to prevent invalid states.

### HTTP API

**Web Server:** Port 80 (line 18)
**CORS:** Enabled for all origins (lines 673-676)

**Endpoints:**
- `GET /api/status` - Quick status (uptime, master, battery, lock state)
- `GET /api/state` - Full device state (all channels, all parameters)
- `POST /api/master` - Set master volume target (smooth ramp)
- `POST /api/input/geq` - 15-band GEQ update
- `POST /api/input/peq` - Input parametric EQ
- `POST /api/output` - Per-channel settings (HPF/LPF/PEQ/limiter/gain/mute/delay/invert)
- `POST /api/gen` - Signal generators (sine/sweep/pink noise)
- `POST /api/seq` - Sequencer control (S1/S2/S3 + interval)
- `POST /api/battery` - Battery voltage update (display-only)
- `POST /api/lock` - Lock/unlock device with 6-digit PIN
- `POST /api/preset/save` - Save current state to slot (0-15)
- `POST /api/preset/load` - Load preset from slot
- `POST /api/preset/copy` - Copy one preset slot to another

**Authentication:** If device is locked (`dev.locked=true`), most endpoints require `code` query parameter matching 6-digit `dev.lockCode` (lines 441-450).

### Visual Feedback

**NeoPixel LED** (line 11):
- Green flash (0,32,0): Successful parameter update (line 484, 501, etc.)
- Yellow flash (32,32,0): Lock/unlock operations (line 629)
- Duration: 80ms on, then off
- Helper: `ack(r,g,b,ms)` (lines 27-33)

### Main Loop Tasks

**loop()** performs two operations (lines 733-758):
1. HTTP request handling (`server.handleClient()`)
2. Master volume ramping (smooth interpolation every cycle)
3. Sequencer tick logging (when S1/S2/S3 enabled)

No actual DSP processing occurs in this firmware - this is a control plane only. The device likely interfaces with external DSP hardware (not shown in code).

## Important Constants

- **NVS Namespace:** "whir"
- **Max Presets:** 16 (slots 0-15)
- **GEQ Bands:** 15 (±12 dB each)
- **Output Channels:** 4
- **Master Ramp Rate:** 1.0 per second (0→100% in 1s)
- **Lock Code:** 6 digits (000000-999999)

## Typical Parameter Ranges

- **Master/Gain:** 0-1 (0-100%)
- **GEQ/PEQ Gain:** -12 to +12 dB
- **Frequency:** 10 to 22000 Hz
- **Q Factor:** 0.4 to 10.0
- **Limiter Threshold:** -24 to 0 dB
- **Limiter Attack:** 0.1 to 100 ms
- **Limiter Release:** 1 to 1600 ms
- **Delay:** 0 to 8 ms
- **Output Gain:** -45 to +15 dB

## Dependencies

**Arduino Libraries** (lines 1-6):
- `WiFi.h` - ESP32 WiFi stack
- `WebServer.h` - HTTP server
- `ArduinoJson.h` - JSON serialization (StaticJsonDocument)
- `Preferences.h` - NVS storage wrapper
- `Adafruit_NeoPixel.h` - LED control
- `math.h` - Math utilities (fabsf, etc.)

## Code Style Notes

- All clamping uses helper functions `clampf()` and `clampu8()` (lines 25-26)
- State changes always followed by `saveWorking()` to persist
- API handlers use consistent pattern: auth check → validate → update → ack LED → save → respond
- Serial logging uses descriptive prefixes: `[API]`, `[SEQ]`, `[RAMP]`

## Modifying WiFi Credentials

Update lines 14-15:
```cpp
const char* SSID = "YourNetworkName";
const char* PASS = "YourPassword";
```

Then recompile and upload.
