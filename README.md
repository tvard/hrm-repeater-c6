# HRM Repeater for ESP32-C6

A BLE Heart Rate Monitor repeater that connects to a real HRM sensor (like a chest strap) and re-broadcasts the data as its own BLE peripheral. This allows devices that can't directly reach your HRM to receive heart rate data via the ESP32-C6 acting as a bridge.

## What it does

```
[HRM Chest Strap] ──BLE──► [ESP32-C6] ──BLE──► [PC / Phone / Watch]
```

- **Central role**: Connects to your real HRM sensor and subscribes to heart rate notifications
- **Peripheral role**: Advertises as `ESP-<SensorName>` with standard Heart Rate Service (0x180D)
- Any app (Wahoo, Zwift, etc.) sees the ESP32 as a normal heart rate monitor

## Hardware

- **ESP32-C6-DevKitM-1** (RISC-V, BLE 5.0)
- Uses onboard **RGB LED** (GPIO8) for status indication
- Uses onboard **BOOT button** (GPIO9) for device switching

## LED Status Indicators

| LED Pattern | Color | Meaning |
|---|---|---|
| Fast blink (100ms) | 🔴 Red | No sensor connected (scanning) |
| Slow blink (500ms) | 🔵 Blue | Sensor connected, no receiver paired |
| Solid | 🟢 Green | Fully connected (sensor + receiver) |
| Triple flash | 🟡 Yellow | Error: lost connection / 0 bpm / 255 bpm |

## Button Controls (BOOT button - GPIO9)

| Action | Function |
|---|---|
| **Short press** (<2s) | Cycle to next predefined HRM target |
| **Long press** (>2s) | Toggle dynamic pair mode |

### Pairing Modes

- **Preset mode** (default): Connects only to devices matching names in `hrm_targets[]`
- **Dynamic pair mode**: Connects to any device advertising Heart Rate Service UUID, or matching any known target name

## Configuration

Edit the `hrm_targets[]` array in `main/main.c` to add your HRM devices:

```c
static const char *hrm_targets[] = {
    "H808S",       /* COOSPO */
    "16821-49",    /* MAGENE */
};
```

Short-press the BOOT button to cycle between these targets.

## Building & Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Build, flash, and monitor
pio run -t upload -t monitor
```

## Monitor Output

When running, the serial monitor shows:

```
I (1234) HRM_REPEATER: [16821-49 → ESP-16821-49] HR: 72 bpm
```

Format: `[sensor_name → advertised_name] HR: X bpm`

## Technical Details

- **Framework**: ESP-IDF 5.5.0 via PlatformIO (espressif32 6.12.0)
- **BLE Stack**: NimBLE (only option for ESP32-C6; Bluedroid not supported)
- **BLE Roles**: Simultaneous Central + Peripheral + Observer + Broadcaster
- **LED Driver**: RMT peripheral driving WS2812-compatible RGB LED
- **Heart Rate Service**: Standard BLE HRS (UUID 0x180D) with Notify + Read

## License

MIT
