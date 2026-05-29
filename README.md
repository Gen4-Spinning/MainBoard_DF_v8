# MainBoard DF v8 — 

**Platform:** STM32G431CBTx (ARM Cortex-M4, 170 MHz)  
**Machine:** Draw Frame (DF)  
**Motors:** 3 — Front Roller (FR), Back Roller (BR), Creel  
**IDE:** STM32CubeIDE | **CAN:** FDCAN1 @ 1 Mbit/s | **BT:** USART1 @ 115,200 bps

---

## How It Works

### Overview

The MainBoard DF v8 is the master controller for a textile Draw Frame machine. It calculates motor RPM targets from operator settings, commands 3 motor drives over CAN, monitors sensors, and communicates machine status to a mobile app over Bluetooth.

```
  [App] <──BT──> [MainBoard STM32] <──CAN──> [FR Motor Drive]
                                   <──CAN──> [BR Motor Drive]
                                   <──CAN──> [Creel Motor Drive]
                   │
                   ├── EEPROM (settings)
                   ├── MCP23017 (sensors + tower lamp)
                   └── USART2 (debug log)
```

---

### Startup Sequence

1. EEPROM is read → settings loaded into `msp` struct
2. Settings validated → defaults loaded if invalid
3. `CalculateMachineParameters()` derives RPM targets for all 3 motors
4. State machine enters `IDLE_STATE`
5. Amber tower lamp turns ON — machine ready

---

### State Machine

The firmware runs a state machine. Each state has its own `while(1)` loop and exits only on a transition event.

| State | What Happens |
|---|---|
| `IDLE_STATE` | Waiting for input. Yellow = inch jog. Green = start run. |
| `RUN_STATE` | All 3 motors running. Monitors sensors, length counter, pause/stop events. |
| `FINISHED_STATE` | Target output length reached. Motors ramp down. |
| `ERROR_STATE` | Fault active. SMPS cut. Red lamp ON. Red button to recover. |
| `SETTINGS_STATE` | Receives new settings from app. Returns to calling state. |
| `DIAGNOSTICS_STATE` | Single-motor test run from app. |
| `DEBUG_STATE` | Manual motor commands via BT console. |

---

### Motor Control Flow

Settings from EEPROM → `CalculateMachineParameters()` → RPM targets sent over CAN

```
delivery_mMin  ──→  FR_MotorRPM    (FR circumference + gear ratio 1.0)
draft ratio    ──→  BR_MotorRPM    (FR speed ÷ draft × gear ratio 3.07)
creelFactor    ──→  Creel_MotorRPM (creel pulley ratio 0.722 × tensionFactor)
```

CAN commands issued to drives:

| Command | Effect |
|---|---|
| `START` | Ramp up to target RPM |
| `RAMPDOWN_STOP` | Controlled deceleration to stop |
| `EMERGENCY_STOP` | Immediate cut — no ramp |
| `CHANGE_RPM` | Update speed while running |
| `RESUME` | Resume from paused |
| `RESET` | Clear motor fault |

After every multi-motor command, TIM17 (15 ms) waits for ACK frames from all drives. Missing ACK → `SYS_ACK_ERROR` → `ERROR_STATE`.

---

### Sensors

| Sensor | Interface | Action on Trigger |
|---|---|---|
| Creel sliver cut | MCP23017 (I2C GPIO expander) | Pause machine, notify app `BT_PAUSE_CREEL_SLIVER_CUT` |
| Lapping sensor | MCP23017 | Pause machine, notify app `BT_PAUSE_LAPPING` |

Both sensors use software hysteresis (100 ms TIM7 ticks) to avoid false triggers. Active only when rotary switch is ON during `RUN_STATE`.

---

### Bluetooth Communication

Framed TLV protocol over USART1. Frame delimiter: `0x7E`.

Key message flows:

```
App ──[0x01 SETTINGS_FROM_APP]──→ MC    Send new delivery speed / draft / length etc.
App ←─[0x06 MC_STATE]────────── MC     Machine status every 500 ms
App ──[0x04 DIAGNOSTICS]────────→ MC    Test a motor drive
App ──[0x0A RESET_LENGTH]───────→ MC    Reset output length counter
App ──[0x0F PID_NEW_VALS]───────→ MC    Write PID gains to a motor drive
```

Settings are stored in EEPROM immediately on receipt. Motor RPMs are recalculated on every settings change.

---

### EEPROM Settings

| Parameter | Address | Default |
|---|---|---|
| `delivery_mMin` | `0x02` | 80 m/min |
| `draft` | `0x06` | 8.0 |
| `lengthLimit_m` | `0x0A` | 100 m |
| `rampUpTime` | `0x0C` | 6 s |
| `rampDownTime` | `0x0E` | 6 s |
| `creelTensionFactor` | `0x12` | 1.0 |

---

### Error Handling

Any module sets `ME.ErrorFlag = 1` and calls `ME_addErrors()`. The active state detects this and transitions to `ERROR_STATE`.

| Error | Code | Source |
|---|---|---|
| CAN bus lost to a motor | 98 | System |
| ACK timeout (15 ms) | 97 | System |
| SMPS feedback lost | 96 | System |
| Motor over-current/voltage/temp | 1–7 | Motor drive (CAN) |
| EEPROM read/write failure | 8–9 | System |

On error: SMPS cut → Red lamp → BT stop broadcast every 500 ms → **Red button** to recover.

---

### CAN Health Monitor (SysObserver)

TIM16 fires every 1 s. For each active motor, checks if new CAN frames were received since last tick. No frames → `SYS_CAN_CUT_ERROR`. Observers enabled only while motors are running.

---

### Tower Lamp & Buttons

| State | Lamp |
|---|---|
| IDLE | Amber ON |
| RUN | Green ON |
| PAUSED | Amber ON |
| FINISHED | Red + Green + Amber ON |
| ERROR | Red ON |
| Stop/complete | Buzzer 1 s |

| Button | Function |
|---|---|
| RED | Emergency stop / exit IDLE / clear ERROR |
| GREEN | Start run / resume after pause |
| YELLOW | Manual pause / inch jog in IDLE |
| ROTARY SW | Enable/disable sliver sensor monitoring |

---

### Timers Quick Reference

| Timer | Period | Purpose |
|---|---|---|
| TIM7 | 100 ms | BT status TX (5 ticks = 500 ms), sensor monitor, length counter |
| TIM16 | 1 s | CAN health check |
| TIM17 | 15 ms one-shot | ACK timeout after motor command |
| TIM15 | 250 ms | Diagnostic data stream to app |

---

### Build & Flash

```bash
# Build in STM32CubeIDE
File → Import → Existing Projects into Workspace
Build Config: Release
Ctrl+B → Output: Release/MainBoard_DF_v8.elf

# Flash via STM32CubeProgrammer
Load: MainBoard_DF_v8.elf  →  Program & Verify
```

Debug UART: USART2 @ **2,000,000 bps** — connect serial monitor to view logs.

> ⚠️ Do not regenerate the `.ioc` file without engineering review.
