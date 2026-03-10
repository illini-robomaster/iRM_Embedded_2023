# Engineer 2026 — Arm & Chassis Controller

**Target MCU:** DM_MC_02 board (STM32H723VGT)  
**Binary:** `vehicle_engineer2026_mc02`  
**License:** GNU GPL v3

---

## Overview

This firmware runs the complete mechatronic system for the 2026 RoboMaster Engineer robot.
It combines a **6-DOF robotic arm** (with gripper) and a **hybrid chassis** (two swerve
modules at the front, two fixed omni-wheels at the rear, and a vertical lift axis) into a
single 200 Hz real-time control loop.

The DM_MC_02 STM32H723 board is the only MCU.  An OrangePi companion computer
connects over a hardware UART and commands the arm joints via a simple ASCII CSV protocol.
The chassis is driven by a human operator through a DJI DBUS remote controller.

---

## Hardware Layout

### MCU Peripherals

| Peripheral | Function |
|---|---|
| `hfdcan1` | Chassis CAN bus (all chassis motors) |
| `hfdcan2` | Arm CAN bus (all arm motors) |
| `huart5`  | DJI DBUS remote controller |
| `huart7`  | Debug serial (ST-Link VCP → `/dev/ttyACM0` @ 921600 baud) |
| `huart10` | OrangePi arm-command link @ 115200 8N1 |

### Chassis Motors (`hfdcan1`)

| Role | Motor | CAN IDs (master/cmd) | Control Mode |
|---|---|---|---|
| Rear-left omni  | DM3519   | `0x20` / `0x21` | VEL (velocity) |
| Rear-right omni | DM3519   | `0x22` / `0x23` | VEL (velocity) |
| Lift            | DMJ10010 | `0x30` / `0x31` | FORCE_POS |
| Front-left drive  | M3508  | `0x202`         | Current (via velocity PID) |
| Front-right drive | M3508  | `0x201`         | Current (via velocity PID) |
| Front-left steer  | M6020  | `0x206`         | Omega PID (via `Steering6020`) |
| Front-right steer | M6020  | `0x205`         | Omega PID (via `Steering6020`) |

### Arm Motors (`hfdcan2`)

| Joint | Motor | CAN IDs (master/cmd) | Control Mode |
|---|---|---|---|
| J1 | Motor4310 (DM4310)     | `0x10` / `0x11` | POS_VEL |
| J2 | MotorDMJ10010 (10010L) | `0x12` / `0x13` | FORCE_POS |
| J3 | MotorDMJ10010 (10010L) | `0x14` / `0x15` | FORCE_POS |
| J4 | Motor4310 (DM4310)     | `0x16` / `0x17` | POS_VEL |
| J5 | Motor4310 (DM4310)     | `0x18` / `0x19` | POS_VEL |
| J6 | MotorDMJ3507 (DM3507)  | `0x00` / `0x01` | POS_VEL |
| Gripper | Motor2006         | RX `0x206`      | Open-loop → stall-hold |

> **Note:** All CAN IDs above must match what is programmed in the DAMIAO
> configuration tool per motor.  The placeholder IDs marked with `TODO` in the
> source must be verified and updated before first power-on.

---

## System Behavior

### Boot Sequence

```
Power on
  │
  ├─ RM_RTOS_Init()
  │     ├─ Setup debug UART (huart7)
  │     ├─ Create hfdcan1 CAN bus
  │     ├─ Create DBUS receiver (huart5)
  │     ├─ Create all chassis motor objects
  │     └─ ArmInit(): create hfdcan2 + huart10 + all arm motor objects
  │
  └─ RM_RTOS_Default_Task()  [single RTOS thread, 200 Hz]
        ├─ Calibration mode (swr DOWN — see §Calibration below)
        ├─ checkAllMotorsConnected()  ← blocks until every motor sends ≥1 CAN frame
        └─ main control loop (osDelay(5) → 200 Hz)
```

The arm motors are **not** enabled at boot.  They are enabled automatically
the first time `ArmUpdate()` receives a valid UART frame from the OrangePi.
After that, a 1-second watchdog ensures motors are disabled if the OrangePi
link goes silent.

### Main Control Loop (200 Hz, 5 ms tick)

Each tick executes in this order:

1. **DBUS kill-switch check** — if `swr == DOWN`, disable all chassis DM motors,
   zero M3508/M6020 outputs, and skip the rest of the tick.
2. **Chassis enable** — on the first tick with `swr != DOWN`, enable RL, RR,
   and lift motors (blocking, with per-motor print statements).
3. **Remote-controller input** — decode `ch0–ch2` into `vx`, `vy`, `vw`
   (clamped to ±1.5 m/s and ±2.0 rad/s).
4. **Rear omni kinematics** — compute RL/RR rotor speed commands to achieve
   `vx` and `vw` using the fixed-rear-longitudinal constraint.
5. **Front swerve kinematics** — compute each module's target drive speed and
   steering angle; apply 90° swerve optimisation to minimise rotation travel.
6. **Lift control** — `swr MID` → position 0 rad (down); `swr UP` → −1 rad
   (raised).  A soft-down threshold releases the current demand once the motor
   reaches its low position to prevent fighting gravity.
7. **Transmit chassis CAN frames** — one burst for DM3519, M3508, M6020, and
   DMJ10010.
8. **`ArmUpdate()`** — handles the complete arm control pipeline (see below).
9. **Debug print** (~1 Hz) — logs `vx`, `vy`, `vw`, rear speeds, and front
   swerve targets/measured angles.

### Arm Control Pipeline (`ArmUpdate`, called every 5 ms)

| Step | Description |
|---|---|
| **1 UART RX** | Accumulate bytes from `huart10` into a line buffer; parse `"j1,j2,j3,j4,j5,j6\n"` when `'\n'` is received.  Stores parsed values as `cmd_target_deg[6]` and resets the watchdog timestamp. |
| **2 Auto-enable** | On the first valid RX frame after power-up, calls `ArmEnable()`, which sends a CAN enable command to each DM motor and sets `arm_enabled = true`. |
| **3 Watchdog** | If no valid frame has arrived within 1000 ms, calls `MotorDisable()` on all DM joints, sends zero current to the gripper, and clears `arm_enabled`.  The arm stays disabled until the next valid UART frame. |
| **4 Setpoint ramp** | For each of the 6 joints, the ramped setpoint (`ramp_target_deg`) is advanced toward `cmd_target_deg` by at most `MAX_VEL_DEG[i] × 0.005 s` per tick, effectively rate-limiting acceleration to match MoveIt `joint_limits.yaml`. |
| **5 Motor commands** | Converts ramped degrees to radians and calls each motor's `SetOutput()`: J1/J4/J5 → `POS_VEL`; J2/J3 → `FORCE_POS` at 50% current; J6 → `POS_VEL`. |
| **6 Gripper state machine** | Starts in `CLOSING`: drives a constant `+2000` raw-unit current until the measured current exceeds `GRIP_STALL_THRESH (4000)`.  Then transitions to `HOLDING`: a `ConstrainedPID` holds the encoder position at the stall angle. |
| **7 CAN TX** | Transmits one batch per motor family: `Motor4310 × 3`, `MotorDMJ10010 × 2`, `MotorDMJ3507 × 1`, `Motor2006 × 1`. |
| **8 UART TX** | Sends `"j1,j2,j3,j4,j5,j6\n"` (encoder angles in degrees, 3 decimal places) back to the OrangePi.  Runs at 200 Hz, giving the OrangePi full-rate feedback even though it commands at 50 Hz. |

### DBUS Remote-Controller Mapping

| Channel | Stick | Direction | Effect |
|---|---|---|---|
| `ch0` | Right horizontal | Right | Lateral motion (−vy) |
| `ch1` | Right vertical   | Forward | Forward motion (+vx) |
| `ch2` | Left horizontal  | Left | CCW rotation (+vw) |
| `swr` | Right switch     | DOWN | **Kill switch** — disables all motors |
| `swr` | Right switch     | MID  | Enabled, chassis low |
| `swr` | Right switch     | UP   | Enabled, chassis raised |

### OrangePi UART Protocol

| Direction | Format | Rate | Notes |
|---|---|---|---|
| OrangePi → MCU | `"j1,j2,j3,j4,j5,j6\n"` | 50 Hz | Target joint angles in **degrees** |
| MCU → OrangePi | `"j1,j2,j3,j4,j5,j6\n"` | 200 Hz | Measured joint angles in **degrees** |

- Physical UART: `huart10` → typically wired to `/dev/ttyS4` on the OrangePi.
- Settings: 115200 baud, 8N1.
- A line with anything other than exactly 6 parsed floats is silently discarded.

---

## Getting Started

### Prerequisites

- **ARM cross-compiler** — `arm-none-eabi-gcc` (≥ 10.x).  Install and add its
  `bin/` directory to `PATH`:
  ```sh
  # macOS
  brew install --cask gcc-arm-embedded
  # Arch Linux
  sudo pacman -S arm-none-eabi-gcc
  # Ubuntu/Debian
  sudo apt install gcc-arm-none-eabi
  ```
- **CMake** ≥ 3.16
- **make** (or Ninja)
- **st-flash** — for flashing over ST-Link:
  ```sh
  brew install stlink          # macOS
  sudo apt install stlink-tools # Ubuntu
  ```

### Build

```sh
# From the repository root
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc) vehicle_engineer2026_mc02
```

The resulting ELF/binary will be in `build/vehicles/Engineer2026/`.

Change `Release` to `Debug` or `RelWithDebInfo` to enable GDB symbols.

### Flash

Connect an ST-Link to the DM_MC_02 board's SWD header, then from inside the
`build/` directory:

```sh
make flash-vehicle_engineer2026_mc02
```

Alternatively, use `st-flash` directly:

```sh
st-flash write vehicles/Engineer2026/vehicle_engineer2026_mc02.bin 0x08000000
```

### Debug Serial

Connect a USB-to-UART adapter (or ST-Link VCP) to `huart7` and open a
terminal at **921600 baud**.  The firmware prints:

- **Calibration offsets** while `swr` is DOWN.
- **Motor connection status** during startup polling.
- **Enable/disable events** for each motor.
- **Watchdog events** if the OrangePi link drops.
- **1 Hz telemetry** once running (velocities, swerve angles).

---

## Calibration Procedure (First Power-On)

The front swerve steering modules need their zero-offset angles measured before
the firmware can steer correctly.

1. With `swr DOWN` (robot is safe to handle), power the robot.
2. Manually rotate both front swerve modules until their wheels point **straight
   forward** (positive X direction).
3. Read the printed offsets from the debug serial, e.g.:
   ```
   FL_STEER_OFFSET = 2.8370f;  FR_STEER_OFFSET = 4.3890f;
   ```
4. Copy these values into the constants at the top of [main_mc02.cc](main_mc02.cc):
   ```cpp
   static const float FL_STEER_OFFSET = 2.8370f;
   static const float FR_STEER_OFFSET = 4.3890f;
   ```
5. Rebuild and reflash.

---

## Known TODOs / Tuning Required

| Item | Location | Description |
|---|---|---|
| **Arm CAN IDs** | `arm_mc02.cc` top constants | Verify `J1_MASTER_ID` … `J6_CAN_ID` match your DAMIAO-tool configuration. |
| **Gripper direction** | `arm_mc02.cc` `GRIP_CLOSE_CURRENT` | Confirm positive current closes (not opens) and negate if needed. |
| **Gripper stall threshold** | `arm_mc02.cc` `GRIP_STALL_THRESH` | Start high (`4000`) and reduce until stall is reliably detected. |
| **Drive PID gains** | `main_mc02.cc` `DRIVE_KP/KI/KD` | Tune `DRIVE_KP` (default `40`) and `DRIVE_KD` (default `5`) on the bench. |
| **Chassis CAN IDs** | `main_mc02.cc` M3508/M6020 constructors | Replace `0x201–0x206` placeholders with your actual ESC IDs. |
| **DBUS sign conventions** | `main_mc02.cc` `vx/vy/vw` block | Verify the robot moves in the expected direction for each stick. |

---

## Source File Map

| File | Responsibility |
|---|---|
| [main_mc02.cc](main_mc02.cc) | `RM_RTOS_Init` + `RM_RTOS_Default_Task`: chassis initialisation, DBUS decoding, kinematics, CAN transmit, and the 200 Hz loop that calls `ArmUpdate()`. |
| [arm_mc02.cc](arm_mc02.cc)   | `ArmInit`, `checkAllMotorsConnected`, `ArmEnable`, `ArmUpdate`: all arm-side logic (UART parse, watchdog, ramp, motor commands, gripper state machine). |
| [arm_mc02.h](arm_mc02.h)     | Public interface for the arm module (four functions + motor type forward declarations). |
| [CMakeLists.txt](CMakeLists.txt) | Declares the `vehicle_engineer2026_mc02` CMake target, links `Steering6020` from the Engineer chassis vehicle. |
