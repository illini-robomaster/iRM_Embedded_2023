# STM32–Jetson UART Communication Protocol — Agent Reference

## Overview

This document summarizes the communication protocol between the **Nvidia Jetson** (vision computer) and the **STM32** (control board) in the iRM Vision system. Communication is over **UART (USB-TTL)** at **115200 baud**, 8N1. The design is **stateless and single-sided**: a response must never directly depend on data from a previous packet (e.g., sending `rel_pitch += 1` is undefined behavior).

**Key source files:**
- Protocol spec: `docs/comm_protocol.md`
- Jetson-side implementation: `Communication/communicator.py`
- Config constants: `config.py`
- STM32-side reference: [iRM_Embedded_2023 minipc example](https://github.com/illini-robomaster/iRM_Embedded_2023/tree/main/examples/minipc)

---

## Packet Structure

All multi-byte integers are **little-endian**. String/ASCII fields are NOT affected by endianness.

| Field         | Type / Content                     | Size    | Offset           |
|---------------|------------------------------------|---------|------------------|
| HEADER        | Fixed ASCII `'ST'` (0x53, 0x54)    | 2 bytes | 0                |
| SEQ_NUM       | `uint16_t` counter (wraps at 2^16) | 2 bytes | 2                |
| DATA_LEN      | `uint8_t` (currently always 0)     | 1 byte  | 4                |
| CMD_ID        | `uint8_t` command identifier       | 1 byte  | 5                |
| DATA          | Fixed-length struct (see below)    | n bytes | 6                |
| CRC_CHECKSUM  | `uint8_t` CRC-8 MAXIM_DOW          | 1 byte  | PACKET_LEN - 3   |
| TAIL          | Fixed ASCII `'ED'` (0x45, 0x44)    | 2 bytes | PACKET_LEN - 2   |

**Total packet size** = DATA length + 9 bytes (header/tail/checksum overhead).  
**Valid range**: 10–21 bytes.

### CRC Checksum

- Standard: **CRC-8 MAXIM_DOW**
- Computed over all bytes from HEADER up to (but NOT including) the CRC byte and TAIL.
- Python library: `crc.Calculator(crc.Crc8.MAXIM_DOW, optimized=True)`

---

## Command IDs and Data Structs

### CMD_ID = 0x00 — GIMBAL_CMD_ID (data: 10 bytes, packet: 19 bytes)

Controls gimbal movement (yaw/pitch).

| Field      | Type      | Size    | Offset (from DATA start) | Description                        |
|------------|-----------|---------|--------------------------|------------------------------------|
| rel_yaw    | float32   | 4 bytes | +0                       | **Angular offset** from camera center to target (radians) |
| rel_pitch  | float32   | 4 bytes | +4                       | **Angular offset** from camera center to target (radians) |
| mode       | uint8_t   | 1 byte  | +8                       | 0 = `'ST'` (Search Target), 1 = `'MY'` (Move Yoke) |
| debug_int  | uint8_t   | 1 byte  | +9                       | Debug integer (user-defined)       |

**Important Protocol Semantics:**
- `rel_yaw` and `rel_pitch` represent the **angular offset** between camera center and the detected target
- **Jetson calculates:** offset = (target_pixel - camera_center_pixel) * (FOV / image_size)
- **MCU should calculate:** target_position = current_gimbal_position + offset
- This allows MCU to handle absolute position control using its own sensors (IMU/encoders)
- Example: If target is 0.1 rad to the right of camera center, `rel_yaw = 0.1`

**Mode semantics:**
- **ST (0)**: AutoAim not engaged OR target not found. MCU should ignore yaw/pitch values and enter search mode (rotate to search for targets).
- **MY (1)**: Target found. MCU should read current gimbal position from sensors, add the offsets (rel_yaw, rel_pitch), and move gimbal to that target position. Filtering for latency compensation should be done on MCU side.

### CMD_ID = 0x01 — COLOR_CMD_ID (data: 1 byte, packet: 10 bytes)

Communicates team color.

| Field    | Type    | Size   | Offset | Description                     |
|----------|---------|--------|--------|---------------------------------|
| my_color | uint8_t | 1 byte | +0     | 0 = RED, 1 = BLUE              |

### CMD_ID = 0x02 — CHASSIS_CMD_ID (data: 12 bytes, packet: 21 bytes)

Controls chassis movement.

| Field | Type    | Size    | Offset | Description                              |
|-------|---------|---------|--------|------------------------------------------|
| vx    | float32 | 4 bytes | +0     | Velocity forward (chassis frame)         |
| vy    | float32 | 4 bytes | +4     | Velocity leftward (chassis frame)        |
| vw    | float32 | 4 bytes | +8     | Counterclockwise angular velocity        |

---

## Config Constants (from `config.py`)

```python
PACK_START = b'ST'
PACK_END   = b'ED'

GIMBAL_CMD_ID  = 0x00
COLOR_CMD_ID   = 0x01
CHASSIS_CMD_ID = 0x02

CMD_TO_LEN = {0x00: 10, 0x01: 1, 0x02: 12}  # data section lengths
HT_LEN = 9  # overhead: header(2) + seq(2) + data_len(1) + cmd_id(1) + crc(1) + tail(2)

GIMBAL_MODE = ['ST', 'MY']  # index 0 = Search Target, index 1 = Move Yoke

# Byte offsets
SEQNUM_OFFSET      = 2
DATA_LENGTH_OFFSET  = 4
CMD_ID_OFFSET       = 5
DATA_OFFSET         = 6
```

---

## Handshake Protocol

To ensure reliable communication between Jetson and MCU, a handshake mechanism is implemented on startup.

### Boot Sequence

**Important:** The system assumes the following boot order:
1. **MCU boots first** (fast boot, typically <1 second)
2. **MCU enters main loop** and begins listening for UART packets
3. **Jetson boots second** (Linux boot, typically 30-60 seconds)
4. **Jetson initiates handshake** once serial port is opened

This boot order is enforced by the hardware design (MCU is embedded hardware, Jetson is a full computer).

### Handshake Sequence

1. **Jetson initiates handshake:**
   - Sends GIMBAL packet with `debug_int = 0xFF` (255)
   - `rel_yaw = 0.0`, `rel_pitch = 0.0`, `mode = 'ST'`
   - Retries up to 3 times with 5-second timeout per attempt

2. **MCU acknowledges:**
   - Upon receiving GIMBAL packet with `debug_int = 0xFF`:
   - MCU should respond with GIMBAL packet with `debug_int = 0xFE` (254)
   - Include current gimbal position in `rel_yaw` and `rel_pitch`
   - **Note:** MCU should be ready to respond immediately (already in main loop)

3. **Jetson confirms:**
   - Waits up to 5 seconds for MCU acknowledgment (per retry)
   - If `debug_int = 0xFE` received, handshake successful
   - If timeout after 3 retries, prints warning but continues operation

### Implementation

**Jetson side:**
```python
communicator = UARTCommunicator(config)
communicator.start_listening()

# Perform handshake
if not communicator.perform_handshake(timeout=5.0):
    print("WARNING: Handshake failed")
```

**MCU side (pseudo-code):**
```cpp
// In packet receive handler
if (gimbal_data.debug_int == 0xFF) {
    // Handshake request received
    gimbal_data_t ack;
    ack.rel_yaw = current_gimbal_yaw;
    ack.rel_pitch = current_gimbal_pitch;
    ack.mode = 0;  // ST mode
    ack.debug_int = 0xFE;  // Handshake ACK

    minipc.Pack(GIMBAL_CMD_ID, &ack);
    minipc.Transmit();
}
```

### Debug Values

| Value | Meaning |
|-------|---------|
| 0xFF (255) | Handshake request from Jetson |
| 0xFE (254) | Handshake acknowledgment from MCU |
| 0x00-0xFD | Normal debug values (user-defined) |

---

## Jetson-Side API (`UARTCommunicator`)

### Initialization

```python
from Communication.communicator import UARTCommunicator
import config
uart = UARTCommunicator(config)
```

Serial port defaults to `/dev/ttyTHS0` at 115200 baud. Auto-detection scans for prefixes: `tty.usbmodem`, `ttyUSB`, `ttyACM`, `ttyTHS`.

### Sending Data

```python
# Gimbal command
uart.create_and_send_packet(config.GIMBAL_CMD_ID, {
    'rel_yaw': 0.5,        # float
    'rel_pitch': -0.2,     # float
    'mode': 'MY',          # 'ST' or 'MY'
    'debug_int': 0         # 0–255
})

# Color command
uart.create_and_send_packet(config.COLOR_CMD_ID, {
    'my_color': 'red',     # 'red' or 'blue'
    'enemy_color': 'blue'  # (derived, only my_color is packed)
})

# Chassis command
uart.create_and_send_packet(config.CHASSIS_CMD_ID, {
    'vx': 1.0,   # float
    'vy': 0.0,   # float
    'vw': 0.5    # float
})
```

### Receiving Data

```python
# Start background listener thread (recommended)
uart.start_listening()

# Poll current state (thread-safe deep copy)
state = uart.get_current_stm32_state()
# Returns dict with keys: my_color, enemy_color, rel_yaw, rel_pitch,
#                          debug_int, mode, vx, vy, vw

# Manual read loop (alternative)
uart.try_read_one()       # reads available bytes into circular buffer
uart.packet_search()      # parses buffer for valid packets
state = uart.get_current_stm32_state()
```

### Internal State Dict

```python
stm32_state_dict = {
    'my_color': 'red',     # team color
    'enemy_color': 'blue', # enemy color (derived)
    'rel_yaw': 0.0,        # last received yaw
    'rel_pitch': 0.0,      # last received pitch
    'debug_int': 0,        # debug value
    'mode': 'ST',          # gimbal mode
    'vx': 0.0,             # chassis vx
    'vy': 0.0,             # chassis vy
    'vw': 0.0,             # chassis vw
}
```

---

## Packet Construction (Byte-Level)

Example: building a GIMBAL packet (`CMD_ID=0x00`, data=10 bytes, total=19 bytes):

```
Offset  Field        Value (example)
------  -----------  ---------------
0–1     HEADER       0x53 0x54  ('ST')
2–3     SEQ_NUM      0x00 0x00  (little-endian uint16)
4       DATA_LEN     0x0A       (10, though currently set to encoded CMD_TO_LEN)
5       CMD_ID       0x00
6–9     rel_yaw      IEEE 754 float32 LE
10–13   rel_pitch    IEEE 754 float32 LE
14      mode         0x00 (ST) or 0x01 (MY)
15      debug_int    0x00–0xFF
16      CRC8         MAXIM_DOW over bytes 0–15
17–18   TAIL         0x45 0x44  ('ED')
```

---

## Known Issues / Design Notes

1. **Circular buffer minimum parse size**: Packets smaller than `STJ_MAX_PACKET_SIZE` (21 bytes) may not be parsed until additional bytes arrive in the buffer. For example, a GIMBAL packet (19 bytes) stays buffered until at least 21 bytes are present, requiring a second packet before the first is parsed.

2. **DATA_LEN field**: Currently hardcoded to `CMD_TO_LEN[cmd_id]` in `create_packet()`. The protocol doc states it is "set to 0" for future extension, but the implementation writes the actual data length. Parsing does not use this field — it derives length from `CMD_ID`.

3. **Thread safety**: `stm32_state_dict` is protected by a `threading.Lock`. Always use `get_current_stm32_state()` to read it (returns a deep copy).

4. **No ACK mechanism**: The protocol is fire-and-forget with no acknowledgment or retransmission. Reliability depends on CRC and the physical UART link.

5. **Gimbal position feedback required**: The MCU **must** send GIMBAL packets with current gimbal position (`rel_yaw`, `rel_pitch`) back to the Jetson for proper tracking. Without this feedback, the vision system cannot properly track targets. The Jetson includes a workaround that uses the last commanded position if no feedback is received, but this is not ideal for accurate tracking.

6. **Handshake recommendation**: Implement the handshake protocol (see above) to verify communication is established before starting operation. The MCU should respond to handshake requests (`debug_int=0xFF`) with an acknowledgment (`debug_int=0xFE`).

---

## Checklist: Adding a New Packet Type

### Jetson side (`iRM_Vision_2023`):
1. Add new `CMD_ID` constant and `CMD_TO_LEN` entry in `config.py`
2. Add data packing logic in `communicator.py` → `create_packet_data()`
3. Add data parsing logic in `communicator.py` → `parse_data()`
4. Update `stm32_state_dict` keys in `__init__()` and `update_current_state()`
5. If new packet length exceeds current bounds, update `STJ_MAX_PACKET_SIZE` / `STJ_MIN_PACKET_SIZE`

### STM32 side (`iRM_Embedded_2023`):
1. Add data struct in `minipc_protocol.h`
2. Update `status_data_t`, `CMD_ID` enum, and `CMD_TO_LEN[]` in `minipc_protocol.h`
3. Add packing helper in `minipc_protocol.cc` → `Pack()`
4. Add parsing in `minipc_protocol.cc` → `ParseData()`
5. If needed, update `MAX_PACKET_LENGTH` / `MIN_PACKET_LENGTH`
