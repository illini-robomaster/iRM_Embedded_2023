# MCU Developer Guide - Jetson Communication Protocol

## Overview

This document provides a complete guide for implementing the Jetson-MCU communication protocol on the STM32 side. It covers packet handling, handshake implementation, gimbal feedback, and testing procedures.

**Target Audience:** Embedded developers working on the STM32/MCU firmware

**Related Documents:**
- [minipc_protocol.md](minipc_protocol.md) - Complete protocol specification
- [HANDSHAKE_IMPLEMENTATION.md](HANDSHAKE_IMPLEMENTATION.md) - Handshake details

---

## System Architecture

### Communication Flow

```
┌─────────────────────────────────────────────────────────────┐
│                         JETSON                              │
│  ┌──────────┐    ┌──────────┐    ┌─────────────┐          │
│  │ Camera   │───>│ Vision   │───>│ UARTComm    │          │
│  │ (MDVS)   │    │ Detection│    │             │          │
│  └──────────┘    └──────────┘    └──────┬──────┘          │
│                                          │                  │
└──────────────────────────────────────────┼──────────────────┘
                                           │ UART
                                           │ 115200 baud
                                           │ 8N1
┌──────────────────────────────────────────┼──────────────────┐
│                                          │                  │
│  ┌─────────────┐    ┌──────────┐    ┌───▼──────┐          │
│  │ Gimbal      │<───│ Control  │<───│ UART RX  │          │
│  │ Motors      │    │ Loop     │    │          │          │
│  └─────────────┘    └────┬─────┘    └──────────┘          │
│                          │                                  │
│  ┌─────────────┐    ┌────▼─────┐    ┌──────────┐          │
│  │ IMU/Encoder │───>│ Feedback │───>│ UART TX  │          │
│  │             │    │          │    │          │          │
│  └─────────────┘    └──────────┘    └──────────┘          │
│                         MCU (STM32)                         │
└─────────────────────────────────────────────────────────────┘
```

### Boot Sequence

**Critical:** MCU boots first, then Jetson boots second.

```
Time 0s:     Power On
Time 0-1s:   MCU boots, enters main loop
Time 1-60s:  MCU waiting, Jetson booting (Linux)
Time 60s:    Jetson opens serial port
Time 60.1s:  Jetson sends handshake request
Time 60.2s:  MCU responds to handshake
Time 60.3s:  Normal operation begins
```

**Key Point:** When Jetson initiates handshake, MCU is already running and should respond immediately.

---

## Packet Format

All multi-byte integers are **little-endian**.

### Packet Structure

```
Byte Offset | Field        | Size    | Type    | Description
------------|--------------|---------|---------|---------------------------
0-1         | HEADER       | 2 bytes | ASCII   | 'ST' (0x53 0x54)
2-3         | SEQ_NUM      | 2 bytes | uint16  | Sequence number (wraps)
4           | DATA_LEN     | 1 byte  | uint8   | Always 0 (reserved)
5           | CMD_ID       | 1 byte  | uint8   | Command identifier
6+          | DATA         | n bytes | varies  | Command-specific data
LEN-3       | CRC8         | 1 byte  | uint8   | CRC-8 MAXIM_DOW
LEN-2,-1    | TAIL         | 2 bytes | ASCII   | 'ED' (0x45 0x44)
```

**Total Packet Size:** DATA length + 9 bytes (overhead)

### Command IDs

| CMD_ID | Name         | Data Size | Total Packet Size | Description |
|--------|--------------|-----------|-------------------|-------------|
| 0x00   | GIMBAL_CMD   | 10 bytes  | 19 bytes         | Gimbal control/feedback |
| 0x01   | COLOR_CMD    | 1 byte    | 10 bytes         | Team color |
| 0x02   | CHASSIS_CMD  | 12 bytes  | 21 bytes         | Chassis velocity |

---

## Implementation Steps

### Step 1: Packet Reception

#### 1.1 UART Configuration

```cpp
// UART Configuration (example)
UART_InitTypeDef uart_init;
uart_init.BaudRate = 115200;
uart_init.WordLength = UART_WORDLENGTH_8B;
uart_init.StopBits = UART_STOPBITS_1;
uart_init.Parity = UART_PARITY_NONE;
uart_init.Mode = UART_MODE_TX_RX;
uart_init.HwFlowCtl = UART_HWCONTROL_NONE;
```

#### 1.2 Circular Buffer

Implement a circular buffer to handle incoming bytes:

```cpp
#define RX_BUFFER_SIZE 256

uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_head = 0;
volatile uint16_t rx_tail = 0;

// In UART RX interrupt
void UART_RX_Handler(uint8_t byte) {
    rx_buffer[rx_head] = byte;
    rx_head = (rx_head + 1) % RX_BUFFER_SIZE;
}
```

#### 1.3 Packet Parsing

```cpp
bool ParsePacket(uint8_t* buffer, uint16_t length) {
    // 1. Check minimum length
    if (length < 10) return false;

    // 2. Verify header
    if (buffer[0] != 'S' || buffer[1] != 'T') return false;

    // 3. Extract CMD_ID
    uint8_t cmd_id = buffer[5];

    // 4. Calculate expected packet length
    uint16_t data_len = GetDataLength(cmd_id);  // Returns 10, 1, or 12
    uint16_t packet_len = data_len + 9;

    if (length < packet_len) return false;

    // 5. Verify tail
    if (buffer[packet_len-2] != 'E' || buffer[packet_len-1] != 'D') return false;

    // 6. Verify CRC
    uint8_t crc_received = buffer[packet_len-3];
    uint8_t crc_calculated = CalculateCRC8(&buffer[0], packet_len-3);

    if (crc_received != crc_calculated) return false;

    // 7. Parse data
    return ParseData(cmd_id, &buffer[6]);
}
```

#### 1.4 CRC-8 Calculation

Use CRC-8 MAXIM_DOW standard (polynomial: 0x31, init: 0x00):

```cpp
uint8_t CalculateCRC8(uint8_t* data, uint16_t length) {
    uint8_t crc = 0x00;

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}
```

### Step 2: Data Parsing

#### 2.1 GIMBAL Command (CMD_ID = 0x00)

**Data Structure (10 bytes):**
```cpp
typedef struct {
    float rel_yaw;      // Bytes 0-3: angular offset from camera center to target (radians)
    float rel_pitch;    // Bytes 4-7: angular offset from camera center to target (radians)
    uint8_t mode;       // Byte 8: 0=ST (search), 1=MY (move yoke)
    uint8_t debug_int;  // Byte 9: debug value
} gimbal_data_t;
```

**Important:** `rel_yaw` and `rel_pitch` are **angular offsets**, not absolute positions!

**Parsing Code:**
```cpp
bool ParseGimbalData(uint8_t* data, gimbal_data_t* output) {
    // Extract floats (little-endian)
    memcpy(&output->rel_yaw, &data[0], 4);
    memcpy(&output->rel_pitch, &data[4], 4);

    output->mode = data[8];
    output->debug_int = data[9];

    return true;
}
```

**Handling - CRITICAL CHANGE:**
```cpp
void HandleGimbalCommand(gimbal_data_t* cmd) {
    // Check for handshake request first
    if (cmd->debug_int == 0xFF) {
        SendHandshakeAck();
        return;
    }

    // Normal gimbal control
    if (cmd->mode == 1) {  // MY mode - move to target
        // IMPORTANT: rel_yaw and rel_pitch are OFFSETS from camera center
        // MCU must ADD these to current gimbal position

        // Get current gimbal position from sensors (IMU/encoders)
        float current_yaw = gimbal.GetCurrentYaw();     // From IMU/encoder
        float current_pitch = gimbal.GetCurrentPitch(); // From IMU/encoder

        // Calculate target position = current + offset
        float target_yaw = current_yaw + cmd->rel_yaw;
        float target_pitch = current_pitch + cmd->rel_pitch;

        // Set target (MCU PID controller will drive to target)
        gimbal.SetTargetYaw(target_yaw);
        gimbal.SetTargetPitch(target_pitch);

    } else {  // ST mode - search target
        gimbal.EnterSearchMode();
    }
}
```

**Example:**
- Camera center: gimbal at yaw=0°, pitch=0°
- Target detected: 10cm to the right, 5cm up
- Jetson calculates: offset = 0.1 rad yaw, -0.05 rad pitch
- Jetson sends: `rel_yaw=0.1, rel_pitch=-0.05, mode=MY`
- MCU reads: current position = (0.0, 0.0) from sensors
- MCU calculates: target = (0.0 + 0.1, 0.0 - 0.05) = (0.1, -0.05)
- MCU moves gimbal to yaw=0.1 rad, pitch=-0.05 rad
```

#### 2.2 COLOR Command (CMD_ID = 0x01)

**Data Structure (1 byte):**
```cpp
typedef struct {
    uint8_t my_color;  // 0=RED, 1=BLUE
} color_data_t;
```

**Parsing Code:**
```cpp
bool ParseColorData(uint8_t* data, color_data_t* output) {
    output->my_color = data[0];
    return true;
}
```

#### 2.3 CHASSIS Command (CMD_ID = 0x02)

**Data Structure (12 bytes):**
```cpp
typedef struct {
    float vx;  // Bytes 0-3: velocity forward
    float vy;  // Bytes 4-7: velocity left
    float vw;  // Bytes 8-11: angular velocity
} chassis_data_t;
```

**Parsing Code:**
```cpp
bool ParseChassisData(uint8_t* data, chassis_data_t* output) {
    memcpy(&output->vx, &data[0], 4);
    memcpy(&output->vy, &data[4], 4);
    memcpy(&output->vw, &data[8], 4);
    return true;
}
```

### Step 3: Packet Transmission

#### 3.1 Build Packet

```cpp
bool BuildPacket(uint8_t cmd_id, void* data, uint8_t* output, uint16_t* length) {
    uint16_t idx = 0;

    // Header
    output[idx++] = 'S';
    output[idx++] = 'T';

    // Sequence number (little-endian)
    static uint16_t seq_num = 0;
    output[idx++] = (seq_num & 0xFF);
    output[idx++] = (seq_num >> 8);
    seq_num++;

    // Data length (always 0 per protocol spec)
    output[idx++] = 0x00;

    // Command ID
    output[idx++] = cmd_id;

    // Data
    uint16_t data_len = GetDataLength(cmd_id);
    memcpy(&output[idx], data, data_len);
    idx += data_len;

    // CRC
    uint8_t crc = CalculateCRC8(output, idx);
    output[idx++] = crc;

    // Tail
    output[idx++] = 'E';
    output[idx++] = 'D';

    *length = idx;
    return true;
}
```

#### 3.2 Send GIMBAL Feedback

**Critical:** MCU must send gimbal position feedback regularly for tracking to work!

```cpp
void SendGimbalFeedback() {
    gimbal_data_t feedback;
    feedback.rel_yaw = gimbal.GetCurrentYaw();
    feedback.rel_pitch = gimbal.GetCurrentPitch();
    feedback.mode = gimbal.GetMode();
    feedback.debug_int = 0;  // Or any debug value

    uint8_t packet[32];
    uint16_t length;
    BuildPacket(0x00, &feedback, packet, &length);

    UART_Transmit(packet, length);
}
```

**Recommended:** Send feedback at 50-100 Hz (every 10-20ms).

### Step 4: Handshake Implementation

**Purpose:** Verify communication is established before operation starts.

#### 4.1 Detect Handshake Request

```cpp
void HandleGimbalCommand(gimbal_data_t* cmd) {
    // Check for handshake request
    if (cmd->debug_int == 0xFF) {
        SendHandshakeAck();
        return;  // Don't process as normal command
    }

    // Normal processing...
}
```

#### 4.2 Send Handshake Acknowledgment

```cpp
void SendHandshakeAck() {
    gimbal_data_t ack;
    ack.rel_yaw = gimbal.GetCurrentYaw();     // Include current position
    ack.rel_pitch = gimbal.GetCurrentPitch();
    ack.mode = 0;  // ST mode
    ack.debug_int = 0xFE;  // Handshake ACK marker

    uint8_t packet[32];
    uint16_t length;
    BuildPacket(0x00, &ack, packet, &length);

    UART_Transmit(packet, length);

    // Optional: Set a flag to indicate handshake complete
    handshake_complete = true;
}
```

---

## Main Loop Structure

### Recommended Control Loop

```cpp
void MainTask() {
    uint32_t last_feedback_time = 0;
    const uint32_t FEEDBACK_INTERVAL_MS = 10;  // 100 Hz

    while (1) {
        // 1. Process incoming packets
        ProcessUARTRx();

        // 2. Run gimbal control
        gimbal.UpdateControl();

        // 3. Send periodic feedback
        uint32_t now = GetTickMs();
        if (now - last_feedback_time >= FEEDBACK_INTERVAL_MS) {
            SendGimbalFeedback();
            last_feedback_time = now;
        }

        // 4. Other tasks...

        // Small delay (optional)
        osDelay(1);
    }
}
```

---

## Testing Procedures

### Phase 1: Loopback Test

**Purpose:** Verify packet building and parsing.

**Setup:**
1. Connect UART TX to RX (loopback)
2. Send a packet, verify you receive it correctly
3. Check CRC calculation

**Expected:** All sent packets should parse successfully.

### Phase 2: Jetson Connection Test

**Purpose:** Verify UART communication with Jetson.

**Setup:**
1. Connect MCU UART to Jetson UART (TX→RX, RX→TX, GND)
2. Run Jetson vision.py
3. Monitor both sides

**Expected Output (Jetson side):**
```
Initiating handshake with MCU...
[TX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=255
[RX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=254
✓ Handshake successful!
```

**Expected Output (MCU side):**
```
RX: Handshake request (debug=0xFF)
TX: Handshake ACK (debug=0xFE)
RX: GIMBAL command (yaw=-0.322, pitch=0.021, mode=MY)
TX: GIMBAL feedback (yaw=-0.322, pitch=0.021)
```

### Phase 3: Gimbal Control Test

**Purpose:** Verify gimbal actually moves to commanded positions.

**Test Procedure:**
1. Point camera at armor plate
2. Jetson should detect and send GIMBAL commands
3. MCU should move gimbal to target
4. MCU should send feedback showing new position
5. Jetson should track smoothly

**Success Criteria:**
- Gimbal moves to target within 0.1 rad accuracy
- Feedback rate: 50-100 Hz
- No packet loss (CRC errors)
- Smooth tracking (no jitter)

### Phase 4: Color Switching Test

**Purpose:** Verify enemy color switching works.

**Test Procedure:**
1. Jetson sends COLOR command (enemy=red)
2. MCU updates internal state
3. MCU sends COLOR feedback
4. Repeat with enemy=blue

### Phase 5: Stress Test

**Purpose:** Verify reliability under high packet rate.

**Test Procedure:**
1. Run for 30+ minutes continuous operation
2. Monitor for packet loss, CRC errors, dropped commands
3. Check for memory leaks, buffer overflows

**Success Criteria:**
- No crashes or resets
- Packet success rate > 99.9%
- Gimbal remains responsive

---

## Debugging Guide

### Common Issues

#### Issue 1: Handshake Timeout

**Symptoms:**
```
✗ Handshake timeout. MCU did not respond.
```

**Possible Causes:**
1. MCU not running or crashed
2. UART not connected (check wiring)
3. Baud rate mismatch (should be 115200)
4. MCU not checking for debug_int==0xFF
5. MCU not sending response packet

**Debug Steps:**
1. Check MCU is powered and running (LED blink)
2. Verify UART TX/RX pins are connected correctly
3. Use oscilloscope to check UART signals
4. Add debug print when receiving debug_int==0xFF
5. Add debug print when sending ACK packet

#### Issue 2: CRC Errors

**Symptoms:**
```
Packet received but crc checksum is wrong
```

**Possible Causes:**
1. CRC algorithm mismatch
2. Packet corruption during transmission
3. Buffer overflow or memory issue

**Debug Steps:**
1. Verify CRC-8 MAXIM_DOW implementation (polynomial 0x31)
2. Test with known packet: `53 54 00 00 00 00 00 00 00 00 00 00 00 00 00 FF XX 45 44`
3. Compare calculated CRC on both sides

#### Issue 3: No Gimbal Feedback

**Symptoms:**
```
Current yaw angle: 0 (never changes)
```

**Possible Causes:**
1. MCU not sending GIMBAL packets back
2. Jetson only receiving COLOR packets
3. Feedback rate too low

**Debug Steps:**
1. Add debug LED toggle when sending GIMBAL feedback
2. Verify feedback loop is running (not blocked)
3. Check feedback interval (should be 10-20ms)
4. Use logic analyzer to verify TX packets

#### Issue 4: Gimbal Not Moving

**Symptoms:**
- Jetson sending commands
- MCU receiving commands
- But gimbal doesn't move

**Possible Causes:**
1. Mode check failing (expecting MY mode)
2. Motor control disabled
3. Position limits preventing movement
4. Control loop not running

**Debug Steps:**
1. Log received yaw/pitch values
2. Verify mode == 1 (MY mode)
3. Check motor PWM signals with oscilloscope
4. Test motor control independently

---

## Debug Values Reference

| Value | Meaning | Usage |
|-------|---------|-------|
| 0xFF (255) | Handshake request | Jetson → MCU only |
| 0xFE (254) | Handshake ACK | MCU → Jetson only |
| 0x00-0xFD | User-defined | Available for custom debugging |

**Recommendations:**
- `0x00`: Normal operation
- `0x01`: Gimbal homing
- `0x02`: Calibration mode
- `0x03-0xFD`: Custom states

---

## Performance Requirements

| Metric | Requirement | Typical |
|--------|-------------|---------|
| Feedback Rate | 50-100 Hz | 100 Hz |
| Position Accuracy | ±0.1 rad | ±0.05 rad |
| Response Latency | <50ms | 20-30ms |
| Packet Loss Rate | <0.1% | <0.01% |
| CRC Error Rate | <0.01% | <0.001% |

---

## Quick Reference Card

### Packet Format Summary
```
ST SEQ(2) LEN CMD DATA... CRC ED
```

### Command IDs
- `0x00`: GIMBAL (10 bytes data)
- `0x01`: COLOR (1 byte data)
- `0x02`: CHASSIS (12 bytes data)

### Handshake Flow
```
Jetson: GIMBAL(debug=0xFF) →
                           ← MCU: GIMBAL(debug=0xFE)
```

### Critical Functions
```cpp
// Must implement
ParsePacket()
BuildPacket()
HandleGimbalCommand()
SendGimbalFeedback()  // Call at 100Hz!
SendHandshakeAck()
```

---

## Contact & Support

**For Protocol Questions:**
- See: [minipc_protocol.md](minipc_protocol.md)
- See: [HANDSHAKE_IMPLEMENTATION.md](HANDSHAKE_IMPLEMENTATION.md)

**For Jetson-Side Code:**
- Repository: `iRM_Vision_2023/`
- Implementation: `Communication/communicator.py`

**For MCU Reference Implementation:**
- Repository: `iRM_Embedded_2023/examples/minipc/`
- Files: `minipc_protocol.h`, `minipc_protocol.cc`

---

## Appendix A: Example Packets (Hex)

### Handshake Request (Jetson → MCU)
```
53 54 00 00 00 00 00 00 00 00 00 00 00 00 00 FF ?? 45 44
└─┘ └───┘ │  │  └──────────────────────┘ │  │  │  └─┘ └─┘
 ST SEQ   LEN ID     DATA (10 bytes)     ┌┘  │  │   ED
                    (yaw=0, pitch=0,     │   │  │
                     mode=ST, debug=FF)  │   │  CRC
                                        mode debug
```

### Handshake ACK (MCU → Jetson)
```
53 54 01 00 00 00 00 00 00 00 00 00 00 00 00 FE ?? 45 44
                                             └┘
                                           debug=0xFE
```

### Normal GIMBAL Command
```
53 54 02 00 00 00 D3 74 9A BE 7C 92 B1 3C 01 01 ?? 45 44
                  └──────────┘ └──────────┘ │  │
                   yaw=-0.302   pitch=0.022 │  debug=1
                                            mode=MY
```

## Appendix B: State Machine

```
┌──────────┐
│  INIT    │
└────┬─────┘
     │
     ▼
┌──────────────┐
│ WAIT_HANDSH  │◄──── timeout (retry)
└──────┬───────┘
       │ RX handshake req
       ▼
┌──────────────┐
│ SEND_ACK     │
└──────┬───────┘
       │
       ▼
┌──────────────┐
│ NORMAL_OP    │◄──┐
└──────┬───────┘   │
       │ RX cmd    │
       │ TX feedback │
       └───────────┘
```

---

**Document Version:** 1.0
**Last Updated:** 2024
**Maintained By:** iRM Vision Team
