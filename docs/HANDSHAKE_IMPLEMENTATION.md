# Handshake Implementation Summary

## Overview

A handshake mechanism has been added to the Jetson-MCU communication system to verify that both devices are properly connected and communicating before starting autonomous operation.

**Boot Sequence:** The system assumes MCU boots first (embedded hardware, ~1s boot time), then Jetson boots second (Linux system, ~30-60s boot time). When Jetson starts, the MCU is already running and ready to respond.

## Changes Made

### 1. Jetson Side (iRM_Vision_2023)

#### File: `Communication/communicator.py`

**Added `perform_handshake()` method:**
- Sends GIMBAL packet with `debug_int=0xFF` (handshake request)
- Waits up to 5 seconds for MCU to respond with `debug_int=0xFE`
- **Retry logic:** Attempts up to 3 times before failing
- Returns `True` if successful, `False` if all retries exhausted
- Thread-safe implementation using existing state_dict_lock

**Location:** Lines 63-115

```python
def perform_handshake(self, timeout=5.0):
    """Perform handshake with MCU to verify communication."""
    # Sends handshake request
    # Waits for ACK
    # Returns success/failure
```

#### File: `vision.py`

**Added handshake call in main():**
- Called after `start_listening()`
- Prints clear status messages with visual separators
- Continues operation even if handshake fails (with warning)

**Location:** Lines 36-44

```python
if not communicator.perform_handshake(timeout=5.0):
    print("WARNING: Handshake failed, but continuing anyway...")
```

### 2. Documentation

#### File: `docs/minipc_protocol.md`

**Added Handshake Protocol section:**
- Complete protocol specification
- Example implementations for both Jetson and MCU
- Debug value table (0xFF = request, 0xFE = ACK)

**Updated Known Issues section:**
- Added note about gimbal feedback requirement
- Added handshake recommendation

## How It Works

### Boot Sequence

```
Power On
    |
    v
MCU Boots (~1 second)
    |
    v
MCU Main Loop Running
    |
    |  (MCU ready and listening)
    |
    v
Jetson Boots (~30-60 seconds)
    |
    v
Jetson Opens Serial Port
    |
    v
Jetson Initiates Handshake
    |
    v
Handshake Exchange
    |
    v
Normal Operation
```

### Handshake Sequence Diagram

```
Jetson                          MCU (already running)
  |                              |
  |-- GIMBAL (debug_int=0xFF) -->|  Request (Attempt 1)
  |                              |
  |     (wait up to 5 sec)       |
  |                              |
  |<-- GIMBAL (debug_int=0xFE) --|  ACK
  |                              |
  |   ✓ Handshake Complete       |
  |                              |
  |-- Normal Operation Begins ---|


(If timeout on attempt 1)
  |                              |
  |-- GIMBAL (debug_int=0xFF) -->|  Request (Attempt 2)
  |                              |
  |     (wait up to 5 sec)       |
  ...                           ...

(Up to 3 total attempts)
```

### Packet Format (Handshake Request)

```
Field       Value
--------    -----
HEADER      'ST' (0x53 0x54)
SEQ_NUM     0x00 0x00
DATA_LEN    0x00
CMD_ID      0x00 (GIMBAL)
rel_yaw     0.0 (4 bytes float)
rel_pitch   0.0 (4 bytes float)
mode        0x00 (ST mode)
debug_int   0xFF ← Handshake marker
CRC8        (computed)
TAIL        'ED' (0x45 0x44)
```

### Packet Format (Handshake ACK)

```
Field       Value
--------    -----
HEADER      'ST' (0x53 0x54)
SEQ_NUM     (varies)
DATA_LEN    0x00
CMD_ID      0x00 (GIMBAL)
rel_yaw     (current position)
rel_pitch   (current position)
mode        0x00 or 0x01
debug_int   0xFE ← ACK marker
CRC8        (computed)
TAIL        'ED' (0x45 0x44)
```

## MCU Implementation Required

To support the handshake, the MCU firmware needs to:

1. **Detect handshake request:**
   ```cpp
   if (received_packet.cmd_id == GIMBAL_CMD_ID &&
       received_packet.data.debug_int == 0xFF) {
       // Handshake request received
   }
   ```

2. **Send acknowledgment:**
   ```cpp
   gimbal_data_t ack;
   ack.rel_yaw = gimbal.current_yaw;      // Current position
   ack.rel_pitch = gimbal.current_pitch;  // Current position
   ack.mode = 0;                          // ST mode
   ack.debug_int = 0xFE;                  // ACK marker

   minipc_protocol.Pack(GIMBAL_CMD_ID, &ack);
   minipc_protocol.Transmit();
   ```

3. **Continue sending feedback:**
   After handshake, MCU should continue sending GIMBAL packets with current position at regular intervals (50-100 Hz recommended).

## Testing the Handshake

### Expected Output (Success on First Try)

```
OPENED SERIAL DEVICE AT: /dev/ttyTHS0

==================================================
Initiating handshake with MCU...
[TX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=255
[TX_HEX] 53 54 00 00 00 00 00 00 00 00 00 00 00 00 00 FF XX 45 44
[RX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=254
✓ Handshake successful! MCU acknowledged connection.
==================================================

(Normal operation begins)
```

### Expected Output (Success After Retry)

```
OPENED SERIAL DEVICE AT: /dev/ttyTHS0

==================================================
Initiating handshake with MCU...
[TX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=255
[TX_HEX] 53 54 00 00 00 00 00 00 00 00 00 00 00 00 00 FF XX 45 44
(5 second timeout)
  No response, retrying...
Handshake retry 1/2...
[TX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=255
[TX_HEX] 53 54 01 00 00 00 00 00 00 00 00 00 00 00 00 FF XX 45 44
[RX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=254
✓ Handshake successful! MCU acknowledged connection.
==================================================

(Normal operation begins)
```

### Expected Output (Failure After All Retries)

```
OPENED SERIAL DEVICE AT: /dev/ttyTHS0

==================================================
Initiating handshake with MCU...
[TX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=255
[TX_HEX] 53 54 00 00 00 00 00 00 00 00 00 00 00 00 00 FF XX 45 44
(5 second timeout)
  No response, retrying...
Handshake retry 1/2...
[TX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=255
[TX_HEX] 53 54 01 00 00 00 00 00 00 00 00 00 00 00 00 FF XX 45 44
(5 second timeout)
  No response, retrying...
Handshake retry 2/2...
[TX] GIMBAL: yaw=0.000, pitch=0.000, mode=ST, debug=255
[TX_HEX] 53 54 02 00 00 00 00 00 00 00 00 00 00 00 00 FF XX 45 44
(5 second timeout)
✗ Handshake failed after all retries.
  Possible causes:
    1. MCU is not running or not responding
    2. UART connection issue (check cable/port)
    3. MCU firmware doesn't implement handshake protocol
  Expected: MCU should reply with GIMBAL packet (debug_int=0xFE)
WARNING: Handshake failed, but continuing anyway...
Communication may not work properly without MCU acknowledgment.
==================================================

(Continues with degraded functionality)
```

## Debug Values Reserved

| Value | Purpose | Direction |
|-------|---------|-----------|
| 0xFF (255) | Handshake request | Jetson → MCU |
| 0xFE (254) | Handshake ACK | MCU → Jetson |
| 0x00-0xFD | Available for user debugging | Both |

## Benefits

1. **Connection verification**: Confirms both devices are communicating
2. **Early error detection**: Identifies comm issues before autonomous operation
3. **Clear status feedback**: User knows if system is ready
4. **Graceful degradation**: System continues even if handshake fails (with warning)
5. **Position sync**: MCU sends initial gimbal position in ACK packet

## Future Enhancements (Optional)

1. Add retry logic (currently tries once)
2. Add timeout configuration in config.py
3. Add strict mode that aborts if handshake fails
4. Add protocol version exchange in handshake
5. Add CRC statistics in handshake ACK

## Related Files

- `Communication/communicator.py` - Handshake implementation
- `vision.py` - Handshake invocation
- `docs/minipc_protocol.md` - Protocol specification
- `config.py` - Debug flags (DEBUG_UART)

## See Also

- [minipc_protocol.md](minipc_protocol.md) - Full protocol specification
- MCU implementation: `iRM_Embedded_2023/examples/minipc/`
