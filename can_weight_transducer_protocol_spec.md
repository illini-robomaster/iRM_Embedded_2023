# CAN Weight Transducer Protocol Specification
## (For Static Analysis & Runtime Validation – Claude Opus)

---

## Scope

This document defines the **CAN application-layer protocol** of a multi-channel
weight transducer.  
It is intended for **automated code inspection, validation, and test generation**.

This specification reflects **actual device behavior**, including
edge cases observed during real measurements.

---

## 1. CAN Transport Constraints

- CAN 2.0
- DLC = 8 bytes (fixed)
- Frame Types:
  - Standard Frame (11-bit ID)
  - Extended Frame (29-bit ID)
- Default node address: `0x01`
- Default bitrate: `500 kbps`

---

## 2. Endianness & Primitive Types (Hard Requirement)

### 2.1 Endianness

> **All multi-byte fields are BIG-ENDIAN**

---

### 2.2 Primitive Types

| Type | Definition |
|----|----|
| `uint32_be` | Unsigned 32-bit integer, big-endian |
| `int32_be` | Signed 32-bit integer (two’s complement), big-endian |
| `float32_be` | IEEE754 single-precision float, big-endian |

⚠️ **Observed behavior override**  
Although the datasheet claims `uint32` for weight values, the device **actually
returns two’s complement negative values** near zero or before calibration.

**Implementations MUST support `int32_be` decoding for weight values.**

---

## 3. CAN Identifier Encoding

### 3.1 Standard Frame (11-bit ID)

```

Bit10..8 : Function Code (3 bits)
Bit7..0  : Device Address (8 bits)

````

**Encoding**
```c
id = (function_code << 8) | address;
````

**Decoding**

```c
function_code = (id >> 8) & 0x07;
address = id & 0xFF;
```

---

### 3.2 Extended Frame (29-bit ID)

```
Bit23..16 : 0xAA (fixed)
Bit15..8  : Function Code
Bit7..0   : Device Address
```

**Encoding**

```c
id = (0xAA << 16) | (function_code << 8) | address;
```

---

## 4. Function Codes

| Code   | Meaning                        |
| ------ | ------------------------------ |
| `0x01` | Tare                           |
| `0x02` | Zero calibration               |
| `0x03` | Read weight (multi-frame)      |
| `0x04` | Communication parameters       |
| `0x05` | Sampling / division parameters |
| `0x06` | Calibration factor (float32)   |

---

## 5. Command Definitions

---

### 5.1 `0x01` – Tare

**Request**

* ID: `(0x01 << 8) | address`
* DATA:

  * Byte0:

    * `0x01–0x0E`: Single channel
    * `0x0F`: All channels
  * Byte1–7: `0x00`

**Response**

* Exact echo of request

---

### 5.2 `0x02` – Zero Calibration

Identical frame layout to **Tare**.

---

### 5.3 `0x03` – Read Weight (Critical)

**Request**

* ID: `(0x03 << 8) | address`
* DATA: `00 00 00 00 00 00 00 00`

---

**Response Rules**

* Device returns **multiple frames**
* Each frame contains **two channels**
* Data layout:

| Bytes | Meaning                  |
| ----- | ------------------------ |
| 0–3   | Channel N (`int32_be`)   |
| 4–7   | Channel N+1 (`int32_be`) |

**Example**

```
FF FF F8 C6 00 00 00 00
```

```c
(int32_be) 0xFFFFF8C6 → -1850
```

⚠️ **Invalid implementation indicators**

* Treating weight strictly as `uint32`
* Assuming only one response frame
* Wrong byte order

---

### 5.4 `0x04` – Communication Parameters

| DATA[0] | Meaning       |
| ------- | ------------- |
| `0xA1`  | Read address  |
| `0xA2`  | Read bitrate  |
| `0xB1`  | Write address |
| `0xB2`  | Write bitrate |

**Bitrate Codes**

| Code   | Bitrate  |
| ------ | -------- |
| `0x02` | 20 kbps  |
| `0x03` | 50 kbps  |
| `0x04` | 100 kbps |
| `0x05` | 125 kbps |
| `0x06` | 200 kbps |
| `0x07` | 250 kbps |
| `0x08` | 400 kbps |
| `0x09` | 500 kbps |
| `0x0A` | 800 kbps |
| `0x0B` | 1 Mbps   |

⚠️ Changes apply **only after power cycle ≥5 seconds**.

---

### 5.5 `0x05` – Sampling Parameters (Non-persistent)

| DATA[0] | Parameter                    |
| ------- | ---------------------------- |
| `0xA1`  | Sample rate (1=10Hz, 2=40Hz) |
| `0xB1`  | Zero tracking range          |
| `0xC1`  | Division                     |

---

### 5.6 `0x06` – Calibration Factor (Critical)

**DATA Layout**

| Byte | Meaning                               |
| ---- | ------------------------------------- |
| 0    | Channel                               |
| 1–4  | Calibration factor `K` (`float32_be`) |
| 5–7  | `0x00`                                |

**Rules**

* Persistent across power loss
* Effective only after reboot
* Calibration formula:

```
K = TARGET_VALUE / MEASURED_RAW_VALUE
```

---

## 6. Two’s Complement Validation Vectors

| Hex (BE)      | Expected int32 |
| ------------- | -------------- |
| `FF FF FF FA` | `-6`           |
| `FF FF FD 09` | `-759`         |
| `FF FF F8 C6` | `-1850`        |

---

## 7. Required Interaction Patterns

* **Echo ACK**: All write operations
* **Multi-frame streaming**: Weight read
* **Apply-on-reboot**: Address, bitrate, calibration factor

---

## 8. Mandatory Validation Checklist

* Correct ID encode/decode (standard & extended)
* Big-endian decoding
* Support signed `int32` weight values
* Multi-frame handling for reads
* Float32 calibration factor correctness
* Enforce reboot after persistent writes
* Prevent `0xFFFFxxxx` → large positive misinterpretation

---

## Final Rule

> Any implementation that parses weight strictly as `uint32`
> is **incorrect by real device behavior**.

````

---

# 二、Copilot（Claude Opus）审查代码用 Prompt（直接复制）

你可以在 Copilot Chat 中直接贴下面这段：

---

```text
You are Claude Opus acting as a protocol compliance auditor.

You are given:
1) A CAN weight transducer protocol specification (see provided document)
2) Application code that sends, receives, or parses CAN frames

Your task:
- Verify the implementation strictly follows the protocol specification
- Flag any deviation, ambiguity, or unsafe assumption

Explicitly check for:
- Incorrect CAN ID bit decoding
- Incorrect endianness
- Misinterpretation of two’s complement weight values as uint32
- Missing multi-frame handling for function code 0x03
- Incorrect float32 encoding for calibration factors
- Missing reboot requirement after persistent writes

When an issue is found:
- Explain the violation
- Show which section of the spec is violated
- Suggest a concrete fix

Be strict. Assume safety-critical industrial usage.
````

---
