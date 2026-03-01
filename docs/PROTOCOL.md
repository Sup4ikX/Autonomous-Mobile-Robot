# 📡 ESP32 LiDAR Robot UDP Protocol

## Overview
This document describes the binary UDP packet formats used for communication between the ESP32 robot and ROS2 nodes. It covers both LiDAR scan packets and telemetry packets, matching the C++ and Python codebase.

---

## 1. LiDAR Packet (0xAA)

**Header (13 bytes, big-endian):**
| Offset | Size | Field             | Type     | Description                                 |
|--------|------|------------------|----------|---------------------------------------------|
| 0      | 1    | start_byte       | uint8_t  | Always 0xAA                                 |
| 1      | 2    | total_length     | uint16_t | Full packet length incl. header/checksum    |
| 3      | 1    | protocol_version | uint8_t  | Protocol version                            |
| 4      | 1    | packet_type      | uint8_t  | 0xAD = scan data, 0xAE = RPM only           |
| 5      | 1    | data_header      | uint8_t  | Always 0xAD                                 |
| 6      | 2    | data_length      | uint16_t | Length after header (measurements+5 bytes)  |
| 8      | 1    | motor_speed      | uint8_t  | Motor speed ×0.05 (rev/s)                   |
| 9      | 2    | zero_offset      | int16_t  | Zero offset ×0.01° (big-endian)             |
| 11     | 2    | start_angle      | uint16_t | Start angle ×0.01° (big-endian)             |

**Measurements:**
- Follows header, N blocks of 3 bytes each:
  - signal_strength (1 byte)
  - distance_raw (2 bytes, big-endian)
- N = (data_length - 5) / 3

**Checksum:**
- Last 2 bytes of packet: uint16_t, big-endian
- Calculated as sum of all bytes except checksum, & 0xFFFF

**Example Layout:**
| Header (13) | [Meas1 (3)] ... [MeasN (3)] | Checksum (2) |

---

## 2. Telemetry Packet (0xBB)

**Fixed size: 23 bytes, big-endian**
| Offset | Size | Field        | Type     | Description                        |
|--------|------|-------------|----------|------------------------------------|
| 0      | 1    | identifier  | uint8_t  | Always 0xBB                        |
| 1      | 4    | ip          | uint8_t[4]| IP address (e.g. 192,168,1,100)    |
| 5      | 2    | tcp_port    | uint16_t | TCP server port                    |
| 7      | 8    | left_hall   | uint64_t | Left hall counter                  |
| 15     | 8    | right_hall  | uint64_t | Right hall counter                 |

All multi-byte fields are big-endian.

---

## 3. Notes
- All packets are sent via UDP from ESP32 to the ROS2 host.
- LiDAR packets are variable length, Telemetry packets are always 23 bytes.
- For parsing, see `python/pj_lidar/udp_structs.py` and C++ headers.

---

## 4. Example (LiDAR)
```
AA 00 1F 01 AD AD 00 14 64 00 00 00 00 ... [measurements] ... [checksum]
```

## 5. Example (Telemetry)
```
BB C0 A8 01 64 0D 05 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

---

For updates, see code and this file.
