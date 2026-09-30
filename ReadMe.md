# Real-Time Modbus RTU Frame Builder & CRC-16 Validator

![Language](https://img.shields.io/badge/Language-C%2B%2B17%20%2F%20C%2B%2B20-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%20Linux-orange)
![Reliability](https://img.shields.io/badge/Protocol-Modbus%20RTU%20%2F%20RS485-brightgreen)

A modular, allocation-free C++ implementation of the Modbus RTU frame assembly and 16-bit Cyclic Redundancy Check (CRC-16) validation algorithm. This library protects robotic actuators and sensor links from command execution failures caused by physical electromagnetic interference (EMI).

---

## 🛰️ System Architecture

Modbus RTU frames are transmitted via physical serial lines (such as RS-485 transceivers) in high-interference environments. 
To guarantee data integrity, every frame ends with a 2-byte CRC checksum:

```text
  ┌──────────────────────────────────────────────────────────┬──────────────┐
  │  Device ID  │ Function Code  │       Data Payload        │  CRC-16 LSB  │  CRC-16 MSB  │
  │   (1 Byte)  │   (1 Byte)     │       (N Bytes)           │  (Low Byte)  │  (High Byte) │
  └──────────────────────────────────────────────────────────┴──────────────┘
  └─────────────────────── Computed Data Block ──────────────┘
