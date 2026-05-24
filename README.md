# 🚗 Smart Autonomous Car — GPS + INS + nRF24L01 + Kalman Filter

<div align="center">

![Arduino](https://img.shields.io/badge/Arduino-Nano-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-0076A8?style=for-the-badge&logo=mathworks&logoColor=white)
![RF](https://img.shields.io/badge/RF-nRF24L01-E74C3C?style=for-the-badge&logo=signal&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-27AE60?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-1D8348?style=for-the-badge)

**Supervised by: Dr. Ahmed**  
*Engineering Department — 2024*

</div>

---

## 📋 Table of Contents

- [Overview](#-overview)
- [System Architecture](#-system-architecture)
- [Features](#-features)
- [Hardware Components](#-hardware-components)
- [Wiring & Pin Assignment](#-wiring--pin-assignment)
- [RF Communication — nRF24L01](#-rf-communication--nrf24l01)
- [Extended Kalman Filter (EKF)](#-extended-kalman-filter-ekf)
- [MATLAB Ground Station](#-matlab-ground-station)
- [Operating Modes](#-operating-modes)
- [Team Members](#-team-members)
- [Installation & Setup](#-installation--setup)
- [Usage](#-usage)
- [MATLAB Dashboard](#-matlab-dashboard)
- [Troubleshooting](#-troubleshooting)
- [Future Work](#-future-work)
- [License](#-license)

---

## 🔍 Overview

This project implements a **fully autonomous Arduino Nano-based car** that combines three navigation modes with real-time wireless telemetry to a MATLAB ground station:

```
┌──────────────────────────────────────────────────────────────────┐
│                        SYSTEM FLOW                               │
│                                                                  │
│  [Car: Arduino Nano]          [Ground Station: MATLAB]           │
│                                                                  │
│  GPS + IMU sensors            Receive RF packets                 │
│       ↓                              ↓                           │
│  Onboard 5-state EKF          3-state Kalman Filter              │
│  Line follow / GPS nav / avoid        ↓                          │
│       ↓                       Live 8-subplot Dashboard           │
│  nRF24L01 TX ──── 2.4GHz ───→ nRF24L01 RX → Arduino → USB      │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

> **Key Innovation:** GPS alone is slow (1 Hz) and noisy (±2.5 m). IMU alone is fast (50 Hz) but drifts. The **Extended Kalman Filter** fuses both sensors optimally — the same algorithm used in aircraft autopilots and self-driving cars.

---

## 🏗 System Architecture

### Three Layers

| Layer | Component | Technology |
|-------|-----------|------------|
| **Sensor** | GPS + IMU + IR + Ultrasonic | NEO-6M, MPU-6050, IR x3, HC-SR04 |
| **Fusion** | Onboard EKF (5-state) | Arduino C++ — `[x, y, heading, vx, vy]` |
| **Control** | FSM + PID + RF Telemetry | Line follow / GPS nav / Obstacle avoid |
| **Ground Station** | MATLAB Kalman + Dashboard | 3-state KF + 8 animated subplots |

### Data Flow

```
Car Arduino Nano
├── reads GPS   → lat, lon, alt, heading
├── reads IMU   → ax, ay, gz
├── runs EKF    → heading for navigation
├── packs struct DataPacket (28 bytes)
└── transmits via nRF24L01 @ 10 Hz
         │
         │  2.4 GHz wireless
         ▼
Ground Station Arduino (receiver)
└── forwards CSV over USB Serial @ 115200 baud
         │
         ▼
MATLAB (PC)
├── reads serial port
├── runs 3-state Kalman Filter
└── updates 8-subplot live dashboard
```

---

## ✨ Features

- 🛣️ **Line Following** — 3 IR sensors with proportional differential speed control
- 🚧 **Obstacle Avoidance** — HC-SR04 ultrasonic with automatic avoidance maneuver
- 📍 **GPS Waypoint Navigation** — Navigate to any GPS coordinate autonomously
- 🧠 **Onboard EKF** — 5-state Extended Kalman Filter fusing GPS + IMU at 50 Hz
- 📡 **Wireless RF Telemetry** — nRF24L01 transmits 7 sensor values at 10 Hz
- 📊 **MATLAB Live Dashboard** — Real-time 8-subplot Kalman filter visualization
- 📐 **Bias Estimation** — Automatically estimates and corrects IMU accelerometer bias
- 🔄 **Seamless Mode Switching** — Automatic transitions between all operating modes

---

## 🔧 Hardware Components

| Component | Model | Qty | Notes |
|-----------|-------|-----|-------|
| Microcontroller | Arduino Nano | 2 | One for car, one for ground station receiver |
| RF Module | nRF24L01 | 2 | 2.4 GHz, 32-byte packet, ~100 m range |
| IMU | MPU-6050 | 1 | 6-axis: gyro 250 dps + accel 2g |
| GPS | NEO-6M | 1 | UART 9600 baud, **3.3V only**, 2.5 m CEP |
| Ultrasonic | HC-SR04 | 1 | Range 2–400 cm, 15° beam |
| IR Sensors | Digital x3 | 3 | Line detection, active-low (LOW = on line) |
| Motor Driver | L298N | 1 | Dual H-Bridge, 2A/ch, onboard 5V reg |
| DC Motors | Geared TT | 2 | 6V gear motors |
| Battery | LiPo 7.4V 2S | 1 | 1500 mAh, ~45 min runtime |
| Capacitors | 10µF + 100nF | 2 pairs | **Required** for nRF24L01 noise filtering |

> ⚠️ **CRITICAL:** NEO-6M GPS operates at **3.3V only**. Connecting to 5V will permanently destroy the module.

---

## 🔌 Wiring & Pin Assignment

### Car Arduino Nano

| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| D2 | TRIG — Ultrasonic | Output | 10 µs trigger pulse |
| D3 | ECHO — Ultrasonic | Input | Pulse width → distance |
| D4 | IR_LEFT | Input | LOW = on black line |
| D5 | IR_MID | Input | LOW = on black line |
| D6 | IR_RIGHT | Input | LOW = on black line |
| D7 | nRF24L01 CE | Output | RF Chip Enable |
| D8 | nRF24L01 CSN | Output | RF SPI Chip Select |
| D9~ | ENA — Left motor | Output | PWM speed 0–255 |
| D10~ | ENB — Right motor | Output | PWM speed 0–255 |
| A0 | IN1 — Motor dir | Output | L298N direction |
| A1 | IN2 — Motor dir | Output | L298N direction |
| D11 | MOSI — SPI | Output | RF data out |
| D12 | MISO — SPI | Input | RF data in |
| D13 | SCK — SPI | Output | RF clock |
| A4 | SDA — MPU-6050 | I2C | Hardware I2C data |
| A5 | SCL — MPU-6050 | I2C | Hardware I2C clock |
| A2 | GPS RX (SW Serial) | Input | GPS TX → Nano |
| A3 | GPS TX (SW Serial) | Output | Nano → GPS RX |

### Power Distribution

```
LiPo 7.4V
    ├── → L298N VM pin (motor power)
    │       └── L298N 5V out → Nano VIN
    │               ├── 5V pin → HC-SR04 VCC, IR sensors VCC
    │               └── 3.3V pin → NEO-6M VCC, nRF24L01 VCC
    └── Common GND → ALL modules
```

---

## 📡 RF Communication — nRF24L01

### Packet Structure (28 bytes)

```cpp
struct DataPacket {
    float lat;       // GPS latitude  (degrees)   4 bytes
    float lon;       // GPS longitude (degrees)   4 bytes
    float alt;       // GPS altitude  (meters)    4 bytes
    float ax;        // IMU accel X   (m/s²)      4 bytes
    float ay;        // IMU accel Y   (m/s²)      4 bytes
    float gz;        // IMU gyro Z    (deg/s)     4 bytes
    float heading;   // EKF heading   (degrees)   4 bytes
};   // Total: 28 bytes — fits nRF24L01 max 32-byte payload
```

### RF Configuration

```cpp
radio.setPALevel(RF24_PA_HIGH);    // high power for range
radio.setDataRate(RF24_250KBPS);   // 250 kbps = more reliable
radio.setChannel(108);             // channel 108 avoids WiFi
```

> 💡 **Tip:** Add a 10 µF electrolytic + 100 nF ceramic capacitor between nRF24L01 VCC and GND. Without decoupling caps, motor noise will corrupt the RF signal causing packet drops.

---

## 🧮 Extended Kalman Filter (EKF)

### Onboard 5-State EKF (Arduino)

The car runs a full **5-state EKF** for navigation:

```
State vector X = [x, y, heading, vx, vy]
                  │   │    │      │   │
                  │   │    │      │   └─ velocity North (m/s)
                  │   │    │      └───── velocity East  (m/s)
                  │   │    └──────────── orientation (degrees, 0=North)
                  │   └───────────────── position North (meters)
                  └───────────────────── position East  (meters)
```

| Step | Rate | Source | Operation |
|------|------|--------|-----------|
| **Predict** | 50 Hz | MPU-6050 | `X = F·X + u`, `P = F·P·Fᵀ + Q` |
| **Update** | 1 Hz | NEO-6M GPS | `K = P·Hᵀ·S⁻¹`, `X = X + K·innov`, `P = (I-KH)·P` |

### Noise Parameters (tune for your hardware)

```cpp
const float Q_X     = 0.05f;   // position process noise     (m²)
const float Q_Y     = 0.05f;
const float Q_HDG   = 0.01f;   // heading process noise      (deg²)
const float Q_VX    = 0.10f;   // velocity process noise     (m/s)²
const float Q_VY    = 0.10f;
const float R_GPS_X = 2.50f;   // GPS position noise — NEO-6M CEP 2.5m
const float R_GPS_Y = 2.50f;
const float R_GPS_H = 5.00f;   // GPS course noise           (deg²)
```

---

## 📊 MATLAB Ground Station

### 3-State Kalman Filter

MATLAB runs a separate **3-state Kalman filter** `[altitude, velocity, accel_bias]` on the received RF data:

```matlab
dt = 0.1;   % 10 Hz
x  = [0; 0; 0];   % [alt; vel; bias]

A = [1  dt  -0.5*dt^2;   % altitude prediction
     0   1     -dt   ;   % velocity prediction
     0   0       1   ];  % bias: constant random walk

B  = [0.5*dt^2; dt; 0];  % measured acceleration input
H  = [1 0 0; 0 1 0];     % GPS observes altitude and velocity
Q  = diag([1e-2, 5e-2, 1e-5]);
R  = diag([9, 0.25]);     % 3m altitude std, 0.5 m/s velocity std
P  = eye(3) * 50;
```

**Predict + Update loop:**
```matlab
% Predict
x = A*x + B*accel;
P = A*P*A' + Q;

% Update (when GPS arrives)
innov = [gps_alt; gps_vel] - H*x;
K     = P*H' / (H*P*H' + R);
x     = x + K*innov;
P     = (I3 - K*H)*P*(I3 - K*H)' + K*R*K';
```

### Live Dashboard — 8 Subplots

```
┌─────────────────────┬─────────────────────┐
│ [1,1] Altitude      │ [1,2] Velocity       │
│  GPS (red dots)     │  GPS (blue dots)     │
│  Kalman (green)     │  Kalman (magenta)    │
├─────────────────────┼─────────────────────┤
│ [2,1] Raw Accel ax  │ [2,2] Accel Bias     │
│  IMU noise+bias     │  Converges to true   │
│  visible            │  bias ~10 seconds    │
├─────────────────────┼─────────────────────┤
│ [3,1] σ_altitude    │ [3,2] σ_velocity     │
│  Uncertainty drops  │  Uncertainty drops   │
│  as GPS updates     │  to steady state     │
├─────────────────────┼─────────────────────┤
│ [4,1] Innovation    │ [4,2] GPS Car Path   │
│  GPS - KF predict   │  Live 2D trajectory  │
│  near zero=healthy  │  lon vs lat plot     │
└─────────────────────┴─────────────────────┘
```

### Expected Performance

| System | Altitude RMSE | Velocity RMSE | Notes |
|--------|--------------|---------------|-------|
| **INS only** | ~15–30 m | ~5–12 m/s | Drift from bias + noise |
| **GPS only** | ~3.0 m | ~0.5 m/s | Direct GPS accuracy |
| **Kalman Filter** | **~1.5–2.5 m** | **~0.2–0.4 m/s** | ✅ Best — fuses both |

---

## 🔄 Operating Modes

The car runs as a **Finite State Machine (FSM)** with 4 modes:

```
┌─────────────────────────────────────────────────────────┐
│  Every loop (~20 ms): CHECK OBSTACLE FIRST              │
│  if (distance < 20 cm) → MODE_AVOID  (always wins)     │
└─────────────────────────────────────────────────────────┘
          ↓
┌─────────────────────────────────────────────────────────┐
│  Priority 1 │ MODE_AVOID       │ Stop→Reverse→Turn→Go  │
│  Priority 2 │ MODE_LINE_FOLLOW │ L/M/R IR sensor PID   │
│  Priority 3 │ MODE_GPS_NAV     │ Bearing + P-controller│
│  Priority 4 │ MODE_ARRIVED     │ Stop. Mission done.   │
└─────────────────────────────────────────────────────────┘
```

### Line Following Truth Table

| L | M | R | Condition | Action |
|---|---|---|-----------|--------|
| 0 | 1 | 0 | Straight | Both motors: 160 PWM |
| 1 | 0 | 0 | Drift left | Left: 110, Right: 160 |
| 0 | 0 | 1 | Drift right | Left: 160, Right: 110 |
| 1 | 1 | 1 | Intersection | Go straight |
| 0 | 0 | 0 | **Line lost** | **Switch to GPS_NAV** |

---

## 👥 Team Members

| Member | Sub-Team | Role |
|--------|----------|------|
| Member 1 | Team A | Hardware Lead — chassis, motors, L298N |
| Member 2 | Team A | RF Wiring — nRF24L01 TX/RX installation |
| Member 3 | Team B | Line Follow Lead — IR sensors + PID code |
| Member 4 | Team B | Line Follow Testing — track validation |
| Member 5 | Team C | Obstacle Lead — HC-SR04 + avoidance logic |
| Member 6 | Team C | Obstacle Testing — distance calibration |
| Member 7 | Team D | GPS Integration — NEO-6M + TinyGPSPlus |
| Member 8 | Team D | IMU Integration — MPU-6050 + bias calibration |
| Member 9 | Team E | EKF Developer — Arduino EKF + RF protocol |
| Member 10 | Team E | MATLAB Ground Station — Kalman + dashboard |

> 📌 **Supervised by: Dr. Ahmed** — Engineering Department

---

## ⚙️ Installation & Setup

### 1. Arduino Libraries (Library Manager)

```
RF24          by TMRh20
TinyGPSPlus   by Mikal Hart
MPU6050       by Electronic Cats
Wire          (built-in)
SoftwareSerial (built-in)
```

### 2. Clone the Repository

```bash
git clone https://github.com/your-username/smart-autonomous-car.git
cd smart-autonomous-car
```

### 3. Repository Structure

```
smart-autonomous-car/
├── arduino/
│   ├── car_transmitter/
│   │   └── car_transmitter.ino    # Car code (upload to car Nano)
│   └── ground_receiver/
│       └── ground_receiver.ino   # Receiver code (upload to ground Nano)
├── matlab/
│   └── kalman_dashboard.m        # MATLAB Kalman + live dashboard
├── docs/
│   ├── wiring_diagram.png
│   └── SmartCar_Documentation.docx
└── README.md
```

### 4. Upload Arduino Code

```bash
# Upload to car Arduino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano arduino/car_transmitter/

# Upload to ground station Arduino
arduino-cli upload -p /dev/ttyUSB1 --fqbn arduino:avr:nano arduino/ground_receiver/
```

### 5. Run MATLAB Dashboard

```matlab
% Open MATLAB, navigate to /matlab folder
% Edit the COM port first:
portName = 'COM3';   % Windows: COM3, COM4...
                     % Linux:  /dev/ttyUSB0
                     % Mac:    /dev/cu.usbserial-...

% Run the script
kalman_dashboard
```

---

## 🚀 Usage

### Step 1 — Verify RF Link

Upload the ping test to both Arduinos and check Serial Monitor. You should see `Sent OK` at > 95% rate.

### Step 2 — Set GPS Waypoint

In `car_transmitter.ino`, set your target coordinates:

```cpp
const float TARGET_LAT = 30.0444f;   // ← your target latitude
const float TARGET_LON = 31.2357f;   // ← your target longitude
const float ARRIVE_RADIUS = 1.5f;    // arrival threshold (meters)
```

### Step 3 — Power On Sequence

1. Power car with LiPo battery
2. Connect ground station Arduino via USB
3. Wait for GPS fix (LED on NEO-6M goes solid, ~60 s outdoors)
4. Run MATLAB script — dashboard should start filling with data
5. Place car on line track and release

### Step 4 — Watch the Dashboard

| Subplot | What to Watch |
|---------|---------------|
| `[2,2]` Bias | Should converge to stable value in ~15 s |
| `[3,1]` σ_alt | Should drop from ~7 m to under 2 m |
| `[4,1]` Innovation | Should stay near zero, random |
| `[4,2]` GPS Path | Should draw recognizable car trajectory |

---

## 📺 MATLAB Dashboard

> Insert MATLAB screenshot here

```
[ Screenshot of live 8-subplot dashboard ]
[ Replace this block with your actual MATLAB figure export ]
```

Export your dashboard screenshot with:

```matlab
exportgraphics(fig, 'dashboard.png', 'Resolution', 300)
```

---

## 🔧 Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| MATLAB receives nothing | Wrong COM port | Check Device Manager, set correct `portName` |
| All data is NaN | `\r` in CSV line | Add `strtrim()`: `data = str2double(split(strtrim(readline(s)),','))` |
| Packet loss > 10% | Motor RF noise | Add 10µF + 100nF caps to nRF24L01 VCC-GND |
| Bias never converges | Q(3,3) too large | Decrease `Q(3,3)` from `1e-5` to `1e-6` |
| σ_alt stays high | No GPS fix | Confirm `gps.location.isValid() == true`. Go outdoors. |
| Innovation grows | Filter diverging | Reset: `P = eye(3)*50; x = [0;0;0]`. Re-tune Q and R. |
| Car ignores waypoint | EKF origin not set | Check `ekf.originSet == true` before GPS_NAV |
| nRF24L01 gets hot | Power issue | Verify 3.3V supply. Check MOSI/MISO not swapped. |
| Car wobbles on line | TURN_SPEED too low | Increase `TURN_SPEED` from 110 toward 130 |

---

## 🔮 Future Work

- [ ] **Upgrade to STM32 / ESP32** — more RAM for 9-state EKF + WiFi dashboard
- [ ] **LoRa SX1276** — extend RF range from 100 m to 1–10 km
- [ ] **MATLAB geoplot()** — overlay GPS path on real map tiles
- [ ] **Multiple GPS waypoints** — sequential mission planning
- [ ] **Magnetometer (MPU-9250)** — absolute heading, eliminates gyro drift
- [ ] **Web dashboard** — Node.js + Chart.js real-time browser interface
- [ ] **RMSE auto-logging** — compute and save filter accuracy after each run
- [ ] **Full 3D EKF** — extend to `[x,y,z,heading,pitch,roll,vx,vy,vz]`

---

## 📚 References

- Kalman, R.E. (1960). *A New Approach to Linear Filtering and Prediction Problems.* ASME Trans.
- [RF24 Library](https://github.com/nRF24/RF24) — TMRh20 (MIT License)
- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) — Mikal Hart (MIT License)
- [MPU-6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) — InvenSense
- [NEO-6M Datasheet](https://www.u-blox.com/en/product/neo-6-series) — u-blox
- Grewal, M.S. & Andrews, A.P. — *Kalman Filtering: Theory and Practice.* Wiley-IEEE Press
- [MATLAB serialport()](https://www.mathworks.com/help/matlab/ref/serialport.html) — MathWorks Docs

---

## 📄 License

```
MIT License

Copyright (c) 2024 Smart Autonomous Car Team — Dr. Ahmed

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software.
```

---

<div align="center">

**Built with ❤️ by the Engineering Team — 2024**  
*Supervised by Dr. Ahmed*

[![GitHub Stars](https://img.shields.io/github/stars/your-username/smart-autonomous-car?style=social)](https://github.com/your-username/smart-autonomous-car)

</div>
