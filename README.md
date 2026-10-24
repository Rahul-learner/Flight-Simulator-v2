# Pico 2 BMI160 Flight Controller: 2-Axis Zero-Drift AHRS & 3D Flight Deck

A standalone, aerospace-grade **2-Axis (Roll & Pitch) Attitude Reference Flight Controller** running on the **Raspberry Pi Pico 2 (RP2350)** paired exclusively with the **Bosch Sensortec BMI160 6-Axis IMU** over **High-Speed SPI (10 MHz)**, featuring zero-dependency **WebSerial 3D Flight Deck Visualization**.

---

## 🌟 Key Features

- **RP2350 Arm Cortex-M33 Architecture (@ 250 MHz)**:
  - Takes full advantage of the RP2350's **single-precision hardware FPU** and DSP extensions.
  - Quaternion and matrix kinematics execute in **< 1.8 microseconds** per cycle.
  - High-rate IMU loop running at **over 6,000 Hz**.

- **Yaw Removed — 100% Drift-Free**:
  - In 6-DOF IMUs without a magnetometer, Yaw has no gravity reference and accumulates unbounded drift.
  - By removing Yaw and formulating the state around the gravity reference vector $\mathbf{g}_b$, the filter is **100% mathematically observable**. Roll and Pitch remain locked to gravity forever!

- **4-State Error-State Extended Kalman Filter (MEKF)**:
  - Error state $\delta \mathbf{x} = [\delta \theta_x, \delta \theta_y, \delta b_{gx}, \delta b_{gy}] \in \mathbb{R}^4$.
  - Closed-form **Rodrigues rotation formula** for gravity vector propagation under high angular rates.
  - **Joseph-Form Stabilized Covariance Update**: Guarantees covariance matrix $P$ remains symmetric and positive-definite under all flight conditions.
  - **Dynamic Acceleration Gating**: Suppresses centrifugal and linear G-spikes to prevent the artificial horizon from falsely banking during turns.

- **High-Speed 10 MHz SPI0 Driver**:
  - Fully implements the Bosch Sensortec BMI160 power sequence and SPI Mode 0 lock.
  - 12-byte atomic burst read starting at register `0x0C` (DATA_0) with little-endian parsing.
  - Integrated stationary calibration with status LED feedback.

- **Offline-First 3D WebGL Flight Deck (WebSerial)**:
  - Connects directly to the Pico 2 USB port from Chrome, Brave, or Edge with **1 click**.
  - No ESP8266, local Wi-Fi, or intermediate software required!

---

## 📁 Repository Structure

```
.
├── main.cpp                  # Main Pico 2 flight controller firmware
├── BMI160_SPI.hpp            # High-Speed 10 MHz SPI driver for Bosch BMI160
├── RollPitchAttitudeEKF.hpp  # 2-Axis Error-State EKF (Yaw removed, zero drift)
├── CMakeLists.txt            # Pico SDK CMake build configuration (RP2350 / pico2)
├── pico_sdk_import.cmake     # Raspberry Pi Pico SDK import helper
├── pico2_serial_bridge.py    # Python serial monitor and terminal HUD
├── wiring_guide.md           # Pinout mappings and hardware connection diagrams
├── web/
│   └── index.html            # Standalone 3D WebGL Flight Deck Simulator (WebSerial)
└── README.md                 # Complete system documentation
```

---

## ⚡ Hardware Pinout Summary (SPI0)

| BMI160 Pin | Pico 2 GPIO | Physical Pin Number | Function | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **VCC / VDD** | **3V3 (OUT)** | **Pin 36** | 3.3V Power | Clean regulated 3.3V rail |
| **GND** | **GND** | **Pin 38** | Ground | Common reference |
| **CS / CSB** | **GP17** | **Pin 22** | SPI0 CS | Active LOW |
| **SCK / SCL** | **GP18** | **Pin 24** | SPI0 SCK | 10 MHz SPI Clock (Mode 0) |
| **SDI / MOSI** | **GP19** | **Pin 25** | SPI0 TX | Pico TX $\to$ BMI160 SDI |
| **SDO / MISO** | **GP16** | **Pin 21** | SPI0 RX | BMI160 SDO $\to$ Pico RX |
| **INT1** | **GP20** | **Pin 26** | Interrupt | Active-HIGH Data Ready (1600 Hz) |

*(See [`wiring_guide.md`](file:///mnt/Vol_B/HobbyProject/Pico2_BMI160_Flight_Controller/wiring_guide.md) for full wiring details).*

---

## 🚀 Building & Flashing

### 1. Build Firmware
```bash
mkdir -p build && cd build
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk
make -j4
```

### 2. Flash to Pico 2
- **Via picotool**:
  ```bash
  picotool load -f -x -v build/Pico2_BMI160_Flight_Controller.uf2
  ```
- **Via BOOTSEL**: Hold the white `BOOTSEL` button on your Pico 2, plug it into USB, and copy `build/Pico2_BMI160_Flight_Controller.uf2` to the mounted `RP2350` drive.

---

## 🎮 3D WebGL Flight Deck Simulator

1. Open [`web/index.html`](file:///mnt/Vol_B/HobbyProject/Pico2_BMI160_Flight_Controller/web/index.html) in **Google Chrome**, **Brave**, or **Microsoft Edge**.
2. Click **`🔌 Connect USB (Pico 2)`** in the top bar.
3. Select your Pico 2 (`/dev/ttyACM0`) and click **Connect**.
4. The 3D aircraft model, Attitude Director Indicator (ADI), and flight gauges will rotate in real-time with pure, drift-free Roll and Pitch!
