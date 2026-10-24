# Raspberry Pi Pico 2 + BMI160 Flight Controller Wiring & Setup Guide

A complete, standalone **2-Axis (Roll & Pitch) Zero-Drift Attitude Reference Flight Controller** running on the **Raspberry Pi Pico 2 (RP2350 Arm Cortex-M33 @ 250 MHz)** paired exclusively with the **Bosch Sensortec BMI160 6-Axis IMU** over **High-Speed SPI0 (10 MHz)**.

---

## ⚡ Hardware Pinout & Wiring Connections

The Bosch BMI160 connects directly to the Raspberry Pi Pico 2 via the hardware **SPI0** bus and an active-high Data Ready interrupt line:

| BMI160 Pin | Pico 2 GPIO | Physical Pin Number | Function | Details |
| :--- | :--- | :--- | :--- | :--- |
| **VCC / VDD** | **3V3 (OUT)** | **Pin 36** | Power (3.3V) | Clean regulated 3.3V rail from Pico 2 onboard buck-boost |
| **GND** | **GND** | **Pin 38** (or Pin 3, 8, 13, 18, 23, 28, 33) | Ground | Common reference ground |
| **CS / CSB** | **GP17** | **Pin 22** | SPI0 Chip Select | Active-LOW chip select driven by firmware |
| **SCK / SCL** | **GP18** | **Pin 24** | SPI0 Clock | High-speed SPI clock operating at **10 MHz** (Mode 0) |
| **SDI / MOSI / SDA** | **GP19** | **Pin 25** | SPI0 TX (Master Out) | Pico TX $\to$ BMI160 SDI data line |
| **SDO / MISO / SA0** | **GP16** | **Pin 21** | SPI0 RX (Master In) | BMI160 SDO $\to$ Pico RX data line |
| **INT1** | **GP20** | **Pin 26** | Hardware Interrupt | Active-HIGH Data Ready trigger (1600 Hz) |

> [!NOTE]
> **Bosch SPI Mode Switch**:
> The BMI160 powers up in I2C mode by default. The driver automatically performs a single dummy read on register `0x7F` over SPI, which switches the internal interface multiplexer into **SPI Mode 0** permanently.

---

## 🎯 Why Yaw is Removed (Mathematical Stability)

In a 6-DOF IMU (accelerometer + gyroscope only, without an external magnetic compass):
- The accelerometer only measures the gravity vector $\mathbf{g} = [0, 0, 1]^T$.
- Gravity provides an absolute, bounded reference for **Roll** and **Pitch**.
- Any rotation around the vertical gravity axis (**Yaw**) produces **zero accelerometer change**.
- As a consequence, estimating Yaw in 6-DOF relies on open-loop gyro integration ($\int \omega_z dt$), which inevitably wanders and drifts over time.

By removing Yaw from the attitude state:
1. **100% State Observability**: Every state in the 4-state error vector $[\delta \theta_x, \delta \theta_y, \delta b_x, \delta b_y]$ is directly observable from gravity.
2. **Zero Yaw Drift Forever**: The attitude indicator and artificial horizon remain rock-solid and aligned with gravity for hours of operation.
3. **No Covariance Degeneracy**: The Kalman filter covariance matrix $P$ remains strictly positive-definite under all flight conditions.

---

## 🚀 Flashing & Running

### Option 1: Automatic Flash via picotool
If the Pico 2 is connected to USB:
```bash
picotool load -f -x -v build/Pico2_BMI160_Flight_Controller.uf2
```

### Option 2: Drag and Drop via BOOTSEL
1. Hold down the **`BOOTSEL`** button on your Pico 2.
2. Plug the USB cable into your computer and release the button.
3. Drag and drop `build/Pico2_BMI160_Flight_Controller.uf2` onto the mounted `RP2350` drive.

---

## 🎮 3D WebGL Flight Simulator (Zero Dependencies)

1. Open [`web/index.html`](file:///mnt/Vol_B/HobbyProject/Pico2_BMI160_Flight_Controller/web/index.html) in **Google Chrome**, **Brave**, or **Microsoft Edge**.
2. Click the **`🔌 Connect USB (Pico 2)`** button in the top bar.
3. Select `Pico - Board CDC (/dev/ttyACM0)` and click **Connect**.
4. The 3D aircraft and Artificial Horizon (ADI) will instantly track the physical roll and pitch of your BMI160!
