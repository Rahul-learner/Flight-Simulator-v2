/**
 * ============================================================================
 *   Pico 2 BMI160 Flight Controller - 2-Axis Zero-Drift Attitude Deck
 * ============================================================================
 * 
 * Target Hardware:
 * - Microcontroller: Raspberry Pi Pico 2 (RP2350 Arm Cortex-M33 @ 250 MHz)
 * - Sensor: Bosch Sensortec BMI160 6-Axis IMU (SPI0 @ 10 MHz)
 * 
 * Mathematical Architecture:
 * - 4-State Error-State Extended Kalman Filter (MEKF)
 * - Estimator: Pure Roll & Pitch (Yaw removed to eliminate unobservable heading drift)
 * - Gravity vector tracking with closed-form Rodrigues rotation propagation
 * - Joseph-form stabilized covariance update (P guaranteed positive-definite)
 * - Dynamic centrifugal / linear G-force rejection
 * 
 * Pinout Connections (SPI0):
 * - GP16 (Pin 21, SPI0 RX / MISO) <--- BMI160 SDO / MISO
 * - GP17 (Pin 22, SPI0 CSn / CS)   ---> BMI160 CS
 * - GP18 (Pin 24, SPI0 SCK)       ---> BMI160 SCK
 * - GP19 (Pin 25, SPI0 TX / MOSI)  ---> BMI160 SDI / MOSI
 * - GP20 (Pin 26, GPIO Input)     <--- BMI160 INT1 (Data Ready)
 * - 3V3  (Pin 36, 3.3V Power)     ---> BMI160 VDD / VDDIO
 * - GND  (Pin 38, Ground)         ---> BMI160 GND
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

#include "BMI160_SPI.hpp"
#include "RollPitchAttitudeEKF.hpp"

// --- Pin Definitions ---
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

#define BMI160_SPI_PORT  spi0
#define BMI160_PIN_MISO  16 // GP16 (Pin 21)
#define BMI160_PIN_CS    17 // GP17 (Pin 22)
#define BMI160_PIN_SCK   18 // GP18 (Pin 24)
#define BMI160_PIN_MOSI  19 // GP19 (Pin 25)
#define BMI160_PIN_INT   20 // GP20 (Pin 26)

// Optional Secondary UART0 (Pin 1 TX, Pin 2 RX)
#define TELEM_UART_PORT      uart0
#define TELEM_UART_TX_PIN    0
#define TELEM_UART_RX_PIN    1
#define TELEM_UART_BAUD_RATE 115200

// --- 23-Byte Binary Telemetry Packet ---
#pragma pack(push, 1)
struct TelemetryPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    int16_t roll;            // deg * 100 (-18000 to 18000)
    int16_t pitch;           // deg * 100 (-9000 to 9000)
    int16_t yaw = 0;         // 0 (Yaw removed)
    int16_t dt;              // loop microseconds
    int16_t alt_cm = 0;      // cm AGL
    int16_t vsi_cms = 0;     // cm/s vertical speed
    int16_t prs_hpa10 = 10132;// hPa * 10
    int16_t roll_pid = 0;
    int16_t pitch_pid = 0;
    int16_t yaw_pid = 0;
    uint8_t checksum;        // XOR of 20 bytes from roll to yaw_pid
};
#pragma pack(pop)

// --- Telemetry Output Modes ---
enum TelemetryMode {
    TELEM_MODE_JSON = 0,   // JSON Lines for 3D Flight Deck WebSerial
    TELEM_MODE_BINARY = 1, // 23-byte binary packets for ESP8266
    TELEM_MODE_HUMAN = 2   // Human-readable terminal HUD
};

// --- Instances ---
BMI160_SPI imu(BMI160_SPI_PORT,
               BMI160_PIN_CS,
               BMI160_PIN_SCK,
               BMI160_PIN_MOSI,
               BMI160_PIN_MISO,
               BMI160_PIN_INT);

RollPitchAttitudeEKF ekf;

// Volatile flag toggled by BMI160 Data Ready interrupt
volatile bool new_imu_data = false;

// 2-Axis Attitude angles (degrees)
float roll_deg = 0.0f;
float pitch_deg = 0.0f;

// Physical sensor values
float ax_g = 0.0f, ay_g = 0.0f, az_g = 1.0f;
float gx_rad = 0.0f, gy_rad = 0.0f, gz_rad = 0.0f;

// Telemetry configuration
static TelemetryMode telem_mode = TELEM_MODE_JSON;
static uint32_t telem_divider = 32; // At 1600 Hz IMU rate, 32 iterations = 50 Hz telemetry
static uint32_t loop_counter = 0;
static uint32_t imu_sample_count = 0;
static uint32_t actual_imu_hz = 0;

// Command line buffer for USB / UART RX
#define CMD_BUF_SIZE 64
static char usb_cmd_buf[CMD_BUF_SIZE];
static uint8_t usb_cmd_idx = 0;

// --- Interrupt Service Routine for BMI160 Data Ready ---
void bmi160_interrupt_handler(uint gpio, uint32_t events) {
    if (gpio == BMI160_PIN_INT) {
        new_imu_data = true;
    }
}

// --- Sensor Alignment and Calibration ---
void calibrate_and_init_ahrs() {
    printf("\n===========================================================\n");
    printf("   Calibrating BMI160 & Initializing 2-Axis Roll-Pitch EKF \n");
    printf("   Keep aircraft STATIONARY on a level surface...          \n");
    printf("===========================================================\n");

    // Perform stationary sample collection (600 samples)
    imu.calibrateStationary(600, PICO_DEFAULT_LED_PIN);

    // Initial gravity vector readings
    float ax_init, ay_init, az_init, gx_init, gy_init, gz_init;
    imu.readCalibrated(ax_init, ay_init, az_init, gx_init, gy_init, gz_init);

    // Initialize EKF gravity vector directly from level accelerometer measurements
    ekf.initFromSensors(ax_init, ay_init, az_init, 0.0f, 0.0f, 0.0f);
    ekf.getEuler(roll_deg, pitch_deg);

    printf(">>> Attitude Filter Online (Zero-Drift Roll & Pitch):\n");
    printf("    Gravity Norm : %.3f g\n", sqrtf(ax_init*ax_init + ay_init*ay_init + az_init*az_init));
    printf("    Initial Roll : %+.2f° | Pitch: %+.2f° (Yaw removed)\n", roll_deg, pitch_deg);
    printf("    Telemetry    : %s @ 50 Hz\n\n",
           telem_mode == TELEM_MODE_JSON ? "JSON (WebSerial / WebGL)" :
           telem_mode == TELEM_MODE_BINARY ? "Binary (23-byte)" : "Human Debug");

    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

// --- Process Interactive Commands ---
void process_command_string(const char* cmd) {
    if (strcasecmp(cmd, "CALIB") == 0 || strcasecmp(cmd, "C") == 0) {
        calibrate_and_init_ahrs();
        printf("{\"msg\":\"Recalibration complete\"}\n");
    }
    else if (strcasecmp(cmd, "LEVEL") == 0 || strcasecmp(cmd, "L") == 0 || strcasecmp(cmd, "TARE") == 0) {
        // Re-align gravity vector to current level reading
        ekf.initFromSensors(ax_g, ay_g, az_g);
        printf("{\"msg\":\"Level orientation aligned to zero roll & pitch\"}\n");
    }
    else if (strncasecmp(cmd, "MODE", 4) == 0) {
        int m = atoi(cmd + 4);
        if (m >= 0 && m <= 2) {
            telem_mode = (TelemetryMode)m;
            printf("{\"msg\":\"Telemetry mode switched\",\"mode\":%d}\n", m);
        } else {
            printf("{\"error\":\"Invalid mode. Use: 0=JSON, 1=Binary, 2=Human\"}\n");
        }
    }
    else if (strncasecmp(cmd, "RATE", 4) == 0) {
        int hz = atoi(cmd + 4);
        if (hz >= 1 && hz <= 500) {
            telem_divider = 1600 / hz;
            if (telem_divider < 1) telem_divider = 1;
            printf("{\"msg\":\"Telemetry rate set\",\"hz\":%d,\"divider\":%lu}\n", hz, telem_divider);
        } else {
            printf("{\"error\":\"Invalid rate (1 - 500 Hz)\"}\n");
        }
    }
    else if (strcasecmp(cmd, "INFO") == 0 || strcasecmp(cmd, "?") == 0) {
        printf("\n--- Pico 2 BMI160 2-Axis Attitude Flight Controller ---\n");
        printf("MCU Core        : RP2350 Arm Cortex-M33 @ %lu MHz (Hardware FPU)\n", clock_get_hz(clk_sys) / 1000000);
        printf("Sensor          : Bosch BMI160 (Chip ID: 0x%02X)\n", imu.getChipId());
        printf("Filter Type     : 2-Axis Roll-Pitch MEKF (Yaw Removed / Zero Drift)\n");
        printf("SPI Bus         : SPI0 @ 10 MHz (SCK=GP%d, MOSI=GP%d, MISO=GP%d, CS=GP%d)\n",
               BMI160_PIN_SCK, BMI160_PIN_MOSI, BMI160_PIN_MISO, BMI160_PIN_CS);
        printf("Actual IMU Rate : %lu Hz\n", actual_imu_hz);
        printf("Attitude State  : Roll=%+.2f°, Pitch=%+.2f° (Yaw=0.0° locked)\n", roll_deg, pitch_deg);
        printf("Gravity Vector  : [%+.3f, %+.3f, %+.3f]\n", ekf.gb[0], ekf.gb[1], ekf.gb[2]);
        printf("Gyro Biases     : X=%+.3f, Y=%+.3f deg/s\n", ekf.bg[0] * 57.2958f, ekf.bg[1] * 57.2958f);
        printf("-------------------------------------------------------\n\n");
    }
    else {
        printf("{\"unknown_command\":\"%s\",\"help\":\"Commands: CALIB, LEVEL, MODE <0-2>, RATE <hz>, INFO\"}\n", cmd);
    }
}

// --- Check Incoming Commands from USB Serial ---
void handle_serial_commands() {
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (usb_cmd_idx > 0) {
                usb_cmd_buf[usb_cmd_idx] = '\0';
                process_command_string(usb_cmd_buf);
                usb_cmd_idx = 0;
            }
        } else if (usb_cmd_idx < (CMD_BUF_SIZE - 1)) {
            usb_cmd_buf[usb_cmd_idx++] = (char)c;
        }
        c = getchar_timeout_us(0);
    }
}

int main() {
    // --- 1. Clock Configuration: Overclock RP2350 Cortex-M33 to 250 MHz ---
    set_sys_clock_khz(250000, true);

    // --- 2. Initialize USB Serial ---
    stdio_init_all();
    sleep_ms(1500); // Allow PC serial terminal to connect

    printf("\n===========================================================\n");
    printf("   Pico 2 BMI160 Flight Controller - Roll & Pitch VRU     \n");
    printf("   RP2350 Core @ %lu MHz | Hardware FPU | Yaw-Free MEKF   \n", clock_get_hz(clk_sys) / 1000000);
    printf("===========================================================\n\n");

    // --- 3. Initialize Status LED ---
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // --- 4. Initialize Optional Secondary Hardware UART0 ---
    uart_init(TELEM_UART_PORT, TELEM_UART_BAUD_RATE);
    gpio_set_function(TELEM_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TELEM_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(TELEM_UART_PORT, false, false);
    uart_set_format(TELEM_UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(TELEM_UART_PORT, true);

    // --- 5. Initialize BMI160 IMU via SPI0 ---
    printf("[INIT] Initializing BMI160 over SPI0 (SCK=GP%d, MOSI=GP%d, MISO=GP%d, CS=GP%d)...\n",
           BMI160_PIN_SCK, BMI160_PIN_MOSI, BMI160_PIN_MISO, BMI160_PIN_CS);

    if (!imu.init(BMI160_SPI::ACCEL_RANGE_8G, BMI160_SPI::GYRO_RANGE_2000DPS)) {
        printf("[ERROR] Failed to communicate with BMI160! Check wiring.\n");
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    }

    // --- 6. Attach Hardware Interrupt on INT1 Pin ---
    gpio_set_irq_enabled_with_callback(BMI160_PIN_INT, GPIO_IRQ_EDGE_RISE, true, &bmi160_interrupt_handler);
    printf("[INIT] Data Ready interrupt attached to GP%d.\n", BMI160_PIN_INT);

    // --- 7. Perform Calibration & State Estimator Initialization ---
    calibrate_and_init_ahrs();

    // Loop timing
    uint64_t last_update_us = time_us_64();
    uint64_t last_rate_check_us = time_us_64();

    // Pure Roll/Pitch Quaternion buffer
    float q[4];

    // Main flight loop
    while (true) {
        // Check for interactive user commands over USB
        handle_serial_commands();

        // Check if new IMU data is available from BMI160 Data Ready interrupt
        if (new_imu_data || gpio_get(BMI160_PIN_INT)) {
            new_imu_data = false;
            imu_sample_count++;

            // 1. Read calibrated physical measurements from BMI160 (12-byte burst read)
            imu.readCalibrated(ax_g, ay_g, az_g, gx_rad, gy_rad, gz_rad);

            // 2. Calculate precise elapsed time dt
            uint64_t now_us = time_us_64();
            float dt_s = (float)(now_us - last_update_us) * 1.0e-6f;
            last_update_us = now_us;

            if (dt_s <= 0.0f || dt_s > 0.05f) {
                dt_s = 0.000625f; // Nominal 1600 Hz step
            }

            // 3. EKF Prediction: Exact Rodrigues gravity vector rotation
            ekf.predict(dt_s, gx_rad, gy_rad, gz_rad);

            // 4. EKF Measurement Update: Gravity alignment with dynamic G-rejection
            ekf.update(ax_g, ay_g, az_g);

            // 5. Extract Roll & Pitch Euler angles (Zero Yaw)
            ekf.getEuler(roll_deg, pitch_deg);
            ekf.getQuaternion(q);

            // 6. Real-Time Telemetry Stream
            loop_counter++;
            if (loop_counter >= telem_divider) {
                loop_counter = 0;

                // Format 1: JSON Line (Native format for 3D Flight Deck WebSerial)
                if (telem_mode == TELEM_MODE_JSON) {
                    printf("{\"r\":%.2f,\"p\":%.2f,\"y\":0.00,\"q\":[%.4f,%.4f,%.4f,%.4f],\"a\":[%.2f,%.2f,%.2f],\"g\":[%.3f,%.3f,%.3f],\"dt\":%.5f}\n",
                           roll_deg, pitch_deg,
                           q[0], q[1], q[2], q[3],
                           ax_g, ay_g, az_g,
                           gx_rad, gy_rad, gz_rad,
                           dt_s);
                }
                // Format 2: 23-Byte Binary Packet (Yaw = 0)
                else if (telem_mode == TELEM_MODE_BINARY) {
                    TelemetryPacket pkt;
                    pkt.roll = (int16_t)(roll_deg * 100.0f);
                    pkt.pitch = (int16_t)(pitch_deg * 100.0f);
                    pkt.yaw = 0; // Pure Roll & Pitch
                    pkt.dt = (int16_t)(dt_s * 1000000.0f);
                    pkt.alt_cm = 0;
                    pkt.vsi_cms = 0;
                    pkt.prs_hpa10 = 10132;
                    pkt.roll_pid = (int16_t)(gx_rad * 100.0f);
                    pkt.pitch_pid = (int16_t)(gy_rad * 100.0f);
                    pkt.yaw_pid = 0;

                    uint8_t chk = 0;
                    uint8_t* p = (uint8_t*)&pkt.roll;
                    for (size_t i = 0; i < 20; i++) chk ^= p[i];
                    pkt.checksum = chk;

                    // Send to USB CDC
                    fwrite(&pkt, sizeof(pkt), 1, stdout);
                    // Also mirror to secondary UART
                    uart_write_blocking(TELEM_UART_PORT, (const uint8_t*)&pkt, sizeof(pkt));
                }
                // Format 3: Human-Readable Debug HUD
                else if (telem_mode == TELEM_MODE_HUMAN) {
                    printf("> Roll: %+.1f° | Pitch: %+.1f° | (Yaw Removed) | Rate: %4lu Hz | |A|: %.2f g\n",
                           roll_deg, pitch_deg, actual_imu_hz,
                           sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g));
                }
            }

            // Periodic Sample Rate Counter (every 1 second)
            if ((now_us - last_rate_check_us) >= 1000000) {
                actual_imu_hz = imu_sample_count;
                imu_sample_count = 0;
                last_rate_check_us = now_us;
            }

        } else {
            sleep_us(1);
        }
    }

    return 0;
}
