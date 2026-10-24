#pragma once

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

/**
 * @file BMI160_SPI.hpp
 * @brief High-Performance SPI Driver for Bosch Sensortec BMI160 6-Axis IMU
 *        optimized for Raspberry Pi Pico 2 (RP2350 Cortex-M33).
 * 
 * Hardware Connections (Default SPI0):
 * - SCK   -> Pico GP18 (Pin 24, SPI0 SCK)
 * - MOSI  -> Pico GP19 (Pin 25, SPI0 TX)
 * - MISO  -> Pico GP16 (Pin 21, SPI0 RX)
 * - CS    -> Pico GP17 (Pin 22, Software / Hardware Chip Select)
 * - INT1  -> Pico GP20 (Pin 26, Data Ready Interrupt)
 * - VCC   -> Pico 3V3  (Pin 36, 3.3V)
 * - GND   -> Pico GND  (Pin 38, Ground)
 */

class BMI160_SPI {
public:
    // --- Register Addresses ---
    static constexpr uint8_t REG_CHIP_ID       = 0x00;
    static constexpr uint8_t REG_ERR_REG       = 0x02;
    static constexpr uint8_t REG_PMU_STATUS    = 0x03;
    static constexpr uint8_t REG_DATA_START    = 0x0C; // Gyro X Low Byte starts here (12-byte burst)
    static constexpr uint8_t REG_STATUS        = 0x1B;
    static constexpr uint8_t REG_TEMPERATURE_0 = 0x20;
    static constexpr uint8_t REG_TEMPERATURE_1 = 0x21;
    static constexpr uint8_t REG_ACC_CONF      = 0x40;
    static constexpr uint8_t REG_ACC_RANGE     = 0x41;
    static constexpr uint8_t REG_GYR_CONF      = 0x42;
    static constexpr uint8_t REG_GYR_RANGE     = 0x43;
    static constexpr uint8_t REG_INT_EN_1      = 0x51;
    static constexpr uint8_t REG_INT_OUT_CTRL  = 0x53;
    static constexpr uint8_t REG_INT_MAP_1     = 0x56;
    static constexpr uint8_t REG_CMD           = 0x7E;

    // --- Commands ---
    static constexpr uint8_t CMD_SOFT_RESET    = 0xB6;
    static constexpr uint8_t CMD_ACC_NORMAL    = 0x11;
    static constexpr uint8_t CMD_GYR_NORMAL    = 0x15;
    static constexpr uint8_t CMD_INT_RESET     = 0x0A;

    // --- Expected Chip IDs ---
    static constexpr uint8_t CHIP_ID_DEFAULT   = 0xD1; // Standard Bosch BMI160
    static constexpr uint8_t CHIP_ID_ALT1      = 0xD3;
    static constexpr uint8_t CHIP_ID_ALT2      = 0xD8;

    // Accel Full Scale Ranges
    enum AccelRange : uint8_t {
        ACCEL_RANGE_2G  = 0x03, // +/- 2g  (16384 LSB/g)
        ACCEL_RANGE_4G  = 0x05, // +/- 4g  (8192 LSB/g)
        ACCEL_RANGE_8G  = 0x08, // +/- 8g  (4096 LSB/g)
        ACCEL_RANGE_16G = 0x0C  // +/- 16g (2048 LSB/g)
    };

    // Gyro Full Scale Ranges
    enum GyroRange : uint8_t {
        GYRO_RANGE_2000DPS = 0x00, // +/- 2000 deg/s (16.4 LSB/(deg/s))
        GYRO_RANGE_1000DPS = 0x01, // +/- 1000 deg/s (32.8 LSB/(deg/s))
        GYRO_RANGE_500DPS  = 0x02, // +/- 500 deg/s  (65.6 LSB/(deg/s))
        GYRO_RANGE_250DPS  = 0x03, // +/- 250 deg/s  (131.2 LSB/(deg/s))
        GYRO_RANGE_125DPS  = 0x04  // +/- 125 deg/s  (262.4 LSB/(deg/s))
    };

    // Output Data Rates (ODR)
    enum ODR : uint8_t {
        ODR_100HZ  = 0x08,
        ODR_200HZ  = 0x09,
        ODR_400HZ  = 0x0A,
        ODR_800HZ  = 0x0B,
        ODR_1600HZ = 0x0C,
        ODR_3200HZ = 0x0D // Gyro only
    };

private:
    spi_inst_t* _spi;
    uint _cs_pin;
    uint _sck_pin;
    uint _mosi_pin;
    uint _miso_pin;
    uint _int_pin;

    float _accel_scale; // LSB per g
    float _gyro_scale;  // LSB per (deg/s)

    // Calibration bias offsets
    float _gyro_bias_x;
    float _gyro_bias_y;
    float _gyro_bias_z;
    float _accel_bias_x;
    float _accel_bias_y;
    float _accel_bias_z;

    uint8_t _detected_chip_id;

    inline void cs_select() {
        asm volatile("nop \n nop \n nop \n nop");
        gpio_put(_cs_pin, 0);
        asm volatile("nop \n nop \n nop \n nop");
    }

    inline void cs_deselect() {
        asm volatile("nop \n nop \n nop \n nop");
        gpio_put(_cs_pin, 1);
        asm volatile("nop \n nop \n nop \n nop");
    }

public:
    BMI160_SPI(spi_inst_t* spi_port = spi0,
               uint cs_pin = 17,
               uint sck_pin = 18,
               uint mosi_pin = 19,
               uint miso_pin = 16,
               uint int_pin = 20)
        : _spi(spi_port),
          _cs_pin(cs_pin),
          _sck_pin(sck_pin),
          _mosi_pin(mosi_pin),
          _miso_pin(miso_pin),
          _int_pin(int_pin),
          _accel_scale(4096.0f),
          _gyro_scale(16.4f),
          _gyro_bias_x(0.0f),
          _gyro_bias_y(0.0f),
          _gyro_bias_z(0.0f),
          _accel_bias_x(0.0f),
          _accel_bias_y(0.0f),
          _accel_bias_z(0.0f),
          _detected_chip_id(0) {}

    void writeRegister(uint8_t reg, uint8_t data) {
        uint8_t buf[2];
        buf[0] = reg & 0x7F; // Write operation: MSB = 0
        buf[1] = data;
        cs_select();
        spi_write_blocking(_spi, buf, 2);
        cs_deselect();
    }

    uint8_t readRegister(uint8_t reg) {
        uint8_t reg_addr = reg | 0x80; // Read operation: MSB = 1
        uint8_t data = 0;
        cs_select();
        spi_write_blocking(_spi, &reg_addr, 1);
        spi_read_blocking(_spi, 0x00, &data, 1);
        cs_deselect();
        return data;
    }

    void readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
        uint8_t reg_addr = reg | 0x80; // Read operation: MSB = 1
        cs_select();
        spi_write_blocking(_spi, &reg_addr, 1);
        spi_read_blocking(_spi, 0x00, buffer, length);
        cs_deselect();
    }

    uint8_t getChipId() const {
        return _detected_chip_id;
    }

    /**
     * @brief Initialize SPI bus and verify communication with BMI160.
     * @return true if device detected and validated, false otherwise.
     */
    bool init(AccelRange a_range = ACCEL_RANGE_8G, GyroRange g_range = GYRO_RANGE_2000DPS) {
        // 1. Initialize SPI at safe initialization speed (1 MHz)
        spi_init(_spi, 1000 * 1000);
        spi_set_format(_spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

        // 2. Configure GPIO pin functions
        gpio_set_function(_sck_pin, GPIO_FUNC_SPI);
        gpio_set_function(_mosi_pin, GPIO_FUNC_SPI);
        gpio_set_function(_miso_pin, GPIO_FUNC_SPI);

        // Configure CS pin as output and set HIGH (deselected)
        gpio_init(_cs_pin);
        gpio_set_dir(_cs_pin, GPIO_OUT);
        gpio_put(_cs_pin, 1);

        // Small stabilization pause
        sleep_ms(15);

        // 3. MANDATORY BOSCH QUIRK:
        // BMI160 powers up in I2C protocol mode. Performing a dummy read over SPI
        // toggles the internal interface selector into pure SPI mode.
        readRegister(0x7F);
        sleep_ms(5);

        // 4. Verify WHO_AM_I / CHIP_ID
        _detected_chip_id = readRegister(REG_CHIP_ID);
        if (_detected_chip_id != CHIP_ID_DEFAULT &&
            _detected_chip_id != CHIP_ID_ALT1 &&
            _detected_chip_id != CHIP_ID_ALT2) {
            printf("[BMI160] ERROR: Invalid Chip ID 0x%02X (expected 0x%02X)!\n",
                   _detected_chip_id, CHIP_ID_DEFAULT);
            return false;
        }
        printf("[BMI160] Detected Chip ID: 0x%02X\n", _detected_chip_id);

        // 5. Issue Soft Reset
        writeRegister(REG_CMD, CMD_SOFT_RESET);
        sleep_ms(100); // Allow internal regulator & state machine reboot

        // Perform dummy read again after reset to lock SPI mode
        readRegister(0x7F);
        sleep_ms(5);

        // Clear any startup error flags in ERR_REG
        readRegister(REG_ERR_REG);

        // 6. BOSCH POWER-UP SEQUENCE:
        // Must command normal power modes before configuring registers!
        writeRegister(REG_CMD, CMD_ACC_NORMAL);
        sleep_ms(10);

        writeRegister(REG_CMD, CMD_GYR_NORMAL);
        sleep_ms(80);

        // Wait until both Accel and Gyro are awake (PMU_STATUS == 0x14)
        uint8_t pmu = readRegister(REG_PMU_STATUS);
        int retry = 0;
        while ((pmu & 0x3C) != 0x14 && retry++ < 20) {
            printf("[BMI160] Awaiting PMU wake (curr=0x%02X, retry %d)...\n", pmu, retry);
            writeRegister(REG_CMD, CMD_ACC_NORMAL);
            sleep_ms(10);
            writeRegister(REG_CMD, CMD_GYR_NORMAL);
            sleep_ms(50);
            pmu = readRegister(REG_PMU_STATUS);
        }

        if ((pmu & 0x3C) != 0x14) {
            printf("[BMI160] WARNING: PMU status 0x%02X; sensors may not be fully awake!\n", pmu);
        } else {
            printf("[BMI160] Power Online: Accel & Gyro in Normal Mode (PMU=0x%02X)\n", pmu);
        }

        // 7. Configure Ranges & Scales
        writeRegister(REG_ACC_RANGE, (uint8_t)a_range);
        switch (a_range) {
            case ACCEL_RANGE_2G:  _accel_scale = 16384.0f; break;
            case ACCEL_RANGE_4G:  _accel_scale = 8192.0f;  break;
            case ACCEL_RANGE_8G:  _accel_scale = 4096.0f;  break;
            case ACCEL_RANGE_16G: _accel_scale = 2048.0f;  break;
        }

        writeRegister(REG_GYR_RANGE, (uint8_t)g_range);
        switch (g_range) {
            case GYRO_RANGE_2000DPS: _gyro_scale = 16.4f;  break;
            case GYRO_RANGE_1000DPS: _gyro_scale = 32.8f;  break;
            case GYRO_RANGE_500DPS:  _gyro_scale = 65.6f;  break;
            case GYRO_RANGE_250DPS:  _gyro_scale = 131.2f; break;
            case GYRO_RANGE_125DPS:  _gyro_scale = 262.4f; break;
        }

        // 8. Configure Output Data Rate (ODR) & Filters
        // Accel: ODR = 1600 Hz (0x0C), Normal filter (0x20) -> 0x2C
        writeRegister(REG_ACC_CONF, 0x20 | (uint8_t)ODR_1600HZ);
        // Gyro: ODR = 1600 Hz (0x0C), Normal filter (0x20) -> 0x2C
        writeRegister(REG_GYR_CONF, 0x20 | (uint8_t)ODR_1600HZ);

        // 9. Configure Data Ready Interrupt on INT1 pin
        // INT_OUT_CTRL: Bit 1 (INT1_OUTPUT_EN) = 1, Bit 2 (INT1_OD) = 0 (push-pull), Bit 3 (INT1_LVL) = 1 (active high) -> 0x0A
        writeRegister(REG_INT_OUT_CTRL, 0x0A);
        // INT_MAP_1: Map Data Ready interrupt to INT1 (Bit 7 = 1) -> 0x80
        writeRegister(REG_INT_MAP_1, 0x80);
        // INT_EN_1: Enable Data Ready interrupt (Bit 4 = 1) -> 0x10
        writeRegister(REG_INT_EN_1, 0x10);

        // 10. Configure Interrupt Pin on Pico
        gpio_init(_int_pin);
        gpio_set_dir(_int_pin, GPIO_IN);
        gpio_pull_down(_int_pin); // Active high interrupt

        // 11. Ramp up SPI speed to 10 MHz for low-latency burst reads
        uint actual_baud = spi_set_baudrate(_spi, 10 * 1000 * 1000);
        printf("[BMI160] SPI Speed escalated to %u Hz (10 MHz nominal).\n", actual_baud);

        return true;
    }

    /**
     * @brief Fast 12-byte atomic burst read of raw Gyro and Accel values.
     * Starts at register 0x0C (DATA_0) and reads through 0x17.
     */
    inline void readRaw(int16_t& gx_raw, int16_t& gy_raw, int16_t& gz_raw,
                        int16_t& ax_raw, int16_t& ay_raw, int16_t& az_raw) {
        uint8_t buffer[12];
        readRegisters(REG_DATA_START, buffer, 12);

        // Little-endian byte reassembly:
        gx_raw = (int16_t)(((uint16_t)buffer[1] << 8)  | buffer[0]);
        gy_raw = (int16_t)(((uint16_t)buffer[3] << 8)  | buffer[2]);
        gz_raw = (int16_t)(((uint16_t)buffer[5] << 8)  | buffer[4]);

        ax_raw = (int16_t)(((uint16_t)buffer[7] << 8)  | buffer[6]);
        ay_raw = (int16_t)(((uint16_t)buffer[9] << 8)  | buffer[8]);
        az_raw = (int16_t)(((uint16_t)buffer[11] << 8) | buffer[10]);
    }

    /**
     * @brief Read calibrated physical measurements.
     * @param ax Accelerometer X in g's
     * @param ay Accelerometer Y in g's
     * @param az Accelerometer Z in g's
     * @param gx Gyroscope X in rad/s (for EKF)
     * @param gy Gyroscope Y in rad/s (for EKF)
     * @param gz Gyroscope Z in rad/s (for EKF)
     */
    inline void readCalibrated(float& ax, float& ay, float& az,
                               float& gx, float& gy, float& gz) {
        int16_t raw_gx, raw_gy, raw_gz;
        int16_t raw_ax, raw_ay, raw_az;

        readRaw(raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az);

        // Convert accelerometer to g's and subtract bias
        ax = ((float)raw_ax / _accel_scale) - _accel_bias_x;
        ay = ((float)raw_ay / _accel_scale) - _accel_bias_y;
        az = ((float)raw_az / _accel_scale) - _accel_bias_z;

        // Convert gyro to deg/s, subtract bias, then convert to rad/s
        float gx_dps = ((float)raw_gx / _gyro_scale) - _gyro_bias_x;
        float gy_dps = ((float)raw_gy / _gyro_scale) - _gyro_bias_y;
        float gz_dps = ((float)raw_gz / _gyro_scale) - _gyro_bias_z;

        constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
        gx = gx_dps * DEG_TO_RAD;
        gy = gy_dps * DEG_TO_RAD;
        gz = gz_dps * DEG_TO_RAD;
    }

    /**
     * @brief Read die temperature in degrees Celsius.
     */
    float readTemperature() {
        uint8_t buf[2];
        readRegisters(REG_TEMPERATURE_0, buf, 2);
        int16_t raw_temp = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        // BMI160 temperature formula: 23.0 C + raw_temp * 0.001953125 C
        return 23.0f + ((float)raw_temp * 0.001953125f);
    }

    /**
     * @brief Run stationary calibration routine to determine gyro and accel zero offsets.
     */
    void calibrateStationary(uint16_t samples = 600, uint led_pin = 25) {
        printf("\n>>> Starting BMI160 Stationary Calibration (%u samples)...\n", samples);
        printf(">>> Keep aircraft LEVEL and completely STILL...\n");

        float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
        float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;

        // Reset previous biases
        _gyro_bias_x = _gyro_bias_y = _gyro_bias_z = 0.0f;
        _accel_bias_x = _accel_bias_y = _accel_bias_z = 0.0f;

        for (uint16_t i = 0; i < samples; i++) {
            int16_t raw_gx, raw_gy, raw_gz;
            int16_t raw_ax, raw_ay, raw_az;
            readRaw(raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az);

            sum_gx += (float)raw_gx / _gyro_scale;
            sum_gy += (float)raw_gy / _gyro_scale;
            sum_gz += (float)raw_gz / _gyro_scale;

            sum_ax += (float)raw_ax / _accel_scale;
            sum_ay += (float)raw_ay / _accel_scale;
            sum_az += (float)raw_az / _accel_scale;

            // Blink LED during calibration
            if (led_pin != 0xFF) {
                gpio_put(led_pin, (i / 30) % 2);
            }
            sleep_ms(3);
        }

        _gyro_bias_x = sum_gx / samples;
        _gyro_bias_y = sum_gy / samples;
        _gyro_bias_z = sum_gz / samples;

        // For accelerometer, assuming level surface on Earth: Z should measure +1.0g
        _accel_bias_x = sum_ax / samples;
        _accel_bias_y = sum_ay / samples;
        _accel_bias_z = (sum_az / samples) - 1.0f; // Align Z to 1.0g

        printf(">>> BMI160 Calibration Completed!\n");
        printf("    Gyro Bias (deg/s) : X=%+.3f, Y=%+.3f, Z=%+.3f\n",
               _gyro_bias_x, _gyro_bias_y, _gyro_bias_z);
        printf("    Accel Bias (g)    : X=%+.3f, Y=%+.3f, Z=%+.3f\n",
               _accel_bias_x, _accel_bias_y, _accel_bias_z);
        printf("    Sensor Temp       : %.1f °C\n\n", readTemperature());

        if (led_pin != 0xFF) {
            gpio_put(led_pin, 1); // Solid LED indicates calibration done
        }
    }

    void setGyroBias(float bx, float by, float bz) {
        _gyro_bias_x = bx;
        _gyro_bias_y = by;
        _gyro_bias_z = bz;
    }

    void getGyroBias(float& bx, float& by, float& bz) const {
        bx = _gyro_bias_x;
        by = _gyro_bias_y;
        bz = _gyro_bias_z;
    }

    uint getIntPin() const { return _int_pin; }
    uint getCsPin() const { return _cs_pin; }
    uint getSckPin() const { return _sck_pin; }
    uint getMosiPin() const { return _mosi_pin; }
    uint getMisoPin() const { return _miso_pin; }
};
