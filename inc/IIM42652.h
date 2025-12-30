/**
 * @file IIM42652.hpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief The header file for the IIM42652 (IMU) driver.
 * @version 1.0
 *
 * @date 2025-12-18
 */

#ifndef IIM42652_H
#define IIM42652_H

#include "IMU.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>

struct SpiConfig {
    // Chip select
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    // Interrupt
    // GPIO_TypeDef *int_port;
    uint16_t int_pin;

    // SPI handle
    SPI_HandleTypeDef* hspi;
};

struct AccelerometerConfig {
    uint8_t accel_fs_sel;
    uint8_t accel_odr;
    uint8_t accel_ui_filt_bw;
    uint8_t accel_ui_filt_ord;
    uint8_t accel_dec2_m2_ord;
    // offset should also probably be here
};

struct GyroConfig {
    uint8_t gyro_fs_sel;
    uint8_t gyro_odr;
    uint8_t gyro_ui_filt_ord;
    uint8_t gyro_ui_filt_bw;
    uint8_t gyro_dec2_m2_ord;
};

class IIM42652 : public IMU<ImuData> {
private:
    /**
     * @brief Relevant data registers.
     */
    static constexpr uint8_t TEMP_DATA1_UI      = 0x1D;
    static constexpr uint8_t ACCEL_DATA_X1_UI   = 0x1F;
    static constexpr uint8_t GYRO_DATA_X1_UI    = 0x25;
    static constexpr uint8_t ACCEL_CONFIG0      = 0x50;
    static constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
    static constexpr uint8_t ACCEL_CONFIG1      = 0x53;
    static constexpr uint8_t GYRO_CONFIG0       = 0x4F;
    static constexpr uint8_t GYRO_CONFIG1       = 0x51;
    static constexpr uint8_t PWR_MGMT0          = 0x4E;
    static constexpr uint8_t WHO_AM_I           = 0x75;
    static constexpr uint8_t DEVICE_CONFIG      = 0x11;
    static constexpr uint8_t REG_BANK_SEL       = 0x76;

    /**
     * @brief Values for acceleration config.
     */
    enum ACCEL_FS_SEL {
        ACCEL_FS_SEL_16g = 0x00,
        ACCEL_FS_SEL_8g  = 0x01,
        ACCEL_FS_SEL_4g  = 0x02,
        ACCEL_FS_SEL_2g  = 0x03
    };

    /**
     * @brief Acceleration filter order options.
     */
    enum ACCEL_UI_FILT_ORD { ACCEL_UI_FILT_ORD_1st = 0x00, ACCEL_UI_FILT_ORD_2nd = 0x01, ACCEL_UI_FILT_ORD_3rd = 0x02 };

    /**
     * @brief Acceleration output data rate options.
     * @note There are more config settings, but these are probably the most relevant.
     */
    enum ACCEL_ODR {
        ACCEL_ODR_4kHz  = 0x04,
        ACCEL_ODR_2kHz  = 0x05,
        ACCEL_ODR_1kHz  = 0x06,
        ACCEL_ODR_200Hz = 0x07,
        ACCEL_ODR_100Hz = 0x08
    };

    /**
     * @brief Gyroscope full-scale range options.
     * @note The range the sampled data can have.
     * @note Some of the options are rounded (e.g. not. 31 but 31.25)
     */
    enum GYRO_FS_SEL {
        GYRO_FS_SEL_2000 = 0x00,
        GYRO_FS_SEL_1000 = 0x01,
        GYRO_FS_SEL_500  = 0x02,
        GYRO_FS_SEL_250  = 0x03,
        GYRO_FS_SEL_125  = 0x04,
        GYRO_FS_SEL_63   = 0x05,
        GYRO_FS_SEL_31   = 0x06,
        GYRO_FS_SEL_16   = 0x07,
    };

    /**
     * @brief Gyroscope output data rate options.
     */
    enum GYRO_ODR {
        GYRO_ODR_32kHz = 0x01,
        GYRO_ODR_16kHz = 0x02,
        GYRO_ODR_8kHz  = 0x03,
        GYRO_ODR_4kHz  = 0x04,
        GYRO_ODR_2kHz  = 0x05,
        GYRO_ODR_1kHz  = 0x06,
        GYRO_ODR_500Hz = 0x0F,
        GYRO_ODR_200Hz = 0x07,
        GYRO_ODR_100Hz = 0x08,
        GYRO_ODR_50Hz  = 0x09,
        GYRO_ODR_25Hz  = 0x0A,
        GYRO_ODR_12Hz  = 0x0B
    };

    /**
     * Gyroscope filter order options.
     */
    enum GYRO_UI_FILT_ORD {
        GYRO_UI_FILT_ORD_1st = 0x00,
        GYRO_UI_FILT_ORD_2nd = 0x01,
        GYRO_UI_FILT_ORD_3rd = 0x02,
    };

    /**
     * Register banks.
     */
    enum BANK_SEL {
        BANK_SEL_0 = 0x00,
        BANK_SEL_1 = 0x01,
        BANK_SEL_2 = 0x02,
        BANK_SEL_3 = 0x03,
        BANK_SEL_4 = 0x04,
    };

    /**
     * User programmable offset values.
     */
    enum OFFSET_USER {
        OFFSET_USER0 = 0x77,
        OFFSET_USER1 = 0x78,
        OFFSET_USER2 = 0x79,
        OFFSET_USER3 = 0x7A,
        OFFSET_USER4 = 0x7B,
        OFFSET_USER5 = 0x7C,
        OFFSET_USER6 = 0x7D,
        OFFSET_USER7 = 0x7E,
        OFFSET_USER8 = 0x7F,

    };

    // Others
    static constexpr uint8_t ACCEL_UI_FILT_BW_4    = 0x01;
    static constexpr uint8_t ACCEL_DEC2_M2_ORD_3rd = 0x02;
    static constexpr uint8_t GYRO_UI_FILT_BW_4     = 0x01;
    static constexpr uint8_t GYRO_DEC2_M2_ORD_3rd  = 0x02;
    static constexpr uint8_t WHO_AM_I_RESPONSE_ID  = 0x6F;

public:
    IIM42652(const SpiConfig spi)
        : m_spi(spi) { };

    /**
     * @brief Init function to setup configuration for the sensor.
     *
     * @return The status.
     */
    uint8_t init();

    /**
     * @brief Reads the value of the IIM42652 sensor.
     *
     * @param data& Location to fill with data.
     *
     * @return The status.
     */
    uint8_t read(ImuData& data);

    /**
     * @brief Read gyroscope data directly over SPI.
     *
     * @return Gyroscope data in degrees.
     */
    GyroData read_gyroscope() override;

    /**
     * @brief Read acceleration data directly over SPI.
     *
     * @return Acceleration data in G's.
     */
    AccelerationData read_acceleration() override;

    /**
     * @brief Read temperature data directly over SPI.
     *
     * @return Themperature data converted to celcius.
     */
    TemperatureData read_temperature() override;

    /**
     * @brief Reads entire relevant memory register at once.
     *
     * @note The idea with this function is to read the entire relevant register with one SPI transfer instead of
     * multiple.
     */
    uint8_t sample_data();

    /**
     * @brief Get the last gyroscope value sampled by calling sample_values().
     *
     * @note Reads the gyro data last sampled from the sample_data function. The idea with the use of this function is
     * to be able to transmit all relevant data over SPI once to save time.
     *
     * @return Gyro data.
     */
    GyroData get_last_gyroscope();

    /**
     * @brief Get the last acceleration value sampled by calling sample_values().
     *
     * @note Reads the acceleration data last sampled from the sample_data function. The idea with the use of this
     * function is to be able to transmit all relevant data over SPI once to save time.
     *
     * @return Acceleration data.
     */
    AccelerationData get_last_acceleration();

    /**
     * @brief Get the last temperature value sampled by calling sample_values().
     *
     * @note Reads the temperature data last sampled from the sample_data function. The idea with the use of this
     * function is to be able to transmit all relevant data over SPI once to save time.
     *
     * @return Temperature data in celcius.
     */
    TemperatureData get_last_temperature();

    /**
     * @brief Sets offset for the acceleration in the +- x direction.
     *
     * @note Units are in g's.
     *
     * @note Max value is ± 1 g. So, -0.55 is valid, 1.1 is not.
     */
    uint8_t set_offset_acc_x(float offset) override;

    /**
     * @brief Sets offset for the acceleration in the +- y direction.
     *
     * @note Units are in g's.
     *
     * @note Max value is ± 1 g. So, -0.55 is valid, 1.1 is not.
     */
    uint8_t set_offset_acc_y(float offset) override;

    /**
     * @brief Sets offset for the acceleration in the +- z direction.
     *
     * @note Units are in g's.
     *
     * @note Max value is ± 1 g. So, -0.55 is valid, 1.1 is not.
     */
    uint8_t set_offset_acc_z(float offset) override;
    
    // Note: I could not make sense of gyroscope offset from the datasheet.
    // It looked like the only offset you can set is the sensitivity, not the initial
    // rotation, which is not that interesting.
    // uint8_t set_offset_gyro_x(float offset) override;
    // uint8_t set_offset_gyro_y(float offset) override;
    // uint8_t set_offset_gyro_z(float offset) override;

    // Note: No way to set offset on the device, better to calculate temperature offset by usecase.
    // uint8_t set_offset_temp(float offset) override;

private:
    /**
     * @brief Helper function to set the accelerometer configuration.
     *
     * @return The status.
     */
    uint8_t _set_config_accelerometer();

    /**
     * @brief Helper function to set the gyro configuration.
     *
     * @return The status.
     */
    uint8_t _set_config_gyro();

    /**
     * @brief Helper function to get the byte scale factor for the acceleration data.
     *
     * @param full_scale_select The selected FS_SEL.
     *
     * @note If you want a wider range of data (e.g. +- 16g), you get less precision (e.g. 2,048 LSB/g). This is why you
     * need to convert back to real values depending on the selected precision.
     */
    float _accel_sensitivity_scale_factor_from_full_range_width(ACCEL_FS_SEL full_scale_select);

    /**
     * @brief Helper function to get the byte scale factor for the gyro data.
     *
     * @param full_scale_select The selected FS_SEL.
     *
     * @note If you want a wider range of data (e.g. +- 16g), you get less precision (e.g. 2,048 LSB/g). This is why you
     * need to convert back to real values depending on the selected precision.
     */
    float _gyro_sensitivity_scale_factor_from_full_range_width(GYRO_FS_SEL full_scale_select);

    /**
     * @brief Helper function to read data over SPI.
     */
    uint8_t _spi_receive_data_from_register();

    /**
     * @brief Get the device id from the WHO_AM_I register.
     */
    uint8_t _get_device_id(uint8_t& id);

    /**
     * @brief Turn Accel, Gyro, and Temperature off/on.
     *
     * @note This has to be done to modify the register values. See page 63 in datasheet.
     * https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf
     */
    uint8_t _toggle_accel_gyro_temp(DEVICE_STATUS st);

    /**
     * @brief Soft resets the IIM42652.
     */
    uint8_t _soft_reset();

    /**
     * @brief Helper to set chip select low.
     */
    void _CS_LOW();

    /**
     * @brief Helper to set chip select high.
     */
    void _CS_HIGH();

    /**
     * @brief Helper to get the register value.
     *
     * @param reg Register address.
     */
    uint8_t _get_register_value(const uint8_t reg);
    
    /**
     * @brief Set the register bank.
     *
     * @param bank The register bank you want to select.
     */
    uint8_t _bank_select(BANK_SEL bank);

private:
    SpiConfig m_spi;
    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_degree;
    AccelerometerConfig m_accel_cfg;
    GyroConfig m_gyro_cfg;
};

/**
 * @brief Convert two uint8_t [(1) msb, (2) lsb] -> to one int16_t <msb, lsb>.
 *
 * @note Often in datasheets, the bits of large numbers will be split into two uint8_t's for transfer, this function
 * combines them into a single number.
 *
 * @param msb Most significant byte
 * @param lsb Least significant byte
 *
 * @return The combined value.
 */
constexpr int16_t be16_to_i16(uint8_t msb, uint8_t lsb);

/**
 * @brief Convert two uint8_t [(1) msb, (2) lsb] -> to one float.
 *
 * @note Often in datasheets, the bits of large numbers will be split into two uint8_t's for transfer, this function
 * combines them into a single number.
 *
 * @param msb Most significant byte
 * @param lsb Least significant byte
 *
 * @return The combined value.
 */
constexpr float be16_to_float(uint8_t msb, uint8_t lsb);

#endif