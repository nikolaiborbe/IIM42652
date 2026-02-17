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

/**
 * GUIDE:
 * The sensor is NOT automatically initilized when you create an object. This means you need to call init() before using
 * the IIM42652 object.
 *
 * It is recommended to use the read(ImuData &data) function. Both for safety, and for speed. If the user
 * want to sample a specific type of data they can use the read_<type>() functions. There is also implementation to
 * sample data from the entire register and only read the previous sampled data. You do this by first calling the
 * sample_data() function and then the get_last_<type>().
 *
 * In the base class "IMU" there are two offset functions that should be used if you want to transform from the sensors
 * axis to the rocket axis.
 */

#include "IMU.h"
#include "logger.h"
#include "main.h"
#include "tx_api.h"


#ifdef __cplusplus
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>
#endif

typedef unsigned long ULONG;

// #include "stm32h7xx_hal_def.h"
// #include "stm32h7xx_hal_gpio.h"
// #include "stm32h7xx_hal_spi.h"
// #include "tx_api.h"

struct SpiConfig {
    // Chip select
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    // Interrupt
    // GPIO_TypeDef *int_port;
    // uint16_t int_pin;

    // SPI handle
    SPI_HandleTypeDef* hspi;
};

struct AccelerometerConfig {
    u8 accel_fs_sel;
    u8 accel_odr;
    u8 accel_ui_filt_bw;
    u8 accel_ui_filt_ord;
    u8 accel_dec2_m2_ord;
    // offset should also probably be here
};

struct GyroConfig {
    u8 gyro_fs_sel;
    u8 gyro_odr;
    u8 gyro_ui_filt_ord;
    u8 gyro_ui_filt_bw;
    u8 gyro_dec2_m2_ord;
};

#ifdef __cplusplus
class IIM42652 : public IMU<ImuData>{
public:
    IIM42652(const SpiConfig& spi)
        : IMU<ImuData>(), m_spi(spi) { };

private:
    ImuData m_sensor_data {};

private:
    /**
     * @brief Relevant data registers.
     */
    static constexpr u8 TEMP_DATA1_UI      = 0x1D;
    static constexpr u8 ACCEL_DATA_X1_UI   = 0x1F;
    static constexpr u8 GYRO_DATA_X1_UI    = 0x25;
    static constexpr u8 ACCEL_CONFIG0      = 0x50;
    static constexpr u8 GYRO_ACCEL_CONFIG0 = 0x52;
    static constexpr u8 ACCEL_CONFIG1      = 0x53;
    static constexpr u8 GYRO_CONFIG0       = 0x4F;
    static constexpr u8 GYRO_CONFIG1       = 0x51;
    static constexpr u8 PWR_MGMT0          = 0x4E;
    static constexpr u8 WHO_AM_I           = 0x75;
    static constexpr u8 DEVICE_CONFIG      = 0x11;
    static constexpr u8 REG_BANK_SEL       = 0x76;

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
    static constexpr u8 ACCEL_UI_FILT_BW_4    = 0x01;
    static constexpr u8 ACCEL_DEC2_M2_ORD_3rd = 0x02;
    static constexpr u8 GYRO_UI_FILT_BW_4     = 0x01;
    static constexpr u8 GYRO_DEC2_M2_ORD_3rd  = 0x02;
    static constexpr u8 WHO_AM_I_RESPONSE_ID  = 0x6F;

public:
    /**
     * @brief Init function to setup configuration for the sensor.
     *
     * @return The status.
     */
    LOGGER::STATUS init();

    /**
     * @brief Reads the value of the IIM42652 sensor.
     *
     * @param data& Location to fill with data.
     *
     * @return The status.
     */
    LOGGER::STATUS read(ImuData& data);

    /**
     * @brief Read gyroscope data directly over SPI.
     *
     * @return Gyroscope data in degrees.
     */
    GyroData read_gyroscope();

    /**
     * @brief Read acceleration data directly over SPI.
     *
     * @return Acceleration data in G's.
     */
    AccelerationData read_acceleration();

    /**
     * @brief Read temperature data directly over SPI.
     *
     * @return Themperature data converted to celcius.
     */
    TemperatureData read_temperature();

    /**
     * @brief Reads entire relevant memory register at once.
     *
     * @note The idea with this function is to read the entire relevant register with one SPI transfer instead of
     * multiple.
     */
    LOGGER::STATUS sample_data();

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
    LOGGER::STATUS set_offset_acc_x(float offset);

    /**
     * @brief Sets offset for the acceleration in the +- y direction.
     *
     * @note Units are in g's.
     *
     * @note Max value is ± 1 g. So, -0.55 is valid, 1.1 is not.
     */
    LOGGER::STATUS set_offset_acc_y(float offset);

    /**
     * @brief Sets offset for the acceleration in the +- z direction.
     *
     * @note Units are in g's.
     *
     * @note Max value is ± 1 g. So, -0.55 is valid, 1.1 is not.
     */
    LOGGER::STATUS set_offset_acc_z(float offset);

    // Note: I could not make sense of gyroscope offset from the datasheet.
    // It looked like the only offset you can set is the sensitivity, not the initial
    // rotation, which is not that interesting.
    // LOGGER::STATUS set_offset_gyro_x(float offset) override;
    // LOGGER::STATUS set_offset_gyro_y(float offset) override;
    // LOGGER::STATUS set_offset_gyro_z(float offset) override;

    // Note: No way to set offset on the device, better to calculate temperature offset by usecase.
    // LOGGER::STATUS set_offset_temp(float offset) override;

private:
    /**
     * @brief Helper function to set the accelerometer configuration.
     *
     * @return The status.
     */
    LOGGER::STATUS _set_config_accelerometer();

    /**
     * @brief Helper function to set the gyro configuration.
     *
     * @return The status.
     */
    LOGGER::STATUS _set_config_gyro();

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
    LOGGER::STATUS _spi_receive_data_from_register();

    /**
     * @brief Get the device id from the WHO_AM_I register.
     */
    LOGGER::STATUS _get_device_id(u8& id);

    /**
     * @brief Turn Accel, Gyro, and Temperature off/on.
     *
     * @note This has to be done to modify the register values. See page 63 in datasheet.
     * https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf
     */
    LOGGER::STATUS _toggle_accel_gyro_temp(DEVICE_STATUS st);

    /**
     * @brief Soft resets the IIM42652.
     */
    LOGGER::STATUS _soft_reset();

    /**
     * @brief Helper to set chip select low.
     */
    void _cs_low();

    /**
     * @brief Helper to set chip select high.
     */
    void _cs_high();

    /**
     * @brief Transmit data over SPI.
     *
     * @param data Pointer to data buffer to transmit.
     * @param len Number of bytes to transmit.
     *
     * @return The status.
     */
    LOGGER::STATUS _spi_transmit(u8* data, u16 len);

    /**
     * @brief Receive data from a register over SPI.
     *
     * @param reg Register address to read from (read bit will be set automatically).
     * @param data Pointer to buffer to store received data.
     * @param len Number of bytes to receive.
     *
     * @return The status.
     */
    LOGGER::STATUS _spi_receive(u8 reg, u8* data, u16 len);

    /**
     * @brief Write a single byte to a register.
     */
    LOGGER::STATUS _write_reg(u8 reg, u8 value);

    /**
     * @brief Helper to get the register value.
     *
     * @param reg Register address.
     */
    u8 _get_register_value(const u8 reg);

    /**
     * @brief Set the register bank.
     *
     * @param bank The register bank you want to select.
     */
    LOGGER::STATUS _bank_select(BANK_SEL bank);

private:
    SpiConfig m_spi;
    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_degree;
    AccelerometerConfig m_accel_cfg;
    GyroConfig m_gyro_cfg;
};

/**
 * @brief Convert two u8 [(1) msb, (2) lsb] -> to one int16_t <msb, lsb>.
 *
 * @note Often in datasheets, the bits of large numbers will be split into two u8's for transfer, this function
 * combines them into a single number.
 *
 * @param msb Most significant byte
 * @param lsb Least significant byte
 *
 * @return The combined value.
 */
constexpr s16 be16_to_i16(u8 msb, u8 lsb);

/**
 * @brief Convert two u8 [(1) msb, (2) lsb] -> to one float.
 *
 * @note Often in datasheets, the bits of large numbers will be split into two u8's for transfer, this function
 * combines them into a single number.
 *
 * @param msb Most significant byte
 * @param lsb Least significant byte
 *
 * @return The combined value.
 */
constexpr float be16_to_float(u8 msb, u8 lsb);

#endif

#endif