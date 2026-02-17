/**
 * @file IIM42652.cpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief This file contains the implementation for the IIM42652 (IMU) driver.
 * @version 0.1
 *
 * @date 2025-12-18
 */

#include "IIM42652.h"

constexpr s16 be16_to_i16(u8 msb, u8 lsb)
{
    return static_cast<s16>((static_cast<u16>(msb) << 8) | static_cast<u16>(lsb));
}

constexpr float be16_to_float(u8 msb, u8 lsb) { return static_cast<float>(be16_to_i16(msb, lsb)); }

void IIM42652::_cs_low() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_RESET); }
void IIM42652::_cs_high() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_SET); }

LOGGER::STATUS IIM42652::_spi_transmit(u8* data, u16 len)
{
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, data, len, SPI_TIMEOUT_MS);
    _cs_high();
    return (status == HAL_OK) ? LOGGER::STATUS::OK : LOGGER::STATUS::LOGGER_ERROR;
}

LOGGER::STATUS IIM42652::_spi_receive(u8 reg, u8* data, u16 len)
{
    u8 tx_reg = (reg & 0x7F) | 0x80;
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, &tx_reg, 1, SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(m_spi.hspi, data, len, SPI_TIMEOUT_MS);
    }
    _cs_high();
    return (status == HAL_OK) ? LOGGER::STATUS::OK : LOGGER::STATUS::LOGGER_ERROR;
}

LOGGER::STATUS IIM42652::_write_reg(u8 reg, u8 value)
{
    u8 tx[2] = { static_cast<u8>(reg & 0x7F), value };
    return _spi_transmit(tx, 2);
}

float IIM42652::_accel_sensitivity_scale_factor_from_full_range_width(ACCEL_FS_SEL full_scale_select)
{
    switch (full_scale_select) {
    case ACCEL_FS_SEL::ACCEL_FS_SEL_16g: return 2048.f;
    case ACCEL_FS_SEL::ACCEL_FS_SEL_8g: return 4096.f;
    case ACCEL_FS_SEL::ACCEL_FS_SEL_4g: return 8192.f;
    case ACCEL_FS_SEL::ACCEL_FS_SEL_2g: return 16384.f;
    default: return 2048.f;
    }
}

float IIM42652::_gyro_sensitivity_scale_factor_from_full_range_width(GYRO_FS_SEL full_scale_select)
{
    switch (full_scale_select) {
    case GYRO_FS_SEL::GYRO_FS_SEL_2000: return 16.4f;
    case GYRO_FS_SEL::GYRO_FS_SEL_1000: return 32.8f;
    case GYRO_FS_SEL::GYRO_FS_SEL_500: return 65.5f;
    case GYRO_FS_SEL::GYRO_FS_SEL_250: return 131.f;
    case GYRO_FS_SEL::GYRO_FS_SEL_125: return 262.f;
    case GYRO_FS_SEL::GYRO_FS_SEL_63: return 524.3f;
    case GYRO_FS_SEL::GYRO_FS_SEL_31: return 1048.6f;
    case GYRO_FS_SEL::GYRO_FS_SEL_16: return 2097.2f;
    default: return 16.4f;
    }
}

LOGGER::STATUS IIM42652::_toggle_accel_gyro_temp(DEVICE_STATUS st)
{
    u8 val {};

    switch (st) {
    // see page 80 in datasheet.
    case DEVICE_STATUS::OFF: val = (1u << 5) | (0u << 2) | 0u; break;
    case DEVICE_STATUS::ON: val = (0u << 5) | (3u << 2) | 3u; break;
    }

    u8 tx[2]              = { (PWR_MGMT0 & 0x7F), val };
    LOGGER::STATUS status = _spi_transmit(tx, 2);
    HAL_Delay(45); // delay for turing on Gyroscope.

    return status;
}

LOGGER::STATUS IIM42652::_soft_reset()
{
    u8 tx[2]              = { (DEVICE_CONFIG & 0x7F), 0x01 };
    LOGGER::STATUS status = _spi_transmit(tx, 2);
    HAL_Delay(1); // required by sensor (see datasheet)
    return status;
}

LOGGER::STATUS IIM42652::init()
{
    HAL_Delay(1);

    // Reset and power down for configuration
    if (_soft_reset() != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    if (_toggle_accel_gyro_temp(DEVICE_STATUS::OFF) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Verify device identity
    u8 id {};
    if (_get_device_id(id) != LOGGER::STATUS::OK || id != WHO_AM_I_RESPONSE_ID)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Configure accelerometer
    m_accel_cfg
        = { ACCEL_FS_SEL_16g, ACCEL_ODR_1kHz, ACCEL_UI_FILT_BW_4, ACCEL_UI_FILT_ORD_2nd, ACCEL_DEC2_M2_ORD_3rd };
    if (_set_config_accelerometer() != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Configure gyroscope
    m_gyro_cfg = { GYRO_FS_SEL_2000, GYRO_ODR_1kHz, GYRO_UI_FILT_ORD_2nd, GYRO_UI_FILT_BW_4, GYRO_DEC2_M2_ORD_3rd };
    if (_set_config_gyro() != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Configure shared filter bandwidth
    u8 filt_bw = (m_accel_cfg.accel_ui_filt_bw << 4) | m_gyro_cfg.gyro_ui_filt_bw;
    if (_write_reg(GYRO_ACCEL_CONFIG0, filt_bw) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Precompute sensitivity scale factors
    m_accel_lsb_per_g
        = _accel_sensitivity_scale_factor_from_full_range_width(static_cast<ACCEL_FS_SEL>(m_accel_cfg.accel_fs_sel));
    m_gyro_lsb_per_degree
        = _gyro_sensitivity_scale_factor_from_full_range_width(static_cast<GYRO_FS_SEL>(m_gyro_cfg.gyro_fs_sel));

    // Power on sensors
    if (_toggle_accel_gyro_temp(DEVICE_STATUS::ON) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS IIM42652::_set_config_accelerometer()
{
    // ACCEL_CONFIG0
    u8 tx0[2] = { (ACCEL_CONFIG0 & 0x7F), 0 };
    tx0[1] |= m_accel_cfg.accel_fs_sel << 5;
    tx0[1] |= m_accel_cfg.accel_odr;
    if (_spi_transmit(tx0, 2) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    HAL_Delay(1);

    // ACCEL_CONFIG1
    u8 tx1[2] = { (ACCEL_CONFIG1 & 0x7F), 0 };
    tx1[1] |= m_accel_cfg.accel_ui_filt_ord << 3;
    tx1[1] |= m_accel_cfg.accel_dec2_m2_ord << 1;
    if (_spi_transmit(tx1, 2) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    HAL_Delay(1);

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS IIM42652::_set_config_gyro()
{
    u8 tx0[2] = { (GYRO_CONFIG0 & 0x7F), 0 };
    tx0[1] |= m_gyro_cfg.gyro_fs_sel << 5;
    tx0[1] |= m_gyro_cfg.gyro_odr;
    if (_spi_transmit(tx0, 2) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    HAL_Delay(1);

    u8 tx1[2] = { (GYRO_CONFIG1 & 0x7F), 0 };
    tx1[1] |= m_gyro_cfg.gyro_ui_filt_ord << 2;
    tx1[1] |= m_gyro_cfg.gyro_dec2_m2_ord;
    if (_spi_transmit(tx1, 2) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    HAL_Delay(1);

    return LOGGER::STATUS::OK;
}

TemperatureData IIM42652::read_temperature()
{
    u8 rx[2] = {};
    if (_spi_receive(TEMP_DATA1_UI, rx, 2) != LOGGER::STATUS::OK) {
        return TemperatureData { false, 0.f };
    }

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    float raw_assembled = be16_to_float(rx[0], rx[1]);
    return TemperatureData { true, (raw_assembled / 132.48f) + 25.f };
}

AccelerationData IIM42652::read_acceleration()
{
    u8 rx[6] = { 0 };
    if (_spi_receive(ACCEL_DATA_X1_UI, rx, 6) != LOGGER::STATUS::OK)
        return AccelerationData { false, 0.f, 0.f, 0.f };

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    float raw_assembled[3] { be16_to_float(rx[0], rx[1]), be16_to_float(rx[2], rx[3]), be16_to_float(rx[4], rx[5]) };

    // convert to ±g
    return AccelerationData { true, raw_assembled[0] / m_accel_lsb_per_g, raw_assembled[1] / m_accel_lsb_per_g,
        raw_assembled[2] / m_accel_lsb_per_g };
}

GyroData IIM42652::read_gyroscope()
{
    u8 rx[6] = {};
    if (_spi_receive(GYRO_DATA_X1_UI, rx, 6) != LOGGER::STATUS::OK)
        return GyroData { false, 0.f, 0.f, 0.f };

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    float raw_assembled[3] = { be16_to_float(rx[0], rx[1]), be16_to_float(rx[2], rx[3]), be16_to_float(rx[4], rx[5]) };

    return GyroData { true, raw_assembled[0] / m_gyro_lsb_per_degree, raw_assembled[1] / m_gyro_lsb_per_degree,
        raw_assembled[2] / m_gyro_lsb_per_degree };
}

GyroData IIM42652::get_last_gyroscope()
{
    if (!m_sensor_data.gyro) {
        return GyroData { false, 0.f, 0.f, 0.f };
    }
    return m_sensor_data.gyro.value();
}

AccelerationData IIM42652::get_last_acceleration()
{
    if (!m_sensor_data.acc) {
        return AccelerationData { false, 0.f, 0.f, 0.f };
    }
    return m_sensor_data.acc.value();
}

TemperatureData IIM42652::get_last_temperature()
{
    if (!m_sensor_data.temperature) {
        return TemperatureData { false, 0.f };
    }
    return m_sensor_data.temperature.value();
}

LOGGER::STATUS IIM42652::read(ImuData& data)
{
    LOGGER::STATUS status = _spi_receive_data_from_register();
    data                  = m_sensor_data;
    return status;
}

LOGGER::STATUS IIM42652::sample_data()
{
    LOGGER::STATUS status = _spi_receive_data_from_register();
    return status;
}

LOGGER::STATUS IIM42652::_get_device_id(u8& id)
{
    u8 rx                 = 0;
    LOGGER::STATUS status = _spi_receive(WHO_AM_I, &rx, 1);
    id                    = rx;
    return status;
}

LOGGER::STATUS IIM42652::_spi_receive_data_from_register()
{
    u8 rx[14] = { 0 };
    if (_spi_receive(TEMP_DATA1_UI, rx, 14) != LOGGER::STATUS::OK) {
        m_sensor_data.valid = false;
        return LOGGER::STATUS::LOGGER_ERROR;
    }

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    float raw_assembled[7] = { be16_to_float(rx[0], rx[1]), be16_to_float(rx[2], rx[3]), be16_to_float(rx[4], rx[5]),
        be16_to_float(rx[6], rx[7]), be16_to_float(rx[8], rx[9]), be16_to_float(rx[10], rx[11]),
        be16_to_float(rx[12], rx[13]) };

    TemperatureData temperature_in_celsius { true, (raw_assembled[0] / 132.48f) + 25.f };

    AccelerationData acceleration_data { true, raw_assembled[1] / m_accel_lsb_per_g,
        raw_assembled[2] / m_accel_lsb_per_g, raw_assembled[3] / m_accel_lsb_per_g };

    GyroData gyro_data { true, raw_assembled[4] / m_gyro_lsb_per_degree, raw_assembled[5] / m_gyro_lsb_per_degree,
        raw_assembled[6] / m_gyro_lsb_per_degree };

    m_sensor_data = { true, temperature_in_celsius, gyro_data, acceleration_data, tx_time_get() };

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS IIM42652::_bank_select(BANK_SEL bank)
{
    u8 tx[2] = { (REG_BANK_SEL & 0x7F), static_cast<u8>(bank) };
    return _spi_transmit(tx, 2);
}

u8 IIM42652::_get_register_value(const u8 reg)
{
    u8 rx = 0;
    _spi_receive(reg, &rx, 1);
    return rx;
}

LOGGER::STATUS IIM42652::set_offset_acc_x(float offset)
{
    constexpr float res = 0.0005f;
    constexpr s16 min   = -2000;
    constexpr s16 max   = 2000;

    _bank_select(BANK_SEL_4);

    u8 upper_gyro_z = _get_register_value(OFFSET_USER4);

    s16 steps        = std::clamp(static_cast<s16>(std::lround(offset / res)), min, max);
    u16 raw12        = static_cast<u16>(steps) & 0x0FFF;
    u8 lower         = static_cast<u8>(raw12 & 0x00FF);
    u8 upper         = static_cast<u8>((raw12 >> 8) & 0x0F);
    u8 propper_upper = (upper << 4) | (upper_gyro_z & 0x0F); // combine previous gyro offset with new accel offset.

    u8 tx[3]              = { (OFFSET_USER4 & 0x7F), propper_upper, lower };
    LOGGER::STATUS status = _spi_transmit(tx, 3);

    _bank_select(BANK_SEL_0);

    return status;
}

LOGGER::STATUS IIM42652::set_offset_acc_y(float offset)
{
    constexpr float res = 0.0005f;
    constexpr s16 min   = -2000;
    constexpr s16 max   = 2000;

    _bank_select(BANK_SEL_4);

    u8 upper_accel_z = _get_register_value(OFFSET_USER7);

    s16 steps        = std::clamp(static_cast<s16>(std::lround(offset / res)), min, max);
    u16 raw12        = static_cast<u16>(steps) & 0x0FFF;
    u8 lower         = static_cast<u8>(raw12 & 0x00FF);
    u8 upper         = static_cast<u8>((raw12 >> 8) & 0x0F);
    u8 propper_upper = (upper_accel_z & 0xF0) | upper; // combine previous accel offset with new accel offset.

    u8 tx[3]              = { (OFFSET_USER6 & 0x7F), lower, propper_upper };
    LOGGER::STATUS status = _spi_transmit(tx, 3);

    _bank_select(BANK_SEL_0);

    return status;
}

LOGGER::STATUS IIM42652::set_offset_acc_z(float offset)
{
    constexpr float res = 0.0005f;
    constexpr s16 min   = -2000;
    constexpr s16 max   = 2000;

    _bank_select(BANK_SEL_4);

    u8 upper_accel_y = _get_register_value(OFFSET_USER7);

    s16 steps        = std::clamp(static_cast<s16>(std::lround(offset / res)), min, max);
    u16 raw12        = static_cast<u16>(steps) & 0x0FFF;
    u8 lower         = static_cast<u8>(raw12 & 0x00FF);
    u8 upper         = static_cast<u8>((raw12 >> 8) & 0x0F);
    u8 propper_upper = (upper << 4) | (upper_accel_y & 0x0F); // combine previous accel offset with new accel offset.

    u8 tx[3]              = { (OFFSET_USER7 & 0x7F), propper_upper, lower };
    LOGGER::STATUS status = _spi_transmit(tx, 3);

    _bank_select(BANK_SEL_0);

    return status;
}