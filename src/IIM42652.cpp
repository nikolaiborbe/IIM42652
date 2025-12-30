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

constexpr int16_t be16_to_i16(uint8_t msb, uint8_t lsb)
{
    return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8) | static_cast<uint16_t>(lsb));
}

constexpr float be16_to_float(uint8_t msb, uint8_t lsb) { return static_cast<float>(be16_to_i16(msb, lsb)); }

void IIM42652::_CS_LOW() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_RESET); }
void IIM42652::_CS_HIGH() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_SET); }

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

uint8_t IIM42652::_toggle_accel_gyro_temp(DEVICE_STATUS st)
{
    uint8_t val {};

    switch (st) {
    // see page 80 in datasheet.
    case DEVICE_STATUS::OFF: val = (1u << 5) | (0u << 2) | 0u; break;
    case DEVICE_STATUS::ON: val = (0u << 5) | (3u << 2) | 3u; break;
    }

    std::array<uint8_t, 2> tx = { PWR_MGMT0, val };
    _CS_LOW();
    HAL_StatusTypeDef tx_status = HAL_SPI_Transmit(m_spi.hspi, tx.data(), 2, SPI_TIMEOUT_MS);
    _CS_HIGH();
    HAL_Delay(45); // delay for turing on Gyroscope.

    if (tx_status != HAL_OK)
        return 1;

    return 0;
}

uint8_t IIM42652::_soft_reset()
{
    uint8_t reg               = DEVICE_CONFIG;
    uint8_t enable_reset      = 0x01;
    std::array<uint8_t, 2> tx = { reg, enable_reset };

    _CS_LOW();
    HAL_StatusTypeDef tx_status = HAL_SPI_Transmit(m_spi.hspi, tx.data(), 2, SPI_TIMEOUT_MS);
    _CS_HIGH();

    HAL_Delay(1); // required by sensor (see datasheet)

    if (tx_status != HAL_OK)
        return 1;

    return 0;
}

uint8_t IIM42652::init()
{
    HAL_Delay(1);

    if (_soft_reset() != 0)
        return 1;

    if (_toggle_accel_gyro_temp(DEVICE_STATUS::OFF) != 0)
        return 1;

    // make sure it's the correct device
    uint8_t id {};
    if (_get_device_id(id) != 0)
        return 1;
    if (id != WHO_AM_I_RESPONSE_ID)
        return 1;

    // set accel configs
    m_accel_cfg.accel_fs_sel      = ACCEL_FS_SEL_16g;
    m_accel_cfg.accel_odr         = ACCEL_ODR_1kHz;
    m_accel_cfg.accel_ui_filt_bw  = ACCEL_UI_FILT_BW_4;
    m_accel_cfg.accel_ui_filt_ord = ACCEL_UI_FILT_ORD_2nd;
    m_accel_cfg.accel_dec2_m2_ord = ACCEL_DEC2_M2_ORD_3rd;

    // set gyro configs
    m_gyro_cfg.gyro_fs_sel      = GYRO_FS_SEL_2000;
    m_gyro_cfg.gyro_odr         = GYRO_ODR_1kHz;
    m_gyro_cfg.gyro_ui_filt_ord = GYRO_UI_FILT_ORD_2nd;
    m_gyro_cfg.gyro_ui_filt_bw  = GYRO_UI_FILT_BW_4;
    m_gyro_cfg.gyro_dec2_m2_ord = GYRO_DEC2_M2_ORD_3rd;

    uint8_t status_gyro_config = _set_config_gyro();
    if (status_gyro_config != 0)
        return status_gyro_config;

    uint8_t status_acc_config = _set_config_accelerometer();
    if (status_acc_config != 0)
        return status_acc_config;

    // calculate LSB scale factors
    m_accel_lsb_per_g
        = _accel_sensitivity_scale_factor_from_full_range_width(static_cast<ACCEL_FS_SEL>(m_accel_cfg.accel_fs_sel));
    m_gyro_lsb_per_degree
        = _gyro_sensitivity_scale_factor_from_full_range_width(static_cast<GYRO_FS_SEL>(m_gyro_cfg.gyro_fs_sel));

    // GYRO_ACCEL_CONFIG0
    std::array<uint8_t, 2> tx = { GYRO_ACCEL_CONFIG0, 0 };
    tx[1] |= m_accel_cfg.accel_ui_filt_bw << 4;
    tx[1] |= m_gyro_cfg.gyro_ui_filt_bw;
    _CS_LOW();
    HAL_StatusTypeDef tx_status = HAL_SPI_Transmit(m_spi.hspi, tx.data(), 2, SPI_TIMEOUT_MS);
    _CS_HIGH();
    HAL_Delay(1);

    if (tx_status != HAL_OK)
        return 1;

    if (_toggle_accel_gyro_temp(DEVICE_STATUS::ON) != 0)
        return 1;

    return 0;
}

uint8_t IIM42652::_set_config_accelerometer()
{
    HAL_StatusTypeDef status_config0 = HAL_ERROR, status_config1 = HAL_ERROR;

    {
        // ACCEL_CONFIG0
        std::array<uint8_t, 2> tx = { ACCEL_CONFIG0, 0 };
        tx[1] |= m_accel_cfg.accel_fs_sel << 5;
        tx[1] |= m_accel_cfg.accel_odr;

        _CS_LOW();
        status_config0 = HAL_SPI_Transmit(m_spi.hspi, tx.data(), 2, SPI_TIMEOUT_MS);
        _CS_HIGH();
        HAL_Delay(1);
    }

    {
        // ACCEL_CONFIG1
        std::array<uint8_t, 2> tx = { ACCEL_CONFIG1, 0 };
        tx[1] |= m_accel_cfg.accel_ui_filt_ord << 3;
        tx[1] |= m_accel_cfg.accel_dec2_m2_ord << 1;

        _CS_LOW();
        status_config1 = HAL_SPI_Transmit(m_spi.hspi, tx.data(), 2, SPI_TIMEOUT_MS);
        _CS_HIGH();
        HAL_Delay(1);
    }

    if ((status_config0 != HAL_OK) || (status_config1 != HAL_OK))
        return 1;

    return 0;
}

uint8_t IIM42652::_set_config_gyro()
{
    HAL_StatusTypeDef status_config0 = HAL_ERROR, status_config1 = HAL_ERROR;

    std::array<uint8_t, 2> tx0 = { GYRO_CONFIG0, 0 };
    tx0[1] |= m_gyro_cfg.gyro_fs_sel << 5;
    tx0[1] |= m_gyro_cfg.gyro_odr;

    _CS_LOW();
    status_config0 = HAL_SPI_Transmit(m_spi.hspi, tx0.data(), 2, SPI_TIMEOUT_MS);
    _CS_HIGH();
    HAL_Delay(1);

    std::array<uint8_t, 2> tx1 = { GYRO_CONFIG1, 0 };
    tx1[1] |= m_gyro_cfg.gyro_ui_filt_ord << 2;
    tx1[1] |= m_gyro_cfg.gyro_dec2_m2_ord;

    _CS_LOW();
    status_config1 = HAL_SPI_Transmit(m_spi.hspi, tx1.data(), 2, SPI_TIMEOUT_MS);
    _CS_HIGH();
    HAL_Delay(1);

    if ((status_config0 != HAL_OK) || (status_config1 != HAL_OK))
        return 1;

    return 0;
}

TemperatureData IIM42652::read_temperature()
{
    TemperatureData temperature_in_celsius {};
    HAL_StatusTypeDef status = HAL_ERROR;

    std::array<uint8_t, 2> rx_buffer {};
    uint8_t reg = TEMP_DATA1_UI | 0x80;

    _CS_LOW();
    status = HAL_SPI_TransmitReceive(
        m_spi.hspi, &reg, rx_buffer.data(), static_cast<uint16_t>(rx_buffer.size()), SPI_TIMEOUT_MS);
    _CS_HIGH();

    if (status != HAL_OK) {
        return TemperatureData { false, 0.f };
    }

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    float raw_assembled                = be16_to_float(rx_buffer[0], rx_buffer[1]);
    temperature_in_celsius.valid       = true;
    temperature_in_celsius.temperature = (raw_assembled / 132.48f) + 25.f; // convertion from datasheet

    return temperature_in_celsius;
}

AccelerationData IIM42652::read_acceleration()
{
    AccelerationData acceleration_data {};
    HAL_StatusTypeDef status = HAL_ERROR;

    std::array<uint8_t, 6> rx_buffer {};
    uint8_t reg = ACCEL_DATA_X1_UI | 0x80;

    _CS_LOW();
    status = HAL_SPI_TransmitReceive(
        m_spi.hspi, &reg, rx_buffer.data(), static_cast<uint16_t>(rx_buffer.size()), SPI_TIMEOUT_MS);
    _CS_HIGH();

    if (status != HAL_OK)
        return AccelerationData { false, 0.f, 0.f, 0.f };

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    std::array<float, 3> raw_assembled { be16_to_float(rx_buffer[0], rx_buffer[1]),
        be16_to_float(rx_buffer[2], rx_buffer[3]), be16_to_float(rx_buffer[4], rx_buffer[5]) };

    // convert to ±g
    acceleration_data.valid = true;
    acceleration_data.acc_x = raw_assembled[0] / m_accel_lsb_per_g;
    acceleration_data.acc_y = raw_assembled[1] / m_accel_lsb_per_g;
    acceleration_data.acc_z = raw_assembled[2] / m_accel_lsb_per_g;

    return acceleration_data;
}

GyroData IIM42652::read_gyroscope()
{
    GyroData gyro_data {};
    HAL_StatusTypeDef status = HAL_ERROR;

    std::array<uint8_t, 6> rx_buffer {};
    uint8_t reg = GYRO_DATA_X1_UI | 0x80;

    _CS_LOW();
    status = HAL_SPI_TransmitReceive(
        m_spi.hspi, &reg, rx_buffer.data(), static_cast<uint16_t>(rx_buffer.size()), SPI_TIMEOUT_MS);
    _CS_HIGH();

    if (status != HAL_OK)
        return GyroData { false, 0.f, 0.f, 0.f };

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    std::array<float, 3> raw_assembled { be16_to_float(rx_buffer[0], rx_buffer[1]),
        be16_to_float(rx_buffer[2], rx_buffer[3]), be16_to_float(rx_buffer[4], rx_buffer[5]) };

    gyro_data.valid  = true;
    gyro_data.gyro_x = raw_assembled[0] / m_gyro_lsb_per_degree;
    gyro_data.gyro_y = raw_assembled[1] / m_gyro_lsb_per_degree;
    gyro_data.gyro_z = raw_assembled[2] / m_gyro_lsb_per_degree;

    return gyro_data;
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

uint8_t IIM42652::read(ImuData& data)
{
    uint8_t status = _spi_receive_data_from_register();
    data                  = m_sensor_data;
    return status;
}

uint8_t IIM42652::sample_data()
{
    uint8_t status = _spi_receive_data_from_register();
    return status;
}

uint8_t IIM42652::_get_device_id(uint8_t& id)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    uint8_t reg = WHO_AM_I | 0x80;
    uint8_t rx_buffer {};

    _CS_LOW();
    status = HAL_SPI_TransmitReceive(m_spi.hspi, &reg, &rx_buffer, 1, SPI_TIMEOUT_MS);
    _CS_HIGH();

    if (status != HAL_OK)
        return 1;

    id = rx_buffer;

    return 0;
}

uint8_t IIM42652::_spi_receive_data_from_register()
{
    AccelerationData acceleration_data {};
    GyroData gyro_data {};
    TemperatureData temperature_in_celsius {};
    HAL_StatusTypeDef status = HAL_ERROR;

    constexpr std::size_t buffer_size = 14;
    std::array<uint8_t, buffer_size> rx_buffer {};

    uint8_t reg = TEMP_DATA1_UI | 0x80;

    _CS_LOW();
    status = HAL_SPI_TransmitReceive(
        m_spi.hspi, &reg, rx_buffer.data(), static_cast<uint16_t>(rx_buffer.size()), SPI_TIMEOUT_MS);
    _CS_HIGH();

    if (status != HAL_OK) {
        m_sensor_data.valid = false;
        return 1;
    }

    // convert data from: DATA[15:8], DATA[7:0] -> DATA[15:0]
    std::array<float, 7> raw_assembled { be16_to_float(rx_buffer[0], rx_buffer[1]),
        be16_to_float(rx_buffer[2], rx_buffer[3]), be16_to_float(rx_buffer[4], rx_buffer[5]),
        be16_to_float(rx_buffer[6], rx_buffer[7]), be16_to_float(rx_buffer[8], rx_buffer[9]),
        be16_to_float(rx_buffer[10], rx_buffer[11]), be16_to_float(rx_buffer[12], rx_buffer[13]) };

    temperature_in_celsius.valid       = true;
    temperature_in_celsius.temperature = (raw_assembled[0] / 132.48f) + 25.f; // convertion from datasheet

    // convert to ±g
    acceleration_data.valid = true;
    acceleration_data.acc_x = raw_assembled[1] / m_accel_lsb_per_g;
    acceleration_data.acc_y = raw_assembled[2] / m_accel_lsb_per_g;
    acceleration_data.acc_z = raw_assembled[3] / m_accel_lsb_per_g;

    gyro_data.valid  = true;
    gyro_data.gyro_x = raw_assembled[4] / m_gyro_lsb_per_degree;
    gyro_data.gyro_y = raw_assembled[5] / m_gyro_lsb_per_degree;
    gyro_data.gyro_z = raw_assembled[6] / m_gyro_lsb_per_degree;

    // TODO: BIG TIME WARNING, tx_time_get() should be used here
    m_sensor_data = { true, temperature_in_celsius, gyro_data, acceleration_data, static_cast<ULONG>(1) };

    return 0;
}

uint8_t IIM42652::_bank_select(BANK_SEL bank)
{
    HAL_StatusTypeDef status  = HAL_ERROR;
    std::array<uint8_t, 2> tx = { REG_BANK_SEL, static_cast<uint8_t>(bank) };

    _CS_LOW();
    status = HAL_SPI_Transmit(m_spi.hspi, tx.data(), 2, SPI_TIMEOUT_MS);
    _CS_HIGH();

    return (status == HAL_OK) ? 0 : 1;
}

uint8_t IIM42652::_get_register_value(const uint8_t reg)
{
    uint8_t tx = reg | 0x80;
    uint8_t rx {};

    _CS_LOW();
    HAL_SPI_TransmitReceive(m_spi.hspi, &tx, &rx, 1, SPI_TIMEOUT_MS);
    _CS_HIGH();
    return rx;
}

uint8_t IIM42652::set_offset_acc_x(float offset)
{
    constexpr float res      = 0.0005f;
    constexpr int16_t min    = -2000;
    constexpr int16_t max    = 2000;
    HAL_StatusTypeDef status = HAL_ERROR;

    _bank_select(BANK_SEL_4);

    uint8_t upper_gyro_z = _get_register_value(OFFSET_USER4);

    int16_t steps         = std::clamp(static_cast<int16_t>(std::lround(offset / res)), min, max);
    uint16_t raw12        = static_cast<uint16_t>(steps) & 0x0FFF;
    uint8_t lower         = static_cast<uint8_t>(raw12 & 0x00FF);
    uint8_t upper         = static_cast<uint8_t>((raw12 >> 8) & 0x0F);
    uint8_t propper_upper = (upper << 4) | upper_gyro_z; // combine previous gyro offset with new accel offset.

    uint8_t reg                     = OFFSET_USER4;
    std::array<uint8_t, 3> tx_upper = { reg, propper_upper, lower };
    _CS_LOW();
    status = HAL_SPI_Transmit(m_spi.hspi, tx_upper.data(), tx_upper.size(), SPI_TIMEOUT_MS);
    _CS_HIGH();

    _bank_select(BANK_SEL_0);

    return (status == HAL_OK) ? 0 : 1;
}

uint8_t IIM42652::set_offset_acc_y(float offset)
{
    constexpr float res      = 0.0005f;
    constexpr int16_t min    = -2000;
    constexpr int16_t max    = 2000;
    HAL_StatusTypeDef status = HAL_ERROR;

    _bank_select(BANK_SEL_4);

    uint8_t upper_accel_z = _get_register_value(OFFSET_USER7);

    int16_t steps         = std::clamp(static_cast<int16_t>(std::lround(offset / res)), min, max);
    uint16_t raw12        = static_cast<uint16_t>(steps) & 0x0FFF;
    uint8_t lower         = static_cast<uint8_t>(raw12 & 0x00FF);
    uint8_t upper         = static_cast<uint8_t>((raw12 >> 8) & 0x0F);
    uint8_t propper_upper = (upper_accel_z & 0xF0) | upper; // combine previous accel offset with new accel offset.

    uint8_t reg                     = OFFSET_USER6;
    std::array<uint8_t, 3> tx_upper = { reg, lower, propper_upper };
    _CS_LOW();
    status = HAL_SPI_Transmit(m_spi.hspi, tx_upper.data(), tx_upper.size(), SPI_TIMEOUT_MS);
    _CS_HIGH();

    _bank_select(BANK_SEL_0);

    return (status == HAL_OK) ? 0 : 1;
}

uint8_t IIM42652::set_offset_acc_z(float offset)
{
    constexpr float res      = 0.0005f;
    constexpr int16_t min    = -2000;
    constexpr int16_t max    = 2000;
    HAL_StatusTypeDef status = HAL_ERROR;

    _bank_select(BANK_SEL_4);

    uint8_t upper_accel_y = _get_register_value(OFFSET_USER7);

    int16_t steps         = std::clamp(static_cast<int16_t>(std::lround(offset / res)), min, max);
    uint16_t raw12        = static_cast<uint16_t>(steps) & 0x0FFF;
    uint8_t lower         = static_cast<uint8_t>(raw12 & 0x00FF);
    uint8_t upper         = static_cast<uint8_t>((raw12 >> 8) & 0x0F);
    uint8_t propper_upper = (upper << 4) | upper_accel_y; // combine previous accel offset with new accel offset.

    uint8_t reg                     = OFFSET_USER7;
    std::array<uint8_t, 3> tx_upper = { reg, propper_upper, lower };
    _CS_LOW();
    status = HAL_SPI_Transmit(m_spi.hspi, tx_upper.data(), tx_upper.size(), SPI_TIMEOUT_MS);
    _CS_HIGH();

    _bank_select(BANK_SEL_0);

    return (status == HAL_OK) ? 0 : 1;
}