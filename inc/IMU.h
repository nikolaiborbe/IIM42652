/**
 * @file IMU.hpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief This file contains the base class for the innertial measurement drivers.
 * @version 1.0
 *
 * @date 2025-10-29
 */

#ifndef IMU_H
#define IMU_H

#include "sensors.h"
#include "quaternion.h"

struct GyroData {
    bool valid;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

// in g
struct AccelerationData {
    bool valid;
    float acc_x;
    float acc_y;
    float acc_z;
};

struct TemperatureData {
    bool valid;
    float temperature;
};

struct ImuData {
    bool valid;
    std::optional<TemperatureData> temperature;
    std::optional<GyroData> gyro;
    std::optional<AccelerationData> acc;
    ULONG timestamp;
};

// in radians
struct Orientation {
    float z; // yaw
    float y; // pitch
    float x; // roll
};

enum class DEVICE_STATUS { ON, OFF };

template <typename T> class IMU : public Sensor<T> {

public:
    IMU()
        : Sensor<T>()
    {
    }

    std::optional<AccelerationData> convert_acceleration();

    virtual GyroData read_gyroscope();
    virtual TemperatureData read_temperature();
    virtual AccelerationData read_acceleration();

    virtual uint8_t set_offset_acc_x(float offset);
    virtual uint8_t set_offset_acc_y(float offset);
    virtual uint8_t set_offset_acc_z(float offset);

protected:
    ImuData m_sensor_data {};
};

/**
 * @brief Change the measured values depending on the IMU's orientation.
 *
 * @param data Acceleration data.
 * @param orientation The orientation of IIM42652 in euler angles using YAW-PITCH-ROLL (Z-Y'-X'').
 *
 * @note See diagram on page 56:
 * https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf
 */
void accel_mount_offset(AccelerationData& data, const Orientation& e);

/**
 * @brief Change the measured values depending on the IMU's orientation.
 * 
 * @param data Acceleration data.
 * @param orientation The orientation of IIM42652 in euler angles using YAW-PITCH-ROLL (Z-Y'-X'').
 *
 * @note See diagram on page 56:
 * https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf
 */
void gyro_mount_offset(GyroData& data, const Orientation& e);

#endif