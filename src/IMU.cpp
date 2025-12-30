#include "IMU.h"

void accel_mount_offset(AccelerationData& data, const Orientation& e)
{
    auto q = Quaternion::fromEuler(e.x, e.y, e.z);

    // we need conjugate because we are transforming from rocket rotation -> sensor rotation.
    q.conjugate().rotateVector(data.acc_x, data.acc_y, data.acc_z);
}

void gyro_mount_offset(GyroData& data, const Orientation& e)
{
    auto q = Quaternion::fromEuler(e.x,e.y,e.z);
    q.conjugate().rotateVector(data.gyro_x, data.gyro_y, data.gyro_z);
}