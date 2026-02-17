#include "IMU.h"

namespace IMU_methods {

void accel_mount_offset(AccelerationData& data, const Orientation& e)
{
    auto q = Quaternion::fromEuler(e.z, e.y, e.x);

    // we need conjugate because we are transforming from rocket rotation -> sensor rotation.
    q.conjugate().rotateVector(data.acc_x, data.acc_y, data.acc_z);
}

void gyro_mount_offset(GyroData& data, const Orientation& e)
{
    auto q = Quaternion::fromEuler(e.z, e.y, e.x);
    q.conjugate().rotateVector(data.gyro_x, data.gyro_y, data.gyro_z);
}

} // namespace IMU_methods
