// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

#include "IMU.h"
#include "Globals.h"
#include <math.h>

void IMU::init() {
	gyroADCToRad_dt = FASTLOOP_DT_F_S * M_PI / mpu->gyroToDeg_s() / 180.0;
	accComplFilterConstant = (float) MEDIUMLOOP_DT_F_S / (config.accTimeConstant + MEDIUMLOOP_DT_F_S);
}
