/*
 * lsm6dso_quat.hpp
 *
 * Tilt-compensated attitude and dead-reckoned position estimate for the LSM6DSO.
 * Runs a Mahony-style complementary filter: the gyro integrates full 3D attitude (quaternion),
 * continuously corrected by a proportional feedback term that pulls roll/pitch back toward the
 * accelerometer's gravity vector. Yaw still has no absolute reference and drifts like plain gyro
 * integration; the correction is also skipped away from a resting 1g, since a collision's impact
 * force is indistinguishable from tilt in the accelerometer reading at that instant.
 */

#ifndef LSM6DSO_LSM6DSO_QUAT_HPP_
#define LSM6DSO_LSM6DSO_QUAT_HPP_

#include <stdint.h>
#include <math.h>

// Set to 0 to compile out the attitude/position math entirely (Update() becomes a no-op), e.g.
// for robot variants that don't need it and want to skip the per-sample quaternion + integration cost.
#ifndef LSM6DSO_QUAT_ESTIMATION_ENABLED
#define LSM6DSO_QUAT_ESTIMATION_ENABLED 1
#endif

typedef struct {
	double_t x;
	double_t y;
	double_t z;
} PositionTypeDef;

class LSM6DSOQuat{
public:
	LSM6DSOQuat();
	// gx/gy/gz: bias-corrected gyro rate in millidegrees/s. ax/ay/az: accelerometer reading in mg.
	void Update(double_t gx_mdps, double_t gy_mdps, double_t gz_mdps,
			double_t ax_mg, double_t ay_mg, double_t az_mg, double_t dt_s);
	PositionTypeDef GetPosition(void);
	void CalibratePosition(void);
private:
	void NormalizeQuaternion(void);
	double_t q[4]; // w, x, y, z; body-to-world attitude
	double_t velocity_mps[3];
	double_t position_m[3];
};

#endif /* LSM6DSO_LSM6DSO_QUAT_HPP_ */
