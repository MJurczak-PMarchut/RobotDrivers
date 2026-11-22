/*
 * lsm6dso.hpp
 *
 *  Created on: Apr 13, 2025
 *      Author: jurcz
 */

#ifndef LSM6DSO_LSM6DSO_HPP_
#define LSM6DSO_LSM6DSO_HPP_

#include "CommManager.hpp"
#include "lsm6dso_reg.h"
#include "lsm6dso_quat.hpp"

#ifndef USES_RTOS
#error "This lib cannot function without RTOS!"
#endif

// One single-tap event of the embedded tap engine (TAP_SRC).
struct LSM6DSOTapEvent
{
	bool x = false;          // axes on which the tap was detected
	bool y = false;
	bool z = false;
	bool negative = false;   // sign of the acceleration of the tap
};

class LSM6DSO{
public:
	LSM6DSO(CommManager *CommunicationManager, SPI_HandleTypeDef *hspi);
	HAL_StatusTypeDef Init(void);
	void InterruptCallback(uint16_t InterruptPin);
	void StartCalibrationOrientation(void);
	void CalibrateOrientation(void);
	void ResetAngularOrientation(void);
	HAL_StatusTypeDef IsInitCompleted(void);
	void GyDataReceivedCB(MessageInfoTypeDef<SPI>* MsgInfo);
	void XlDataReceivedCB(MessageInfoTypeDef<SPI>* MsgInfo);
	double GetAngularOrientationForAxis(uint8_t axis);
	// Last sample: angular rate [deg/s] with the calibrated gyro offset removed, and
	// acceleration [g] (sensor axes). NAN for an axis above 2.
	double GetAngularRateForAxis(uint8_t axis);
	double GetAccelerationForAxis(uint8_t axis);
#if LSM6DSO_QUAT_ESTIMATION_ENABLED
	// Tilt-corrected orientation from the Mahony quaternion filter (see lsm6dso_quat.hpp);
	// same axis convention and units (degrees) as GetAngularOrientationForAxis.
	double GetCorrectedAngularOrientationForAxis(uint8_t axis);
	PositionTypeDef GetPosition(void);
	void CalibratePosition(void);
#endif
	bool IsCollisionDetected(void);
	void ClearCollisionDetected(void);
	// Embedded single-tap detection, latched until ReadTap(); no interrupt pin is routed.
	// Thresholds 0..31 in steps of full scale / 32 (250 mg at +-8 g); 0 turns the axis off.
	// shock / quiet 0..3 (INT_DUR2) at 1667 Hz ODR: shock 0 = 2.4 ms, n = n * 4.8 ms (max
	// time above threshold); quiet 0 = 1.2 ms, n = n * 2.4 ms (time after a tap with no new one).
	// hp_filter: the tap engine sees the high-pass filter (cut-off set by Init(), ODR/100 =
	// 16.7 Hz), i.e. the acceleration of the tap itself; false: the slope filter (half the
	// difference of consecutive samples, small at 1667 Hz for a tap spread over a few ms).
	// Blocking SPI: call only after Init() and from the task that ran Init().
	HAL_StatusTypeDef EnableTapDetection(uint8_t ths_x, uint8_t ths_y, uint8_t ths_z,
			uint8_t shock, uint8_t quiet, bool hp_filter);
	// Reads TAP_SRC, which also clears the latch. True when it holds a tap (`event` filled).
	// Blocking SPI, same task rule as EnableTapDetection().
	bool ReadTap(LSM6DSOTapEvent& event);
protected:
	void GetGyData(void);
	void GetXlData(void);
private:
	std::function<void(MessageInfoTypeDef<SPI> *MsgInfo)> _CallbackFuncGy;
	std::function<void(MessageInfoTypeDef<SPI> *MsgInfo)> _CallbackFuncXl;
	uint8_t pRxGyData[7];
	uint8_t pTxGyData[7];
	uint8_t pRxXlData[7];
	uint8_t pTxXlData[7];
	volatile uint32_t measurements_no;
	int16_t data_raw_acceleration[3];
	int16_t data_raw_angular_rate[3];
	double_t acceleration_mg[3];
	double_t angular_rate_mdps[3];
	double_t angular_orientation[3];
	double_t gyro_offset[3];
	double_t gyro_cal_offset[3];
	double_t prev_horizontal_accel_mg;
	double_t prev_yaw_rate_mdps;
	volatile bool collision_detected;
	bool __init_completed;
	SPI_HandleTypeDef *_hspi;
	CommManager *__CommManager;
	stmdev_ctx_t dev_ctx;
#if LSM6DSO_QUAT_ESTIMATION_ENABLED
	LSM6DSOQuat _positionEstimator;
#endif
//	uint16_t _LSM6DSO_nCS_PIN;
//	GPIO_TypeDef *_LSM6DSO_nCS_Port;
};


#endif /* LSM6DSO_LSM6DSO_HPP_ */
