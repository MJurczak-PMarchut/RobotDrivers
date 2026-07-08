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
#if LSM6DSO_QUAT_ESTIMATION_ENABLED
	// Tilt-corrected orientation from the Mahony quaternion filter (see lsm6dso_quat.hpp);
	// same axis convention and units (degrees) as GetAngularOrientationForAxis.
	double GetCorrectedAngularOrientationForAxis(uint8_t axis);
	PositionTypeDef GetPosition(void);
	void CalibratePosition(void);
#endif
	bool IsCollisionDetected(void);
	void ClearCollisionDetected(void);
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
