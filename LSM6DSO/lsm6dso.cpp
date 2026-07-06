/*
 * lsm6dso.cpp
 *
 *  Created on: Apr 13, 2025
 *      Author: jurcz
 */
#include "lsm6dso.hpp"
#include "semphr.h"
#include "string.h"
#include "Configuration.h"
#include "RobotSpecificDefines.hpp"

static CommManager *_CommunicationManagerStatic = NULL;
static SemaphoreHandle_t SPIMutex;

volatile uint32_t g_imu_int1_count = 0;
volatile uint32_t g_imu_int2_count = 0;
volatile HAL_StatusTypeDef g_imu_pushRequestStatus = HAL_ERROR;
// Live readback of the sensor's own interrupt-routing registers, refreshed by
// LSM6DSO::DiagnoseInterruptConfig(). Expected: drdy_g=1, drdy_mode=LSM6DSO_DRDY_PULSED (1).
// If either drifts from that after INT1 has frozen, the sensor's config got corrupted
// (e.g. a corrupted SPI write meant for a motor controller landing here instead).
volatile uint8_t g_imu_diag_int1_drdy_g = 0xFF;
volatile int32_t g_imu_diag_drdy_mode = -1;

#define RX_CONTEXT 10
#define TX_CONTEXT 15

// Sample-to-sample jumps above these counts as a collision; own motor-driven acceleration/turning
// ramps up over several samples (motor + drivetrain inertia), so it stays below these even at a
// similar peak value. Both are per-sample (1667 Hz ODR) deltas and need tuning on real hardware.
#define COLLISION_ACCEL_JERK_THRESHOLD_MG 400.0
#define COLLISION_GYRO_JERK_THRESHOLD_MDPS 500000.0

#define ACCEL_SAMPLE_DT_S (1.0/1667.0)

static uint8_t pTXBuf[257] = {0};
static uint8_t pRxBuf[257] = {0};

//static int16_t data_raw_temperature;

// static float angle[3] = {0};
// static float angle_rate_offset[3] ={910, 560, -350};

void RxCallbackFunc(MessageInfoTypeDef<SPI>* MsgInfo)
{
	BaseType_t pxH;
	xSemaphoreGiveFromISR(SPIMutex, &pxH);
}

void TxCallbackFunc(MessageInfoTypeDef<SPI>* MsgInfo)
{
	BaseType_t pxH;
	xSemaphoreGiveFromISR(SPIMutex, &pxH);
}


// These are blocking functions
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef status;
	if(_CommunicationManagerStatic == NULL)
	{
		return -1;
	}
	if(len > 256){
		Error_Handler();
		return -1;
	}
	pTXBuf[0]= reg;
	memcpy(pTXBuf + 1, bufp, len);
	MessageInfoTypeDef<SPI> MsgInfo = {0};
	MsgInfo.IntHandle = (SPI_HandleTypeDef*)handle;
	MsgInfo.len = len + 1;
	MsgInfo.spi_cpol_high = 1;
	MsgInfo.context = TX_CONTEXT;
	MsgInfo.pTxData = pTXBuf;
	MsgInfo.eCommType = COMM_INT_TX;
	MsgInfo.pTxCompletedCB = TxCallbackFunc;
	MsgInfo.GPIO_PIN = LSM6DSO_nCS_PIN;
	MsgInfo.GPIOx = LSM6DSO_nCS_PORT;
	status = _CommunicationManagerStatic->PushCommRequestIntoQueue(&MsgInfo);
	if(status != HAL_OK)
	{
		return -1;
	}
	if(xSemaphoreTake(SPIMutex, 20) == pdFALSE){
		_CommunicationManagerStatic->MsgReceivedCB((SPI_HandleTypeDef*)handle);
		xSemaphoreTake(SPIMutex, 1);
		_CommunicationManagerStatic->PushCommRequestIntoQueue(&MsgInfo); // try again
		if(xSemaphoreTake(SPIMutex, 20) == pdFALSE){
			_CommunicationManagerStatic->MsgReceivedCB((SPI_HandleTypeDef*)handle);
			return -1;
		}
	}
	return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_StatusTypeDef status;
	if(_CommunicationManagerStatic == NULL)
	{
		return -1;
	}
	if(len > 256){
		Error_Handler();
		return -1;
	}
	pTXBuf[0]= reg | (1<<7);
	memset(pTXBuf + 1, 0, len);
	MessageInfoTypeDef<SPI> MsgInfo = {0};
	MsgInfo.IntHandle = (SPI_HandleTypeDef*)handle;
	MsgInfo.len = len + 1;
	MsgInfo.spi_cpol_high = 1;
	MsgInfo.context = RX_CONTEXT;
	MsgInfo.pTxData = pTXBuf;
	MsgInfo.pRxData = pRxBuf;
	MsgInfo.eCommType = COMM_INT_TXRX;
	MsgInfo.pRxCompletedCB = RxCallbackFunc;
	MsgInfo.GPIO_PIN = LSM6DSO_nCS_PIN;
	MsgInfo.GPIOx = LSM6DSO_nCS_PORT;
	status = _CommunicationManagerStatic->PushCommRequestIntoQueue(&MsgInfo);
	if(status != HAL_OK)
	{
		return -1;
	}
	if(xSemaphoreTake(SPIMutex, 20) == pdFALSE){
		_CommunicationManagerStatic->MsgReceivedCB((SPI_HandleTypeDef*)handle);
		xSemaphoreTake(SPIMutex, 1);
		_CommunicationManagerStatic->PushCommRequestIntoQueue(&MsgInfo); // try again
		if(xSemaphoreTake(SPIMutex, 20) == pdFALSE){
			_CommunicationManagerStatic->MsgReceivedCB((SPI_HandleTypeDef*)handle);
			return -1;
		}
	}
	memcpy(bufp, pRxBuf+1, len);
	return 0;
}

static void platform_delay(uint32_t ms)
{
	vTaskDelay(ms);
}

LSM6DSO::LSM6DSO(CommManager *CommunicationManager, SPI_HandleTypeDef *hspi)
{
	__CommManager = CommunicationManager;
	_CommunicationManagerStatic = CommunicationManager;
	this->dev_ctx.handle = (void*)hspi;
	this->dev_ctx.mdelay = platform_delay;
	this->dev_ctx.read_reg = platform_read;
	this->dev_ctx.write_reg = platform_write;
	this->_hspi = hspi;
	this->__init_completed = false;
	this->collision_detected = false;
	this->prev_horizontal_accel_mg = 0;
	this->prev_yaw_rate_mdps = 0;
	memset(this->pTxGyData, 0, 7);
	this->pTxGyData[0] = LSM6DSO_OUTX_L_G | (1<<7);
	this->_CallbackFuncGy = std::bind(&LSM6DSO::GyDataReceivedCB, this, std::placeholders::_1);
	memset(this->pTxXlData, 0, 7);
	this->pTxXlData[0] = LSM6DSO_OUTX_L_A | (1<<7);
	this->_CallbackFuncXl = std::bind(&LSM6DSO::XlDataReceivedCB, this, std::placeholders::_1);
	SPIMutex = xSemaphoreCreateBinary();
	if(SPIMutex == NULL){
		Error_Handler();
	}
}

HAL_StatusTypeDef LSM6DSO::Init(void)
{
	static uint8_t whoamI, rst;
	lsm6dso_pin_int1_route_t int1_route = {0};
	lsm6dso_pin_int2_route_t int2_route = {0};
	whoamI = 0;
	vTaskDelay(100);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);

	if (whoamI != LSM6DSO_ID)
	    return HAL_ERROR;
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	}while (rst);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_1667Hz);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_1667Hz);
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_8g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);
	lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dso_gy_filter_lp1_set(&dev_ctx, PROPERTY_ENABLE);
	int1_route.drdy_g = 1;
	int2_route.drdy_xl = 1;
	lsm6dso_dataready_pulsed_t drdy_mode = LSM6DSO_DRDY_PULSED;
	lsm6dso_data_ready_mode_set(&dev_ctx, drdy_mode);
	lsm6dso_pin_int1_route_set(&dev_ctx, int1_route);
	lsm6dso_pin_int2_route_set(&dev_ctx, NULL, int2_route);
	this->__init_completed = true;
	return HAL_OK;
}

void LSM6DSO::GyDataReceivedCB(MessageInfoTypeDef<SPI>* MsgInfo)
{
	uint8_t buffer_index = 0;
	for(uint8_t plane=0; plane < 3; plane++){
		buffer_index = 1 + (plane*2);
		this->data_raw_angular_rate[plane] = (this->pRxGyData[buffer_index] | (this->pRxGyData[buffer_index + 1] << 8));
		this->angular_rate_mdps[plane] = lsm6dso_from_fs2000_to_mdps(this->data_raw_angular_rate[plane]);
		this->angular_orientation[plane] = this->angular_orientation[plane] +
				((this->angular_rate_mdps[plane] - this->gyro_offset[plane])/(1000 * 1667));
		this->gyro_cal_offset[plane] = this->gyro_cal_offset[plane] + this->angular_rate_mdps[plane];
	}
	// A collision that spins the robot shows up as a sudden jump in yaw rate between samples;
	// an intentional turn ramps up gradually, so it stays below this even at a similar peak rate.
	if(fabs(this->angular_rate_mdps[2] - this->prev_yaw_rate_mdps) > COLLISION_GYRO_JERK_THRESHOLD_MDPS){
		this->collision_detected = true;
	}
	this->prev_yaw_rate_mdps = this->angular_rate_mdps[2];
	this->measurements_no = this->measurements_no + 1;

}

void LSM6DSO::XlDataReceivedCB(MessageInfoTypeDef<SPI>* MsgInfo)
{
	uint8_t buffer_index = 0;
	for(uint8_t plane=0; plane < 3; plane++){
		buffer_index = 1 + (plane*2);
		this->data_raw_acceleration[plane] = (this->pRxXlData[buffer_index] | (this->pRxXlData[buffer_index + 1] << 8));
		this->acceleration_mg[plane] = lsm6dso_from_fs8_to_mg(this->data_raw_acceleration[plane]);
	}
	// Horizontal plane (X/Y) excludes the Z axis, which mostly carries gravity and vertical bounce.
	double_t horizontal_accel_mg = sqrt(this->acceleration_mg[0]*this->acceleration_mg[0]
			+ this->acceleration_mg[1]*this->acceleration_mg[1]);
	// A collision arrives as a sudden jump between samples; the robot's own motor-driven
	// acceleration ramps up over several samples due to motor/drivetrain inertia, so its
	// sample-to-sample delta stays well below this even at a similar peak magnitude.
	if(fabs(horizontal_accel_mg - this->prev_horizontal_accel_mg) > COLLISION_ACCEL_JERK_THRESHOLD_MG){
		this->collision_detected = true;
	}
	this->prev_horizontal_accel_mg = horizontal_accel_mg;

	#if LSM6DSO_QUAT_ESTIMATION_ENABLED
	// Attitude (tilt-compensated) + dead-reckoned position, fed with the latest gyro sample
	// (updated independently by GyDataReceivedCB at the same 1667 Hz ODR, just phase-shifted).
	this->_positionEstimator.Update(
			this->angular_rate_mdps[0] - this->gyro_offset[0],
			this->angular_rate_mdps[1] - this->gyro_offset[1],
			this->angular_rate_mdps[2] - this->gyro_offset[2],
			this->acceleration_mg[0], this->acceleration_mg[1], this->acceleration_mg[2],
			ACCEL_SAMPLE_DT_S);
	#endif
}

void LSM6DSO::GetGyData(void)
{
	memset(pRxGyData, 0, 7);
	MessageInfoTypeDef<SPI> MsgInfo = {0};
	MsgInfo.IntHandle = this->_hspi;
	MsgInfo.len = 7;
	MsgInfo.spi_cpol_high = 1;
	MsgInfo.context = RX_CONTEXT;
	MsgInfo.pTxData = this->pTxGyData;
	MsgInfo.pRxData = this->pRxGyData;
	MsgInfo.eCommType = COMM_INT_TXRX;
	MsgInfo.pCB = &this->_CallbackFuncGy;
	MsgInfo.GPIO_PIN = LSM6DSO_nCS_PIN;
	MsgInfo.GPIOx = LSM6DSO_nCS_PORT;
	g_imu_pushRequestStatus = __CommManager->PushCommRequestIntoQueue(&MsgInfo);
}

void LSM6DSO::GetXlData(void)
{
	memset(pRxXlData, 0, 7);
	MessageInfoTypeDef<SPI> MsgInfo = {0};
	MsgInfo.IntHandle = this->_hspi;
	MsgInfo.len = 7;
	MsgInfo.spi_cpol_high = 1;
	MsgInfo.context = RX_CONTEXT;
	MsgInfo.pTxData = this->pTxXlData;
	MsgInfo.pRxData = this->pRxXlData;
	MsgInfo.eCommType = COMM_INT_TXRX;
	MsgInfo.pCB = &this->_CallbackFuncXl;
	MsgInfo.GPIO_PIN = LSM6DSO_nCS_PIN;
	MsgInfo.GPIOx = LSM6DSO_nCS_PORT;
	__CommManager->PushCommRequestIntoQueue(&MsgInfo);
}

void LSM6DSO::StartCalibrationOrientation(void)
{
	this->measurements_no = 0;
	for(uint8_t plane=0; plane < 3; plane++){
		this->gyro_offset[plane] = 0;
		this->gyro_cal_offset[plane] = 0;
	}
}

void LSM6DSO::CalibrateOrientation(void)
{
	for(uint8_t plane=0; plane < 3; plane++){
		this->gyro_offset[plane] = this->gyro_cal_offset[plane] / this->measurements_no;
		this->angular_orientation[plane] = 0;
	}
}

void LSM6DSO::ResetAngularOrientation(void)
{
	for(uint8_t plane=0; plane < 3; plane++){
		this->angular_orientation[plane] = 0;
	}
}

HAL_StatusTypeDef LSM6DSO::IsInitCompleted(void)
{
	return (this->__init_completed)? HAL_OK : HAL_ERROR;
}

void LSM6DSO::InterruptCallback(uint16_t InterruptPin){
	if(IsInitCompleted() != HAL_OK)
	{
		return;
	}
	if(InterruptPin == IMU_INT1_Pin)
	{
		g_imu_int1_count++;
		GetGyData();
	}
	else if(InterruptPin == IMU_INT2_Pin)
	{
		g_imu_int2_count++;
		GetXlData();
	}
}

double LSM6DSO::GetAngularOrientationForAxis(uint8_t axis)
{
	if(axis < 3)
		return angular_orientation[axis];
	else
		return NAN;
}

#if LSM6DSO_QUAT_ESTIMATION_ENABLED
PositionTypeDef LSM6DSO::GetPosition(void)
{
	return this->_positionEstimator.GetPosition();
}

void LSM6DSO::CalibratePosition(void)
{
	this->_positionEstimator.CalibratePosition();
}
#endif

bool LSM6DSO::IsCollisionDetected(void)
{
	return this->collision_detected;
}

void LSM6DSO::ClearCollisionDetected(void)
{
	this->collision_detected = false;
}

void LSM6DSO::DiagnoseInterruptConfig(void)
{
	lsm6dso_pin_int1_route_t route = {0};
	lsm6dso_dataready_pulsed_t mode = LSM6DSO_DRDY_LATCHED;

	lsm6dso_pin_int1_route_get(&dev_ctx, &route);
	lsm6dso_data_ready_mode_get(&dev_ctx, &mode);

	g_imu_diag_int1_drdy_g = route.drdy_g;
	g_imu_diag_drdy_mode = mode;
}

