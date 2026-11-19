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


#define RX_CONTEXT 10

// Sample-to-sample jumps above these counts as a collision; own motor-driven acceleration/turning
// ramps up over several samples (motor + drivetrain inertia), so it stays below these even at a
// similar peak value. Both are per-sample (1667 Hz ODR) deltas and need tuning on real hardware.
#define COLLISION_ACCEL_JERK_THRESHOLD_MG 950.0
#define COLLISION_GYRO_JERK_THRESHOLD_MDPS 600000.0

#define ACCEL_SAMPLE_DT_S (1.0/1667.0)

static uint8_t pTXBuf[257] = {0};
static uint8_t pRxBuf[257] = {0};

//static int16_t data_raw_temperature;

// static float angle[3] = {0};
// static float angle_rate_offset[3] ={910, 560, -350};

/*
 * Blocking-call request tagging (B2/B7, SPI part), same scheme as vl53l1_platform.cpp:
 * every attempt gets a fresh context, the completion callback only wakes the matching
 * request, a stale token is drained before each push, and a timeout removes the head
 * message only if it is ours (it may be an L9960T message on the shared hspi2).
 * Bit 15 keeps these contexts apart from the async IMU reads (RX_CONTEXT) and L9960T.
 * Only LSM6DSO::Init() uses these, from a single task, so no mutex is needed.
 */
#define BLOCKING_CTX_FLAG  0x8000u
static uint16_t _seq = 0;
static volatile uint16_t _waiting_ctx = 0;
static volatile HAL_StatusTypeDef _result = HAL_ERROR;
// TransactionStatus target; static because the ISR may write it after a timed-out call.
static HAL_StatusTypeDef _xfer_status = HAL_OK;

static void BlockingDoneCB(MessageInfoTypeDef<SPI>* MsgInfo)
{
	if(MsgInfo->context != _waiting_ctx)
	{
		return;   // completion of an abandoned request
	}
	_result = (MsgInfo->TransactionStatus != NULL)? *MsgInfo->TransactionStatus : HAL_ERROR;
	BaseType_t pxH = pdFALSE;
	xSemaphoreGiveFromISR(SPIMutex, &pxH);
	portYIELD_FROM_ISR(pxH);   // B11
}

// Up to two attempts on timeout. Returns 0 on success, -1 otherwise (stmdev_ctx convention).
static int32_t _PushAndWait(MessageInfoTypeDef<SPI> *MsgInfo)
{
	for(uint8_t attempt = 0; attempt < 2; attempt++)
	{
		uint16_t ctx = (uint16_t)(BLOCKING_CTX_FLAG | (++_seq & 0x7FFFu));
		MsgInfo->context = ctx;
		MsgInfo->TransactionStatus = &_xfer_status;
		xSemaphoreTake(SPIMutex, 0);   // drop a token left by an abandoned request
		_waiting_ctx = ctx;
		// HAL_BUSY means queued behind a motor-driver transfer: wait for it anyway.
		_CommunicationManagerStatic->PushCommRequestIntoQueue(MsgInfo);
		bool done = (xSemaphoreTake(SPIMutex, 20) == pdTRUE);
		_waiting_ctx = 0;
		if(done)
		{
			return (_result == HAL_OK)? 0 : -1;
		}
		_CommunicationManagerStatic->RemoveHeadIfContext((SPI_HandleTypeDef*)MsgInfo->IntHandle, ctx);
	}
	return -1;
}


// These are blocking functions
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
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
	MsgInfo.pTxData = pTXBuf;
	MsgInfo.eCommType = COMM_INT_TX;
	MsgInfo.pTxCompletedCB = BlockingDoneCB;
	MsgInfo.GPIO_PIN = LSM6DSO_nCS_PIN;
	MsgInfo.GPIOx = LSM6DSO_nCS_PORT;
	return _PushAndWait(&MsgInfo);
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
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
	MsgInfo.pTxData = pTXBuf;
	MsgInfo.pRxData = pRxBuf;
	MsgInfo.eCommType = COMM_INT_TXRX;
	MsgInfo.pRxCompletedCB = BlockingDoneCB;
	MsgInfo.GPIO_PIN = LSM6DSO_nCS_PIN;
	MsgInfo.GPIOx = LSM6DSO_nCS_PORT;
	int32_t ret = _PushAndWait(&MsgInfo);
	if(ret == 0)
	{
		memcpy(bufp, pRxBuf+1, len);   // copy out only a confirmed result
	}
	return ret;
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
	__CommManager->PushCommRequestIntoQueue(&MsgInfo);
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
		GetGyData();
	}
	else if(InterruptPin == IMU_INT2_Pin)
	{
		GetXlData();
	}
}

double LSM6DSO::GetAngularOrientationForAxis(uint8_t axis)
{
	if(axis < 3)
	#if LSM6DSO_QUAT_ESTIMATION_OVERRIDE  and  LSM6DSO_QUAT_ESTIMATION_ENABLED
		return this->GetCorrectedAngularOrientationForAxis(axis);
	#else
		return angular_orientation[axis];
	#endif
	else
		return NAN;
}

#if LSM6DSO_QUAT_ESTIMATION_ENABLED
double LSM6DSO::GetCorrectedAngularOrientationForAxis(uint8_t axis)
{
	return this->_positionEstimator.GetOrientationForAxis(axis);
}

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
