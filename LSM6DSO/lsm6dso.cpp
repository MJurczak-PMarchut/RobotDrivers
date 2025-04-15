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
#define TX_CONTEXT 15

static uint8_t pTXBuf[257] = {0};
static uint8_t pRxBuf[257] = {0};

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t angular_rate_mdps[3];

static float angle[3] = {0};
static float angle_rate_offset[3] ={910, 560, -350};

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
	SPIMutex = xSemaphoreCreateBinary();
	if(SPIMutex == NULL){
		Error_Handler();
	}
}

HAL_StatusTypeDef LSM6DSO::Init(void)
{
	static uint8_t whoamI, rst;
	whoamI = 0;
	vTaskDelay(100);
//	platform_read(this->dev_ctx.handle, 0xf, pRxBuf, 3);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);

//	if (whoamI != LSM6DSO_ID)
//	    return HAL_ERROR;
	lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(&dev_ctx, &rst);
	}while (rst);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
	lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_12Hz5);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	lsm6dso_device_id_get(&dev_ctx, &whoamI);
	lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_12Hz5);
	lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);
	lsm6dso_xl_hp_path_on_out_set(&dev_ctx, LSM6DSO_LP_ODR_DIV_100);
	lsm6dso_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
	uint8_t reg;
	while (1){
		lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);
		if (reg) {
		  /* Read acceleration field data */
		  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		  lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		  acceleration_mg[0] =
			lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
		  acceleration_mg[1] =
			lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
		  acceleration_mg[2] =
			lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);
		}
	    lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);

	    if (reg) {
	      /* Read angular rate field data */
	      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	      lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
	      angular_rate_mdps[0] =
	        lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[0]);
	      angular_rate_mdps[1] =
	        lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[1]);
	      angular_rate_mdps[2] =
	        lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate[2]);
		    angle[0] = angle[0] + ((angular_rate_mdps[0] - angle_rate_offset[0])/1000)/12.5;
		    angle[1] = angle[1] + ((angular_rate_mdps[1] - angle_rate_offset[1])/1000)/12.5;
		    angle[2] = angle[2] + ((angular_rate_mdps[2] - angle_rate_offset[2])/1000)/12.5;
	    }

	}
	return HAL_OK;
}




