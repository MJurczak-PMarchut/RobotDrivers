/*
 * vl53l5cx.cpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 *
 * History:
 * 	--Date: 12.11.2022
 * 	--Author: Mateusz
 * 	--Changes:
 * 		--A
 */

#include "vl53l5cx.hpp"
#include "vl53l5cx_buffers.h"

const static uint8_t __ToFAddr[] = { 0x54, 0x56, 0x58, 0x60, 0x62, 0x64 };
const static uint16_t __ToFX_SHUT_Pin[] = { XSHUT_3_Pin, XSHUT_3_Pin,
XSHUT_3_Pin,
XSHUT_4_Pin, XSHUT_5_Pin, XSHUT_6_Pin };
static GPIO_TypeDef *__ToFX_SHUT_Port[] = { XSHUT_3_GPIO_Port,
XSHUT_3_GPIO_Port,
XSHUT_3_GPIO_Port, XSHUT_4_GPIO_Port,
XSHUT_5_GPIO_Port, XSHUT_6_GPIO_Port };

uint8_t VL53L5CX::__sensor_nb = 0;
uint8_t VL53L5CX::__sensor_init_tbd = 0;

uint8_t VL53L5CX::null_data_sink = 0;

uint16_t VL53L5CX_INIT_REGS[] = {
		0x7FFF, 0x0009, 0x000F, 0x000A,
		0x7FFF,
		0x000C, 0x0101, 0x0102, 0x010A, 0x4002, 0x4002, 0x010A, 0x0103, 0x000C, 0x000F,
		0x0000,
		0x000F, 0x000A,
		0x0000,
		0x7FFF,
		0x0006,
		0x000E, 0x7FFF, 0x0003, 0x7FFF,
		0x0021,
		0x7FFF,
		0x7FFF,
		0x000C, 0x7FFF, 0x0101, 0x0102, 0x010A, 0x4002, 0x4002, 0x010A, 0x0103, 0x400F, 0x021A, 0x021A, 0x021A, 0x021A, 0x0219, 0x021B, 0x7FFF,
		0x7FFF,
		0x000C, 0x7FFF, 0x0020, 0x0020, 0x7FFF,
		0x0000,
		0x7FFF,
		0x0000,
		0x7FFF,
		0x0000,
		0x7FFF, 0x7FFF, 0x0003, 0x7FFF,
		0x0021,
		0x7FFF,
		0x7FFF,
		0x000C, 0x7FFF, 0x0114, 0x0115, 0x0116, 0x0117, 0x000B,
		0x7FFF,
		0x000C, 0x000B,
		0x0006,
		0x7FFF,
		/* Get offset NVM data and store them into the offset buffer */
		0x2FD8,
		VL53L5CX_UI_CMD_STATUS,
		VL53L5CX_UI_CMD_START,
		/* _vl53l5cx_send_offset_data */
		0x2E18,
		VL53L5CX_UI_CMD_STATUS

};
uint8_t VL53L5CX_INIT_REG_VALUES[] = {
		0x00, 0x04, 0x40, 0x03,
		0x00,
		0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x03, 0x01, 0x00, 0x43,
		0x00,
		0x40,0x01,
		0x00,
		0x00,
		0x00,
		0x01, 0x02, 0x0D, 0x01,
		0x00,
		0x00,
		0x00,
		0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x03, 0x01, 0x00, 0x43, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00,
		0x00,
		0x00, 0x01, 0x07, 0x06, 0x09,
		0x00,
		0x0A,
		0x00,
		0x0B,
		0x00,
		0x01, 0x02, 0x0D, 0x01,
		0x00,
		0x00,
		0x00,
		0x01, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00,
		0x00,
		0x00, 0x01,
		0x00,
		0x02,
		/* Get offset NVM data and store them into the offset buffer */
		0x00,
		0x00,
		0x00,
		/* _vl53l5cx_send_offset_data */
		0x00,
		0x00

};

VL53L5CX::VL53L5CX(e_ToF_Position position, CommManager *comm) :
		ToF_Sensor(vl53l5, position, comm), __InitSequenceID { 0 }, __wait_until_tick {
				0 }, __Status { TOF_STATE_NOT_INITIALIZED } {

	this->__sensor_index = __sensor_nb;
	__sensor_nb++;
	this->__sensor_conf.platform.__CommunicationManager = comm;
	this->__sensor_conf.platform.address = DEFAULT_ADDR;

	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_RESET);
}

/*
 * Initializes Sensors
 * Could probably be made static to initialize all sensors present
 * Or be implemented in parent class
 */
HAL_StatusTypeDef VL53L5CX::SensorInit(MessageInfoTypeDef* MsgInfo) {
	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef MsgInfoToSend = { 0 };

	MsgInfoToSend.context = this->__InitSequenceID;
	MsgInfoToSend.pCB = std::bind(&VL53L5CX::SensorInit, this, std::placeholders::_1);
	MsgInfoToSend.I2C_Addr = this->__address;
	MsgInfoToSend.I2C_MemAddr = VL53L5CX_INIT_REGS[this->__InitSequenceID];
	MsgInfoToSend.len = 1;
	MsgInfoToSend.pTxData = &VL53L5CX_INIT_REG_VALUES[this->__InitSequenceID];
	//sink for rx
	MsgInfoToSend.pRxData = &null_data_sink;
	//Init sensor
	if (this->__Status == TOF_STATE_NOT_INITIALIZED) {
		//Basically wait for sensor boot
		MsgInfoToSend.len = 0;
		this->__waitInit(1);
	} else {
		switch (this->__InitSequenceID) {
		case 0 ... 3:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 4:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
			break;
		case 5 ... 14:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 15:
			MsgInfoToSend.len = 0;
			this->__waitInit(1);
			break;
		case 16 ... 17:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 18:
			MsgInfoToSend.len = 0;
			this->__waitInit(100);
			break;
		case 19:
			this->__CheckInitPollingMessage(1, 0, 0xff, 1, MsgInfo, &MsgInfoToSend);
			break;
		case 20 ... 23:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 24:
			this->__CheckInitPollingMessage(1, 0, 0x10, 0x10, MsgInfo, &MsgInfoToSend);
			break;
		case 25:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 26:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
			break;
		case 27 ... 43:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 44:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
			break;
		case 45 ... 49:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 50:
			//FW download
			MsgInfoToSend.len = 0x8000;
			MsgInfoToSend.pTxData = (uint8_t*)&VL53L5CX_FIRMWARE[0];
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 51:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 52:
			//FW download #2
			MsgInfoToSend.len = 0x8000;
			MsgInfoToSend.pTxData = (uint8_t*)&VL53L5CX_FIRMWARE[0x8000];
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 53:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 54:
			//FW download #3
			MsgInfoToSend.len = 0x5000;
			MsgInfoToSend.pTxData = (uint8_t*)&VL53L5CX_FIRMWARE[0x10000];
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 55 ... 58:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 59:
			this->__CheckInitPollingMessage(1, 0, 0x10, 0x10, MsgInfo, &MsgInfoToSend);
			break;
		case 60:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 61:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
			break;
		case 62 ... 68:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 69:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
			break;
		case 70 ... 71:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			break;
		case 72:
			if (__vl53l5cx_poll_for_mcu_boot(&MsgInfoToSend) != 0)
			{
				Error_Handler();
			}
		case 73:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			MsgInfoToSend.len = sizeof(VL53L5CX_GET_NVM_CMD);
			MsgInfoToSend.pTxData = (uint8_t*)VL53L5CX_GET_NVM_CMD;
			break;
		case 74:
			this->__CheckInitPollingMessage(4, 0, 0xFF, 2, MsgInfo, &MsgInfoToSend);
			break;
		case 75:
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
			MsgInfoToSend.len = VL53L5CX_NVM_DATA_SIZE;
			MsgInfoToSend.pRxData = this-> __comm_buffer;
			break;
		case 76:
			//Send offset buffer;
			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
			(void)memcpy(this->__offset_buffer, this->__comm_buffer,
				VL53L5CX_OFFSET_BUFFER_SIZE);
			this->__vl53l5cx_send_offset_data(VL53L5CX_RESOLUTION_4X4);
			MsgInfoToSend.len = VL53L5CX_OFFSET_BUFFER_SIZE;
			MsgInfoToSend.pTxData = this->__comm_buffer;
			break;
		case 77:
			this->__CheckInitPollingMessage(4, 1, 0xFF, 2, MsgInfo, &MsgInfoToSend);
			break;
		}
		this->__InitSequenceID++;
	}
	if(MsgInfoToSend.len != 0) do{
		ret = this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
	}while(ret != HAL_OK);

	return ret;
}

HAL_StatusTypeDef VL53L5CX::SetI2CAddress() {
	uint8_t ret = vl53l5cx_set_i2c_address(&this->__sensor_conf,
			__ToFAddr[this->__sensor_index]);
	while(this->__Status != TOF_STATE_INIT_ONGOING){
		//Wait for address change
	}
	return (ret == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::IsAlive(uint8_t *is_alive) {
	uint8_t ret = vl53l5cx_is_alive(&this->__sensor_conf, is_alive);
	return (ret == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::StartRanging(void) {
	uint8_t ret = vl53l5cx_start_ranging(&this->__sensor_conf);
	return (ret == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::GetRangingData(void) {
	uint8_t ret = vl53l5cx_get_ranging_data(&this->__sensor_conf,
			&this->result);
	return (ret == 0) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef VL53L5CX::CheckDataReady(void) {
}

void VL53L5CX::__waitInit(uint32_t waitms) {
	this->__Status = TOF_STATE_INIT_WAIT;
	this->__wait_until_tick = pdMS_TO_TICKS(waitms) + xTaskGetTickCount();
}

ToF_Status_t VL53L5CX::CheckSensorStatus(void) {
	switch (this->__Status) {
	case TOF_STATE_INIT_WAIT:
		if (xTaskGetTickCount() >= this->__wait_until_tick) {
			this->__Status = TOF_STATE_INIT_ONGOING;
			this->SensorInit(NULL);
		}
		break;
	}
	return this->__Status;
}

HAL_StatusTypeDef VL53L5CX::__CheckInitPollingMessage(
		uint8_t					size,
		uint8_t					pos,
		uint8_t					mask,
		uint8_t					expected_value,
		MessageInfoTypeDef* 	MsgInfo,
		MessageInfoTypeDef*		MsgInfoToSend)
{
	if((MsgInfo != NULL) && (MsgInfo->context == this->__InitSequenceID)){
		//Received data
		if((this->__comm_buffer[pos] & mask) == expected_value){
			//The expected value
			MsgInfoToSend->len = 0;
			return HAL_OK;
		}
	}
	//keep polling
	MsgInfoToSend->pRxData = this->__comm_buffer;
	MsgInfoToSend->eCommType = COMM_INT_I2C_MEM_RX;
	this->__InitSequenceID--;
	//We don't want to actually callback this function, we can wait 10ms
	MsgInfoToSend->pCB = 0;
	this->__waitInit(10);
	return HAL_BUSY;
}


uint8_t VL53L5CX::__vl53l5cx_poll_for_mcu_boot(MessageInfoTypeDef* MsgInfoToSend)
{
	if((MsgInfoToSend != NULL) && (MsgInfoToSend->context == this->__InitSequenceID)){
		if((this->__comm_buffer[0] & (uint8_t)0x80) != (uint8_t)0){
			return this->__comm_buffer[1];
		}
		else if((this->__comm_buffer[0] & (uint8_t)0x1) != (uint8_t)0)
		{
			return 0;
		}
	}
	MsgInfoToSend->pRxData = this->__comm_buffer;
	MsgInfoToSend->eCommType = COMM_INT_I2C_MEM_RX;
	MsgInfoToSend->len = 2;
	this->__InitSequenceID--;
	MsgInfoToSend->pCB = 0;
	this->__waitInit(10);
	return 0;
}


uint8_t VL53L5CX::__vl53l5cx_send_offset_data(uint8_t resolution)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	uint32_t signal_grid[64];
	int16_t range_grid[64];
	uint8_t dss_4x4[] = {0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
	uint8_t footer[] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};
	int8_t i, j;
	uint16_t k;

	(void)memcpy(this->__comm_buffer,
               this->__offset_buffer, VL53L5CX_OFFSET_BUFFER_SIZE);

	/* Data extrapolation is required for 4X4 offset */
	if(resolution == (uint8_t)VL53L5CX_RESOLUTION_4X4){
		(void)memcpy(&(this->__comm_buffer[0x10]), dss_4x4, sizeof(dss_4x4));
		SwapBuffer(this->__comm_buffer, VL53L5CX_OFFSET_BUFFER_SIZE);
		(void)memcpy(signal_grid,&(this->__comm_buffer[0x3C]),
			sizeof(signal_grid));
		(void)memcpy(range_grid,&(this->__comm_buffer[0x140]),
			sizeof(range_grid));

		for (j = 0; j < (int8_t)4; j++)
		{
			for (i = 0; i < (int8_t)4 ; i++)
			{
				signal_grid[i+(4*j)] =
				(signal_grid[(2*i)+(16*j)+ (int8_t)0]
				+ signal_grid[(2*i)+(16*j)+(int8_t)1]
				+ signal_grid[(2*i)+(16*j)+(int8_t)8]
				+ signal_grid[(2*i)+(16*j)+(int8_t)9])
                                  /(uint32_t)4;
				range_grid[i+(4*j)] =
				(range_grid[(2*i)+(16*j)]
				+ range_grid[(2*i)+(16*j)+1]
				+ range_grid[(2*i)+(16*j)+8]
				+ range_grid[(2*i)+(16*j)+9])
                                  /(int16_t)4;
			}
		}
	    (void)memset(&range_grid[0x10], 0, (uint16_t)96);
	    (void)memset(&signal_grid[0x10], 0, (uint16_t)192);
            (void)memcpy(&(this->__comm_buffer[0x3C]),
		signal_grid, sizeof(signal_grid));
            (void)memcpy(&(this->__comm_buffer[0x140]),
		range_grid, sizeof(range_grid));
            SwapBuffer(this->__comm_buffer, VL53L5CX_OFFSET_BUFFER_SIZE);
	}

	for(k = 0; k < (VL53L5CX_OFFSET_BUFFER_SIZE - (uint16_t)4); k++)
	{
		this->__comm_buffer[k] = this->__comm_buffer[k + (uint16_t)8];
	}

	(void)memcpy(&(this->__comm_buffer[0x1E0]), footer, 8);
	return status;
}
