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
		0x000C, 0x7FFF, 0x0101, 0x0102, 0x010A, 0x4002, 0x4002, 0x010A, 0x0103, 0x400F, 0x021A, 0x021A, 0x021A, 0x021A, 0x0219, 0x021B, 0x7FFF
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
		0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x03, 0x01, 0x00, 0x43, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00
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
	MsgInfoToSend.pCB = std::bind(&VL53L5CX::Init, this, std::placeholders::_1);
	MsgInfoToSend.I2C_Addr = this->__address;
	MsgInfoToSend.I2C_MemAddr = VL53L5CX_INIT_REGS[this->__InitSequenceID];
	MsgInfoToSend.len = 1;
	MsgInfoToSend.pTxData = &VL53L5CX_INIT_REG_VALUES[this->__InitSequenceID];
	//sink for rx
	MsgInfoToSend.pRxData = &null_data_sink;
	//Init sensor
	if (this->__Status == TOF_STATE_NOT_INITIALIZED) {
		//Basically wait for sensor boot
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

HAL_StatusTypeDef VL53L5CX::__CheckPollingMessage(
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
			//Not the expected value
			MsgInfoToSend.len = 0;
		}
	}
	else{
		//keep polling
		MsgInfoToSend->pRxData = this->__comm_buffer;
		MsgInfoToSend->eCommType = COMM_INT_I2C_MEM_RX;
		this->__InitSequenceID--;
		//We don't want to actually callback this function, we can wait 10ms
		MsgInfoToSend.pCB = 0;
		this->__waitInit(10);
	}
}
