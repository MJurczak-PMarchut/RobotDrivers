/*
 * vl53l5cx.cpp
 *
 *  Created on: 29 paź 2022
 *      Author: Paulina
 *
 * History:
 * 	--Date: 15.11.2022
 * 	--Author: Mateusz
 * 	--Changes: 	Added SensorInit and helper functions
 *
 */

#include "vl53l5cx.hpp"
#include "vl53l5cx_buffers.h"
#include "RobotSpecificDefines.hpp"

const static uint8_t __ToFAddr[] = { 0x54, 0x56, 0x58, 0x60, 0x62, 0x64 };

const static uint16_t __ToFX_SHUT_Pin[] = XSHUT_PINS;
const static uint16_t __ToFX_GPIO_Pin[] = TOFx_GPIO_PINS;
static GPIO_TypeDef *__ToFX_SHUT_Port[] = TOFx_XSHUT_PORTS;

uint8_t VL53L5CX::__sensor_init_tbd = 0;

uint8_t VL53L5CX::null_data_sink = 0;

uint16_t VL53L5CX_INIT_REGS[] = {
		/* SW reboot sequence */
		0x7FFF, 0x0009, 0x000F, 0x000A,
		0x7FFF,
		0x000C, 0x0101, 0x0102, 0x010A, 0x4002, 0x4002, 0x010A, 0x0103, 0x000C, 0x000F,
		0x0000,
		0x000F, 0x000A,
		0x0000,
		/* Wait for sensor booted (several ms required to get sensor ready ) */
		0x7FFF,
		0x0006,
		0x000E, 0x7FFF,
		/* Enable FW access */
		0x0003, 0x7FFF,
		0x0021,
		0x7FFF,
		/* Enable host access to GO1 */
		0x7FFF,
		0x000C,
		/* Power ON status */
		0x7FFF, 0x0101, 0x0102, 0x010A, 0x4002, 0x4002, 0x010A, 0x0103, 0x400F, 0x021A, 0x021A, 0x021A, 0x021A, 0x0219, 0x021B,
		/* Wake up MCU */
		0x7FFF,
		0x7FFF,
		0x000C, 0x7FFF, 0x0020, 0x0020,
		/* Download FW into VL53L5 */
		0x7FFF,
		0x0000,
		0x7FFF,
		0x0000,
		0x7FFF,
		0x0000,
		0x7FFF,
		/* Check if FW correctly downloaded */
		0x7FFF, 0x0003, 0x7FFF,
		0x0021,
		0x7FFF,
		0x7FFF,
		0x000C,
		/* Reset MCU and wait boot */
		0x7FFF, 0x0114, 0x0115, 0x0116, 0x0117, 0x000B,
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
		VL53L5CX_UI_CMD_STATUS,
		/*__vl53l5cx_send_xtalk_data */
		0x2CF8,
		/* Send default configuration to VL53L5CX firmware */
		0x2C34,
		VL53L5CX_UI_CMD_STATUS,
		/* send dci #1 */
		0x0000,
		VL53L5CX_UI_CMD_STATUS,
		/* send dci #2 */
		0x0000,
		VL53L5CX_UI_CMD_STATUS,
		/* Init ends here, now send start ranging command and set frequency*/
		0x0000,
		VL53L5CX_UI_CMD_STATUS,
		/* second dci in start ranging */
		0x0000,
		VL53L5CX_UI_CMD_STATUS,
		/* third dci */
		0x0000,
		VL53L5CX_UI_CMD_STATUS,
		/* Start xshut bypass (interrupt mode) */
		0x7FFF, 0x0009, 0x7FFF,
		/* WrMulti*/
		VL53L5CX_UI_CMD_END - (uint16_t)(4 - 1),
		VL53L5CX_UI_CMD_STATUS,
		/* Read ui range data content and compare if data size is the correct one */
		/* Request data reading from FW */
		VL53L5CX_UI_CMD_END-(uint16_t)11,
		VL53L5CX_UI_CMD_STATUS,
		/* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
		VL53L5CX_UI_CMD_START,



};
uint8_t VL53L5CX_INIT_REG_VALUES[] = {
		/* SW reboot sequence */
		0x00, 0x04, 0x40, 0x03,
		0x00,
		0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x03, 0x01, 0x00, 0x43,
		0x00,
		0x40,0x01,
		0x00,
		/* Wait for sensor booted (several ms required to get sensor ready ) */
		0x00,
		0x00,
		0x01, 0x02,
		/* Enable FW access */
		0x0D, 0x01,
		0x00,
		0x00,
		0x00,
		0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x03, 0x01, 0x00, 0x43, 0x03, 0x01, 0x00, 0x00, 0x00,
		/* Wake up MCU */
		0x00,
		0x00,
		0x00, 0x01, 0x07, 0x06,
		/* Download FW into VL53L5 */
		0x09,
		0x00,
		0x0A,
		0x00,
		0x0B,
		0x00,
		0x01,
		/* Check if FW correctly downloaded */
		0x02, 0x0D, 0x01,
		0x00,
		0x00,
		0x00,
		0x01,
		/* Reset MCU and wait boot */
		0x00, 0x00, 0x00, 0x42, 0x00, 0x00,
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
		0x00,
		/*__vl53l5cx_send_xtalk_data */
		0x00,
		/* Send default configuration to VL53L5CX firmware */
		0x00,
		0x00,
		/* send dci #1 */
		0x00,
		0x00,
		/* send dci #2 */
		0x00,
		0x00,
		/* Init ends here, now send start ranging command and set frequency*/
		0x00,
		0x00,
		/* second dci in start ranging */
		0x00,
		0x00,
		/* third dci */
		0x00,
		0x00,
		/* Start xshut bypass (interrupt mode) */
		0x00, 0x05, 0x02,
		/* WrMulti*/
		0x00,
		0x00,
		/* Read ui range data content and compare if data size is the correct one */
		/* Request data reading from FW */
		0x00,
		0x00,
		/* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
		0x00

};

/* Enable mandatory output (meta and common data) */
uint32_t output_bh_enable[] = {
	0x00000007U,
	0x00000000U,
	0x00000000U,
	0xC0000000U};

/* Send addresses of possible output */
uint32_t output[] ={VL53L5CX_START_BH,
	VL53L5CX_METADATA_BH,
	VL53L5CX_COMMONDATA_BH,
	VL53L5CX_AMBIENT_RATE_BH,
	VL53L5CX_SPAD_COUNT_BH,
	VL53L5CX_NB_TARGET_DETECTED_BH,
	VL53L5CX_SIGNAL_RATE_BH,
	VL53L5CX_RANGE_SIGMA_MM_BH,
	VL53L5CX_DISTANCE_BH,
	VL53L5CX_REFLECTANCE_BH,
	VL53L5CX_TARGET_STATUS_BH,
	VL53L5CX_MOTION_DETECT_BH};

uint8_t start_ranging_cmd[] = {0x00, 0x03, 0x00, 0x00};
uint32_t header_config[2] = {0, 0};

VL53L5CX::VL53L5CX(e_ToF_Position position, CommManager *comm, I2C_HandleTypeDef *hi2c1) :
		ToF_Sensor(vl53l5, position, comm), __hi2c1{hi2c1}, __InitSequenceID{0}, __wait_until_tick{0}, __Status {TOF_INIT_NOT_DONE}, __data_count{0}, eOrientation{ROTATE_0} {

	this->__sensor_index = __no_of_sensors - 1;
	this->__sensor_conf.platform.__CommunicationManager = comm;
	this->__sensor_conf.platform.address = DEFAULT_ADDR;

	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_RESET);
	last_update_tick = HAL_GetTick();
	this->_CallbackFunc = std::bind(&VL53L5CX::DataReceived, this, std::placeholders::_1);

}

HAL_StatusTypeDef VL53L5CX::SetRotation(SensorSpatialOrientation Orientation)
{
	if(Orientation <= ROTATE_270){
		eOrientation = Orientation;
		return HAL_OK;
	}
	return HAL_ERROR;
}

uint16_t VL53L5CX::GetSensorITPin(void)
{
	return __ToFX_GPIO_Pin[this->__sensor_index];
}

HAL_StatusTypeDef VL53L5CX::SensorInit(void)
{
	uint8_t ret;

	ret = vl53l5cx_init(&this->__sensor_conf);
	ret |= vl53l5cx_set_ranging_mode(&this->__sensor_conf, VL53L5CX_RANGING_MODE_CONTINUOUS);
	ret |= vl53l5cx_set_ranging_frequency_hz(&this->__sensor_conf, 60);
	ret |= vl53l5cx_start_ranging(&this->__sensor_conf);
	this->__Status = (ret)?TOF_STATE_ERROR:TOF_STATE_OK;
	this->last_update_tick = HAL_GetTick();
	return (ret)?HAL_OK:HAL_ERROR;
}

/*
 * Initializes Sensors
 * Could probably be made static to initialize all sensors present
 * Or be implemented in parent class
 */
//HAL_StatusTypeDef VL53L5CX::SensorInit(MessageInfoTypeDef* MsgInfo) {
//	HAL_StatusTypeDef ret = HAL_OK;
//	MessageInfoTypeDef MsgInfoToSend = { 0 };
//	uint32_t header_config[2] = {0, 0};
//	uint8_t pipe_ctrl[] = {VL53L5CX_NB_TARGET_PER_ZONE, 0x00, 0x01, 0x00};
//	uint32_t single_range = 0x00;
//	uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00,
//			0x00, 0x00, 0x00, 0x0f,
//			0x00, 0x02, 0x00, 0x08};
//	uint16_t tmp;
//	MsgInfoToSend.context = this->__InitSequenceID;
//	MsgInfoToSend.pCB = std::bind(&VL53L5CX::SensorInit, this, std::placeholders::_1);
//	MsgInfoToSend.I2C_Addr = this->__address;
//	MsgInfoToSend.I2C_MemAddr = VL53L5CX_INIT_REGS[this->__InitSequenceID];
//	MsgInfoToSend.len = 1;
//	MsgInfoToSend.pTxData = &VL53L5CX_INIT_REG_VALUES[this->__InitSequenceID];
//	MsgInfoToSend.uCommInt.hi2c = this->__hi2c1;
//	//sink for rx
//	MsgInfoToSend.pRxData = &null_data_sink;
//	//Init sensor
//	if (this->__Status == TOF_INIT_NOT_DONE) {
//		//Basically wait for sensor boot
//		MsgInfoToSend.len = 0;
//		this->__waitInit(1);
//	} else if (this->__Status == TOF_STATE_INIT_ONGOING){
//		switch (this->__InitSequenceID) {
//		case 0 ... 3:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 4:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
//			break;
//		case 5 ... 14:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 15:
//			MsgInfoToSend.len = 0;
//			this->__waitInit(1);
//			break;
//		case 16 ... 17:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 18:
//			MsgInfoToSend.len = 0;
//			this->__waitInit(100);
//			break;
//		/* Wait for sensor booted (several ms required to get sensor ready ) */
//		case 19:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 20:
//			this->__CheckInitPollingMessage(1, 0, 0xff, 1, MsgInfo, &MsgInfoToSend);
//			break;
//		case 21 ... 24:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 25:
//			this->__CheckInitPollingMessage(1, 0, 0x10, 0x10, MsgInfo, &MsgInfoToSend);
//			break;
//		case 26:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 27:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
//			break;
//		case 28 ... 44:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 45:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
//			break;
//		case 46 ... 50:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 51:
//			//FW download
//			MsgInfoToSend.len = 0x8000;
//			MsgInfoToSend.pTxData = (uint8_t*)&VL53L5CX_FIRMWARE[0];
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 52:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 53:
//			//FW download #2
//			MsgInfoToSend.len = 0x8000;
//			MsgInfoToSend.pTxData = (uint8_t*)&VL53L5CX_FIRMWARE[0x8000];
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 54:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 55:
//			//FW download #3
//			MsgInfoToSend.len = 0x5000;
//			MsgInfoToSend.pTxData = (uint8_t*)&VL53L5CX_FIRMWARE[0x10000];
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 56 ... 59:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 60:
//			this->__CheckInitPollingMessage(1, 0, 0x10, 0x10, MsgInfo, &MsgInfoToSend);
//			break;
//		case 61:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 62:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
//			break;
//		case 63 ... 69:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 70:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
//			break;
//		case 71 ... 72:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 73:
//			if (__vl53l5cx_poll_for_mcu_boot(&MsgInfoToSend) != 0)
//			{
//				Error_Handler();
//			}
//			break;
//		case 74:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 75:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.len = sizeof(VL53L5CX_GET_NVM_CMD);
//			MsgInfoToSend.pTxData = (uint8_t*)VL53L5CX_GET_NVM_CMD;
//			break;
//		case 76:
//			this->__CheckInitPollingMessage(4, 0, 0xFF, 2, MsgInfo, &MsgInfoToSend);
//			break;
//		case 77:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
//			MsgInfoToSend.len = VL53L5CX_NVM_DATA_SIZE;
//			MsgInfoToSend.pRxData = this-> __comm_buffer;
//			break;
//		case 78:
//			//Send offset buffer;
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			(void)memcpy(this->__offset_buffer, this->__comm_buffer,
//				VL53L5CX_OFFSET_BUFFER_SIZE);
//			this->__vl53l5cx_send_offset_data(VL53L5CX_RESOLUTION_4X4);
//			MsgInfoToSend.len = VL53L5CX_OFFSET_BUFFER_SIZE;
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 79:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 80:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.len = VL53L5CX_XTALK_BUFFER_SIZE;
//			this->__vl53l5cx_send_xtalk_data(VL53L5CX_RESOLUTION_4X4);
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 81:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 82:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.len = sizeof(VL53L5CX_DEFAULT_CONFIGURATION);
//			MsgInfoToSend.pTxData = (uint8_t*)VL53L5CX_DEFAULT_CONFIGURATION;
//			break;
//		case 83:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 84:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.I2C_MemAddr = this->__vl53l5cx_dci_write_data((uint8_t*)&pipe_ctrl,
//					VL53L5CX_DCI_PIPE_CONTROL, (uint16_t)sizeof(pipe_ctrl));
//			MsgInfoToSend.len = (uint16_t)sizeof(pipe_ctrl) + 12;
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 85:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 86:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.I2C_MemAddr = this->__vl53l5cx_dci_write_data((uint8_t*)&single_range,
//					VL53L5CX_DCI_SINGLE_RANGE, (uint16_t)sizeof(single_range));
//			MsgInfoToSend.len = (uint16_t)sizeof(single_range) + 12;
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 87:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//			/* Init ends here, now send start ranging command and set frequency*/
//		case 88:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			this->__vl53l5cx_start_ranging(&MsgInfoToSend);
//			MsgInfoToSend.I2C_MemAddr = this->__vl53l5cx_dci_write_data((uint8_t*)&(output), VL53L5CX_DCI_OUTPUT_LIST, (uint16_t)sizeof(output));
//			MsgInfoToSend.len = (uint16_t)sizeof(output) + 12;
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 89:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 90:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.I2C_MemAddr = this->__vl53l5cx_dci_write_data((uint8_t*)&(header_config), VL53L5CX_DCI_OUTPUT_CONFIG, (uint16_t)sizeof(header_config));
//			MsgInfoToSend.len = (uint16_t)sizeof(header_config) + 12;
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 91:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 92:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.I2C_MemAddr = this->__vl53l5cx_dci_write_data((uint8_t*)&(output_bh_enable), VL53L5CX_DCI_OUTPUT_CONFIG, (uint16_t)sizeof(output_bh_enable));
//			MsgInfoToSend.len = (uint16_t)sizeof(output_bh_enable) + 12;
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 93:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 94 ... 96:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			break;
//		case 97:
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.len = sizeof(start_ranging_cmd);
//			MsgInfoToSend.pTxData = (uint8_t*)start_ranging_cmd;
//			break;
//		case 98:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 99:
//		/* Read ui range data content and compare if data size is the correct one */
//			/* Request data reading from FW */
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_TX;
//			MsgInfoToSend.len = sizeof(cmd);
//			this->__vl53l5cx_dci_read_data(cmd, 0x5440, 12);
//			(void)memcpy(this->__comm_buffer, cmd, sizeof(cmd));
//			MsgInfoToSend.pTxData = this->__comm_buffer;
//			break;
//		case 100:
//			this->__CheckInitPollingMessage(4, 1, 0xFF, 3, MsgInfo, &MsgInfoToSend);
//			break;
//		case 101:
//			/* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
//			MsgInfoToSend.eCommType = COMM_INT_I2C_MEM_RX;
//			MsgInfoToSend.len = 24;
//			MsgInfoToSend.pRxData = this->__comm_buffer;
//			break;
//		case 102:
//			SwapBuffer(this->__comm_buffer, 24);
//			/* Copy data from FW into input structure (-4 bytes to remove header) */
//			for(uint8_t i = 0 ; i < 24; i++){
//				this->__comm_buffer[i] = this->__comm_buffer[i + 4];
//			}
//			(void)memcpy(&tmp, &(this->__comm_buffer[0x8]), sizeof(tmp));
//			if(tmp != this->__data_count)
//			{
//				Error_Handler();
//			}
//			MsgInfoToSend.len = 0;
//			this->__Status = TOF_STATE_OK;
//			/* End Init sequence */
//			break;
//		default:
//			MsgInfoToSend.len = 0;
//		}
//		this->__InitSequenceID++;
//	}
//	if(MsgInfoToSend.len != 0) do{
//		ret = this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
//	}while(ret != HAL_OK);
//
//	return ret;
//}

uint16_t VL53L5CX::GetDataFromSensor(uint8_t x, uint8_t y)
{
	uint8_t index;
	switch (eOrientation) {
		case ROTATE_0:
				index = (y*4)+(3-x);
			break;
		case ROTATE_90:
				index = ((3-x)*4)+(3-y);
			break;
		case ROTATE_180:
				index = ((3-y)*4)+x;
			break;
		case ROTATE_270:
				index = (x*4)+y;
			break;
		default:
			index = (y*4)+x;
			break;
	}
	if(index < 16)
	{
		return this->result.distance_mm[index];
	}
	else{
		return 0xFFFF;
	}
}

uint8_t VL53L5CX::GetStatusFromSensor(uint8_t x, uint8_t y)
{
	uint8_t index = (y*4)+x;
	if(index < 16)
	{
		return this->result.target_status[index];
	}
	else{
		return 0xFF;
	}
}

HAL_StatusTypeDef VL53L5CX::DisableSensorComm(void)
{
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_RESET);
	return HAL_OK;
}

HAL_StatusTypeDef VL53L5CX::SetI2CAddress() {
	HAL_GPIO_WritePin(__ToFX_SHUT_Port[this->__sensor_index],
			__ToFX_SHUT_Pin[this->__sensor_index], GPIO_PIN_SET);
	uint8_t ret = vl53l5cx_set_i2c_address(&this->__sensor_conf,
			__ToFAddr[this->__sensor_index]);
	this->__address = __ToFAddr[this->__sensor_index];
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


void VL53L5CX::DataReceived(MessageInfoTypeDef<I2C_HandleTypeDef>* MsgInfo)
{
	SwapBuffer((uint8_t*)this->__comm_buffer, (uint16_t)32);
	for(uint8_t i = 0; i< 16; i++)
	{
		this->result.distance_mm[i] = (((uint16_t*)this->__comm_buffer)[i] < 0)? 0 : ((uint16_t*)this->__comm_buffer)[i]/4;
	}
	SwapBuffer(this->__comm_buffer + 32, (uint16_t)16);
	memcpy(this->result.target_status, this->__comm_buffer + 32, 16);
	this->__Status = TOF_STATE_OK;
	xEventGroupSetBitsFromISR(EventGroupHandle, (1<<this->__sensor_index), NULL);
}

HAL_StatusTypeDef VL53L5CX::GetRangingData(void) {

	HAL_StatusTypeDef ret = HAL_OK;
	MessageInfoTypeDef<I2C_HandleTypeDef> MsgInfoToSend = { 0 };
	MsgInfoToSend.context = __pos;
	MsgInfoToSend.eCommType = COMM_INT_MEM_RX;
	MsgInfoToSend.I2C_Addr = this->__address;
	MsgInfoToSend.I2C_MemAddr = 304;
	MsgInfoToSend.len = 32;
	MsgInfoToSend.pRxData = (uint8_t*)this->__comm_buffer;
	MsgInfoToSend.IntHandle = this->__hi2c1;
	ret = this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
	MsgInfoToSend.I2C_Addr = this->__address;
	MsgInfoToSend.I2C_MemAddr = 360;
	MsgInfoToSend.len = 16;
	MsgInfoToSend.pRxData = (uint8_t*)this->__comm_buffer + 32;
	MsgInfoToSend.IntHandle = this->__hi2c1;
	MsgInfoToSend.pCB = &this->_CallbackFunc;
	ret = this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
	this->__Status = TOF_STATE_DATA_RDY;

	return ret;
}

HAL_StatusTypeDef VL53L5CX::CheckDataReady(void) {
	return HAL_ERROR;
}

void VL53L5CX::__waitInit(uint32_t waitms) {
	this->__Status = TOF_STATE_INIT_WAIT;
	this->__wait_until_tick = pdMS_TO_TICKS(waitms) + xTaskGetTickCount();
}

ToF_Status_t VL53L5CX::CheckSensorStatus(void) {
	switch (this->__Status) {
	case TOF_STATE_INIT_WAIT:

		break;
	case TOF_STATE_OK:
	case TOF_STATE_DATA_RDY:
		if(HAL_GetTick() - last_update_tick > 100)//no update since 100ms;
		{
			if(GetRangingData() == HAL_ERROR)
			{
				this->__Status = TOF_STATE_ERROR;
			}
		}
		break;
	default:
		break;
	}
	return this->__Status;
}

HAL_StatusTypeDef VL53L5CX::__GetData(void)
{
	if(this->__Status == TOF_STATE_OK){
		this->__Status = TOF_STATE_DATA_RDY;
	}
	return HAL_OK;
}

HAL_StatusTypeDef VL53L5CX::__CheckInitPollingMessage(
		uint8_t					size,
		uint8_t					pos,
		uint8_t					mask,
		uint8_t					expected_value,
		MessageInfoTypeDef<I2C_HandleTypeDef>* 	MsgInfo,
		MessageInfoTypeDef<I2C_HandleTypeDef>*		MsgInfoToSend)
{
	//Received data
	if((this->__comm_buffer[pos] & mask) == expected_value){
		//The expected value
		MsgInfoToSend->len = 0;
		this->__waitInit(0); //to restart sequence
		return HAL_OK;
	}

	//keep polling
	MsgInfoToSend->pRxData = this->__comm_buffer;
	MsgInfoToSend->eCommType = COMM_INT_RX;
	this->__InitSequenceID--;
	//We don't want to actually callback this function, we can wait 10ms
	MsgInfoToSend->pCB = 0;
	this->__waitInit(10);
	return HAL_BUSY;
}


uint8_t VL53L5CX::__vl53l5cx_poll_for_mcu_boot(MessageInfoTypeDef<I2C_HandleTypeDef>* MsgInfoToSend)
{
	if((MsgInfoToSend != NULL) && (MsgInfoToSend->context == this->__InitSequenceID)){
		if((this->__comm_buffer[0] & (uint8_t)0x80) != (uint8_t)0){
			return this->__comm_buffer[1];
		}
		else if((this->__comm_buffer[0] & (uint8_t)0x1) != (uint8_t)0)
		{
			MsgInfoToSend->len = 0;
			this->__waitInit(0);
			return 0;
		}
	}
	MsgInfoToSend->pRxData = this->__comm_buffer;
	MsgInfoToSend->eCommType = COMM_INT_RX;
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


uint8_t VL53L5CX::__vl53l5cx_send_xtalk_data(uint8_t resolution)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	uint8_t res4x4[] = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07};
	uint8_t dss_4x4[] = {0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
	uint8_t profile_4x4[] = {0xA0, 0xFC, 0x01, 0x00};
	uint32_t signal_grid[64];
	int8_t i, j;

	(void)memcpy(this->__comm_buffer, (uint8_t*)VL53L5CX_DEFAULT_XTALK,
		VL53L5CX_XTALK_BUFFER_SIZE);

	/* Data extrapolation is required for 4X4 Xtalk */
	if(resolution == (uint8_t)VL53L5CX_RESOLUTION_4X4)
	{
		(void)memcpy(&(this->__comm_buffer[0x8]),
			res4x4, sizeof(res4x4));
		(void)memcpy(&(this->__comm_buffer[0x020]),
			dss_4x4, sizeof(dss_4x4));

		SwapBuffer(this->__comm_buffer, VL53L5CX_XTALK_BUFFER_SIZE);
		(void)memcpy(signal_grid, &(this->__comm_buffer[0x34]),
			sizeof(signal_grid));

		for (j = 0; j < (int8_t)4; j++)
		{
			for (i = 0; i < (int8_t)4 ; i++)
			{
				signal_grid[i+(4*j)] =
				(signal_grid[(2*i)+(16*j)+0]
				+ signal_grid[(2*i)+(16*j)+1]
				+ signal_grid[(2*i)+(16*j)+8]
				+ signal_grid[(2*i)+(16*j)+9])/(uint32_t)4;
			}
		}
	    (void)memset(&signal_grid[0x10], 0, (uint32_t)192);
	    (void)memcpy(&(this->__comm_buffer[0x34]),
                  signal_grid, sizeof(signal_grid));
	    SwapBuffer(this->__comm_buffer, VL53L5CX_XTALK_BUFFER_SIZE);
	    (void)memcpy(&(this->__comm_buffer[0x134]),
	    profile_4x4, sizeof(profile_4x4));
	    (void)memset(&(this->__comm_buffer[0x078]),0 ,
                         (uint32_t)4*sizeof(uint8_t));
	}

	return status;
}


uint16_t VL53L5CX::__vl53l5cx_dci_write_data(
		uint8_t				*data,
		uint32_t			index,
		uint16_t			data_size)
{
	int16_t i;

	uint8_t headers[] = { 0x00, 0x00, 0x00, 0x00 };
	uint8_t footer[] = { 0x00, 0x00, 0x00, 0x0f, 0x05, 0x01,
			(uint8_t) ((data_size + (uint16_t) 8) >> 8), (uint8_t) ((data_size
					+ (uint16_t) 8) & (uint8_t) 0xFF) };

	uint16_t address = (uint16_t) VL53L5CX_UI_CMD_END
			- (data_size + (uint16_t) 12) + (uint16_t) 1;

	headers[0] = (uint8_t) (index >> 8);
	headers[1] = (uint8_t) (index & (uint32_t) 0xff);
	headers[2] = (uint8_t) (((data_size & (uint16_t) 0xff0) >> 4));
	headers[3] = (uint8_t) ((data_size & (uint16_t) 0xf) << 4);

	/* Copy data from structure to FW format (+4 bytes to add header) */
	SwapBuffer(data, data_size);
	for (i = (int16_t) data_size - (int16_t) 1; i >= 0; i--) {
		this->__comm_buffer[i + 4] = data[i];
	}

	/* Add headers and footer */
	(void) memcpy(&this->__comm_buffer[0], headers, sizeof(headers));
	(void) memcpy(&this->__comm_buffer[data_size + (uint16_t) 4], footer,
			sizeof(footer));

	SwapBuffer(data, data_size);

	return address;
}

uint8_t VL53L5CX::__vl53l5cx_start_ranging(MessageInfoTypeDef<I2C_HandleTypeDef>* MsgInfoToSend)
{
	uint8_t resolution, status = VL53L5CX_STATUS_OK;
	uint32_t i;

	union Block_header *bh_ptr;

	resolution = VL53L5CX_RESOLUTION_4X4;
	this->__data_count = 0;

	/* Enable selected outputs in the 'platform.h' file */
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
	output_bh_enable[0] += (uint32_t)256;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
	output_bh_enable[0] += (uint32_t)1024;
#endif

	/* Update data size */
	for (i = 0; i < (uint32_t)(sizeof(output)/sizeof(uint32_t)); i++)
	{
		if ((output[i] == (uint8_t)0)
                    || ((output_bh_enable[i/(uint32_t)32]
                         &((uint32_t)1 << (i%(uint32_t)32))) == (uint32_t)0))
		{
			continue;
		}

		bh_ptr = (union Block_header *)&(output[i]);
		if (((uint8_t)bh_ptr->type >= (uint8_t)0x1)
                    && ((uint8_t)bh_ptr->type < (uint8_t)0x0d))
		{
			if ((bh_ptr->idx >= (uint16_t)0x54d0)
                            && (bh_ptr->idx < (uint16_t)(0x54d0 + 960)))
			{
				bh_ptr->size = resolution;
			}
			else
			{
				bh_ptr->size = (uint16_t)((uint16_t)resolution
                                  * (uint16_t)VL53L5CX_NB_TARGET_PER_ZONE);
			}
			this->__data_count += bh_ptr->type * bh_ptr->size;
		}
		else
		{
			this->__data_count += bh_ptr->size;
		}
		this->__data_count += (uint32_t)4;
	}
	this->__data_count += (uint32_t)24;

	header_config[0] = this->__data_count;
	header_config[1] = i + (uint32_t)1;

	(void)memcpy(this->__comm_buffer, output, sizeof(output));
	MsgInfoToSend->len = (uint16_t)sizeof(output);
	MsgInfoToSend->pTxData = this->__comm_buffer;


	return status;
}

uint8_t VL53L5CX::__vl53l5cx_dci_read_data(
		uint8_t 			*_cmd,
		uint32_t			index,
		uint16_t			data_size)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	_cmd[0] = (uint8_t)(index >> 8);
	_cmd[1] = (uint8_t)(index & (uint32_t)0xff);
	_cmd[2] = (uint8_t)((data_size & (uint16_t)0xff0) >> 4);
	_cmd[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);


	return status;
}

void VL53L5CX::SetMutex(osapi::Mutex *pmutex)
{
	PlatformSetMutex(pmutex);
}
