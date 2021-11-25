/*
 * L9960T.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#include "L9960T.hpp"

uint16_t Compose16BitNumber(bool tab[]){
	uint16_t number = 0;
	for(int i = 0; i < sizeof(tab); i++){
		number += tab[i] * 2^i;
	}
	return number;
}

void Decompose16BitNumber(bool *tab, uint16_t number){
	for(int i = 0; i < 16; i++){
		if(number%2 == 1) tab[i] = true;
		else tab[i] = false;
		number = number/2;
	}
}


L9960T::L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi,  uint16_t CS_Pin, GPIO_TypeDef *CS_Port, CommManager *CommunicationManager)
{

	this->__CommunicationManager = CommunicationManager;
	this->__hspi = hspi;
	this->__side = side;
	this->__CS_Pin = CS_Pin;
	this->__CS_Port = CS_Port;

	if((side == MOTOR_LEFT) && ((__Instantiated_sides & (1 << MOTOR_LEFT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_A_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_A_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_A_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_A_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_LEFT);
#ifdef LEFT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
	else if ((side == MOTOR_RIGHT) && ((__Instantiated_sides & (1 << MOTOR_RIGHT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_B_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_B_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_B_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_B_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_RIGHT);
#ifdef RIGHT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
}

L9960T::L9960T(MotorSideTypeDef side)
{

	this->__CommunicationManager = 0;
	this->__hspi = 0;
	this->__side = side;
	this->__CS_Pin = 0;
	this->__CS_Port = NULL;

	if((side == MOTOR_LEFT) && ((__Instantiated_sides & (1 << MOTOR_LEFT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_A_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_A_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_A_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_A_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_LEFT);
#ifdef LEFT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
	else if ((side == MOTOR_RIGHT) && ((__Instantiated_sides & (1 << MOTOR_RIGHT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_B_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_B_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_B_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_B_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_RIGHT);
#ifdef RIGHT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
}

HAL_StatusTypeDef L9960T::AttachPWMTimerAndChannel(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	assert_param(IS_TIM_CHANNELS(Channel));
	this->__htim = htim;
	this->__Channel = Channel;
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::SetMotorPowerPWM(uint16_t PowerPWM)
{
	if(PowerPWM < 1000)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, PowerPWM);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef L9960T::SetMotorDirection(MotorDirectionTypeDef Dir)
{

	//TODO Check in what mode we are, and change direction
	//(Nah, in the future, probably best to focus on most basic configuration for now, 3 days till deadline)
	GPIO_PinState Pin_STATE;
	if((Dir == MOTOR_DIR_FORWARD) || (Dir == MOTOR_DIR_BACKWARD))
	{
		Pin_STATE = (GPIO_PinState)((Dir ^ __Direction) & 0x01); //Do xor and take last bit
		HAL_GPIO_WritePin(__IN2_DIR_PORT, __IN2_DIR_PIN, Pin_STATE);
		return HAL_OK;
	}
	return HAL_ERROR;

}

HAL_StatusTypeDef L9960T::EmergencyStop(void)
{
	__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, 0);
	this->Disable();
}

HAL_StatusTypeDef L9960T::Disable()
{
	//TODO Disable Motor
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::ComposeSPIMess(uint8_t command){
	bool tab[16] = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

	switch(command){

		case overcurrent_monitoring:
			tab[12] = true;

		case restart_trigger:
			tab[13] = true;
			tab[11] = this->RegisterWrite.SWreset_bit1;
			tab[10] = this->RegisterWrite.SWreset_bit0;
			tab[9] = this->RegisterWrite.HWSC_LBIST_Trigger;
			tab[8] = this->RegisterWrite.ConfigCC;

		case configuration_1:
			tab[13] = true;
			tab[12] = true;
			tab[11] = this->RegisterWrite.CL_bit1;
			tab[10] = this->RegisterWrite.CL_bit0;
			tab[9] = this->RegisterWrite.NOSR;
			tab[8] = this->RegisterWrite.ISR;
			tab[7] = this->RegisterWrite.VSR;
			tab[6] = this->RegisterWrite.TDIAG1_bit2;
			tab[5] = this->RegisterWrite.TDIAG1_bit1;
			tab[4] = this->RegisterWrite.TDIAG1_bit0;
			tab[3] = this->RegisterWrite.TSW_low_current;
			tab[2] = true;
			tab[1] = this->RegisterWrite.DIAG_CLR_EN;

		case configuration_2:
			tab[14] = true;
			tab[11] = this->RegisterWrite.in1_in2_if;
			tab[10] = this->RegisterWrite.OTsd_thr_var_bit2;
			tab[9] = this->RegisterWrite.OTsd_thr_var_bit1;
			tab[8] = this->RegisterWrite.OTsd_thr_var_bit0;
			tab[7] = this->RegisterWrite.OTwarn_thr_var_bit2;
			tab[6] = this->RegisterWrite.OTwarn_thr_var_bit1;
			tab[5] = this->RegisterWrite.OTwarn_thr_var_bit0;
			tab[4] = this->RegisterWrite.UV_PROT_EN;
			tab[3] = this->RegisterWrite.NSPREAD;
			tab[1] = this->RegisterWrite.UV_WIN;

		case configuration_3:
			tab[14] = true;
			tab[12] = true;
			tab[11] = this->RegisterWrite.WL_MODE;
			tab[10] = this->RegisterWrite.TVVL_bit3;
			tab[9] = this->RegisterWrite.TVVL_bit2;
			tab[8] = this->RegisterWrite.TVVL_bit1;
			tab[7] = this->RegisterWrite.TVVL_bit0;
			tab[1] = this->RegisterWrite.OTWARN_TSEC_EN;

		case configuration_4:
			tab[14] = true;
			tab[13] = true;
			tab[11] = this->RegisterWrite.TDSR;
			tab[10] = this->RegisterWrite.OL_ON;

		case configuration_request_1:
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[1] = true;
			tab[0] = true;

		case configuration_request_2:
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[2] = true;
			tab[0] = true;

		case configuration_request_3:
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[3] = true;
			tab[0] = true;

		case configuration_request_4:
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[4] = true;
			tab[0] = true;

		case configuration_request_5:
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[5] = true;
			tab[0] = true;

		case states_request_1:
			tab[15] = true;

		case states_request_2:
			tab[15] = true;
			tab[1] = true;
			tab[0] = true;

		case states_request_3:
			tab[15] = true;
			tab[2] = true;
			tab[0] = true;

		case OFF_STATE_diagnosis:
			tab[15] = true;
			tab[12] = true;
			tab[1] = true;

		case component_traceability_number_request_1:
			tab[15] = true;
			tab[14] = true;
			tab[12] = true;

		case component_traceability_number_request_2:
			tab[15] = true;
			tab[14] = true;
			tab[12] = true;
			tab[1] = true;
			tab[0] = true;

		case electronic_id_request:
			tab[15] = true;
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[0] = true;

		case silicon_version_request:
			tab[15] = true;
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[1] = true;

		case Logic_HW_version_request:
			tab[15] = true;
			tab[14] = true;
			tab[13] = true;
			tab[12] = true;
			tab[2] = true;
	}
	this->LastCommand = command;
	this->SPI_TX = Compose16BitNumber(tab);
}

HAL_StatusTypeDef L9960T::AnalizeSPIMess(){

	bool tab[16];
	uint8_t address;
	Decompose16BitNumber(tab, this->SPI_RX);

	address = tab[15]*8 + tab[14]*4 + tab[13]*2 + tab[12]*1;

	switch(address){

		case 1:
			this->RegisterRead.OCH1_bit1 = tab[10];
			this->RegisterRead.OCH1_bit0 = tab[9];
			this->RegisterRead.OCH0_bit1 = tab[7];
			this->RegisterRead.OCH0_bit0 = tab[6];
			this->RegisterRead.OCL1_bit1 = tab[4];
			this->RegisterRead.OCL1_bit0 = tab[3];
			this->RegisterRead.OCL0_bit1 = tab[1];
			this->RegisterRead.OCL0_bit0 = tab[0];
		case 7:
			switch(this->LastCommand){
				case configuration_request_1:
					this->RegisterRead.CL_echo_bit1 = tab[11];
					this->RegisterRead.CL_echo_bit0 = tab[10];
					this->RegisterRead.NOSR_echo = tab[9];
					this->RegisterRead.ISR_echo = tab[8];
					this->RegisterRead.VSR_echo = tab[7];
					this->RegisterRead.TDIAG1_echo_bit2 = tab[6];
					this->RegisterRead.TDIAG1_echo_bit1 = tab[5];
					this->RegisterRead.TDIAG1_echo_bit0 = tab[4];
					this->RegisterRead.TSW_low_current_echo = tab[3];
					this->RegisterRead.DIAG_CLR_EN = tab[1];

				case configuration_request_2:
					this->RegisterRead.in1_in2_if_echo = tab[11];
					this->RegisterRead.in1_in2_if_latch = tab[10];
					this->RegisterRead.OT_sd_thr_var_echo_bit2 = tab[9];
					this->RegisterRead.OT_sd_thr_var_echo_bit1 = tab[8];
					this->RegisterRead.OT_sd_thr_var_echo_bit0 = tab[7];
					this->RegisterRead.OTwarn_thr_var_echo_bit2 = tab[6];
					this->RegisterRead.OTwarn_thr_var_echo_bit1 = tab[5];
					this->RegisterRead.OTwarn_thr_var_echo_bit0 = tab[4];
					this->RegisterRead.UV_PROT_EN_echo = tab[3];
					this->RegisterRead.NSPREAD_echo = tab[2];
					this->RegisterRead.UV_WIN_echo = tab[0];

				case configuration_request_3:
					this->RegisterRead.WLMODE_echo = tab[11];
					this->RegisterRead.TVVL_echo_bit3 = tab[10];
					this->RegisterRead.TVVL_echo_bit2 = tab[9];
					this->RegisterRead.TVVL_echo_bit1 = tab[8];
					this->RegisterRead.TVVL_echo_bit0 = tab[7];
					this->RegisterRead.OTWARN_TSEC_EN_echo = tab[0];

				case configuration_request_4:
					this->RegisterRead.TDSR_echo = tab[11];

				case configuration_request_5:
					this->RegisterRead.POR_status= tab[11];
					this->RegisterRead.config_CC_status_echo = tab[10];
					this->RegisterRead.CC_latch_state = tab[9];
			}
		case 8:
			switch(this->LastCommand){
				case states_request_1:
					this->RegisterRead.NDIS_status = tab[11];
					this->RegisterRead.DIS_status = tab[10];
					this->RegisterRead.BRIDGE_EN = tab[9];
					this->RegisterRead.HWSC_LBIST_status_bit2 = tab[8];
					this->RegisterRead.HWSC_LBIST_status_bit1 = tab[7];
					this->RegisterRead.HWSC_LBIST_status_bit0 = tab[6];
					this->RegisterRead.VPS_UV_REG = tab[5];
					this->RegisterRead.NGFAIL = tab[4];
					this->RegisterRead.ILIM_REG = tab[3];
					this->RegisterRead.VDO_OV_REG = tab[2];
					this->RegisterRead.VDO_UV_REG = tab[1];
					this->RegisterRead.VPS_UV = tab[0];

				case states_request_2:
					this->RegisterRead.OTSDcnt_bit5 = tab[11];
					this->RegisterRead.OTSDcnt_bit4 = tab[10];
					this->RegisterRead.OTSDcnt_bit3 = tab[9];
					this->RegisterRead.OTSDcnt_bit2 = tab[8];
					this->RegisterRead.OTSDcnt_bit1 = tab[7];
					this->RegisterRead.OTSDcnt_bit0 = tab[6];
					this->RegisterRead.OTWARN = tab[5];
					this->RegisterRead.OTWARN_REG = tab[4];
					this->RegisterRead.NOTSD = tab[3];
					this->RegisterRead.NOTSD_REG = tab[2];
					this->RegisterRead.OL_ON_STATUS_bit1 = tab[1];
					this->RegisterRead.OL_ON_STATUS_bit0 = tab[0];

				case states_request_3:
					this->RegisterRead.UV_CNT_REACHED = tab[5];
					this->RegisterRead.Error_count_bit3 = tab[4];
					this->RegisterRead.Error_count_bit2 = tab[3];
					this->RegisterRead.Error_count_bit1 = tab[2];
					this->RegisterRead.Error_count_bit0 = tab[1];
			}
		case 9:
			this->RegisterRead.DIAG_OFF_bit2 = tab[2];
			this->RegisterRead.DIAG_OFF_bit1 = tab[1];
			this->RegisterRead.DIAG_OFF_bit0 = tab[0];

		case 13:
			switch(this->LastCommand){
				case component_traceability_number_request_1:
					this->RegisterRead.I[11] = tab[11];
					this->RegisterRead.I[10] = tab[10];
					this->RegisterRead.I[9] = tab[9];
					this->RegisterRead.I[8] = tab[8];
					this->RegisterRead.I[7] = tab[7];
					this->RegisterRead.I[6] = tab[6];
					this->RegisterRead.I[5] = tab[5];
					this->RegisterRead.I[4] = tab[4];
					this->RegisterRead.I[3] = tab[3];
					this->RegisterRead.I[2] = tab[2];
					this->RegisterRead.I[1] = tab[1];
					this->RegisterRead.I[0] = tab[0];

				case component_traceability_number_request_2:
					this->RegisterRead.I[23] = tab[23];
					this->RegisterRead.I[22] = tab[22];
					this->RegisterRead.I[21] = tab[21];
					this->RegisterRead.I[20] = tab[20];
					this->RegisterRead.I[19] = tab[19];
					this->RegisterRead.I[18] = tab[18];
					this->RegisterRead.I[17] = tab[17];
					this->RegisterRead.I[16] = tab[16];
					this->RegisterRead.I[15] = tab[15];
					this->RegisterRead.I[14] = tab[14];
					this->RegisterRead.I[13] = tab[13];
					this->RegisterRead.I[12] = tab[12];
			}
		case 15:
			switch(this->LastCommand){
				case electronic_id_request:
					this->RegisterRead.ASIC_name[9] = tab[11];
					this->RegisterRead.ASIC_name[8] = tab[10];
					this->RegisterRead.ASIC_name[7] = tab[9];
					this->RegisterRead.ASIC_name[6] = tab[8];
					this->RegisterRead.ASIC_name[5] = tab[7];
					this->RegisterRead.ASIC_name[4] = tab[6];
					this->RegisterRead.ASIC_name[3] = tab[5];
					this->RegisterRead.ASIC_name[2] = tab[4];
					this->RegisterRead.ASIC_name[1] = tab[3];
					this->RegisterRead.ASIC_name[0] = tab[2];
					this->RegisterRead.ASSP = tab[0];

				case silicon_version_request:
					this->RegisterRead.Silicon_version[3] = tab[9];
					this->RegisterRead.Silicon_version[2] = tab[8];
					this->RegisterRead.Silicon_version[1] = tab[7];
					this->RegisterRead.Silicon_version[0] = tab[6];

				case Logic_HW_version_request:
					this->RegisterRead.code_version[7] = tab[7];
					this->RegisterRead.code_version[6] = tab[6];
					this->RegisterRead.code_version[5] = tab[5];
					this->RegisterRead.code_version[4] = tab[4];
					this->RegisterRead.code_version[3] = tab[3];
					this->RegisterRead.code_version[2] = tab[2];
					this->RegisterRead.code_version[1] = tab[1];
					this->RegisterRead.code_version[0] = tab[0];
			}
	}

}

