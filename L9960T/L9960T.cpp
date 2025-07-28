/*
 * L9960T.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#include "L9960T.hpp"


L9960T::L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi, CommManager *CommunicationManager, uint32_t Channel, TIM_HandleTypeDef *htim, bool inverted_pwm, bool use_sw_pwm, bool linerize_change):
	MCInterface(side),
	__hspi{hspi},
	__CommunicationManager{CommunicationManager},
	_prev_context{0},
	__htim{htim},
	__Channel{Channel},
	__inverted_pwm{inverted_pwm},
	__use_sw_pwm{use_sw_pwm},
	__linerize_change(linerize_change),
	SCS_index{0},
	_StatusSemaphore{NULL}
{
	__InitMessageID = 0;
#ifdef MOTOR_INVERTED_SIDE
	if((side == MOTOR_LEFT) && ((__Instantiated_sides & (1 << side)) == 0))
#else
	if((side == MOTOR_RIGHT) && ((__Instantiated_sides & (1 << side)) == 0))
#endif
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_B_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_B_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_B_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_B_GPIO_Port;
		this->__CS_Pin = MD_CS_2_Pin;
		this->__CS_Port = MD_CS_2_GPIO_Port;
		this->__DIS_PORT = MD_DIS_2_GPIO_Port;
		this->__DIS_PIN = MD_DIS_2_Pin;
#ifdef LEFT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
#ifdef MOTOR_INVERTED_SIDE
	else if ((side == MOTOR_RIGHT) && ((__Instantiated_sides & (1 << side)) == 0))
#else
	if((side == MOTOR_LEFT) && ((__Instantiated_sides & (1 << side)) == 0))
#endif
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_A_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_A_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_A_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_A_GPIO_Port;
		this->__CS_Pin = MD_CS_1_Pin;
		this->__CS_Port = MD_CS_1_GPIO_Port;
		this->__DIS_PORT = MD_DIS_1_GPIO_Port;
		this->__DIS_PIN = MD_DIS_1_Pin;
#ifdef RIGHT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
	HAL_GPIO_WritePin(__CS_Port, __CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	this->__Instantiated_sides |= (1 << this->__side);
	this->__Instantiated_sides |=  ((1 << this->__side) << MOTOR_NDIS_OFFSET);
	HAL_GPIO_WritePin(this->__DIS_PORT, this->__DIS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(this->__IN1_PWM_PORT, this->__IN1_PWM_PIN, GPIO_PIN_RESET);

#ifdef USES_RTOS
	_StatusSemaphore = xSemaphoreCreateBinaryStatic(&_pxSemphrMemory);
	_CallbackFunc = std::bind(&L9960T::_ControllerStateCB, this, std::placeholders::_1);
#endif
}

void L9960T::Init(MessageInfoTypeDef<SPI>* MsgInfo)
{
	uint16_t Message, Comp_message;
	static uint16_t retry_count = 0;
	MessageInfoTypeDef<SPI> MsgInfoToSend = {0};
	MsgInfoToSend.GPIO_PIN = this->__CS_Pin;
	MsgInfoToSend.GPIOx = this->__CS_Port;
	MsgInfoToSend.context = (1 << this->__side) |
					  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) | //Tis a Init sequence!
			          (this->__InitMessageID << 8);
	MsgInfoToSend.eCommType = COMM_INT_TXRX;
	MsgInfoToSend.len = 2;
	MsgInfoToSend.pRxData = this->pRxData;
	MsgInfoToSend.IntHandle = this->__hspi;
#ifdef USES_RTOS
	MsgInfoToSend.pCB = &_CallbackFunc;
#else
	MsgInfoToSend.pCB = std::bind(&L9960T::Init, this, std::placeholders::_1);
#endif
	switch(this->__InitMessageID)
	{
		case 0://Reset SW
			Message = (RESET_TRIGGER_CONF_ADDR << ADDRESS_OFFSET) | (RESET_TRIGGER_CONF_SW_RESET << MESSAGE_OFFSET);
			Message |= (~__builtin_parity(Message) & 1);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			this->__InitMessageID++;
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			break;
		case 1://Check POR status
			Message = (CONFIGURATION_REQUEST_ADDR << ADDRESS_OFFSET) | (CONFIGURATION_REQUEST_CONF(5) << MESSAGE_OFFSET);
			Message |= (~__builtin_parity(Message) & 1);
			MsgInfoToSend.context = (1 << this->__side) |
							  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) |
					          (this->__InitMessageID << 8);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			retry_count++;
			if((this->pRxData[0] & POR_STATUS_MSK) == POR_STATUS_MSK)
			{
				retry_count = 0;
				this->__InitMessageID++;
			}
			else if(retry_count > 6)
			{
				retry_count = 0;
				this->__InitMessageID--;
			}
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			break;
		case 2://Trigger HWSC & BIST
			Message = (RESET_TRIGGER_CONF_ADDR << ADDRESS_OFFSET) | (RESET_TRIGGER_CONF_HWSC << MESSAGE_OFFSET);
			Message |= (~__builtin_parity(Message) & 1);
			MsgInfoToSend.context = (1 << this->__side) |
							  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) |
					          (this->__InitMessageID << 8);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			this->__InitMessageID++;
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			break;
		case 3://Check BIST status
			Message = (STATUS_REQUEST_ADDR << ADDRESS_OFFSET) | (STATUS_REQUEST_1 << MESSAGE_OFFSET);
			Message |= (~__builtin_parity(Message) & 1);
			MsgInfoToSend.context = (1 << this->__side) |
							  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) |
					          (this->__InitMessageID << 8);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			Comp_message = 0;
			Comp_message = (this->pRxData[0]<<8) | this->pRxData[1];
			if(Comp_message & HWSC_BIST_RUN_STATUS_MSK)
			{
				if((Comp_message & HWSC_BIST_PASS) == HWSC_BIST_PASS)
				{
					//PASS
					retry_count=0;
					this->__InitMessageID++;
				}
				else if((Comp_message & HWSC_BIST_PASS) == HWSC_BIST_RUN_STATUS_MSK)
				{
					//FAIL
					this->__InitMessageID = 0;
				}
				else if((Comp_message & HWSC_BIST_PASS) == HWSC_RUNNING)
				{
					//RUNNING
				}
				else if((Comp_message & HWSC_BIST_PASS) == HWSC_BIST_FAIL)
				{
					//GENERAL FAIL
					this->__InitMessageID = 0;
				}
			}
			else if(Comp_message ==0)
			{
				if(retry_count > 6){
					this->__InitMessageID = 0;
					retry_count = 0;
				}
				else{
					retry_count++;
				}
			}
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			break;
		case 4://Clear Communication Check bit and start timers
#ifndef USES_RTOS
			Message = (RESET_TRIGGER_CONF_ADDR << ADDRESS_OFFSET) | (0 << RESET_TRIGGER_CONF_CC_CONFIG_SHIFT);
#else
			Message = (RESET_TRIGGER_CONF_ADDR << ADDRESS_OFFSET) | (1 << RESET_TRIGGER_CONF_CC_CONFIG_SHIFT);
#endif
			Message |= (~__builtin_parity(Message) & 1);
			MsgInfoToSend.context = (1 << this->__side) |
							  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) |
							  (this->__InitMessageID << 8);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			this->__InitMessageID++;
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			break;
		case 5://Set current range
			Message = 	(CONFIGURATION_ADDR(1) << ADDRESS_OFFSET) 			|
						(0x3 << CONFIGURATION_CL_OFFSET) | (1 << ISR_OFFSET)|
						(1 << VSR_OFFSET) | (7 << TDIAG1_OFFSET) 			|
						(1 << MD_ONE_CONF1_OFFSET) | (1 << DIAG_CLR_OFFSET);
			Message |= (~__builtin_parity(Message) & 1);
			MsgInfoToSend.context = (1 << this->__side) |
							  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) |
							  (this->__InitMessageID << 8);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			this->StartPWM();
			this->__InitMessageID++;
			break;
		default:
			this->__InitMessageID++;
			//this ends init
			break;
	}

}

HAL_StatusTypeDef L9960T::AttachPWMTimerAndChannel(TIM_HandleTypeDef *htim, uint32_t Channel)
{
	assert_param(IS_TIM_CHANNELS(Channel));
	this->__htim = htim;
	this->__Channel = Channel;
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::SetMotorPower(float Power)
{
	if(__linerize_change)
	{
		int16_t space_diff = 0;
		uint16_t current_power_space = 0;
		uint16_t set_power_space = 0;
		MotorDirectionTypeDef current_dir = MOTOR_DIR_BACKWARD;
		MotorDirectionTypeDef expected_dir = (Power>0)? MOTOR_DIR_FORWARD :
					(Power == 0)? __motorDir : MOTOR_DIR_BACKWARD;
		float _power = (Power > 1)? 1:
				(Power < -1)? -1:
				(Power < 0)?  -Power: Power;
		uint16_t _PWM = _power * 999;
		//Prepare sequence
		SCS_index = 0; //
		current_power_space = (__motorDir == MOTOR_DIR_FORWARD)? 999 + __powerPWM : 999 - __powerPWM;
		set_power_space = (expected_dir == MOTOR_DIR_FORWARD)? 999 + _PWM : 999 - _PWM;
		int8_t sign = ((current_power_space - set_power_space) > 0)? 1 : -1;
		while(current_power_space != set_power_space)
		{
			space_diff = current_power_space - set_power_space;
			space_diff = (space_diff > 0)? space_diff : -space_diff;
			// move closer to set space
			if(space_diff < 150){
				current_power_space = set_power_space;
			}
			else{
				current_power_space = current_power_space - (sign * 100);
			}
			if(current_power_space > 1998){
				current_power_space = set_power_space;
			}
			// translate to scs
			if(current_power_space < 999){
				// Move backward
				current_dir = MOTOR_DIR_BACKWARD;
				_L9660_SCS[SCS_index].dir = current_dir;
				_L9660_SCS[SCS_index].power = 999 - current_power_space;
			}
			else if(current_power_space > 999){
				// Move forward
				current_dir = MOTOR_DIR_FORWARD;
				_L9660_SCS[SCS_index].dir = current_dir;
				_L9660_SCS[SCS_index].power = current_power_space - 999;
			}
			else if(current_power_space == 999){
				// PWM to 0, no dir change
				_L9660_SCS[SCS_index].dir = current_dir;
				_L9660_SCS[SCS_index].power = 0;
			}
			SCS_index++;
		}
		return HAL_OK;
	}
	else{
		return MCInterface::SetMotorPower(Power);
	}
}


HAL_StatusTypeDef L9960T::SetMotorPowerPWM(uint16_t PowerPWM)
{
	__powerPWM = PowerPWM;
	if(((1 << this->__side) << MOTOR_NDIS_OFFSET) & this->__Instantiated_sides)
	{
		return HAL_ERROR;
	}
	else if(PowerPWM < 1000)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, ((__inverted_pwm)?999-PowerPWM:PowerPWM));
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef L9960T::SetMotorDirection(MotorDirectionTypeDef Dir)
{

	GPIO_PinState Pin_STATE;
	__motorDir = Dir;
	if((Dir == MOTOR_DIR_FORWARD) || (Dir == MOTOR_DIR_BACKWARD))
	{
		Pin_STATE = (GPIO_PinState)(((GPIO_PinState)Dir ^ __Direction) & 0x01); //Do xor and take last bit
		HAL_GPIO_WritePin(__IN2_DIR_PORT, __IN2_DIR_PIN, Pin_STATE);
		return HAL_OK;
	}
	return HAL_ERROR;

}

HAL_StatusTypeDef L9960T::EmergencyStop(void)
{
	__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, 0);
	return this->Disable();
}

HAL_StatusTypeDef L9960T::Disable(void)
{
	SetMotorPowerPWM(0);
	this->__Instantiated_sides |=  ((1 << this->__side) << MOTOR_NDIS_OFFSET);
	HAL_GPIO_WritePin(this->__DIS_PORT, this->__DIS_PIN, GPIO_PIN_SET);
	HAL_TIM_PWM_Stop(__htim, __Channel);
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::Enable(void)
{
	if(((1 << this->__side) << MOTOR_NDIS_OFFSET) & this->__Instantiated_sides)
	{
		this->__Instantiated_sides &= ~ ((1 << this->__side) << MOTOR_NDIS_OFFSET);
		HAL_GPIO_WritePin(this->__DIS_PORT, this->__DIS_PIN, GPIO_PIN_RESET);
	}
	this->__delay_ms(1);
	this->StartPWM();//Wait 1ms to satisfy wait on dis condition (1us is enough but too much work)
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::CheckIfControllerInitializedOk(void)
{
	return (__InitMessageID >= 7)? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef L9960T::StartPWM(void)
{
	if(this->__use_sw_pwm)
	{
		HAL_TIM_Base_Start_IT(__htim);
		__htim->State = HAL_TIM_STATE_READY;
	}
	return HAL_TIM_PWM_Start_IT(__htim, __Channel);
}

void L9960T::SoftPWMCB_pulse()
{
	__IN1_PWM_PORT->BSRR = __IN1_PWM_PIN;
}

void L9960T::SoftPWMCB_period()
{
	if(__linerize_change & (SCS_index > 0))
	{
		SCS_index--;
		SetMotorPowerPWM(_L9660_SCS[SCS_index].power);
		SetMotorDirection(_L9660_SCS[SCS_index].dir);
	}
	__IN1_PWM_PORT->BSRR = (uint32_t)__IN1_PWM_PIN << 16;
}

#ifdef USES_RTOS
/*
 * 	Should be called periodically to ensure that the controller is behaving correctly
 */
HAL_StatusTypeDef L9960T::CheckControllerState(void)
{

	static uint8_t state = 0;
	uint16_t Message;
	HAL_StatusTypeDef ret;
	static HAL_StatusTypeDef transactionstatud = HAL_ERROR;
	if(xSemaphoreTake(this->_StatusSemaphore, ( TickType_t ) 10) == pdTRUE)
	{
		MessageInfoTypeDef<SPI> MsgInfoToSend = {0};

		//Send status request


		MsgInfoToSend.GPIO_PIN = this->__CS_Pin;
		MsgInfoToSend.GPIOx = this->__CS_Port;
		MsgInfoToSend.context = (STATUS_CHECK_CONTEXT << CONTEXT_OFFSET) | (state << 8);
		MsgInfoToSend.eCommType = COMM_INT_TXRX;
		MsgInfoToSend.len = 2;
		MsgInfoToSend.IntHandle = this->__hspi;
		MsgInfoToSend.pTxData = pTxData;
		MsgInfoToSend.TransactionStatus = &transactionstatud;
		MsgInfoToSend.pCB = &_CallbackFunc;
		MsgInfoToSend.pRxData = this->pRxData;


		switch(state)
		{
			case 0:
			{
				Message = (STATUS_REQUEST_ADDR << ADDRESS_OFFSET) | (STATUS_REQUEST_1 << MESSAGE_OFFSET);
			}
			break;

			case 1:
			{
				Message = (STATUS_REQUEST_ADDR << ADDRESS_OFFSET) | (STATUS_REQUEST_2 << MESSAGE_OFFSET);
			}
			break;

			case 2:
			{
				Message = (STATUS_REQUEST_ADDR << ADDRESS_OFFSET) | (STATUS_REQUEST_3 << MESSAGE_OFFSET);

			}
			break;
		}

		Message |= (~__builtin_parity(Message) & 1);
		this->pTxData[1] = (Message & 0xFF);
		this->pTxData[0] = ((Message >> 8) & 0xFF);
		ret = this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);

		if(ret == HAL_OK)
		{
			state++;
			if(state >=3) state = 0;
		}
	}
/*
 * Handle errors here, but remember that reading the faults clears them if bit @DIAG_CLR_OFFSET is set to 1
 */

	if(~_status_regs[0] & (1<<9))
	{
		// bridge disabled ?
		if(((1 << this->__side) << MOTOR_NDIS_OFFSET) & this->__Instantiated_sides)
		{
			//disabled correctly
		}
		else
		{
			//error
			this->Disable();
			this->Enable();
			this->SetMotorPowerPWM(__powerPWM);

		}

	}
	return HAL_OK; //dummy
}

void L9960T::_ControllerStateCB(MessageInfoTypeDef<SPI>* MsgInfo)
{
	xSemaphoreGiveFromISR(this->_StatusSemaphore, NULL);
	if(((MsgInfo->context & 0xFF) >> CONTEXT_OFFSET) == STATUS_CHECK_CONTEXT) //redundant check
	{
//		if((this->_status_regs[MsgInfo->context >> 8] >> ADDRESS_OFFSET) != STATUS_REQUEST_ADDR){
//			//this is different msg, can not be handled here
//			return;
//		}
	}
	//we can handle this message here
	switch((this->_prev_context >> CONTEXT_OFFSET) & 0xF)
	{
		case 0:
		{
			//Init started, continue
			this->Init(MsgInfo);
		}
		break;
		case INIT_SEQUENCE_CONTEXT:
		{
			//Init Continue
			this->Init(MsgInfo);
		}
		break;
		case STATUS_CHECK_CONTEXT:
		{
			if((this->_prev_context >> 8) > 2) return;
			this->_status_regs[this->_prev_context >> 8] = (this->pRxData[0]<<8) | this->pRxData[1];;
			//We can call check again here, but for now lets exit, parent class task will call that function periodically
		}
		break;
		default:
		//Context not handled here
			break;
	}
	this->_prev_context = MsgInfo->context;
}

#endif

void L9960T::__delay_us(uint32_t TimeUs)
{
	uint32_t actWaitCNT = 0;
	TimeUs = TimeUs + 1;
	if(TimeUs > 998)
	{
		return;  //wrong value
	}
	if (this->__htim->Instance->CNT + TimeUs > 999)
	{
		actWaitCNT = TimeUs - (999 - this->__htim->Instance->CNT);
	}
	else
	{
		actWaitCNT = this->__htim->Instance->CNT + TimeUs;
	}
	while(this->__htim->Instance->CNT != actWaitCNT)
	{}
}

void L9960T::__delay_ms(uint32_t TimeMs)
{
#ifdef USES_RTOS
//Use vTaskDelay when RTOS is in use
	vTaskDelay(TimeMs);
#else
	HAL_Delay(TimeMs)
#endif
}
