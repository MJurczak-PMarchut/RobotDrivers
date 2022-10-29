/*
 * L9960T.cpp
 *
 *  Created on: Nov 13, 2021
 *      Author: Mateusz
 */

#include "L9960T.hpp"

#ifdef USES_RTOS
//Use vTaskDelay when RTOS is in use

void HAL_Delay(uint32_t Delay)
{
	vTaskDelay(Delay);
}
#endif

L9960T::L9960T(MotorSideTypeDef side, SPI_HandleTypeDef *hspi, CommManager *CommunicationManager, uint32_t Channel, TIM_HandleTypeDef *htim):
	__side{side},
	__hspi{hspi},
	__CommunicationManager{CommunicationManager},
	__htim{htim},
	__Channel{Channel}
{
	__InitMessageID = 0;
	if((side == MOTOR_LEFT) && ((__Instantiated_sides & (1 << MOTOR_LEFT)) == 0))
	{
		this->__IN1_PWM_PIN = MD_IN1_PWM_A_Pin;
		this->__IN2_DIR_PIN = MD_IN2_DIR_A_Pin;
		this->__IN1_PWM_PORT = MD_IN1_PWM_A_GPIO_Port;
		this->__IN2_DIR_PORT = MD_IN2_DIR_A_GPIO_Port;
		this->__Instantiated_sides |= (1 << MOTOR_LEFT);
		this->__Instantiated_sides |= MOTOR_LEFT_NDIS_ENABLED;
		this->__CS_Pin = MD_CS_1_Pin;
		this->__CS_Port = MD_CS_1_GPIO_Port;
		this->__DIS_PORT = MD_DIS_1_GPIO_Port;
		this->__DIS_PIN = MD_DIS_1_Pin;
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
		this->__Instantiated_sides |= MOTOR_RIGHT_NDIS_ENABLED;
		this->__CS_Pin = MD_CS_2_Pin;
		this->__CS_Port = MD_CS_2_GPIO_Port;
		this->__DIS_PORT = MD_DIS_2_GPIO_Port;
		this->__DIS_PIN = MD_DIS_2_Pin;
#ifdef RIGHT_MOTOR_INVERT_DIRECTION
		this->__Direction = GPIO_PIN_RESET;
#else
		this->__Direction = GPIO_PIN_SET;
#endif
	}
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	this->__Instantiated_sides |=  ((1 << this->__side) << MOTOR_NDIS_OFFSET);
	HAL_GPIO_WritePin(this->__DIS_PORT, this->__DIS_PIN, GPIO_PIN_SET);
}

void L9960T::Init(MessageInfoTypeDef* MsgInfo)
{
	uint16_t Message;
	MessageInfoTypeDef MsgInfoToSend = {0};
	MsgInfoToSend.GPIO_PIN = this->__CS_Pin;
	MsgInfoToSend.GPIOx = this->__CS_Port;
	MsgInfoToSend.context = (1 << this->__side) |
					  (INIT_SEQUENCE_CONTEXT << CONTEXT_OFFSET) | //Tis a Init sequence!
			          (this->__InitMessageID << 8);
	MsgInfoToSend.eCommType = COMM_INT_SPI_TXRX;
	MsgInfoToSend.len = 2;
	MsgInfoToSend.pRxData = this->pRxData;
	MsgInfoToSend.uCommInt.hspi = this->__hspi;
	MsgInfoToSend.pCB = std::bind(&L9960T::Init, this, std::placeholders::_1);
	switch(this->__InitMessageID)
	{
		case 0://Reset SW
			Message = (RESET_TRIGGER_CONF_ADDR << ADDRESS_OFFSET) | (RESET_TRIGGER_CONF_SW_RESET << MESSAGE_OFFSET);
			Message |= (~__builtin_parity(Message) & 1);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			this->__InitMessageID++;
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
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			if((this->pRxData[0] & POR_STATUS_MSK) == POR_STATUS_MSK)
			{
				this->__InitMessageID++;
			}
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
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			this->__InitMessageID++;
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
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			Message = 0;
			Message = (this->pRxData[0]<<8) | this->pRxData[1];
			if(Message & HWSC_BIST_RUN_STATUS_MSK)
			{
				if((Message & HWSC_BIST_PASS) == HWSC_BIST_PASS)
				{
					//PASS
					this->__InitMessageID++;
				}
				else if((Message & HWSC_BIST_PASS) == HWSC_BIST_RUN_STATUS_MSK)
				{
					//FAIL
					this->__InitMessageID++;
					this->__InitMessageID--;
				}
				else if((Message & HWSC_BIST_PASS) == HWSC_RUNNING)
				{
					//RUNNING
					this->__InitMessageID++;
					this->__InitMessageID--;
				}
				else if((Message & HWSC_BIST_PASS) == HWSC_BIST_FAIL)
				{
					//GENERAL FAIL
					this->__InitMessageID++;
					this->__InitMessageID--;
				}
			}
			break;
		case 4://Clear Communication Check bit and start timers
			Message = (RESET_TRIGGER_CONF_ADDR << ADDRESS_OFFSET) | (0 << RESET_TRIGGER_CONF_CC_CONFIG_SHIFT);
			Message |= (~__builtin_parity(Message) & 1);
			MsgInfoToSend.context = (1 << this->__side) |
							  (0 << CONTEXT_OFFSET) | //End Init
							  (this->__InitMessageID << 8);
			this->pTxData[1] = (Message & 0xFF);
			this->pTxData[0] = ((Message >> 8) & 0xFF);
			MsgInfoToSend.pTxData = pTxData;
			this->__CommunicationManager->PushCommRequestIntoQueue(&MsgInfoToSend);
			this->__InitMessageID++;
			this->StartPWM();
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

HAL_StatusTypeDef L9960T::SetMotorPowerPWM(uint16_t PowerPWM)
{
	if(((1 << this->__side) << MOTOR_NDIS_OFFSET) & this->__Instantiated_sides)
	{
		return HAL_ERROR;
	}
	else if(PowerPWM < 1000)
	{
		__HAL_TIM_SET_COMPARE(this->__htim, this->__Channel, PowerPWM);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef L9960T::SetMotorDirection(MotorDirectionTypeDef Dir)
{

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
	HAL_Delay(1); 	//Wait 1ms to satisfy wait on dis condition (1us is enough but too much work)
	return HAL_OK;
}

HAL_StatusTypeDef L9960T::CheckIfControllerInitializedOk(void)
{
	return (__InitMessageID == 5)? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef L9960T::StartPWM(void)
{
	return HAL_TIM_PWM_Start(__htim, __Channel);
}
