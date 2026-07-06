/*
 * CommSPI.cpp
 *
 *  Created on: 30 gru 2022
 *      Author: Paulina
 */

#include "CommSPI.hpp"

CommSPI::CommSPI(SPI_HandleTypeDef *hint, DMA_HandleTypeDef *hdma, CommModeTypeDef CommMode)
:CommBaseClass(hint, hdma, CommMode, "SPI")
{}

// Makes the free-check -> configure -> start-transfer sequence in
// __CheckIfFreeAndSendRecv atomic against concurrent callers (the "MC Status
// check" task, the IMU's EXTI interrupt, and the SPI completion ISR all reach
// this same code path). Without this, a task can be preempted between
// confirming the peripheral is idle and actually starting its transfer, and
// the preempting interrupt can start its own transfer on the same shared bus
// in the meantime - corrupting whichever transaction loses. RAII so every
// return path (including the existing early return) exits correctly.
// COMM_WAIT uses blocking HAL calls with a timeout of up to 1000ms - holding this
// critical section through one of those would mask every interrupt (including
// SysTick) for that whole duration, which is far worse than the race it prevents.
// Call Release() before making a COMM_WAIT call; the destructor then becomes a
// no-op. COMM_INTERRUPT/COMM_DMA calls are quick and non-blocking, so those stay
// protected all the way through by leaving the guard active until it goes out of scope.
class SPICriticalSection
{
public:
	SPICriticalSection() : _isISR(xPortIsInsideInterrupt() == pdTRUE), _savedStatus(0), _active(true)
	{
		if(_isISR)
		{
			_savedStatus = taskENTER_CRITICAL_FROM_ISR();
		}
		else
		{
			taskENTER_CRITICAL();
		}
	}
	~SPICriticalSection()
	{
		Release();
	}
	void Release()
	{
		if(!_active)
		{
			return;
		}
		_active = false;
		if(_isISR)
		{
			taskEXIT_CRITICAL_FROM_ISR(_savedStatus);
		}
		else
		{
			taskEXIT_CRITICAL();
		}
	}
private:
	bool _isISR;
	UBaseType_t _savedStatus;
	bool _active;
};

HAL_StatusTypeDef CommSPI::__CheckIfInterfaceFree(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret = HAL_BUSY;
	if(_Handle->State != HAL_SPI_STATE_READY)
	{
		return ret;
	}
	return HAL_OK;
}

HAL_StatusTypeDef CommSPI::__CheckIfFreeAndSendRecv(MessageInfoTypeDef<SPI_HandleTypeDef> *MsgInfo)
{
	HAL_StatusTypeDef ret =HAL_BUSY;
	SPICriticalSection guard;
	if(__CheckIfInterfaceFree(MsgInfo) != HAL_OK)
	{
		return ret;
	}
	// check if cpol change needed
	if(MsgInfo->spi_cpol_high != this->cpol_high)
	{
		//disable spi
		__HAL_SPI_DISABLE(MsgInfo->IntHandle);
		// chsnge cpol
		if(MsgInfo->spi_cpol_high)
			SET_BIT(MsgInfo->IntHandle->Instance->CFG2, SPI_POLARITY_HIGH);
		else
			CLEAR_BIT(MsgInfo->IntHandle->Instance->CFG2, SPI_POLARITY_HIGH);
		// enable spi
		__HAL_SPI_ENABLE(MsgInfo->IntHandle);
	}
	this->cpol_high = MsgInfo->spi_cpol_high;
	if(_commType != COMM_DUMMY)
	{
		// Abort any leftover transaction state before selecting the new target's CS -
		// otherwise any bus-level side effect from the abort lands on the device we
		// just selected instead of whatever (already-idle) state preceded it.
		HAL_SPI_Abort(MsgInfo->IntHandle);
		__CheckAndSetCSPinsGeneric(MsgInfo);
	}
	if(_commType == COMM_WAIT)
	{
		// Blocking call about to happen (up to 1000ms) - must not hold interrupts off for that.
		guard.Release();
	}
	switch(MsgInfo->eCommType)
	{
		case COMM_INT_RX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Receive_DMA(_Handle, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Receive_IT(_Handle, MsgInfo->pRxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Receive(_Handle, MsgInfo->pRxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		case COMM_INT_TX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Transmit_DMA(_Handle, MsgInfo->pTxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Transmit_IT(_Handle, MsgInfo->pTxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_Transmit(_Handle, MsgInfo->pTxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		case COMM_INT_TXRX:
		{
			if(_commType == COMM_DMA)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_TransmitReceive_DMA(_Handle, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
				__HAL_DMA_DISABLE_IT(hdmaRx, DMA_IT_HT);
			}
			else if(_commType == COMM_INTERRUPT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_TransmitReceive_IT(_Handle, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len);
			}
			else if(_commType == COMM_WAIT)
			{
				//Queue empty and peripheral ready, send message
				ret = HAL_SPI_TransmitReceive(_Handle, MsgInfo->pTxData, MsgInfo->pRxData, MsgInfo->len, 1000);
			}
			else
			{
				ret = HAL_ERROR;
			}
		}
		break;
		default:
			ret = HAL_ERROR;
			break;
	}
	return ret;
}

HAL_StatusTypeDef CommSPI::MsgReceivedCB(SPI_HandleTypeDef *hint)
{
	HAL_StatusTypeDef ret;
	// MessageInfoTypeDef<SPI_HandleTypeDef> MsgInfo ={0};
//	xQueuePeekFromISR(_MsgQueue, &MsgInfo);
	// check if cpol change needed
	ret = CommBaseClass::MsgReceivedCB(hint);
	return ret;
}



