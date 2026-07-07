/*
 * CommInterface.hpp
 *
 *  Created on: Dec 26, 2022
 *      Author: Mateusz
 */
#ifndef COMMMANAGER_COMMINTERFACE_HPP_
#define COMMMANAGER_COMMINTERFACE_HPP_
#include "Configuration.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Makes a check-then-act sequence (e.g. "confirm peripheral idle, then configure
// and start a transfer") atomic against concurrent callers. Any Comm* interface's
// __CheckIfFreeAndSendRecv can be reached from a task, from the peripheral's own
// completion ISR (chaining the next queued message), and from other interrupts
// that push requests directly (e.g. a sensor's data-ready EXTI line) - without
// this, a task can be preempted between confirming the peripheral is idle and
// actually starting its transfer, and the preempting interrupt can start its own
// transfer on the same shared peripheral in the meantime, corrupting whichever
// transaction loses. RAII so every return path exits correctly.
//
// COMM_WAIT-style blocking HAL calls (up to a ~1000ms timeout) must not be made
// while this is held - that would mask every interrupt (including SysTick) for
// the whole wait. Call Release() first in that case; the destructor then becomes
// a no-op. COMM_INTERRUPT/COMM_DMA calls are quick and non-blocking, so leave the
// guard active until it goes out of scope to keep those fully protected.
class CommCriticalSection
{
public:
	CommCriticalSection() : _isISR(xPortIsInsideInterrupt() == pdTRUE), _savedStatus(0), _active(true)
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
	~CommCriticalSection()
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

template <class T>
class CommBaseClass
{
	public:
		CommBaseClass(T *hint, DMA_HandleTypeDef *hdmaRx, CommModeTypeDef CommMode, const char* CommName="");
		virtual HAL_StatusTypeDef PushMessageIntoQueue(MessageInfoTypeDef<T> *MsgInfo);
		HAL_StatusTypeDef CheckIfSameInstance(const T *pIntStruct);
		virtual HAL_StatusTypeDef MsgReceivedCB(T *hint);
		virtual HAL_StatusTypeDef MsgReceivedCB(T *hint, uint16_t len){return HAL_ERROR;};
		virtual HAL_StatusTypeDef MsgReceivedRxCB(T *hint){return HAL_ERROR;};

	protected:
		virtual HAL_StatusTypeDef __CheckIfInterfaceFree(MessageInfoTypeDef<T> *MsgInfo) = 0;
		virtual HAL_StatusTypeDef __CheckIfFreeAndSendRecv(MessageInfoTypeDef<T> *MsgInfo) = 0;
		virtual HAL_StatusTypeDef __CheckForNextCommRequestAndStart(MessageInfoTypeDef<T> *MsgInfo);
		HAL_StatusTypeDef __CheckAndSetCSPinsGeneric(MessageInfoTypeDef<T> *MsgInfo);

		static CriticalSectionState __EnterCriticalSection();
		static void __LeaveCriticalSection(CriticalSectionState &state);

		T *_Handle;
		DMA_HandleTypeDef *hdmaRx;
		CommModeTypeDef _commType;
		const char* name;
		QueueHandle_t _MsgQueue;

	private:
		uint8_t _xQueueStaticBuffer[MAX_MESSAGE_NO_IN_QUEUE*sizeof(MessageInfoTypeDef<T>)];
		StaticQueue_t  _xStQueue;
		uint16_t GPIO_PIN;
		GPIO_TypeDef *GPIOx;

};

#endif /* COMMMANAGER_COMMINTERFACE_HPP_ */
