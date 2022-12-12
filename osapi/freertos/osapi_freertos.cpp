
#include "FreeRTOS.h"
#include "task.h"
#include <csignal>
#include "osapi_thread_freertos.h"



unsigned int getSystemTime() {
    return xTaskGetTickCount();
}
Thread::Thread(int priority, unsigned int stackSize, Joinable isJoinable, const char* name):
    __taskHandle{0}, __isJoinable{isJoinable},  __name{name}, __stackSize{stackSize}, __priority{priority}
{
    if(isJoinable == JOINABLE){
        this->__xSemaphore = xSemaphoreCreateBinary();
    }
    else{
        this->__xSemaphore = NULL;
    }
    
}

bool Thread::run() {
    return (xTaskCreate(threadFunction, this->__name, this->__stackSize, this, this->__priority, &__taskHandle) == pdPASS)? true : false;
}

void Thread::threadFunction(void* argument) {
    Thread* osapiThreadObject = reinterpret_cast<Thread*>(argument);
    if (osapiThreadObject) {
        osapiThreadObject->job();   
        if(osapiThreadObject->isJoinable()){
           xSemaphoreGive(osapiThreadObject->__xSemaphore);
        }
    }
    vTaskDelete(NULL); // this is required in FreeRTOS to make sure that the thread does not just simply return
}

Thread::~Thread() 
{
    if(this->isJoinable()){
        vSemaphoreDelete(this->__xSemaphore);
    }
}
bool Thread::isRunning() 
{
    return (__taskHandle != 0)? true : false ;
}

bool Thread::join(unsigned int timeout) 
{
    if(this->isJoinable()){
        return (xSemaphoreTake(this->__xSemaphore, timeout) == pdTRUE)? true : false;
    }
    else{
        return false;
    }
}
bool Thread::isJoinable()
{
    return (this->__isJoinable == JOINABLE)? true : false;           
}

bool Thread::suspend() 
{
    vTaskSuspend(this->__taskHandle);
    return (eTaskGetState(this->__taskHandle) == eSuspended)? true : false;
} 

bool Thread::resume() 
{
    if(eTaskGetState(this->__taskHandle) == eSuspended){
        vTaskResume(this->__taskHandle);
        return true;
    }
    return false;
}  

bool Thread::setPriority(int priority) 
{
    this->__priority = priority;
    vTaskPrioritySet(this->__taskHandle, priority);
    return (this->getPriority() == priority)? true : false;           
}

int Thread::getPriority() 
{
    return uxTaskPriorityGet(__taskHandle);
}

const char* Thread::getName() 
{
    return __name;
}
