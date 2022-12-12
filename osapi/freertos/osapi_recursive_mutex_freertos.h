#ifndef OSAPI_RECURSIVE_MUTEX_FREERTOS_H
#define OSAPI_RECURSIVE_MUTEX_FREERTOS_H

#include "osapi_mutex_interface.h"
#include "semphr.h"

class RecursiveMutex : public MutexInterface
{
public:

    RecursiveMutex(): __xMutex{NULL}
    {
        this->__xMutex = xSemaphoreCreateRecursiveMutex();
    }
    ~RecursiveMutex()
    {
        vSemaphoreDelete(this->__xMutex);
    }
    bool lock(unsigned int timeout)
    {
        if(this->__xMutex != NULL){
            return (xSemaphoreTakeRecursive(this->__xMutex, timeout/portTICK_PERIOD_MS) == pdTRUE)? true:false ;
        }
        return false;
    }
    /** Unlocks the mutex */
    void unlock()
    {
        xSemaphoreGiveRecursive(this->__xMutex);
    }
private:
    SemaphoreHandle_t  __xMutex;
};
#endif // OSAPI_RECURSIVE_MUTEX_FREERTOS_H
