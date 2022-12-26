#ifndef OSAPI_MUTEX_FREERTOS_H
#define OSAPI_MUTEX_FREERTOS_H


class Mutex : public MutexInterface
{
public:

    Mutex(): __xMutex{NULL}
    {
        this->__xMutex = xSemaphoreCreateMutex();
    }
    ~Mutex()
    {
        vSemaphoreDelete(this->__xMutex);
    }
    bool lock(unsigned int timeout)
    {
        if(this->__xMutex != NULL){
            return (xSemaphoreTake(this->__xMutex, timeout/portTICK_PERIOD_MS) == pdTRUE)? true:false ;
        }
        return false;
    }
    /** Unlocks the mutex */
    void unlock()
    {
        xSemaphoreGive(this->__xMutex);
    }
private:
    SemaphoreHandle_t  __xMutex;
        
};
#endif // OSAPI_MUTEX_FREERTOS_H
