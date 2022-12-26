#ifndef OSAPI_RECURSIVE_MUTEX_FREERTOS_H
#define OSAPI_RECURSIVE_MUTEX_FREERTOS_H

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
    /** Locks the mutex. In case the mutex is already locked, it may cause the calling thread to block,
     *  waiting for the mutex to become unlocked, for the maximum given timeout.
     *  @param[in] timeout maximum number of milliseconds allowed to block the calling thread while waiting for mutex to become unlocked
     *  @retval true if the mutex was successfully locked (calling thread owns now this lock)
     *  @retval false if the mutex was not locked within the given time
     */
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
