#ifndef OSAPI_THREAD_FREERTOS_H
#define OSAPI_THREAD_FREERTOS_H

/** Thread interface implementation for FreeRTOS. */
class Thread : public ThreadInterface
{
    public:
    
        /** Thread constructor.
         *  @param[in] priority thread priority
         *  @param[in] stackSize thread stack size in bytes
         *  @param[in] isJoinable decides if the thread supports join operation or not
         *  @param[in] name optional thread name
         */
        Thread(int priority, unsigned int stackSize, Joinable isJoinable, const char* name = "unnamed") {
            // TODO
        }
        
        // TODO
};


#endif // OSAPI_THREAD_FREERTOS_H
