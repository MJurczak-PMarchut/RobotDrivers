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
        Thread(int priority, unsigned int stackSize, Joinable isJoinable, const char* name = "unnamed");
        
        /** Virtual destructor required to properly destroy derived class objects. */
        virtual ~Thread();
        
        /** Runs the thread.
        *  @retval true if the thread was started successfully, 
        *  @retval false if the thread was not started successfully, or the thread was already running
        */
        virtual bool run();
        
        /** Checks if the thread is running.
        *  @retval true if the thread is running
        *  @retval false if the thread is not running
        */
        virtual bool isRunning();
        /** Waits for the thread to finish executing, with a given timeout.
         *  @param timeout[in] number of milliseconds to wait for the thread to finish executing
         *  @retval true if the thread was successfully joined in the given time
         *  @retval false if the thread was not joined within the given time or the thread is not joinable at all
         */
        virtual bool join(unsigned int timeout);

        /** Checks, if the thread is joinable.
         *  @retval true if the thread is joinable
         *  @retval false if the thread is not joinable
         */
        virtual bool isJoinable();

        /** Suspends thread execution.
         *  @retval true if the thread was suspended successfully
         *  @retval false if the thread was not suspended for some reason
         */
        virtual bool suspend();        

        /** Resumes thread execution.
         *  @retval true if the thread was resumed successfully
         *  @retval false if the thread was not resumed for some reason
         */
        virtual bool resume();        

        /** Sets thread priority
         *  @param[in] priority new thread priority
         *  @retval true if the priority for the thread was set successfully
         *  @retval false if the priority for the thread was not set successfully for some reason
         */
        virtual bool setPriority(int priority);

        /** Gets the thread priority
         *  @return current thread priority
         */
        virtual int getPriority();
        
        /** Gets thread name
         *  @return name of the thread
         */
        virtual const char* getName();
    
    protected:
        
        /** Delays thread execution for a given time.
         *  @param time[in] number of milliseconds to delay thread execution
         */
        virtual void sleep(unsigned int time) {
            vTaskDelay(time/portTICK_PERIOD_MS);
        }
    private:
        static void threadFunction(void* argument);
        TaskHandle_t __taskHandle;
        int __priority;
        unsigned int __stackSize;
        Joinable __isJoinable;
        const char* __name;
        SemaphoreHandle_t  __xSemaphore;
};


#endif // OSAPI_THREAD_FREERTOS_H
