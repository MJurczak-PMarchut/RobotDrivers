#ifndef OSAPI_MORTAL_THREAD_H
#define OSAPI_MORTAL_THREAD_H

#include "osapi.h"
using namespace osapi;

class MortalThread : public Thread
{
  public:
    MortalThread(int priority, unsigned int stackSize, const char* name = "unnamed") : Thread(priority, stackSize, JOINABLE, name),
    killSignal{0}        {
      ;
    }
    virtual ~MortalThread() {
      ;
    }                   
                        
    /** Sends termination signal to the thread. */
    void kill() {
        this->killSignal = 1;
    }
                
  protected:
        virtual void begin(void)
        {
        }
        virtual void loop(void) = 0;
        virtual void end(void)
        {
        }
  private:
      sig_atomic_t killSignal;
    /** Implementation of the job method */
    virtual void job(void) {
      this->begin();
        while(!this->killSignal) this->loop();
       this->end();
    }           
};
 
#endif // OSAPI_MORTAL_THREAD_H
