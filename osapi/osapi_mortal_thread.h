#ifndef OSAPI_MORTAL_THREAD_H
#define OSAPI_MORTAL_THREAD_H

class MortalThread : public Thread
{
  public:
    MortalThread(int priority, unsigned int stackSize, const char* name = "unnamed") : Thread(priority, stackSize, JOINABLE, name),
    killSignal{0}, InitCompleted{false}
    {
      ;
    }
    virtual ~MortalThread()
    {
      ;
    }

    bool run()
    {
    	InitCompleted = false;
    	return Thread::run();
    }

    /** Sends termination signal to the thread. */
    void kill() {
        this->killSignal = 1;
    }

    bool isInitCompleted(void)
    {
    	return this->InitCompleted;
    }

                
  protected:
        virtual void begin(void) {};

        virtual void loop(void) = 0;

        virtual void end(void) {};
  private:
      sig_atomic_t killSignal;
      bool InitCompleted;


    /** Implementation of the job method */
    virtual void job(void) {

      this->begin();
      InitCompleted = true;
      while(!this->killSignal) this->loop();

      this->end();
    }           
};
 
#endif // OSAPI_MORTAL_THREAD_H
