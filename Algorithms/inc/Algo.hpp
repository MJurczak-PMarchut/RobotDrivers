/*
 * miniTomi.hpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */

#ifndef ALGORITHMS_INC_ALGO_HPP_
#define ALGORITHMS_INC_ALGO_HPP_
#ifdef ROBOT_MT_V1
#include "osapi.h"

class Algorithm : public MortalThread
{
public:
	Algorithm():MortalThread(tskIDLE_PRIORITY, 512){};
protected:
    virtual void begin(void){};
    virtual void loop(void){}
    virtual void end(void){}
private:
};


#endif
#endif /* ALGORITHMS_INC_ALGO_HPP_ */
