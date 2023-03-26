/*
 * StandardTomi.hpp
 *
 *  Created on: 26 mar 2023
 *      Author: Mateusz
 */

#ifndef ALGORITHMS_INC_STANDARDTOMI_HPP_
#define ALGORITHMS_INC_STANDARDTOMI_HPP_

#ifdef ROBOT_STD_V1
#include "osapi.h"

class Robot : public MortalThread
{
public:
	Robot();
protected:
    void begin(void);
    void loop(void);
    void end(void);
private:
};


#endif



#endif /* ALGORITHMS_INC_STANDARDTOMI_HPP_ */
