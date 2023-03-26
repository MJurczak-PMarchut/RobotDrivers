/*
 * miniTomi.hpp
 *
 *  Created on: 19 gru 2022
 *      Author: Mateusz
 */

#ifndef ALGORITHMS_MINITOMI_HPP_
#define ALGORITHMS_MINITOMI_HPP_
#ifdef ROBOT_MT_V1
#include "osapi.h"

class Robot : public MortalThread
{
public:
	miniTomi();
protected:
    void begin(void);
    void loop(void);
    void end(void);
private:
};


#endif
#endif /* ALGORITHMS_MINITOMI_HPP_ */
