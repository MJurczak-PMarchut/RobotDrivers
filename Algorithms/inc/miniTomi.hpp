/*
 * miniTomi.hpp
 *
 *  Created on: 12 gru 2022
 *      Author: Mateusz
 */

#ifndef ALGORITHMS_INC_MINITOMI_HPP_
#define ALGORITHMS_INC_MINITOMI_HPP_
#ifdef ROBOT_MT_V1
#include "osapi_mortal_thread.h"
#include "L9960T.hpp"

class miniTomi : public MortalThread
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
#endif /* ALGORITHMS_INC_MINITOMI_HPP_ */
