/*
 * L9960_regs.hpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */

#ifndef L9960T_L9960_REGS_HPP_
#define L9960T_L9960_REGS_HPP_

#define ADDRESS_OFFSET 12
#define MESSAGE_OFFSET 1
#define CONTEXT_OFFSET 2

#define SW_RESET_Msk (1 << 9)
#define HWSC_LBIST_Msk (1 << 8)
#define CC_CONFIG_Msk (1 < 7)

#define CL_RANGE_Shift (1 << 9)
#define NOSR_Msk (1 << 8)

#define RESET_TRIGGER_CONF_ADDR 2
#define CC_CONFIG (1<<7)

#define ELECTRONIC_ID_REQUEST_ADDR 15
#define ELECTRONIC_ID_REQUEST_MSG 0

#define CONFIGURATION_REQUEST_ADDR 7
#define CONFIGURATION_REQUEST_CONF(CONF_NO) (1<<(CONF_NO-1))

#define STATUS_REQUEST_ADDR 8
#define STATUS_REQUEST_1 0
#define STATUS_REQUEST_2 (1<<0)
#define STATUS_REQUEST_3 (1<<1)


#endif /* L9960T_L9960_REGS_HPP_ */
