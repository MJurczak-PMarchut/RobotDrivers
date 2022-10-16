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

#define ELECTRONIC_ID_REQUEST_ADDR 15
#define ELECTRONIC_ID_REQUEST_MSG 0

#endif /* L9960T_L9960_REGS_HPP_ */
