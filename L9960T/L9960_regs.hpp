/*
 * L9960_regs.hpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */

#ifndef L9960T_L9960_REGS_HPP_
#define L9960T_L9960_REGS_HPP_

#define SW_RESET_Msk (1 << 9)
#define HWSC_LBIST_Msk (1 << 8)
#define CC_CONFIG_Msk (1 < 7)

#define CL_RANGE_Shift (1 << 9)
#define NOSR_Msk (1 << 8)

#define overcurrent_monitoring 0x001
#define restart_trigger 0x002
#define configuration_1 0x003
#define configuration_2 0x004
#define configuration_3 0x005
#define configuration_4 0x006
#define configuration_request_1 0x07a
#define configuration_request_2 0x07b
#define configuration_request_3 0x07c
#define configuration_request_4 0x07d
#define configuration_request_5 0x07e
#define states_request_1 0x08a
#define states_request_2 0x08b
#define states_request_3 0x08c
#define OFF_STATE_diagnosis 0x009
#define component_traceability_number_request_1 0x13a
#define component_traceability_number_request_2 0x13b
#define electronic_id_request 0x15a
#define silicon_version_request 0x15b
#define Logic_HW_version_request 0x15c

//restart trigger
#define SET_SWreset_bit1 rst_trig|=(1<<11);
#define SET_SWreset_bit0 rst_trig|=(1<<10);
#define SET_HWSC_LBIST_Trigger rst_trig|=(1<<9);
#define SET_ConfigCC rst_trig|=(1<<8);
#define RESET_SWreset_bit1 rst_trig&=~(1<<11);
#define RESET_SWreset_bit0 rst_trig&=~(1<<10);
#define RESET_HWSC_LBIST_Trigger rst_trig&=~(1<<9);
#define RESET_ConfigCC rst_trig&=~(1<<8);
//configuration 1
#define SET_CL_bit1 conf1|=(1<<11);
#define SET_CL_bit0 conf1|=(1<<10);
#define SET_NOSR conf1|=(1<<9);
#define SET_ISR conf1|=(1<<8);
#define SET_VSR conf1|=(1<<7);
#define SET_TDIAG1_bit2 conf1|=(1<<6);
#define SET_TDIAG1_bit1 conf1|=(1<<5);
#define SET_TDIAG1_bit0 conf1|=(1<<4);
#define SET_TSW_low_current conf1|=(1<<3);
#define SET_DIAG_CLR_EN conf1|=(1<<1);
#define RESET_CL_bit1 conf1&=~(1<<11);
#define RESET_CL_bit0 conf1&=~(1<<10);
#define RESET_NOSR conf1&=~(1<<9);
#define RESET_ISR conf1&=~(1<<8);
#define RESET_VSR conf1&=~(1<<7);
#define RESET_TDIAG1_bit2 conf1&=~(1<<6);
#define RESET_TDIAG1_bit1 conf1&=~(1<<5);
#define RESET_TDIAG1_bit0 conf1&=~(1<<4);
#define RESET_TSW_low_current conf1&=~(1<<3);
#define RESET_DIAG_CLR_EN conf1&=~(1<<1);
//configuration 2
#define SET_in1_in2_if conf2|=(1<<11);
#define SET_OTsd_thr_var_bit2 conf2|=(1<<10);
#define SET_OTsd_thr_var_bit1 conf2|=(1<<9);
#define SET_OTsd_thr_var_bit0 conf2|=(1<<8);
#define SET_OTwarn_thr_var_bit2 conf2|=(1<<7);
#define SET_OTwarn_thr_var_bit1 conf2|=(1<<6);
#define SET_OTwarn_thr_var_bit0 conf2|=(1<<5);
#define SET_UV_PROT_EN conf2|=(1<<4);
#define SET_NSPREAD conf2|=(1<<3);
#define SET_UV_WIN conf2|=(1<<1);
#define RESET_in1_in2_if conf2&=~(1<<11);
#define RESET_OTsd_thr_var_bit2 conf2&=~(1<<10);
#define RESET_OTsd_thr_var_bit1 conf2&=~(1<<9);
#define RESET_OTsd_thr_var_bit0 conf2&=~(1<<8);
#define RESET_OTwarn_thr_var_bit2 conf2&=~(1<<7);
#define RESET_OTwarn_thr_var_bit1 conf2&=~(1<<6);
#define RESET_OTwarn_thr_var_bit0 conf2&=~(1<<5);
#define RESET_UV_PROT_EN conf2&=~(1<<4);
#define RESET_NSPREAD conf2&=~(1<<3);
#define RESET_UV_WIN conf2&=~(1<<1);
//configuration 3
#define SET_WL_MODE conf3|=(1<<11);
#define SET_TVVL_bit3 conf3|=(1<<10);
#define SET_TVVL_bit2 conf3|=(1<<9);
#define SET_TVVL_bit1 conf3|=(1<<8);
#define SET_TVVL_bit0 conf3|=(1<<7);
#define SET_OTWARN_TSEC_EN conf3|=(1<<1);
#define RESET_WL_MODE conf3&=~(1<<11);
#define RESET_TVVL_bit3 conf3&=~(1<<10);
#define RESET_TVVL_bit2 conf3&=~(1<<9);
#define RESET_TVVL_bit1 conf3&=~(1<<8);
#define RESET_TVVL_bit0 conf3&=~(1<<7);
#define RESET_OTWARN_TSEC_EN conf3&=~(1<<1);
//configuration 4
#define SET_TDSR conf4|=(1<<11);
#define SET_OL_ON conf4|=(1<<10);
#define RESET_TDSR conf4&=~(1<<11);
#define RESET_OL_ON conf4&=~(1<<10);

#endif /* L9960T_L9960_REGS_HPP_ */
