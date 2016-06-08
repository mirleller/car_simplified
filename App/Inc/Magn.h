#ifndef _MAGN_H_
#define _MAGN_H_

#include "common.h"
#include "include.h"

#define ADC_precise ADC_10bit  //ADC采样精度
/*
extern uint32   ad_H_L_pre,               //上一时刻adc通道采集到的值
                ad_H_M_pre,
                ad_H_R_pre,
                ad_V_L_pre,
                ad_V_R_pre;
*/


extern uint32   ad_H_L,               //上一时刻adc通道采集到的值
                ad_H_M,
                ad_H_R,
                ad_V_L,
                ad_V_R;


extern float e_p;                        //期望的两轮速度之比，左轮比右轮

extern float e_p_pre;                   //上一时刻期望的两轮速度之比，左轮比右轮

extern float as;
void calcu_track(void);

#endif