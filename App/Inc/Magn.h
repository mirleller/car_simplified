#ifndef _MAGN_H_
#define _MAGN_H_

#include "common.h"
#include "include.h"

#define ADC_precise ADC_10bit  //ADC��������
/*
extern uint32   ad_H_L_pre,               //��һʱ��adcͨ���ɼ�����ֵ
                ad_H_M_pre,
                ad_H_R_pre,
                ad_V_L_pre,
                ad_V_R_pre;
*/


extern uint32   ad_H_L,               //��һʱ��adcͨ���ɼ�����ֵ
                ad_H_M,
                ad_H_R,
                ad_V_L,
                ad_V_R;


extern float e_p;                        //�����������ٶ�֮�ȣ����ֱ�����

extern float e_p_pre;                   //��һʱ�������������ٶ�֮�ȣ����ֱ�����

extern float as;
void calcu_track(void);

#endif