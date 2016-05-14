#ifndef _calculate_PWM_h_
#define _calculate_PWM_h_

//控制平衡的PID
extern float  P;
extern float  I;       //
extern float  D;
extern float   PV;
extern float   Dv;
extern float v_aim;          //目标速度

//控制差速的PID
extern float   Pe;
extern float   De;

extern int Tu;                          //速度环周期


extern void calculate_PWM(float Angle,float Gyr);
void PID_INIT(void);
#endif