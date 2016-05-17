#ifndef _calculate_PWM_h_
#define _calculate_PWM_h_
#define _flag_st  0x01      //直立标志位
#define _flag_vi  0x02      //速度积分开始标志位
#define _flag_sh  0x04      //显示AD读值标志位
#define _flag_vc  0x08      //中断次数计数标志位
#define _flag_ei  0x10      //转向环积分标志位
#define _flag_va  0x20      //改变目标速度标志位0为已改变，1为没变

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