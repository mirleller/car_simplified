#ifndef _calculate_PWM_h_
#define _calculate_PWM_h_

//����ƽ���PID
extern float  P;
extern float  I;       //
extern float  D;
extern float   PV;
extern float   Dv;
extern float v_aim;          //Ŀ���ٶ�

//���Ʋ��ٵ�PID
extern float   Pe;
extern float   De;

extern int Tu;                          //�ٶȻ�����


extern void calculate_PWM(float Angle,float Gyr);
void PID_INIT(void);
#endif