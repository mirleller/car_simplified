#ifndef _calculate_PWM_h_
#define _calculate_PWM_h_
#define _flag_st  0x01      //ֱ����־λ
#define _flag_vi  0x02      //�ٶȻ��ֿ�ʼ��־λ
#define _flag_sh  0x04      //��ʾAD��ֵ��־λ
#define _flag_vc  0x08      //�жϴ���������־λ
#define _flag_ei  0x10      //ת�򻷻��ֱ�־λ
#define _flag_va  0x20      //�ı�Ŀ���ٶȱ�־λ0Ϊ�Ѹı䣬1Ϊû��

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