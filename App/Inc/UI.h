#ifndef _UI_h_
#define _UI_h_
#include "common.h"

extern uint8 functionFlag;     //���ܱ��

typedef struct{
    char name[6];
    int * value;
}VarInt;

typedef struct{
    char name[6];
    float * value;
}VarDouble;

typedef enum{
    Win0,       //Ĭ�ϴ���Win0���հ״��壬���ڶ�̬��ʾ��ֵ
    Win1,       //�˻�����
    Win2,       //�޸�Int����ֵ����
    Win3,       //�޸�Double����ֵ����
    Win4        //���湦�ܴ���
}WinNum;
extern void UI_init();                          //UI��ʼ�������������Լ�OLED��ʼ��
extern void key_detect(void);                   //�����¼���⼰�����¼�����
extern char UI_Add_Int(char* nam,int *val);      //�������������������,nam����������val������ֵ ��ӳɹ�����1�����򷵻�0
extern char UI_Add_Double(char* nam,float *val); //�����������Double����,nam����������val������ֵ ��ӳɹ�����1�����򷵻�0

//����������ʾֻ��Wincow=win0ʱ��Ч
extern void UI_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);   //UI��ʾ�ַ���
extern void UI_ShowInt(unsigned char x0, unsigned char y0, int num, unsigned char TextSize);    //UI��ʾ������
extern void UI_ShowDouble(unsigned char x, unsigned char y,double num, unsigned char TextSize);//UI��ʾDouble������
#endif