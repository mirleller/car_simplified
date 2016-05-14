#ifndef _UI_H_
#define _UI_H_
#include "common.h"

typedef struct{
    char name[6];
    int * value;
}VarInt;

typedef struct{
    char name[6];
    float * value;
}VarDouble;

typedef enum{
    Win0,
    Win1,
    Win2,
    Win3,
    Win4
}WinNum;
extern void key_detect(void);
extern char UI_Add_Int(char* nam,int *val);  //���������������,nam����������val������ֵ ��ӳɹ�����1�����򷵻�0
extern char UI_Add_Double(char* nam,float *val); 
extern void UI_init();
#endif