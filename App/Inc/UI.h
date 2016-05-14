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
extern char UI_Add_Int(char* nam,int *val);  //向链表中添加数据,nam：变量名，val：变量值 添加成功返回1，否则返回0
extern char UI_Add_Double(char* nam,float *val); 
extern void UI_init();
#endif