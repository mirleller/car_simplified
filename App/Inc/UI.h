#ifndef _UI_h_
#define _UI_h_
#include "common.h"

extern uint8 functionFlag;     //功能标记

typedef struct{
    char name[6];
    int * value;
}VarInt;

typedef struct{
    char name[6];
    float * value;
}VarDouble;

typedef enum{
    Win0,       //默认窗口Win0，空白窗体，用于动态显示数值
    Win1,       //人机界面
    Win2,       //修改Int型数值窗口
    Win3,       //修改Double型数值窗口
    Win4        //保存功能窗口
}WinNum;
extern void UI_init();                          //UI初始化，包括按键以及OLED初始化
extern void key_detect(void);                   //按键事件检测及按键事件处理
extern char UI_Add_Int(char* nam,int *val);      //向链表中添加整形数据,nam：变量名，val：变量值 添加成功返回1，否则返回0
extern char UI_Add_Double(char* nam,float *val); //向链表中添加Double数据,nam：变量名，val：变量值 添加成功返回1，否则返回0

//下列数据显示只在Wincow=win0时生效
extern void UI_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);   //UI显示字符串
extern void UI_ShowInt(unsigned char x0, unsigned char y0, int num, unsigned char TextSize);    //UI显示整形数
extern void UI_ShowDouble(unsigned char x, unsigned char y,double num, unsigned char TextSize);//UI显示Double类型数
#endif