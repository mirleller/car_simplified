#ifndef Scopeinterface_H
#define Scopeinterface_H
extern int16 Outdata[4];
void Scopeinterface_Init();        //串口上位机初始化
void OutPut_Data(int16 *OutData);  //串口向上位机发送数据

#endif