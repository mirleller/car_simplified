#include "Magn.h"
#include "UI.h"

#define adc_precise 1023  //计算精度  三种可选
                         //  ADC_12bit 4095
                        //  ADC_10bit 1023
                        //  ADC_8bit 255

float Pd=1.0;
float Dd=0.2;

float e_p=0.0;                        //期望的转速比，左轮比右轮
float e_p_pre=0.0;                    //上一时刻期望的两轮速度之比，左轮比右轮


extern int16   B_VL,B_VR;           //编码器读值

extern float v_aim;                //目标速度

float cha[7]={0.0,  0.015, 0.027,  0.098,  0.1176, 0.1569, 0.3922};
float fa[5]={0.0, 0.05, 0.1284, 0.2569, 0.3637};

float dia[][7][2]={{{1e10,7.5},{7.5,3.2},{3.2,1.9},{1.9,1.3},{1.3,1.0},{1.0,0.8}},//横排代表相同中间电感不同两边电感差值，竖排代表相同两边电感差值不同中间电感值
                  {{1e10,7.2},{7.0,3.2},{3.2,1.8},{1.8,1.2},{1.2,0.9},{0.9,0.7}},
                  {{1e10,6.5},{6.5,3.0},{3.0,1.7},{1.7,1.1},{1.1,0.8},{0.8,0.6}},
                  {{10.0,1.5},{1.5,1.2},{1.2,1.0},{1.0,0.5},{0.5,0.3},{0.3,0.1}}};
//                  {{0.6,0.5},{0.5,0.4},{0.4,0.3},{0.3,0.2},{0.2,0.1},{0.1,0.01}},
//                  {{0.6,0.5},{0.5,0.4},{0.4,0.3},{0.3,0.2},{0.2,0.1},{0.1,0.01}}};
//////////////////////////////////////
//计算出坐游轮应有的转速比（左轮比右轮）
void calcu_track(void)
{
  float   H_L=0,               //上一时刻adc通道采集到的值
          H_M=0,
          H_R=0,
          V_L=0,
          V_R=0;
  uint32  a_p=255;
  
  switch(ADC_precise)
  {
  case  ADC_8bit:
    a_p=255;
    break;
  case  ADC_10bit:
    a_p=1023;
    break;
  case  ADC_12bit:
    a_p=4095;
    break;
  case  ADC_16bit:
    a_p=65535;
    break;
  }
  
  //归一化处理，不论采样精度如何设置，都按宏定义adc_precise转化成相应的精度计算
  H_L=((float)ad_H_L)*adc_precise/a_p;
  H_M=((float)ad_H_M)*adc_precise/a_p;
  H_R=((float)ad_H_R)*adc_precise/a_p;
  V_L=((float)ad_V_L)*adc_precise/a_p;
  V_R=((float)ad_V_R)*adc_precise/a_p;
  
  //计算转速比
  float   e1=((float)(H_L-H_R))/adc_precise;   //差值
  float   e2=((float)(V_L-V_R))/adc_precise;
  float   e=(e1+e2)/2;
  e=e1;
  
  float   afa=(adc_precise-H_M)/adc_precise; //中间电感值
  int16   si=0;
  if(e>0)si=1;
  else if(e<0)si=-1;
  int8    i=0,
          j=0,
          k=0,
          l=0,
          m=-1,     //标记阀值
          n=-1;     //标记差值
  k=sizeof(fa)/4;
  l=sizeof(cha)/4;
  if(e<0)e=-e;
  for(i=0;i<k;i++)
  {
    if(afa>=fa[i]&&afa<fa[i+1])
    {
     m=i;
     break;
    }
  }
  for(j=0;j<l;j++)
  {
   if(e>=cha[j]&&e<cha[j+1])
   {
    n=j;
    break;
   }
  }
  UI_ShowInt(45,0,m,1);
  UI_ShowInt(45,1,n,1);
  if(m==-1||n==-1)
  {
    if(e_p_pre>0)e_p=0.4;
    else if(e_p_pre<0)e_p=-0.4;
  }
  else
  {
    e_p=(dia[m][n][1]-dia[m][n][0])*(((e-cha[n])/(cha[n+1]-cha[n]))*((afa-fa[m])/(fa[m+1]-fa[m])))+dia[m][n][0];
  }
  if(((float)(V_L+V_R)/2.0/adc_precise)>0.84)
    if(e_p<8.0)
      e_p=8.0;
  if(si==-1)e_p=-e_p;
  float t=e_p;
  e_p=Pd*e_p+Dd*(e_p-e_p_pre);
  e_p_pre =t;
}
