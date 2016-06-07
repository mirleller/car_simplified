#include "Magn.h"
#include "UI.h"
#include "calculate_PWM.h"
#include "math.h"

#define adc_precise 225  //���㾫��  ���ֿ�ѡ
                         //  ADC_12bit 4095
                        //  ADC_10bit 1023
                        //  ADC_8bit 255

float Pd=1.0;
#define Dd (1-Pd)
float Ke1=0.55;
#define Ke2 (1-Ke1)
float e_p=0.0;                        //������ת�ٱȣ����ֱ�����
float e_p_pre=0.0;                    //��һʱ�������������ٶ�֮�ȣ����ֱ�����

float as=1.0;                         //��Ӧ�����ų�ǿ��

extern int16   B_VL,B_VR;           //��������ֵ

extern float v_aim;                //Ŀ���ٶ�

int cv[10]={0,5,10,15,20,25,30,35,40,45};
float diav[][2]={{1.0,0.98},{0.98,0.94},{0.94,0.92},{0.92,0.87},{0.87,0.85},{0.85,0.83},{0.83,0.78},{0.78,0.72},{0.72,0.68}};

float cha[7]={0.0,  0.007, 0.027,  0.048,  0.976, 0.1469, 0.3922};
float fa[7]={0.0, 0.03, 0.054, 0.14, 0.1809, 0.2369, 0.3637};

float dia[][7][2]={{{1e10,9.5},{9.5,7.5},{7.5,6.5},{6.5,5.5},{5.5,4.0},{4.0,3.5}},//���Ŵ�����ͬ�м��в�ͬ���ߵ�в�ֵ�����Ŵ�����ͬ���ߵ�в�ֵ��ͬ�м���ֵ
                  {{10.0,8.0},{8.0,6.0},{6.0,5.0},{5.0,4.7},{4.7,3.2},{3.2,2.8}},
                  {{6.5,6.0},{6.0,5.0},{5.0,4.5},{4.5,3.9},{3.9,3.5},{3.5,2.6}},
                  {{5.1,4.6},{4.6,4.0},{4.0,3.6},{3.6,3.0},{3.0,2.4},{2.4,1.6}},
                  {{3.9,3.5},{3.5,2.8},{2.8,2.2},{2.2,1.7},{1.7,1.4},{1.4,1.2}},
                  {{1.6,1.3},{1.3,1.2},{1.2,1.15},{1.15,1.09},{1.09,1.04},{1.04,1.01}}};
//                  {{0.6,0.5},{0.5,0.4},{0.4,0.3},{0.3,0.2},{0.2,0.1},{0.1,0.01}},
//                  {{0.6,0.5},{0.5,0.4},{0.4,0.3},{0.3,0.2},{0.2,0.1},{0.1,0.01}}};
//////////////////////////////////////
//�����������Ӧ�е�ת�ٱȣ����ֱ����֣�
void calcu_track(void)
{
  float   H_L=0,               //��һʱ��adcͨ���ɼ�����ֵ
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
  
  //��һ���������۲�������������ã������궨��adc_preciseת������Ӧ�ľ��ȼ���
  H_L=((float)ad_H_L)*adc_precise/a_p;
  H_M=((float)ad_H_M)*adc_precise/a_p;
  H_R=((float)ad_H_R)*adc_precise/a_p;
  V_L=((float)ad_V_L)*adc_precise/a_p;
  V_R=((float)ad_V_R)*adc_precise/a_p;
  
  //����ת�ٱ�
  float   e1=((float)(H_L-H_R))/adc_precise;   //��ֵ
  float   e2=((float)(V_L-V_R))/adc_precise;
  float   e=e1*Ke1+e1*Ke2;
  //e=e1;
  float   afa=(adc_precise-H_M)/adc_precise; //�м���ֵ
  int16   si=0;
  int8    i=0,
          j=0,
          k=0,
          l=0,
          m=-1,     //��Ƿ�ֵ
          n=-1;     //��ǲ�ֵ
  k=sizeof(fa)/4;
  l=sizeof(cha)/4;
  if(e>0)si=1;
  else if(e<0)si=-1;
  if(e<0)e=-e;
  if(e==0)
    e_p=e_p_pre;
  else
  {
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
  //UI_ShowInt(45,0,m,1);
  //UI_ShowInt(45,1,n,1);
  if(m==-1||n==-1)
  {/*
    if(e_p_pre>0)e_p=0.4;
    else if(e_p_pre<0)e_p=-0.4;
    */
    e_p=e_p_pre;
  }
  else
  {
    e_p=(dia[m][n][1]-dia[m][n][0])*(((e-cha[n])/(cha[n+1]-cha[n]))*((afa-fa[m])/(fa[m+1]-fa[m])))+dia[m][n][0];
  }
  if(((float)(V_L+V_R)/2.0/adc_precise)>0.85)
    if(e_p<10.0)
      e_p=10.0;
  if(si==-1)e_p=-e_p;
  }
  /*
  uint8 mm=0;
  for(i=0;i<sizeof(cv)/2;i++)
  {
    if(abs(B_V)<cv[i+1]&&abs(B_V)>=cv[i])
    {
     mm=i;
     break;
    }
  }
  e_p*=(diav[mm][0]+(float)(abs(B_V)-cv[mm])/(cv[mm+1]-cv[mm])*(diav[mm][1]-diav[mm][0]));
  */
  float t=e_p;
  e_p=Pd*e_p+Dd*(e_p-e_p_pre);
  e_p_pre =t;
}
