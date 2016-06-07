#include "common.h"
#include "include.h"
#include "Magn.h"
#include "filter_2.h"
/*
#define A 1
#define B 0.18
#define Q 0.0001
#define R 0.3
#define H 1
*/
//#define Qg 0.1
//#define Rg 1000.0

//float Fore_Angle=0;
//float Fore_Gyr=0;

//float PPg=400.0;
//float Kgg=0.0;
// dt=0.016;          //加速度计陀螺仪数据采集时间间隔（见主函数中的设定）

float Filter(float angle_m,float gyro_m)
{
  /*
    Fore_Gyr=Fore_Gyr;
    PPg=PPg+Qg;
    Kgg=PPg/(PPg+Rg);
    Fore_Gyr=Fore_Gyr+Kgg*(gyro_m-Fore_Gyr);
    PPg=(1-Kgg)*PPg;
  */
  
    static double x=0.0;       //最优角度初值
    static double p=300.0;     //最优角度对应协方差初值
    static double Q=0.05;     //加速度计的协方差系数
    static double R=1000.0;         //
    static double k=0.0;
    double B=4.50;//4.70
    
    x=x+gyro_m*B*dt;
    p=p+Q;
    k=p/(p+R);
    x=x+k*(angle_m-x);
    p=(1-k)*p;
    return x;
}

/*
float Qa=1;
float Ra=100;

//float Fore_Adc=0;

float PPa=1;

float Filter_adc(float _adc,float _adc_pre)
{
  static char i=0;
  if(i==0)
  {
    i++;
    switch(ADC_precise)
    {
    case ADC_8bit://0x00  0~255 无信号时190
      Qa=0.1;
      Ra=0.5;
      break;
    case ADC_12bit://0x01 0~4095  无信号时3055
      Qa=0.1;
      Ra=9.0;
      break;
    case ADC_10bit://0x02 0~1023  无信号时765
      Qa=0.1;
      Ra=4.0;
      break;
    case ADC_16bit://0x03 0~65535 无信号时48800 不建议使用
      Qa=1.0;
      Ra=100.0;
      break;
    }
  }
  _adc_pre=_adc_pre;
  PPa=PPa+Qa;
  float Kggg=PPa/(PPa+Ra);
  _adc_pre=_adc_pre+Kggg*(_adc-_adc_pre);
  PPa=(1-Kggg)*PPa;
  
  return _adc_pre;
}
*/
