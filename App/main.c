#include "common.h"
#include "include.h"
#include "IMU_IIC.h"
#include "MMA8451.h"
#include "L3G4200.h"
#include "ANGEL.h"
#include "Scopeinterface.h"
#include <math.h>
#include "OLED_I2C.h"
#include "Filter_2.h"
#include "calculate_PWM.h"
#include "Magn.h"
#include "VCAN_key.h"
#include "UI.h"

//extern unsigned char BMP1[];

//#define gp 0.9
//#define ap (1-gp)

int  D_Z_ACC=10;
int  D_Z_GYR=3;

#define Ngyr  5    //���ٶȲ�������
#define Nacc  4    //���ٶȼƲ�������


extern int16 ACC_X, ACC_Y, ACC_Z;
extern int16 Gyro_X, Gyro_Y, Gyro_Z;
extern float Fore_Gyr;

extern float v_aim;                     //Ŀ���ٶ�

int16   B_VL=0,B_VR=0;          //��������ֵ

extern  int16 gy=0;
extern  float Gyr=0;
extern  int16 ac=0;
extern  float angle=0;
//extern  float an_ac=0;
extern float car_angle=0;

extern int d_;
extern uint8  _flag;

uint32  ad_H_L=0,               //adcͨ���ɼ�����ֵ��ƽ��ֵ
        ad_H_M=0,
        ad_H_R=0,
        ad_V_L=0,
        ad_V_R=0;

uint32  ad_H_L_pre=0,               //��һʱ��adcͨ���ɼ�����ֵ
        ad_H_M_pre=0,
        ad_H_R_pre=0,
        ad_V_L_pre=0,
        ad_V_R_pre=0;

int8 CH=0;                     //��־ADCת�����еĽ׶�

uint32 Timer=0;                 //���ڼ���

extern uint8 pit0_5ms=0;


void PIT0_IRQHandler(void);   //��ȡ���ٶȺͼ��ٶȵ��жϺ���
void myInit();                //���ֳ�ʼ��
void Mydelays(int n);         //��ʱ
void Stop_pwm(void);          //ֹͣƽ�⳵���ж�
void get_adc(void);
void angle_deal();
void angle_get();
void speed_get();             //��ȡ�������ķ�����
void get_D_Z_GYR(void);
/*!
 *  @brief      main����
 *  @since      v5.0

 *  @note       FTM PWM ����
 */
void main(void)
{ 
 //   uint8 i=0;
     DisableInterrupts ;
    PID_INIT();
    myInit();
     UI_Add_Int("D_Z_ACC",&D_Z_ACC);
     UI_Add_Int("Tu     ",&Tu);
     
     UI_Add_Double("P    ",&P);
     UI_Add_Double("I    ",&I);
     UI_Add_Double("D    ",&D);
     UI_Add_Double("PV   ",&PV);
     UI_Add_Double("Dv   ",&Dv);
     UI_Add_Double("v_aim",&v_aim);
     UI_Add_Double("Pe   ",&Pe);
     UI_Add_Double("De   ",&De);
     
     
     UI_ShowInt(25,6,(int16)((B_VR+B_VL)/2),1);
     
    while(1)
    {  
      /*
      OLED_ShowStr(0, 5, (unsigned char *)"B_VL",1);
      OLED_ShowInt(25,5,B_VL,1);
      OLED_ShowStr(0, 6, (unsigned char *)"B_VR",1);
      OLED_ShowInt(25,6,B_VR,1);*/
    #if 0
      
   
  #endif
  
    }
}
void myInit()
{
        //�����ʼ��
        FTM_PWM_init(FTM0, FTM_CH4,200*100,1000);      
	FTM_PWM_init(FTM0, FTM_CH5,200*100,1000);        

	FTM_PWM_init(FTM0, FTM_CH6,200*100,1000);       
	FTM_PWM_init(FTM0, FTM_CH7,200*100,1000);        
         
        FTM_QUAD_Init(FTM1);        
        FTM_QUAD_Init(FTM2);
        
        
        i2c_init(I2C1, 1200*1000);            //����I2Cͨ�� 1�����ò�����Ϊ  1200k   
        port_init_NoALT(PTB0, 19);//����
        port_init_NoALT(PTB1, 19);//���� 
        
        
        MMA8451_Init();                             //��ʼ�����ٶȼ�
        
        L3G4200_Init();                             //��ʼ��������
        /*
        OLED_Init();                                //��ʼ��OLED   
        OLED_Fill(0xFF);//ȫ������
        Mydelays(5);
        OLED_Fill(0x00);//ȫ��Ϩ��
        */
        
        UI_init();
        
        
        uart_init (UART4,115200);                     //��ʼ��uart4,���ò�����Ϊ9600
        
        
        //�ж�����
        NVIC_SetPriorityGrouping((uint32_t)5);        //�������ȼ��飬�߶�λ��ռ���ȼ����Ͷ�λ�����ȼ�
        
        
        NVIC_SetPriority(PORTB_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping(), (uint32_t)0, (uint32_t)0));//��A�˿ڵ��ж���Ϊ������ȼ�
        NVIC_SetPriority(ADC1_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping(), (uint32_t)1, (uint32_t)1));
        NVIC_SetPriority(PIT0_IRQn, NVIC_EncodePriority (NVIC_GetPriorityGrouping(), (uint32_t)2, (uint32_t)2));
        
        set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
        set_vector_handler(ADC1_VECTORn,get_adc);     //����ADC1���жϺ���Ϊget_adc
        set_vector_handler(PORTB_VECTORn ,Stop_pwm);    //�� Stop_pwm ������ӵ��ж�������
        
        

        port_init(PTB3,IRQ_RISING|PULLDOWN|ALT1);            //��B3�˿���ΪGPIO�ڣ�������ƽ�ж�
        port_init(PTB1,PULLDOWN|ALT1);             //��B1�˿���ΪGPIO�ڣ�
        port_init(PTA29,PULLDOWN|ALT1);            //��A29�˿���ΪGPIO�ڣ�
        port_init(PTA27,PULLDOWN|ALT1);            //��A27�˿���ΪGPIO�ڣ�
        port_init(PTA26,PULLDOWN|ALT1);            //��A26�˿���ΪGPIO�ڣ�
        port_init(PTA26,PULLUP|ALT1);
        gpio_init (PTA29, GPI,0);                  //��A26,27,29��B1��Ϊ����
        gpio_init (PTA27, GPI,0);
        gpio_init (PTA26, GPI,0);
        gpio_init (PTB1, GPI,0);
        gpio_init (PTC8, GPO,0);                  //C6��Ϊ���
        /*
        char i=0;
        while(i==0)
        {
          if(gpio_get(PTA27))
          {
            get_D_Z_GYR();
            i++;
          }
        }
        */
        //gpio_set (PTC8,1);
        
        adc_init(ADC1_SE10);                       //��ʼ��ADC�˿ڣ�PTB4,5,6,7,10
        adc_init(ADC1_SE11);
        adc_init(ADC1_SE12);
        adc_init(ADC1_SE13);
        adc_init(ADC1_SE14);
        
        pit_init_ms(PIT0, dt*1000);                      //��ʼ��PIT0����ʱʱ��Ϊ�� 1000ms

        
        enable_irq (PIT0_IRQn);                                 //ʹ��PIT0�ж�
        enable_irq (PORTB_IRQn);                                //ʹ��PORTA26�ж�
        enable_irq (ADC1_IRQn);
        EnableInterrupts ;                                    //ʹ��ȫ���ж�

}




void PIT0_IRQHandler(void)
{
   PIT_Flag_Clear(PIT0);
   
      speed_get();
   
   key_detect() ;

   if(CH==0)
   {
     adc_start_hv(ADC1_SE10, ADC_precise, ADC_8sample);  //����һ������ת����Ӳ��ƽ��
     CH++;
   }
   angle_get();
   angle_deal();
   
   
   
   calculate_PWM(car_angle,Gyr);
  //  OLED_ShowStr(0, 5, (unsigned char *)"B_VL",1);
  //  OLED_ShowInt(25,5,B_VL,1);
  //  OLED_ShowStr(0, 6, (unsigned char *)"B_VR",1);
   
     UI_ShowInt(25,6,(int16)((B_VR+B_VL)/2),1);
      
#if 1
   if(_flag&_flag_sh)
   {
     _flag&=(~_flag_sh);
    UI_ShowInt(0,0,ad_H_L,1);         //��OLED����ʾADCת��ͨ���Ķ���
    UI_ShowInt(0,1,ad_H_M,1);
    UI_ShowInt(0,2,ad_H_R,1);
    UI_ShowInt(0,3,ad_V_L,1);
    UI_ShowInt(0,4,ad_V_R,1);
   }
#endif
   
#if 1
   Outdata[0]=5*angle;
   Outdata[1]=5*car_angle;
   Outdata[2]=5*Gyr;
   Outdata[3]=5*Fore_Gyr;
   OutPut_Data(Outdata);
#endif
   Timer++;
   if(Timer==27027000)Timer=0;
   
   if(~(_flag&_flag_va))             //����ٶȸı�ı�־λ
     _flag|=_flag_va;
   
   while(CH!=0)
   {
     if(PIT_TFLG0==1)
       while(1);
   }
   
}







void angle_get()
{
  //��ȡ���ٶȵ�ֵ��ȡ��ֵ
  int16 gyr[Ngyr]={0};
  uint8 i=0,
        j=0;
  for(i=0;i<Ngyr;i++)
  {
    Get_gyr_X(gyr[i]);
  }
  for(j=Ngyr;j>1;j--)
  {
    for(i=0;i<j-1;i++)
    {
      if(gyr[i]>gyr[i+1])
      {
        int16 t=gyr[i];
        gyr[i]=gyr[i+1];
        gyr[i+1]=t;
      }
    }
  }
   gy=(gyr[Ngyr/2+(Ngyr%2)]+gyr[Ngyr/2+1])/2;
  
  //��ýǶ�
  int16 acc[Nacc]={0};
  for(i=0;i<Ngyr;i++)
  {
    Get_acc_Z(acc[i]);
  }
  for(j=Nacc;j>1;j--)
  {
    for(i=0;i<j-1;i++)
    {
      if(acc[i]>acc[i+1])
      {
        int16 t=acc[i];
        acc[i]=acc[i+1];
        acc[i+1]=t;
      }
    }
  }
   ac=(acc[Nacc/2+(Nacc%2)]+acc[Nacc/2+1])/2;
  angle=ac;
}
void angle_deal()
{
  
  //Gyr=(gy-D_Z_GYR)*0.07;  //һ�׵�ͨ�˲�
  Gyr=gy-D_Z_GYR;
  angle=angle-D_Z_ACC;
 
  car_angle=Filter(angle,Gyr);
 
}

void speed_get()
{
   
   B_VL=FTM_QUAD_get(FTM1);  B_VL=-B_VL;        //��ȡ�������ķ�����
   B_VR=FTM_QUAD_get(FTM2);
   FTM_QUAD_clean(FTM1);              //����Ĵ���
   FTM_QUAD_clean(FTM2);
}


#if 1
void get_adc(void)
{
  ADC_SC1_REG(ADC1_BASE_PTR, 0) &= ~ADC_SC1_COCO_MASK;          //����жϱ�־λ
  switch(CH)
  {
    case  1: ad_H_L = ADC_R_REG(ADC1_BASE_PTR, 0);//ADC1��a
            //ad_H_L_pre=Filter_adc(ad_H_L,ad_H_L_pre);
            adc_start_hv(ADC1_SE11, ADC_precise, ADC_8sample);  //����һ������ת����Ӳ��ƽ��   
            CH++;
            break;
    case  2: ad_V_L = ADC_R_REG(ADC1_BASE_PTR, 0);//ADC1��a
            //ad_V_L_pre=Filter_adc(ad_V_L,ad_V_L_pre);
            adc_start_hv(ADC1_SE12, ADC_precise, ADC_8sample);  //����һ������ת����Ӳ��ƽ��
            CH++;
            break;
    case  3: ad_V_R = ADC_R_REG(ADC1_BASE_PTR, 0);//ADC1��a
            //ad_V_R_pre=Filter_adc(ad_V_R,ad_V_R_pre);
            adc_start_hv(ADC1_SE13, ADC_precise, ADC_8sample);  //����һ������ת����Ӳ��ƽ��
            CH++;
            break;
    case  4:ad_H_R = ADC_R_REG(ADC1_BASE_PTR, 0);//ADC1��a 
            //ad_H_R_pre=Filter_adc(ad_H_R,ad_H_R_pre);
            adc_start_hv(ADC1_SE14, ADC_precise, ADC_8sample);//����һ������ת����Ӳ��ƽ��
            CH++; 
            break;
    case  5:ad_H_M = ADC_R_REG(ADC1_BASE_PTR, 0);//ADC1��a
            //ad_H_M_pre=Filter_adc(ad_H_M,ad_H_M_pre);
            CH=0;
            break;
  }
}
#endif


void Mydelays(int n){
  for(;n>0;n--)
    for(int i=0;i<5000;i++);
}


void Stop_pwm(void)
{
    uint8  n = 0;    //���ź�

    
    n = 3;
    if(PORTB_ISFR & (1 << n))           //PTB3�����ж�
    {
      
        PORTB_ISFR  = (1 << n);        //д1���жϱ�־λ
        if(gpio_get(PTA26)==1)
        {
          v_aim=0;
        }
        else
          if(gpio_get(PTA27)==1)
          {
            _flag|=_flag_sh;
          }
    }
}

void get_D_Z_GYR(void)
{
          gpio_set (PTC8,1);
          int i=0;
          int jl=100;
          int32 g=0;
          while(i++<jl)
          {
            int16 gyr[Ngyr]={0};
            uint8 i=0,
                  j=0;
            for(i=0;i<Ngyr;i++)
            {
              Get_gyr_X(gyr[i]);
            }
            for(j=Ngyr;j>1;j--)
            {
              for(i=0;i<j-1;i++)
              {
                if(gyr[i]>gyr[i+1])
                {
                  int16 t=gyr[i];
                  gyr[i]=gyr[i+1];
                  gyr[i+1]=t;
                }
              }
            }
            gy=(gyr[Ngyr/2+(Ngyr%2)]+gyr[Ngyr/2+1])/2;
            g+=gy;
          }
          D_Z_GYR=g/jl;
          gpio_set (PTC8,1); 
}