#include <math.h>
#include "common.h"
#include "include.h"
#include "math.h"
#include "Magn.h"
#include "PID.h"
#include "Scopeinterface.h"
#include "filter_2.h"
#include "OLED_I2C.h"
#include "calculate_PWM.h"
#include "UI.h"
//控制平衡的PID
extern uint32 Timer;                 //用于计数
extern float Fore_Gyr;


  float d=0;
  float da=0;

#define FTM0_precision  1000
#define pwm_s 25
float  P=1.80;
float  I=0.00008;       //
float  D=0.980;
PID_ angle_PID;
uint8 _flag=0;
float angle_last=0;
int32   A_=0;                       //存储平均值

#define ND 80                      //采样个数
#define GP 4                      //间隔
int16   D_A[ND];                 //存储采集到的数据
uint32  NI=0;                      //用来计数

//控制速度的PID
float   PV=30.0;
#define Pv  (P*PV)   
float   Iv=0.005;
float   Dv=0.09;
PID_  v_PID;


/*
#define NDV 60                      //采样个数
#define GPV 1                      //间隔
int16   D_B_V[NDV];                 //存储采集到的数据
uint32  NIV=0;                      //用来计数
*/

int16   B_V=0;                        //编码器方波数
float   B_V_=0;                       //存储平均值
//float v_aim=10.0*(dt/0.026);          //目标速度

float v_aim=19;          //目标速度
int8  v_flag=0;                       //目标速度改变时，从新开始积分

//控制差速的PID
float   Pe=3.8000;
float   Ie=0.0050;
float   De=1.20000;
float   e_I=0;
PID_  turn_PID;

extern int16   B_VL,B_VR;           //编码器读值

float   Kvl=0.00;     //0.0011
float   Kvr=0.00;
//int16   B_V_pre=0;                  //上一时刻编码器读数
//int16   e_B_V_pre=1;                  //上一时刻转速比


//float d_pre=0;
int Tu=100;                          //速度环周期
int Tt=10;                         //转向环周期

float E_d=0;                            //目标半径与实际半径的差值
  
float xx1=0,
      xx2=0;

float xxl=0,
      xxr=0;
  
float v_1=0;
float v_r=0;
float v_l=0;
//float d_=0.0;

//float Fore_d=0;
int32 d_r_pre=0,
      d_l_pre=0;

void cal_I(float angle);
void cal_I_V(int16 B_V);
void ifstand(float xx1,float xx2,float *da,float d,float Gyr);
void PID_INIT(void)
{
  //PID_init(&angle_PID,P,I,D);
  PID_init(&v_PID,Pv,Iv,Dv);
  PID_init(&turn_PID,Pe,Ie,De);
}

float I_xx(float *xxx,int tu,int b_V,float *xx)
{
  if((Timer%(tu/(uint16)(dt*1000)))==0)
  {
    if(Timer==0)
    {
      if(!(_flag&((unsigned)0x08)))
        _flag|=0x08;
      else
        *xxx+=b_V;
      *xx=*xxx/(tu/(dt*1000));
      *xxx=0;
    }
    else
    {
      *xxx+=b_V;
      *xx=*xxx/(tu/(dt*1000));
      *xxx=0;
    }
  }
  else
    *xxx+=b_V;
}

void calculate_PWM(float angle,float Gyr)
{
  
  
  //cal_I(angle);//算角度积分
  //cal_I_V(B_V);//计算平均速度
  
  
  
  int32 d_l=0,d_r=0;
  float e_B=0;
  float a_al=angle-angle_last;
  float ds=0;
  float v_v=0;
  float ep=0;
  
  B_V=(B_VL+B_VR)/2;
  
  I_xx(&xx1,Tu,B_V,&v_1);
  v_v=v_aim-v_1;
  //OLED_ShowInt(0,7,(int16)(v_v),1);
  I_xx(&xxl,Tt,B_VL,&v_l);
  I_xx(&xxr,Tt,B_VR,&v_r);
  
  /*
  if(Timer&(Tu/5)==0)
  {
    if(Timer==0)
    {
      if(!(_flag&((unsigned)0x08)))
        _flag|=0x08;
      else
        xx1+=(float)(v_aim-B_V);
    }
  }
  else if((Timer-1)&(Tu/5)==0)
  {
    xx1=0;
    xx1+=(float)(v_aim-B_V);
  }
  else
    xx1+=(float)(v_aim-B_V);
  */
  
  if(~(_flag&_flag_va))
    _flag&=~(_flag_vi);
  if(~(_flag&_flag_va))
    _flag&=~(_flag_ei);
  
  if((_flag&(unsigned)_flag_vi)==0)
  {
    if(((B_V-v_aim)<0.1*v_aim)&&((B_V-v_aim)>(-0.1*v_aim)))
    {
      _flag|=(unsigned)_flag_vi;
     xx2=0;
    }
  }
  if((_flag&(unsigned)_flag_vi)==1)
    xx2+=(float)(v_aim-B_V);
  
  v_PID.ek=xx1;
  //cal_I_V(B_V);
  //xx2=(float)(v_aim-B_V_)/dt*0.026;
  
  cal_I(angle);
  
  d=0;
  int b=abs(B_V);
  if(functionFlag&0x01)
  {
      float ga=0;
    //if(fabs(angle)<200.0)
      //ga=0.2*(D*fabs(a_al)+20e-3*a_al*a_al);
    //else
    //ga=D*fabs(a_al)+1e-3*a_al*a_al;
      ga=D*fabs(a_al);
    if(a_al>=0)
      d=P*angle+ga;
    else
      d=P*angle-ga;
    
  }
  
  angle_last=angle;
  
  
  if(!(_flag&((unsigned)_flag_st)))
  {
    if(angle>1000||angle<-1000||B_V>=500||B_V<=-500||B_VL>=500||B_VL<=-500||B_VR>=500||B_VR<=-500)//跌倒了赶紧站起来
    {
      _flag|=(unsigned)_flag_st;
      ds+=I*A_;
      ep=10000000.0;
    }
    else                                //正在跑
    {
      if(Timer%(Tu/5)==0)
      {
        ifstand(v_v,xx2,&da,d,a_al);
      }
      ds=da;
      calcu_track();
      ep=e_p;
    }
  }
  else if((_flag&((unsigned)_flag_st)))
  {
    if((angle<150)&&(angle>-150)&&(Gyr>-500)&&(Gyr<500))//如果站起来了就继续跑
    {
      _flag&=(unsigned)(~_flag_st);
      if(Timer%(Tu/5)==0)
      {
        ifstand(v_v,xx2,&da,d,a_al);
      }
      ds=da;
      calcu_track();
      ep=e_p;
    }
    else
    {
      ds+=I*A_;
      ep=10000000.0;
    }
  }
  
  if(functionFlag&0x04)
    d+=ds;
  
  
  //d_pre=d;
  
  
  ep=e_p;
  if(d>0)
    d+=pwm_s;
  else if(d<0)
    d-=pwm_s;
  
  if(!(functionFlag&0x02))
    ep=100000000.0;
  
  //d_l=d*(1-1/e_p);
  //d_r=d*(1+1/e_p);
 
  if(v_l==v_r)
  {
    if(ep>0)
      e_B=100000000.0;
    else if(e_p<0)
      e_B=-100000000.0;
  }
  
  else
  {
    if(((v_l+v_r)==0))
    {
      if(v_r>v_l)
        e_B=0.01;
      else
        e_B=-0.01;
    }
    else
      e_B=(float)(v_l+v_r)/(v_r-v_l);
  }
 
  if(abs(B_V)<2)
  {
    if(((1000.0/fabs(e_B))>abs(B_V)))
    {
      if(e_B>0)
        e_B=1000.0/abs(B_V);
      else if(e_B<0)
        e_B=-1000.0/abs(B_V);
    }
  }
  else
  {
    if(fabs(e_B)<0.01)
    {
      if(e_B>0)e_B=0.01;
      else if(e_B<0)e_B=-0.01;
    }
    if(((10.0/fabs(e_B))>abs(B_V)))
    {
      if(e_B>0)
        e_B=10.0/abs(B_V);
      else if(e_B<0)
        e_B=-10.0/abs(B_V);
    }
  }
  
    turn_PID.ek=1.0/ep-1.0/e_B;
  
  /*
  if((_flag&(unsigned)_flag_ei)==0)
  {
    if(e_p>0)
    {
      if((turn_PID.ek<(1+0.3)*e_p)&&(turn_PID.ek>(1-0.3)*e_p))
      {
        _flag|=(unsigned)_flag_ei;
      e_I=0;
      }
    }
    else
    {
      if((turn_PID.ek>(1+0.3)*e_p)&&(turn_PID.ek<(1-0.3)*e_p))
      {
        _flag|=(unsigned)_flag_ei;
      e_I=0;
      }
    }
  }
  if((_flag&(unsigned)_flag_ei)==1)
    e_I+=(float)(v_aim-B_V);
  */
  
  //e_I+=turn_PID.ek;
 
    E_d=(Pe*turn_PID.ek+De*(turn_PID.ek-turn_PID.ek_1))*(1.0+(float)abs(B_V)/70.0);
    /*
    float tt=0.0;
    //tt=1.0/ep;
  //if(fabs(E_d)>5.0)
    tt=(1.0/ep)+E_d;
    */
    
    float et=fabs(1.0/ep);
    float ss=1.0+(5.0*et*et*et+1.0*et*et+0.001*et);
    ss=1.0;
    if((d_r-d_r_pre)<-30&&(d_l-d_l_pre)<-30)
    {
      d_l=d*ss*(1.0-1.0/ep);
      d_r=d*ss*(1.0+1.0/ep);
    }
    else if(abs((d_r-d_r_pre)-(d_l-d_l_pre))<50)
    {
      d_l=d*ss*(1.0-1.0/ep);
      d_r=d*ss*(1.0+1.0/ep);
    }
    else if((d_r-d_r_pre)>(d_l-d_l_pre))
    {
      d_l=d*ss*(1.0-1.0/ep-0.2*E_d);
      d_r=d*ss*(1.0+1.0/ep+E_d);
    }
    else if((d_r-d_r_pre)<(d_l-d_l_pre))
    {
      d_l=d*ss*(1.0-1.0/ep-E_d);
      d_r=d*ss*(1.0+1.0/ep+0.2*E_d);
    }
  
  
  //d_l=d*(1.0-tt);
  //d_r=d*(1.0+tt);
  //ep=100000000.0;
  //d_l=d*(1.0-((1.0/ep)+0));
  //d_r=d*(1.0+((1.0/ep)+0));
  

  //d_l+=(-B_VL*dt/0.005)*Kvl*(200/Tu);
  //d_r+=(-B_VR*dt/0.005)*Kvr*(200/Tu);
  
  //E_d+=PID_calcu(turn_PID);
  
  
  v_PID.ek_2=v_PID.ek_1;
  v_PID.ek_1=v_PID.ek;
  
  turn_PID.ek_2=turn_PID.ek_1;
  turn_PID.ek_1=turn_PID.ek;
  
  
  //B_V_pre=B_V;
  //e_B_V_pre=1/e_B;
  
  //d_r=d;
  //d_l=d;
  /*
  if(d_r>0)d_r+=30;
  else  if(d_r<0)d_r-=30;
  */
  //d_r=300;
  //d_l=300;
  d_r_pre=d*(1.0+1.0/ep);
  d_l_pre=d*(1.0-1.0/ep);
  
  ///////////////////////////////////////////////////////////////////////
  if(d_r>(FTM0_precision-5))d_r=FTM0_precision-5;
  if(d_r<(-FTM0_precision+5))d_r=-(FTM0_precision)+5;
  if(d_l>(FTM0_precision-5))d_l=FTM0_precision-5;
  if(d_l<(-FTM0_precision+5))d_l=-(FTM0_precision)+5;
  
   if(d_r>=0)
  {
    FTM_PWM_Duty(FTM0, FTM_CH6, (uint32)FTM0_precision-d_r);
    FTM_PWM_Duty(FTM0, FTM_CH7, (uint32)FTM0_precision);
  }
  else
  {
    FTM_PWM_Duty(FTM0, FTM_CH6, (uint32)FTM0_precision);
    FTM_PWM_Duty(FTM0, FTM_CH7, (uint32)FTM0_precision+d_r);
  }

if(d_l>=0)
  {
    FTM_PWM_Duty(FTM0, FTM_CH4, (uint32)FTM0_precision);
    FTM_PWM_Duty(FTM0, FTM_CH5, (uint32)FTM0_precision-d_l);
  }
  else
  {
    FTM_PWM_Duty(FTM0, FTM_CH4, (uint32)FTM0_precision+d_l);
    FTM_PWM_Duty(FTM0, FTM_CH5, (uint32)FTM0_precision);
  }
}


////////////////////////////////////////////////////////////////////////////////////////////


void ifstand(float xx1,float xx2,float *da,float d,float Gyr)
{
  if(xx2<=0)
  {
    if(xx1>=0)
      *da=Pv*xx1+Pv*0.00*xx2;
    else
      *da=1.4*Pv*xx1+Pv*0.00*xx2;
  }
  else
  {
    if(xx1>=0)
      *da=1*Pv*xx1+Pv*0.00*xx2;
    else
      
      *da=1.4*Pv*xx1+Pv*0.00*xx2;
  }
  
  
  if(xx1>0)
  {
    //if(Gyr>0)
    //  *da+=Gyr*D/4;
    //if(Gyr<0)
      //*da-=(Gyr*D/4-25e-3*Gyr*Gyr)/3*2;
  }
  
  
  //else 
  /*
    if(xx1<0)
  {
    if(Gyr>0)
      *da-=Gyr*D/4+25e-3*Gyr*Gyr;
    //if(Gyr<0)
    //  *da+=Gyr*D/4;
  }
  */
  
  if((*da)>(800))
     *da=800;
  if((*da)<(-8800))
     *da=-800;

}



void cal_I(float angle)
{
  uint8 i=0;
  if(angle>200)angle-=200;
  else if(angle<-200)angle+=200;
  else angle=0;
  if(GP==0)
  {
    if(NI<ND)
    {
      D_A[NI]=angle;
    }
    else
    {
      D_A[NI-ND]=angle;
    }
	A_=0;
    for(i=0;i<ND;i++)
    {
      A_+=D_A[i];
    } 
  }
  else
  {
    if(((NI/(GP+1))<ND)&&((NI%(GP+1))==0))
    {  
      D_A[NI/(GP+1)]=angle;
	  A_=0;
      for(i=0;i<ND;i++)
      {
        A_+=D_A[i];
      } 
    }
    else
    {
      if(((NI/(GP+1)-ND)<ND)&&((NI%(GP+1))==0))
      {
        D_A[NI/(GP+1)-ND]=angle;
		B_V_=0;
        for(i=0;i<ND;i++)
        {
          A_+=D_A[i];
        } 
      }
    } 
  }
  NI++;
  if(NI==2*(GP+1)*ND)NI=ND*GP;
}

/*
void cal_I_V(int16 B_V)
{
  uint8 i=0;
  if(GPV==0)
  {
    if(NIV<NDV)
    {
      D_B_V[NIV]=B_V;
    }
    else
    {
      D_B_V[NIV-NDV]=B_V;
    }
	B_V_=0;
    for(i=0;i<NDV;i++)
    {
      B_V_+=D_B_V[i];
    } 
  }
  else
  {
    if(((NIV/(GPV+1))<NDV)&&((NIV%(GPV+1))==0))
    {  
      D_B_V[NIV/(GPV+1)]=B_V;
	  B_V_=0;
      for(i=0;i<NDV;i++)
      {
        B_V_+=D_B_V[i];
      } 
    }
    else
    {
      if(((NIV/(GPV+1)-NDV)<NDV)&&((NIV%(GPV+1))==0))
      {
        D_B_V[NIV/(GPV+1)-NDV]=B_V;
		B_V_=0;
        for(i=0;i<NDV;i++)
        {
          B_V_+=D_B_V[i];
        } 
      }
    } 
  }
  NIV++;
  if(NIV==2*(GPV+1)*NDV)NIV=NDV*GPV;
}
*/
/*
void ifstand(float xx1,float xx2,float *da,float d,float Gyr)
{
      if((xx2>=0&&xx1>0))              
      {
        if(Gyr>800||Gyr<-800)
          *da=Pv*xx1;
        else
          
   //       *da=Pv*xx1+0.19*xx2;
          
          *da=Pv*xx1;
      }
      else if((xx2<0&&xx1<0))
      {
        float tt=0;
        if(v_aim!=0)
          tt=Pv-(float)(-xx1*xx1*0.001+xx1)/v_aim*0.05*Pv;
        else
          tt=Pv-(float)(-xx1*xx1*0.001+xx1)*0.05*Pv;
        
   //     if(tt>(Pv*3))
   //       tt=Pv*3;
        
        
   //     *da=tt*xx1+0.001*xx2;
        
        *da=tt*xx1;
      }
      else if(xx2>0)
      {
        if(Gyr>0)
          *da+=Gyr*D/4;
        if(Gyr<0)
          *da-=Gyr*D/4;
      }
      else if(xx2<0)
      {
        if(Gyr>0)
          *da-=Gyr*D/4;
        if(Gyr<0)
          *da+=Gyr*D/4;
      }
      //if(xx1>0)
      
        char si=1,
            sia=1;
        float _da=*da;
        float _d=d;
        if((*da)<0){_da=-_da;sia=-1;}
        if(d<0){_d=-_d;si=-1;}
      
        //if(_da>3*_d)
          //*da=sia*3*_d;
        if((*da)>(800))
          *da=800;
        if((*da)<(-8800))
          *da=-800;
      
}
*/

