#include "UI.h"
#include "common.h"
#include "VCAN_key.h"
#include "OLED_I2C.h"
#define MaxLength 20    //宏定义链表的最大长度
#define Igap  1         //每次按下按键，整形数增加的数值
#define Dgap  0.1
#define Irate 5         //每次按下按键，整形数增加的比率
#define Drate 5

uint8 functionFlag=0x07;     //功能标记

VarInt ListInt[MaxLength];              //存放Int型的链表
VarDouble  ListDouble[MaxLength];       //存放Double型的链表

int8 IntLength=0;                       //记录Int数组元素个数
int8 DbLength=0;                        //记录Double数组元素个数

int8 pInt=0;                            //整形数组指针
int8 pDb=0;                             //Double数组指针

WinNum now_window=Win0;                 //默认窗口Win0      

void show_form(uint8 window_num);       //显示当前应显示的窗口
int8 UI_Add_Int(char* nam,int *val);    //向Int链表中添加数据，nam：变量名，字符串类型，val：整形指针，添加成功返回1，否则返回0
int8 UI_Add_Double(char* nam,float *val);//向Double链表中添加数据,nam：变量名，字符串类型，val：double型指针，添加成功返回1，否则返回0
void saveFlash();        //保存成功动画
void deal_key(void);                    //按键事件处理

void UI_init()          //UI初始化
{
   OLED_Init();         //初始化OLED   
   OLED_Fill(0x00);     //全屏熄灭
   key_init(KEY_MAX);   //初始化按键
}

void key_detect()                //按键检测，若有按键按下则进行按键事件处理，否则直接退出
{
  show_form(now_window);
  uint8 times=2;                 
  while(times--)                //循环检测八次按键
    key_IRQHandler();
  uint8 flag=Is_empty();        //获取按键状态
  if(flag==1) 
    deal_key();
}

int8 UI_Add_Int(char* nam,int *val)     //添加整型数
{
    if(IntLength==MaxLength) return 0;
    int8 i=0;
    while(nam[i]!='\0'){
        ListInt[IntLength].name[i]=nam[i];
        i++;
    }
    ListInt[IntLength].name[i]='\0';
    ListInt[IntLength++].value=val;
    return 1;
}
int8 UI_Add_Double(char* nam,float *val)       //添加Double型数
{
    if(DbLength==MaxLength) return 0;
    int8 i=0;
    while(nam[i]!='\0'){
        ListDouble[DbLength].name[i]=nam[i];
        i++;
    }
    ListDouble[DbLength].name[i]='\0';
    ListDouble[DbLength++].value=val;
    return 1;
}
void UI_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
    if(now_window==Win0)        OLED_ShowStr(x,y,ch,TextSize);
}

void UI_ShowInt(unsigned char x0, unsigned char y0, int num, unsigned char TextSize)
{
    if(now_window==Win0)        OLED_ShowInt(x0,y0,num,TextSize);
}

void UI_ShowDouble(unsigned char x, unsigned char y,double num, unsigned char TextSize)
{
    if(now_window==Win0)        OLED_ShowDouble(x,y,num,TextSize);
}

void show_form(uint8 window_num)
{
  switch(window_num){
      case Win0:{       //空窗体
       
      }break;
      case Win1:{       //开机界面
        OLED_ShowStr(1,0,"!^^^^^^^^^^^^^^^^^^^!",1);
        OLED_ShowStr(13,1,"XiaoYing team",2);
        
        OLED_ShowStr(20,4," Lop-Std",1);
        OLED_ShowStr(20,5," Lop-Trn",1);
        OLED_ShowStr(20,6," Lop_Spd",1);

        OLED_ShowStr(2,7,"Fun",1);
        OLED_ShowStr(103,7,"Esc",1);

        if(functionFlag&0x01) OLED_ShowStr(74,4,"open",1);
        else    OLED_ShowStr(74,4,"    ",1);
        if(functionFlag&0x02) OLED_ShowStr(74,5,"open",1);
        else    OLED_ShowStr(74,5,"    ",1);
        if(functionFlag&0x04) OLED_ShowStr(74,6,"open",1);
        else    OLED_ShowStr(74,6,"    ",1);
        /*
        OLED_ShowStr(48,4,"@__@",2);
        for(int i=0;i<10000;i++)
          for(int j=100;j<1000;j++);
        OLED_ShowStr(48,4,"-__-",2);
        for(int i=0;i<1000;i++)
          for(int j=100;j<1000;j++);
        */
      }break;
      case Win2:{       //修改ListInt元素
        //OLED_ShowStr(1,0,"name:      value:",1);
        OLED_ShowStr(1,5,"_____________________",1);
        OLED_ShowStr(1,6," OK|",2);
        OLED_ShowStr(95,6,"|->",2);
        //ListInt
        if(pInt-1>=0)
        {
          OLED_ShowStr(2,0,ListInt[pInt-1].name,1);
          OLED_ShowInt(48,0,*ListInt[pInt-1].value,1);
        }
        else
        {
          OLED_ShowStr(2,0,"             ",1);
        }
        if(pInt>=0&&pInt<IntLength)
        {
          OLED_ShowStr(2,1,ListInt[pInt].name,2);
          OLED_ShowInt(48,1,*ListInt[pInt].value,2);
        }
        else
          pInt=0;
        if(pInt+1<IntLength)
        {
          OLED_ShowStr(2,3,ListInt[pInt+1].name,1);
          OLED_ShowInt(48,3,*ListInt[pInt+1].value,1);
        }
        else
        {
          OLED_ShowStr(2,3,"             ",1);
        }     
        
        if(pInt+2<IntLength)
        {
          OLED_ShowStr(2,4,ListInt[pInt+2].name,1);
          OLED_ShowInt(48,4,*ListInt[pInt+2].value,1);
        }
        else
        {
          OLED_ShowStr(2,4,"             ",1);
        }     
      }break;
      case Win3:{       //修改ListDouble元素
        
        OLED_ShowStr(1,5,"_____________________",1);
        OLED_ShowStr(1,6," OK|",2);
        OLED_ShowStr(95,6,"|->",2);
        //ListDouble
        if(pDb-1>=0)
        {
          OLED_ShowStr(2,0,ListDouble[pDb-1].name,1);
          OLED_ShowDouble(48,0,*ListDouble[pDb-1].value,1);
        }
        else
        {
          OLED_ShowStr(2,0,"             ",1);
        }
        if(pDb>=0&&pDb<DbLength)
        {
          OLED_ShowStr(2,1,ListDouble[pDb].name,2);
          OLED_ShowDouble(48,1,*ListDouble[pDb].value,2);
        }
        else
          pDb=0;
        if(pDb+1<DbLength)
        {
          OLED_ShowStr(2,3,ListDouble[pDb+1].name,1);
          OLED_ShowDouble(48,3,*ListDouble[pDb+1].value,1);
        }
        else
        {
          OLED_ShowStr(2,3,"             ",1);
        }     
        
        if(pDb+2<DbLength)
        {
          OLED_ShowStr(2,4,ListDouble[pDb+2].name,1);
          OLED_ShowDouble(48,4,*ListDouble[pDb+2].value,1);
        }
        else
        {
          OLED_ShowStr(2,4,"             ",1);
        }
        
      }break;
      case Win4:{
        OLED_ShowStr(12,0,"save int +l",2);
        OLED_ShowStr(12,2,"save dbl +r",2);
        OLED_ShowStr(12,4,"save All +A",2);
        
        OLED_ShowStr(1,6,"_____________________",1);
        OLED_ShowStr(1,7," SAVE\\",1);
        OLED_ShowStr(95,7,"  /->",1);
      }break;
  }
}
void deal_key() //按键事件处理
{
    KEY_MSG_t keymsg;
    keymsg=get_key_msg();
    switch(keymsg.key)
    {
      case KEY_U:{      //上键
        if(keymsg.status==KEY_DOWN||keymsg.status==KEY_HOLD)
        {
          switch(now_window){
            case Win0:{}break;
            case Win1:{         //切换显示直立环
              if(functionFlag&0x01==0x01)       functionFlag&=~(0x01);
              else  functionFlag|=0x01; 
            }break;
            case Win2:{
              if(pInt>0)
                 pInt--;
            }break;
            case Win3:{ 
              if(pDb>0)
                 pDb--;
            }break;
            case Win4:{}break;
          }
        }
      }break;
      case KEY_D:{      //下键
        if(keymsg.status==KEY_DOWN||keymsg.status==KEY_HOLD)
        {
          switch(now_window){
            case Win0:{
              
            }break;
            case Win1:{}break;
            case Win2:{
              pInt++;
              if(pInt>=IntLength)
                pInt=IntLength-1;
              
              
            }break;
            case Win3:{
              pDb++;
              if(pDb>=DbLength)
                pDb=DbLength-1;
            }break;
            case Win4:{}break;
          }
        }
      }break;
      case KEY_L:{      //左键
        if(keymsg.status==KEY_DOWN||keymsg.status==KEY_HOLD)
        {
          switch(now_window){
            case Win0:{}break;
            case Win1:{         //切换显示方向环
              if((functionFlag&0x02)==0x02)       functionFlag&=~(0x02);
              else  functionFlag|=0x02; 
            }break;
            case Win2:{
              int tempI=*ListInt[pInt].value;
              
              if(keymsg.status==KEY_DOWN)tempI+=Igap; 
              else tempI=tempI+Igap*Irate; 
              
              *ListInt[pInt].value=tempI;
             
            }break;
            case Win3:{
              double tempD=*ListDouble[pDb].value;
              
              if(keymsg.status==KEY_DOWN)  tempD+=Dgap;
              else tempD=tempD+Dgap*Drate;
              
              *ListDouble[pDb].value=tempD;
            }break;
            case Win4:{ //保存IntList
              saveFlash();
            }break;
          }
        }
      }break;
      case KEY_R:{      //右键
        if(keymsg.status==KEY_DOWN||keymsg.status==KEY_HOLD)
        {
          switch(now_window){
            case Win0:{}break;
            case Win1:{ //切换显示速度环
              if((functionFlag&0x04)==0x04)       functionFlag&=~(0x04);
              else  functionFlag|=0x04; 
            }break;
            case Win2:{
              int tempI=*ListInt[pInt].value;
              
              if(keymsg.status==KEY_DOWN)tempI-=Igap; 
              else tempI=tempI-Igap*Irate; 
              
              *ListInt[pInt].value=tempI;
            }break;
            case Win3:{
              double tempD=*ListDouble[pDb].value;
              
              if(keymsg.status==KEY_DOWN)  tempD-=Dgap;
              else tempD=tempD-Dgap*Drate;
              
              *ListDouble[pDb].value=tempD;
            }break;
            case Win4:{ //保存DoubleList
              saveFlash();
            }break;
          }
        }
      }break;
      case KEY_A:{      //A键(确认键)
        if(keymsg.status==KEY_DOWN){
          switch(now_window){
            case Win0:{
              OLED_Fill(0x00);
              now_window++;
            }break;
            case Win1:{
              OLED_Fill(0x00);
              now_window++;
            }break;
            case Win2:{
              OLED_Fill(0x00);
              now_window++;
            }break;
            case Win3:{
              OLED_Fill(0x00);
              now_window++;
            }break;
            case Win4:{ //保存IntList和DoubleList
              saveFlash();
            }break;
          }
        }  
      }break;
      case KEY_B:{      //B键(取消键)
          if(keymsg.status==KEY_DOWN) 
          {
            if(now_window>Win0) 
            {
              OLED_Fill(0x00);
              now_window--;
            }
            switch(now_window){
            case Win0:{
              OLED_ShowStr(5,2,"Thank for using",2);
              for(int i=0;i<10000;i++)
                  for(int j=100;j<1000;j++);
              OLED_Fill(0x00);
            }break;
          }
        }
      }break;
    }
}

void saveFlash()//保存成功动画
{
  now_window=Win1;
  OLED_Fill(0x00);
  for(int i=0;i<128;i+=8)
  {
    OLED_ShowStr(i,2,">",2);
    OLED_ShowStr(i,4,">",2);
    for(int k=0;k<10000;k++){
      if(i>=56) for(int j=0;j<300;j++);
      else for(int j=0;j<1000;j++);
    }
  }
  OLED_Fill(0x00);
  
  OLED_ShowStr(5,2,"Save Succeed!",2);
  for(int i=0;i<10000;i++)
      for(int j=100;j<1000;j++);
  OLED_Fill(0x00);
}