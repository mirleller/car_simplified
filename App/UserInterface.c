#include "UI.h"
#include "common.h"
#include "VCAN_key.h"
#include "OLED_I2C.h"
#define MaxLength 20    //�궨���������󳤶�
#define Igap  1
#define Dgap  0.1
#define Irate 5
#define Drate 5

VarInt ListInt[MaxLength];
VarDouble  ListDouble[MaxLength];

int8 IntLength=0;                       //��¼��������Ԫ�ظ���
int8 DbLength=0;                        //��¼Double����Ԫ�ظ���

int8 pInt=0;                            //��������ָ��
int8 pDb=0;                             //Double����ָ��

WinNum now_window=Win0;

void show_form(uint8 window_num);       //��ʾ��ǰӦ��ʾ�Ĵ���
int8 UI_Add_Int(char* nam,int *val);    //��Int�������������,nam����������val������ֵ ��ӳɹ�����1�����򷵻�0
int8 UI_Add_Double(char* nam,float *val);//��Double�������������,nam����������val������ֵ ��ӳɹ�����1�����򷵻�0
void deal_key(void);

void UI_init()//UI��ʼ��
{
   OLED_Init();          //��ʼ��OLED   
   OLED_Fill(0x00);     //ȫ��Ϩ��
   key_init(KEY_MAX);   //��ʼ������
}

void key_detect()                       //������⣬���а�����������а����¼���������ֱ���˳�
{
  show_form(now_window);
  uint8 times=2;                 
  while(times--)                //ѭ�����˴ΰ���
    key_IRQHandler();
  uint8 flag=Is_empty();        //��ȡ����״̬
  if(flag==1) 
    deal_key();
}

int8 UI_Add_Int(char* nam,int *val)
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
int8 UI_Add_Double(char* nam,float *val)
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
  if(now_window==Win0)
    OLED_ShowStr(x,y,ch,TextSize);
}

void UI_ShowInt(unsigned char x0, unsigned char y0, int num, unsigned char TextSize)
{
  if(now_window==Win0)  
    OLED_ShowInt(x0,y0,num,TextSize);
}

void UI_ShowDouble(unsigned char x, unsigned char y,double num, unsigned char TextSize)
{
  
    
    int n1=num;
    OLED_ShowInt(x,y,num,TextSize);
    if(num<1e-6)
      num=-num;
    int n2=(num-n1)*100;
    int k=0;
    do{
      k++;
      n1/=10;
    }while(n1>0);
    if(n2>0){
      OLED_ShowStr(x+k*(TextSize*8),y,".",TextSize);
      OLED_ShowInt(x+(k+1)*(TextSize*8),y,n2,TextSize);
    } 
  //OLED_ShowInt(x,y,num,TextSize);
}

void show_form(uint8 window_num)
{
  switch(window_num){
      case Win0:{//�մ���
       
      }break;
      case Win1:{//����
        OLED_ShowStr(1,0,"!^^^^^^^^^^^^^^^^^^^!",1);
        OLED_ShowStr(13,2,"XiaoYing team",2);
        OLED_ShowStr(2,6,"Fun",2);
        OLED_ShowStr(103,6,"Esc",2);
        /*
        OLED_ShowStr(48,4,"@__@",2);
        for(int i=0;i<10000;i++)
          for(int j=100;j<1000;j++);
        OLED_ShowStr(48,4,"-__-",2);
        for(int i=0;i<1000;i++)
          for(int j=100;j<1000;j++);
        */
      }break;
      case Win2:{//�޸�ListIntԪ��
        //OLED_ShowStr(1,0,"name:      value:",1);
        OLED_ShowStr(1,5,"_____________________",1);
        OLED_ShowStr(1,6," OK|",2);
        OLED_ShowStr(95,6,"|->",2);
        //ListInt
        if(pInt-1>0)
        {
          OLED_ShowStr(2,0,ListInt[pInt-1].name,1);
          OLED_ShowInt(48,0,*ListInt[pInt-1].value,1);
        }
        else
        {
          
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
          
        }     
      }break;
      case Win3:{//�޸�ListDoubleԪ��
        //ListDouble
       
        if(pDb>=0&&pDb<DbLength)
        {
         // OLED_ShowStr(2,1,"HelloWorld",2);
          OLED_ShowStr(2,1,ListDouble[pDb].name,2);
          double k=*ListDouble[pDb].value;
          UI_ShowDouble(48,1,k,2);
        }
        else
          pDb=0;
        
      }break;
      case Win4:{}break;
  }
}
void deal_key()
{
    KEY_MSG_t keymsg;
    keymsg=get_key_msg();
    switch(keymsg.key)
    {
      case KEY_U:{      //�ϼ�
        if(keymsg.status==KEY_DOWN||keymsg.status==KEY_HOLD)
        {
          switch(now_window){
            case Win0:{
                
            }break;
            case Win1:{}break;
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
      case KEY_D:{      //�¼�
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
      case KEY_L:{      //���
        if(keymsg.status==KEY_DOWN||keymsg.status==KEY_HOLD)
        {
          switch(now_window){
            case Win0:{}break;
            case Win1:{}break;
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
            case Win4:{}break;
          }
        }
      }break;
      case KEY_R:{      //�Ҽ�
        if(keymsg.status==KEY_DOWN||keymsg.status==KEY_HOLD)
        {
          switch(now_window){
            case Win0:{}break;
            case Win1:{}break;
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
            case Win4:{}break;
          }
        }
      }break;
      case KEY_A:{      //A��(ȷ�ϼ�)
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
            case Win3:{}break;
            case Win4:{}break;
          }
        }  
      }break;
      case KEY_B:{      //B��(ȡ����)
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
            case Win1:{}break;
            case Win2:{}break;
            case Win3:{}break;
            case Win4:{}break;
          }
            
          }
      }break;
    }
}
