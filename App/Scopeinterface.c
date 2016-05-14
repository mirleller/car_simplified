#include "common.h"
#include "Scopeinterface.h"
#include "include.h"
/*
void Scopeinterface_Init()
{
  uart5_init_struct.UART_Uartx = UART5; //使用UART5
  uart5_init_struct.UART_BaudRate = 115200; //设置波特率9600
  //初始化UART
  LPLD_UART_Init(uart5_init_struct);
}*/
int16 Outdata[4]={0};
void OutPut_Data(int16 *OutData)
{
  unsigned short CRC_Temp;  
//  int temp[4] = {0};
//  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i,j;
  unsigned short CRC16 = 0;

 /* for(i=0;i<4;i++)
   {
    
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i];
    
   }
 */  
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = OutData[i]&0xff;
    databuf[i*2+1] = OutData[i]>>8;
   
  }
  ///////add
   CRC_Temp = 0xffff;
   for (i=0;i<8; i++)
   {      
      CRC_Temp ^= databuf[i];
      for (j=0;j<8;j++) 
      {
        if (CRC_Temp & 0x01)
          CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
        else
          CRC_Temp = CRC_Temp >> 1;
      }
    }
  //////
  CRC16 = CRC_Temp;
  databuf[8] = CRC16&0xff;
  databuf[9] = CRC16>>8; 

  for(i=0;i<10;i++)
    uart_putchar(UART4,databuf[i]);
}
