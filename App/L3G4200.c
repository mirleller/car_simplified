#include "common.h"
#include "L3G4200.h"
#include "IMU_IIC.h"
//�˴��������ص�ͷ�ļ�


//#define IOS     //IO��ģ��IIC�Ŀ���


//��ʱ����
void tly_delay(u16 k)		 
{
    u8 i;	 

    while(--k)
    {
        for(i=0;i<100;i++); //��ʱʱ�䳤�̣�0--255��ѡ
    }
}



#ifdef IOS



//��ȡL3G4200��1���ֽ�
u8 L3G4200_readbyte(u8 address)
{
	u8 ret;
	IIC_start();		//����
	send_byte(SlaveAddress);	//д���豸ID��д�ź�
	send_byte(address);	//X��ַ
	IIC_start();		//���·��Ϳ�ʼ
	send_byte(SlaveAddress+1);	//д���豸ID������
	ret = read_byte();	//��ȡһ�ֽ�
	IIC_stop();

	return ret;
}



//д��L3G4200��1���ֽ�
void L3G4200_writebyte(u8 address, u8 thedata)
{
	IIC_start();		//����
	send_byte(SlaveAddress);	//д���豸ID��д�ź�
	send_byte(address);	//X��ַ
	send_byte(thedata);	//д���豸ID������
	IIC_stop();
}


#else



#include "MK60_i2c.h"

//��ȡL3G4200��1���ֽ�
#define L3G4200_readbyte(address) i2c_read_reg(I2C1,SlaveAddress>>1,address)

//д��L3G4200��1���ֽ�
#define L3G4200_writebyte(address,thedata)  i2c_write_reg(I2C1,SlaveAddress>>1,address,thedata)


#endif



/*
 * �������ܣ���L3G4200���ٶ����
 * �����w
 *       Status - ���ݼĴ���״̬
 *       Regs_Addr - ���ݼĴ�����ַ
 * ��������ֵ�����ٶ�ֵ��int16��
 */   
int16 L3G4200_GetResult(u8 Status, u8 Regs_Addr) 
{
  u8 ret=0;
  u16 cnt=0;
  int16 result=0,temp=0;
  
  if(Regs_Addr>L3G4200D_OUT_Z_H)
    return 0;
  
  // �ȴ�ת����ɲ�ȡ��ֵ 
  while(!(ret&Status)) 
  {
    ret = L3G4200_readbyte( L3G4200D_STATUS_REG);
    if(++cnt>500)
      break;
  }
  
  result= L3G4200_readbyte( Regs_Addr);
  temp  = L3G4200_readbyte( Regs_Addr-1);
  result=result<<8;
  result=result|temp;
  
  return result;
}

//��ȡ3����ٶ�
int16 Gyro_X, Gyro_Y, Gyro_Z;
void L3G4200_XYZ()
{
    Gyro_X = L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_X_H);
    Gyro_Y = L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_Y_H);
    Gyro_Z = L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_Z_H);	
}

//*****************************************************************

//��ʼ��L3G4200D��������Ҫ��ο�pdf�����޸�************************
void L3G4200_Init()
{
   while(L3G4200_readbyte(L3G4200D_WHO_AM_I)!=0xD3);
   tly_delay(10);
   L3G4200_writebyte(L3G4200D_CTRL_REG1, 0xFF);   
   tly_delay(10);
   L3G4200_writebyte(L3G4200D_CTRL_REG2, 0x00);   // 
   tly_delay(10);
   L3G4200_writebyte(L3G4200D_CTRL_REG3, 0x08);   //0x08
   tly_delay(10);
   L3G4200_writebyte(L3G4200D_CTRL_REG4, 0x30);  //+-2000dps
   tly_delay(10);
   L3G4200_writebyte(L3G4200D_CTRL_REG5, 0x00);
   tly_delay(10);
}