#include "common.h"
#include "MMA8451.h"
#include "IMU_IIC.h"
//�˴��������ص�ͷ�ļ�


//#define IOS

void mma_delay(u16 k)		 //��ʱ����
{
    u8 i;	 //�ں����У�����������������Ȼ����ܵ���
    while(--k)
    {
       for(i=0;i<100;i++); //��ʱʱ�䳤�̣�0--255��ѡ
    }
}


#ifdef IOS
//��ȡMMA8451��1���ֽ�
u8 MMA8451_readbyte(u8 address)
{
	u8 ret;
	IIC_start();		//����
	send_byte(MMA845x_IIC_ADDRESS);	//д���豸ID��д�ź�
	send_byte(address);	//X��ַ
	IIC_start();		//���·��Ϳ�ʼ
	send_byte(MMA845x_IIC_ADDRESS+1);	//д���豸ID������
	ret = read_byte();	//��ȡһ�ֽ�
	IIC_stop();

	return ret;
}



//д��MMA8451��1���ֽ�
void MMA8451_writebyte(u8 address, u8 thedata)
{
	IIC_start();		//����
	send_byte(MMA845x_IIC_ADDRESS);	//д���豸ID��д�ź�
	send_byte(address);	//X��ַ
	send_byte(thedata);	//д���豸ID������
	IIC_stop();
}

#else
#include "MK60_i2c.h"


//��ȡMMA8451��1���ֽ�
#define MMA8451_readbyte(address) i2c_read_reg(I2C1,MMA845x_IIC_ADDRESS>>1,address)

//д��MMA8451��1���ֽ�
#define MMA8451_writebyte(address,thedata)  i2c_write_reg(I2C1,MMA845x_IIC_ADDRESS>>1,address,thedata)




#endif
/*
 * �������ܣ���MAA8451���ٶ����
 * �����w
 *       Status - ���ݼĴ���״̬
 *       Regs_Addr - ���ݼĴ�����ַ
 * ��������ֵ�����ٶ�ֵ��int16��
 */   
int16 MMA8451_GetResult(u8 Status, u8 Regs_Addr) 
{
  u8 ret=0;
  u16 cnt=0;
  int16 result,temp;
  
  if(Regs_Addr>MMA8451_REG_OUTZ_LSB)
    return 0;
  
  // �ȴ�ת����ɲ�ȡ��ֵ 
  while(!(ret&Status)) 
  {
    ret = MMA8451_readbyte( MMA8451_REG_STATUS);
    if(++cnt>500)
      break;
  }
  
  result= MMA8451_readbyte( Regs_Addr);
  temp  = MMA8451_readbyte( Regs_Addr+1);
  result=result<<8;
  result=result|temp;
  
  return result>>2;
}


//��ȡ3����ٶ�
int16 ACC_X, ACC_Y, ACC_Z;
void MMA8451_XYZ()
{
    ACC_X = MMA8451_GetResult(MMA8451_STATUS_X_READY, MMA8451_REG_OUTX_MSB);
    ACC_Y = MMA8451_GetResult(MMA8451_STATUS_Y_READY, MMA8451_REG_OUTY_MSB);
    ACC_Z = MMA8451_GetResult(MMA8451_STATUS_X_READY, MMA8451_REG_OUTZ_MSB);	
}


//MMA8451��ʼ��
void MMA8451_Init()
{	
    while(MMA8451_readbyte(MMA8451_REG_WHOAMI)!=0x1a);
	mma_delay(10);
	MMA8451_writebyte(MMA8451_REG_SYSMOD,0x00);	   //Ĭ��ģʽStandby Mode
	mma_delay(10);
	MMA8451_writebyte(MMA8451_REG_CTRL_REG2,0x02); //High Resolution
    mma_delay(10);
	MMA8451_writebyte(MMA8451_REG_CTRL_REG1,0x01); //����ģʽ,800HZ
	
}



  		
  
