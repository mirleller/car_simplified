#include "common.h"
#include "MMA8451.h"
#include "IMU_IIC.h"
//此处请添加相关的头文件


//#define IOS

void mma_delay(u16 k)		 //延时函数
{
    u8 i;	 //在函数中，必须先声明变量，然后才能调用
    while(--k)
    {
       for(i=0;i<100;i++); //延时时间长短，0--255可选
    }
}


#ifdef IOS
//读取MMA8451的1个字节
u8 MMA8451_readbyte(u8 address)
{
	u8 ret;
	IIC_start();		//启动
	send_byte(MMA845x_IIC_ADDRESS);	//写入设备ID及写信号
	send_byte(address);	//X地址
	IIC_start();		//重新发送开始
	send_byte(MMA845x_IIC_ADDRESS+1);	//写入设备ID及读信
	ret = read_byte();	//读取一字节
	IIC_stop();

	return ret;
}



//写入MMA8451的1个字节
void MMA8451_writebyte(u8 address, u8 thedata)
{
	IIC_start();		//启动
	send_byte(MMA845x_IIC_ADDRESS);	//写入设备ID及写信号
	send_byte(address);	//X地址
	send_byte(thedata);	//写入设备ID及读信
	IIC_stop();
}

#else
#include "MK60_i2c.h"


//读取MMA8451的1个字节
#define MMA8451_readbyte(address) i2c_read_reg(I2C1,MMA845x_IIC_ADDRESS>>1,address)

//写入MMA8451的1个字节
#define MMA8451_writebyte(address,thedata)  i2c_write_reg(I2C1,MMA845x_IIC_ADDRESS>>1,address,thedata)




#endif
/*
 * 函数功能：读MAA8451加速度输出
 * 参数
 *       Status - 数据寄存器状态
 *       Regs_Addr - 数据寄存器地址
 * 函数返回值：加速度值（int16）
 */   
int16 MMA8451_GetResult(u8 Status, u8 Regs_Addr) 
{
  u8 ret=0;
  u16 cnt=0;
  int16 result,temp;
  
  if(Regs_Addr>MMA8451_REG_OUTZ_LSB)
    return 0;
  
  // 等待转换完成并取出值 
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


//读取3轴加速度
int16 ACC_X, ACC_Y, ACC_Z;
void MMA8451_XYZ()
{
    ACC_X = MMA8451_GetResult(MMA8451_STATUS_X_READY, MMA8451_REG_OUTX_MSB);
    ACC_Y = MMA8451_GetResult(MMA8451_STATUS_Y_READY, MMA8451_REG_OUTY_MSB);
    ACC_Z = MMA8451_GetResult(MMA8451_STATUS_X_READY, MMA8451_REG_OUTZ_MSB);	
}


//MMA8451初始化
void MMA8451_Init()
{	
    while(MMA8451_readbyte(MMA8451_REG_WHOAMI)!=0x1a);
	mma_delay(10);
	MMA8451_writebyte(MMA8451_REG_SYSMOD,0x00);	   //默认模式Standby Mode
	mma_delay(10);
	MMA8451_writebyte(MMA8451_REG_CTRL_REG2,0x02); //High Resolution
    mma_delay(10);
	MMA8451_writebyte(MMA8451_REG_CTRL_REG1,0x01); //主动模式,800HZ
	
}



  		
  
