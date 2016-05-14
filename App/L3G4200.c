#include "common.h"
#include "L3G4200.h"
#include "IMU_IIC.h"
//此处请添加相关的头文件


//#define IOS     //IO口模拟IIC的开关


//延时函数
void tly_delay(u16 k)		 
{
    u8 i;	 

    while(--k)
    {
        for(i=0;i<100;i++); //延时时间长短，0--255可选
    }
}



#ifdef IOS



//读取L3G4200的1个字节
u8 L3G4200_readbyte(u8 address)
{
	u8 ret;
	IIC_start();		//启动
	send_byte(SlaveAddress);	//写入设备ID及写信号
	send_byte(address);	//X地址
	IIC_start();		//重新发送开始
	send_byte(SlaveAddress+1);	//写入设备ID及读信
	ret = read_byte();	//读取一字节
	IIC_stop();

	return ret;
}



//写入L3G4200的1个字节
void L3G4200_writebyte(u8 address, u8 thedata)
{
	IIC_start();		//启动
	send_byte(SlaveAddress);	//写入设备ID及写信号
	send_byte(address);	//X地址
	send_byte(thedata);	//写入设备ID及读信
	IIC_stop();
}


#else



#include "MK60_i2c.h"

//读取L3G4200的1个字节
#define L3G4200_readbyte(address) i2c_read_reg(I2C1,SlaveAddress>>1,address)

//写入L3G4200的1个字节
#define L3G4200_writebyte(address,thedata)  i2c_write_reg(I2C1,SlaveAddress>>1,address,thedata)


#endif



/*
 * 函数功能：读L3G4200角速度输出
 * 参数
 *       Status - 数据寄存器状态
 *       Regs_Addr - 数据寄存器地址
 * 函数返回值：加速度值（int16）
 */   
int16 L3G4200_GetResult(u8 Status, u8 Regs_Addr) 
{
  u8 ret=0;
  u16 cnt=0;
  int16 result=0,temp=0;
  
  if(Regs_Addr>L3G4200D_OUT_Z_H)
    return 0;
  
  // 等待转换完成并取出值 
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

//读取3轴角速度
int16 Gyro_X, Gyro_Y, Gyro_Z;
void L3G4200_XYZ()
{
    Gyro_X = L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_X_H);
    Gyro_Y = L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_Y_H);
    Gyro_Z = L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_Z_H);	
}

//*****************************************************************

//初始化L3G4200D，根据需要请参考pdf进行修改************************
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