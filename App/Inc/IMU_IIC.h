#ifndef _IMU_IIC_H_
#define _IMU_IIC_H_

#include "common.h"
#include "MK60_i2c.h"
#define u8 uint8

void IIC_start(void);
void IIC_stop(void);
void IIC_ack_main(u8 ack_main);
void send_byte(u8 c);
u8 read_byte(void);
void send_to_byte(u8 ad_main,u8 c);
void send_to_nbyte(u8 ad_main,u8 ad_sub,u8 *buf,u8 num);
void read_from_byte(u8 ad_main,u8 *buf);
void read_from_nbyte(u8 ad_main,u8 ad_sub,u8 *buf,u8 num);
void IMU_IIC_Init();



#endif