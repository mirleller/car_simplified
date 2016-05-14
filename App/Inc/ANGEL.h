#ifndef _ANGEL_H_
#define _ANGEL_H_
#include "include.h"
#include "L3G4200.h"
//*********************************************************
//定义获取加速度计和陀螺仪的读值
//*********************************************************


#define Get_acc_X(x)     x=MMA8451_GetResult(MMA8451_STATUS_X_READY, MMA8451_REG_OUTX_MSB);
#define Get_acc_Y(x)     x=MMA8451_GetResult(MMA8451_STATUS_Y_READY, MMA8451_REG_OUTY_MSB);
#define Get_acc_Z(x)     x=MMA8451_GetResult(MMA8451_STATUS_X_READY, MMA8451_REG_OUTZ_MSB);
#define Get_gyr_X(x)     x=L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_X_H);
#define Get_gyr_Y(x)     x=L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_Y_H);
#define Get_gyr_Z(x)     x=L3G4200_GetResult(L3G4200D_STATUS_REG, L3G4200D_OUT_Z_H);	


#endif