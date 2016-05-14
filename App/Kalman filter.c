//卡尔曼滤波算法
//Kalman filter
/*输入：倾角（单位：度）
        角速度（单位：度每秒）
*/
#define A 1
#define B 1
#define H 1
#ifdef 1
#define P0
#else 
float P0=0.005;
#endif
float an_1_0=0;
float 
float filter(float angel,float gyr)
{
  
}
