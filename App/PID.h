#ifndef _PID_H_
#define _PID_H_

struct _PID {
  float P;
  float I;
  float D;
  float ek;
  float ek_1;
  float ek_2;
  void (*a)(struct _PID *A,float P,float I,float D);
};
typedef struct _PID PID_;

void PID_init(PID_ *A,float P,float I,float D);
float PID_calcu(PID_ A);









#endif