#include "PID.h"
#include "common.h"

void PID_init(PID_ *A,float P,float I,float D)
{
A->P=P;
A->I=I;
A->D=D;
A->ek=0;
A->ek_1=0;
A->ek_2=0;
}

float PID_calcu(PID_ A)
{
 return (A.P)*(A.ek-A.ek_1)+(A.I)*(A.ek)+(A.D)*(A.ek-2*(A.ek_1)+A.ek_2);
}