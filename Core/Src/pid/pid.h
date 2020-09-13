#ifndef __PID_H__
#define __PID_H__
#include "main.h"


#define PID_INTEGRAL_ON    //位置式PID是否包含积分项。如果仅用PD控制，注释本行
typedef struct PID
{ 
    float P;               
    float I;
    float D;	
#ifdef 	PID_INTEGRAL_ON
    float Integral;        //位置式PID积分项
    float IntegralMax;     //位置式PID积分项最大值，用于限幅
#endif	
    float Last_Error;      //上一次误差	
    float OutputMax;       //位置式PID输出最大值，用于限幅
    float OutputMin;       //位置式PID输出最小值，用于限幅
}PID;


float PID_Cal(PID *pid, int32_t NowValue, int32_t AimValue);

#endif
