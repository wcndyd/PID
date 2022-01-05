#ifndef _PID_H_
#define _PID_H_

#include "stdint.h"

#define ABS(x)      (((x)>0)? (x): -(x))
enum
{
    LLAST   = 0,
    LAST    = 1,
    NOW     = 2,

    POSITION_PID,
    DELTA_PID,
};

typedef struct _PID_Typedef
{
    float Kp;
    float Ki;
    float Kd;

    float target[3];                    //目标值,包含NOW， LAST， LLAST上上次
    float feedback[3];                  //测量值
    float err[3];                           //误差

    float pout;                             //p输出
    float iout;                             //i输出
    float dout;                             //d输出

    float pos_out;                      //本次位置式输出
    float last_pos_out;                 //上次位置式输出
    float pos_out0;                     //位置式输出最小值

    float delta_u;                      //本次增量值
    float last_delta_out;               //上次增量式输出
    float delta_out;                    //本次增量式输出 = last_delta_out + delta_u

    float max_err;
    float deadband;                     //err < deadband return

    uint32_t pid_mode;
    int MaxOutput;              //输出限幅
    uint32_t IntegralLimit;         //积分限幅
} PID_TypeDef;

void PID_Init(
    PID_TypeDef    *pid,
    uint32_t            mode,
    uint32_t            maxout,
    uint32_t            intergral_limit,
    float               kp,
    float               ki,
    float               kd
);

void PID_Reset(PID_TypeDef  *pid, float kp, float ki, float kd);
		
void abs_limit(float *a, float ABS_MAX);

void PID_Init(
    PID_TypeDef    *pid,
    uint32_t            mode,
    uint32_t            maxout,
    uint32_t            intergral_limit,
    float               kp,
    float               ki,
    float               kd);

float PID_Calculate(PID_TypeDef *pid, float target, float feedback);

#endif

