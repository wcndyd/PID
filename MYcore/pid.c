#include "pid.h"

void PID_Reset(PID_TypeDef  *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void abs_limit(float *a, float ABS_MAX)
{
    if (*a > ABS_MAX)
        *a = ABS_MAX;
    if (*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/**
  * @brief  初始化PID结构体
  * @param  PID结构体指针
    @param  比例系数
        @param  积分系数
        @param  微分系数
        @param  积分最大值
        @param  总输出最大值
  * @retval None
  */
void PID_Init(
    PID_TypeDef    *pid,
    uint32_t            mode,
    uint32_t            maxout,
    uint32_t            intergral_limit,
    float               kp,
    float               ki,
    float               kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;

    pid->target[0] = 0;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

}

float PID_Calculate(PID_TypeDef *pid, float target, float feedback)
{
    pid->feedback[NOW] = feedback;
    pid->target[NOW] = target;
    pid->err[NOW] = target - feedback;

    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err)
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if (pid->pid_mode == POSITION_PID)                   //位置式PID
    {
        pid->pout = pid->Kp * pid->err[NOW];
        pid->iout += pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - pid->err[LAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit);                //限制积分输出
        pid->pos_out = pid->pout + pid->iout + pid->dout;       // 计算总输出
        abs_limit(&(pid->pos_out), pid->MaxOutput);                 // 限制总输出
        pid->last_pos_out = pid->pos_out;                                       //更新上一次总输出
    }
    else if (pid->pid_mode == DELTA_PID)                //增量式PID
    {
        pid->pout = pid->Kp * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;   //update last time
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->feedback[LLAST] = pid->feedback[LAST];
    pid->feedback[LAST] = pid->feedback[NOW];
    pid->target[LLAST] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];

    return pid->pid_mode == POSITION_PID ? pid->pos_out : pid->delta_out;
}
