#include "pid.h"

template<typename T>void __LIMIT(T &a,const T &b)
{
    if (a>b) a=b;
    else if (a<-b) a=-b;
}

PID::PID()
{
    kp=ki=kd=int_max=ctrl_max=int_sum=0;
    last_error=last_delta_error=0;
}

double PID::calc_output(double target, double actual)
{
    double error;
    double delta_error;
    double result;

    error = target - actual;
    delta_error = error - last_error;

    delta_error *= 0.384f;
    delta_error += last_delta_error * 0.615f; //低通滤波

    last_error = error;

    int_sum += error;

    __LIMIT(int_sum, int_max); // 积分限幅
    last_delta_error = delta_error;

    result = error * kp + delta_error * kd + int_sum * ki;
    __LIMIT(result, ctrl_max);
    return result;
}

void PID::setPID(double p, double i, double d)
{
}
