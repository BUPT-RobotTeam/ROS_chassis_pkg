#ifndef PID_H
#define PID_H

class PID
{
private:
    double kp,ki,kd,int_max,ctrl_max,int_sum;
    double last_error,last_delta_error;
    bool I_flag;
public:
    PID();
    double calc_output(double target,double actual);
    void setPID(double p,double i,double d);
};

#endif