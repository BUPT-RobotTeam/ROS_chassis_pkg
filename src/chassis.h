#ifndef CHASSIS_H
#define CHASSIS_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <serial/serial.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include "pid.h"

union serialVelMsgTypeDef
{
    double raw_msg[6];
    uint8_t ui8[48];
};

class Chassis
{
private:
    serial::Serial serial_port;
    serial::Timeout timeout;
    geometry_msgs::Pose actual_pose;
    geometry_msgs::Pose target_pose;
    PID controller[6];
public:
    Chassis();
    void exec();
    void update_chassis_posture(const geometry_msgs::Pose::ConstPtr &);
    void set_target_pose(const geometry_msgs::Pose::ConstPtr &);
    void chassis_move(const geometry_msgs::Twist::ConstPtr &);
    int chassis_serial_init(const std::string &, const int &);    
};



#endif