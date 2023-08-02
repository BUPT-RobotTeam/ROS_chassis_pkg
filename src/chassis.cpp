#include "chassis.h"
#include <tf/transform_datatypes.h> 

Chassis::Chassis()
{
    geometry_msgs::Pose empty;
    empty.position.x=0;
    empty.position.y=0;
    empty.position.z=0;
    empty.orientation.w=0;
    empty.orientation.x=0;
    empty.orientation.y=0;
    empty.orientation.z=0;

    actual_pose=empty;
    target_pose=empty;
}

void Chassis::exec()
{
    geometry_msgs::Twist ctrl;
    ctrl.linear.x=controller.calc_output(target_pose.position.x,actual_pose.position.x);
    ctrl.linear.y=controller.calc_output(target_pose.position.y,actual_pose.position.y);
    ctrl.linear.z=controller.calc_output(target_pose.position.z,actual_pose.position.z); // 无意义 底盘无Z轴自由度
    
    tf::Quaternion q1;
    q.setW(target_pose.orientation.w);  
    q.setX(target_pose.orientation.x);
    q.setY(target_pose.orientation.y);
    q.setZ(target_pose.orientation.z);

    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

int Chassis::chassis_serial_init(const std::string &serial_name, const int &baud)
{
    timeout=serial::Timeout::simpleTimeout(100);
    serial_port.setPort(serial_name);
    serial_port.setBaudrate(baud);
    serial_port.setTimeout(timeout);
    try
    {
        serial_port.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open the serial.Please check your settings.");
        return -1;
    }
    ROS_INFO("Chassis serial has been open.");
    return 0;
}

void Chassis::update_chassis_posture(const geometry_msgs::Pose::ConstPtr &pose)
{
    actual_pose=*pose;
}

void Chassis::set_target_pose(const geometry_msgs::Pose::ConstPtr &target)
{
    target_pose=*target;
}

void Chassis::chassis_move(const geometry_msgs::Twist::ConstPtr &vel)
{
    serialVelMsgTypeDef msg;
    msg.raw_msg[0]=vel->linear.x;
    msg.raw_msg[1]=vel->linear.y;
    msg.raw_msg[2]=vel->linear.z;
    msg.raw_msg[3]=vel->angular.x;
    msg.raw_msg[4]=vel->angular.y;
    msg.raw_msg[5]=vel->angular.z;
    serial_port.write(msg.ui8,sizeof(msg));
}
