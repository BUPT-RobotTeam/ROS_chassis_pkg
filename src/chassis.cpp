#include "chassis.h"
#include <tf2/transform_datatypes.h> 
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

Chassis::Chassis()
{
    geometry_msgs::Pose empty;
    empty.position.x=0;
    empty.position.y=0;
    empty.position.z=0;
    empty.orientation.w=1;
    empty.orientation.x=0;
    empty.orientation.y=0;
    empty.orientation.z=0;
    actual_pose=empty;
    target_pose=empty;
    for (int i=0;i<6;i++)
        controller[i].setPID(1,0,0.5);
}

void Chassis::exec()
{
    geometry_msgs::Twist ctrl;
    ctrl.linear.x=controller[0].calc_output(target_pose.position.x,actual_pose.position.x);
    ctrl.linear.y=controller[1].calc_output(target_pose.position.y,actual_pose.position.y);
    ctrl.linear.z=controller[2].calc_output(target_pose.position.z,actual_pose.position.z); // 无意义 底盘无Z轴自由度
    
    
    tf2::Quaternion q1,q2;
    q1.setW(target_pose.orientation.w);  q2.setW(actual_pose.orientation.w);
    q1.setX(target_pose.orientation.x);  q2.setX(actual_pose.orientation.x);
    q1.setY(target_pose.orientation.y);  q2.setY(actual_pose.orientation.y);
    q1.setZ(target_pose.orientation.z);  q2.setZ(actual_pose.orientation.z);

    double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    tf2::Matrix3x3(q1).getRPY(roll1, pitch1, yaw1);
    tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);

    ctrl.angular.x=controller[3].calc_output(roll1,roll2); // 无意义
    ctrl.angular.y=controller[4].calc_output(pitch1,pitch1); // 无意义
    ctrl.angular.z=controller[5].calc_output(yaw1,yaw2); 

    ROS_INFO("Sending the velocity. x: %f yaw: %f",ctrl.linear.x,ctrl.angular.z);
    geometry_msgs::Twist::ConstPtr ctrlPtr=boost::make_shared<const geometry_msgs::Twist>(ctrl);
    chassis_move(ctrlPtr);
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
