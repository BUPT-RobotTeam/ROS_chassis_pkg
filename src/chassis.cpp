#include "chassis.h"
#include <tf2/transform_datatypes.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    first_tag=false;
    global_frame="map";
    for (int i=0;i<6;i++)
        controller[i].setPID(1.0,0,0.5);
}

void Chassis::exec()
{
    geometry_msgs::Twist ctrl;

    geometry_msgs::Pose target_in_base;
    geometry_msgs::TransformStamped map2base;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);
    try
    {
        map2base=tfBuffer.lookupTransform("base_link",global_frame,ros::Time(0),ros::Duration(0.3));
    }
    catch(const tf2::TransformException &e)
    {
        ROS_ERROR("%s",e.what());
        return ;
    }
    
    tf2::doTransform(target_pose,target_in_base,map2base);

    ctrl.linear.x=controller[0].calc_output(target_in_base.position.x,0);
    ctrl.linear.y=controller[1].calc_output(target_in_base.position.y,0); // 无意义 底盘无Y轴自由度
    ctrl.linear.z=controller[2].calc_output(target_in_base.position.z,0); // 无意义 底盘无Z轴自由度
    

    tf2::Quaternion q1,q2;
    q1.setW(target_in_base.orientation.w);  
    q1.setX(target_in_base.orientation.x);  
    q1.setY(target_in_base.orientation.y);  
    q1.setZ(target_in_base.orientation.z);  

    double roll, pitch, yaw;
    tf2::Matrix3x3(q1).getRPY(roll, pitch, yaw);

    ctrl.angular.x=controller[3].calc_output(roll,0); // 无意义
    ctrl.angular.y=controller[4].calc_output(pitch,0); // 无意义
    ctrl.angular.z=controller[5].calc_output(yaw,0); 

    ROS_INFO("x: %f y: %f yaw: %f",target_in_base.position.x,target_in_base.position.y,yaw);
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
    if (!first_tag)
    {
        target_pose=*pose;
        first_tag=true;
    }
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
