#include <ros/ros.h>
#include "chassis.h"
#include <exception>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


Chassis myChassis;

void velMsgCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    myChassis.chassis_move(msg);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "chassis_node");
    ros::NodeHandle nh;

    std::string vel_topic,serial_name,pose_topic;
    ros::Subscriber vel_sub;
    int serial_baud;
    
    nh.getParam("velTopic",vel_topic);
    nh.getParam("chassisSerialName",serial_name);
    nh.getParam("chassisSerialBaud",serial_baud);
    
    vel_sub=nh.subscribe(vel_topic,10,velMsgCallback);

    if (myChassis.chassis_serial_init(serial_name,serial_baud)==-1) 
    {
        ROS_ERROR("Serial initalizing failed");
        throw std::runtime_error("Serial initalizing failed");
        return -1; 
    }

    ros::spin();
    return 0;
}
