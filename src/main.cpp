#include <ros/ros.h>
#include "chassis.h"
#include <exception>
#include <geometry_msgs/PoseStamped.h>

Chassis myChassis;

void velMsgCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    myChassis.chassis_move(msg);
}

void poseMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::Pose curPose;
    curPose=msg->pose;
    geometry_msgs::Pose::ConstPtr curPosePtr = boost::make_shared<const geometry_msgs::Pose>(curPose);
    myChassis.update_chassis_posture(curPosePtr);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "chassis_node");
    ros::NodeHandle nh;

    std::string vel_topic,serial_name,pose_topic;
    ros::Subscriber vel_sub,pose_sub;
    int serial_baud;
    
    nh.getParam("velTopic",vel_topic);
    nh.getParam("poseTopic",pose_topic);
    nh.getParam("chassisSerialName",serial_name);
    nh.getParam("chassisSerialBaud",serial_baud);
    
    vel_sub=nh.subscribe(vel_topic,10,velMsgCallback);
    pose_sub=nh.subscribe(pose_topic,10,poseMsgCallback);

    if (myChassis.chassis_serial_init(serial_name,serial_baud)==-1) 
    {
        ROS_ERROR("Serial initalizing failed");
        throw std::runtime_error("Serial initalizing failed");
        return -1; 
    }

    ros::Rate loop_rate(50);
    ros::spin();
    return 0;
}
