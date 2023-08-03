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

void poseMsgCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    geometry_msgs::PoseStamped curPose,tf_curPose;
    curPose=*msg;
    tf2_ros::Buffer tfBuffer;
    geometry_msgs::TransformStamped imu2base;
    tf2_ros::TransformListener listener(tfBuffer);
    try
    {
        imu2base=tfBuffer.lookupTransform("base_link","imu_frame",ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return ;
    }
    tf2::doTransform(curPose,tf_curPose,imu2base);
    geometry_msgs::Pose::ConstPtr tf_curPosePtr = boost::make_shared<const geometry_msgs::Pose>(tf_curPose.pose);
    myChassis.update_chassis_posture(tf_curPosePtr);
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
    while (ros::ok())
    {
        ros::spinOnce();
        myChassis.exec();
        loop_rate.sleep();
    }
    return 0;
}
