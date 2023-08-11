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

    // ros::Rate loop_rate(100);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     geometry_msgs::PoseStamped base_pose;
    //     geometry_msgs::TransformStamped base_in_world;
    //     tf2_ros::Buffer tfBuffer;
    //     tf2_ros::TransformListener listener(tfBuffer);
    //     try
    //     {
    //         base_in_world=tfBuffer.lookupTransform("base_link","map",ros::Time(0),ros::Duration(0.2));
    //     }
    //     catch(const tf2::TransformException &e)
    //     {
    //         ROS_ERROR("%s",e.what());
    //         continue;
    //     }
    //     base_pose.header=base_in_world.header;
    //     base_pose.pose.position.x=base_in_world.transform.translation.x;
    //     base_pose.pose.position.y=base_in_world.transform.translation.y;
    //     base_pose.pose.position.z=base_in_world.transform.translation.z;
    //     base_pose.pose.orientation=base_in_world.transform.rotation;

    //     tf2::Quaternion q(
    //     base_pose.pose.orientation.x,
    //     base_pose.pose.orientation.y,
    //     base_pose.pose.orientation.z,
    //     base_pose.pose.orientation.w);
    //     tf2::Matrix3x3 m(q);
    //     double roll, pitch, yaw;
    //     m.getRPY(roll, pitch, yaw);

    //     ROS_INFO("x: %f y: %f yaw: %f",base_pose.pose.position.x,base_pose.pose.position.y,yaw);
    //     geometry_msgs::Pose::ConstPtr basePosePtr = boost::make_shared<const geometry_msgs::Pose>(base_pose.pose);
    //     myChassis.update_chassis_posture(basePosePtr);
    //     // myChassis.exec();
    //     loop_rate.sleep();
    // }
    return 0;
}
