#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <sstream>
#include <memory>
#include <std_msgs/String.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <tf2/convert.h>
#include <tf/transform_broadcaster.h>


float st,th,br;

ros::Publisher control_pub;
void controlCallback(const geometry_msgs::Twist &control_msg) {
    th = control_msg.linear.x;
    if(control_msg.angular.z>0.5)
    {
        st=(control_msg.angular.z-0.5)*2;
    }
    else
    {
        st=-control_msg.angular.z*2;
    }
    br = 0;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    ros::Subscriber control_sub = nh.subscribe("/smart/cmd_vel", 20, controlCallback);
    ros::Publisher control_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/hero/vehicle_control_cmd", 10);
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        carla_msgs::CarlaEgoVehicleControl command;
        command.throttle = th;
        command.steer = st;
        command.brake = br;
        control_pub.publish(command);
        ros::spinOnce();
        loop_rate.sleep();

    }
   ros::spin();

    return 0;
}

