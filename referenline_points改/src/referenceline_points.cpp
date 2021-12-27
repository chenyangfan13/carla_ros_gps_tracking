#include "ros/ros.h"
#include <tf/tf.h>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
using namespace std;


ros::Publisher marker_pub;

struct point{
    double x;
    double y;
    double z;
    double yaw;
};


bool loadPath(string filename , vector<point> &waypoint) {

    ifstream infile(filename);
    cout<<"infile"<<endl;
    string line;
    //string temp;
    int i=0;
    vector<double> arr;
    //stringstream temp_line;
    point temp_point;
    double data;
    istringstream iss;
    while(getline(infile, line))
    {
        iss.clear();
        iss.str(line);
        while(iss>>data)
        {
            cout<<"data = "<<data<<endl;
            //arr[i]=data;
            arr.push_back(data);
            
            // cout<<"    "<<i<<endl;
        }
        // cout<<"   "<<"   "<<endl;
          
     }
    cout<<"   "<<arr.size()<<"   "<<endl;
    //cout<<"test  :  "<<i<<endl;
    for(int j=0;j<arr.size();j+=4)
    {
        temp_point.x = arr[j];
        temp_point.y = arr[j+1];
        temp_point.z = arr[j+2];
        temp_point.yaw = arr[j+3];
        waypoint.push_back(temp_point);
    }
    return 1;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "referenceline_points");

    //setup communication
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    //ros::Publisher  cmd_vel_pub     = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    //ros::Publisher  trajectory_pub  = nh.advertise<geometry_msgs::PoseArray>("planned_trajectory", 10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("test_point_array", 10);
    //load tracks
    ros::Publisher path_pubs_ = nh.advertise<nav_msgs::Path>("splinepoints", 1000, true);
    nav_msgs::Path now_path;
    now_path.header.frame_id = "map";
    // 设置时间戳
    now_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    std::string points_filename = nh_priv.param<std::string>("gps_points", "/home/chen/ros_work/lidar/src/referenline_points/gps_points.txt");
    vector<point> waypoint;
    loadPath(points_filename , waypoint);
    visualization_msgs::Marker test_point;
    test_point.header.frame_id = "map";
    test_point.header.stamp = ros::Time::now();
    test_point.ns = "referenceline_points";
    test_point.action = visualization_msgs::Marker::ADD;
    test_point.pose.orientation.w = 1.0;
    test_point.id = 0;
    test_point.type = visualization_msgs::Marker::LINE_STRIP;
    test_point.scale.x = 0.3;
    test_point.color.a = 1.0;
    test_point.color.g = 0.5;
    test_point.color.b = 0.5;
    test_point.color.r = 0.4;
    test_point.points.clear();
    
    ros::Rate rate(10);
    while (ros::ok())
    {
        for(int i=0;i<waypoint.size();i++)
        {
            cout<<waypoint[i].x<<"   "<<waypoint[i].y<<"   "<<waypoint[i].z<<"   "<<waypoint[i].yaw<<endl;
            geometry_msgs::Point p;
            p.x = waypoint[i].x;
            p.y = waypoint[i].y;
            p.z = 0;
            test_point.points.push_back(p);
        }
        for(int i=0;i<waypoint.size();i++)
        {
            pose.pose.position.x = waypoint[i].x;
            pose.pose.position.y = waypoint[i].y;
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;
            now_path.poses.push_back(pose);
        }
      //  for(int i=0;i<waypoint.size();i++)
      //  {
      //      geometry_msgs::Point p;
      //      p.x = waypoint[i].x;
      //      p.y = waypoint[i].y;
      //      p.z = 0;
      //      test_point.points.push_back(p);
      //  }
        marker_pub.publish(test_point);
        path_pubs_.publish(now_path);
        ros::spinOnce();
        rate.sleep();

    }
    ros::spin();
    return 0;

    //std::cout<<"x :"<<waypoint[5].x<<"  "<<"y :"<<waypoint[5].y<<std::endl;    
}
