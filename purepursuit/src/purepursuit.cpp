#include <ros/ros.h>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"


#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <tf2/convert.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>



#define PREVIEW_DIS 20 //预瞄距离

#define Ld 1.868  //轴距

using namespace std;

ros::Publisher purepersuit_;
ros::Publisher path_pub_;
nav_msgs::Path path;

double carVelocity = 0;
double preview_dis = 0;
double k = 0.1;

// 计算四元数转换到欧拉角
std::array<float, 3> calQuaternionToEuler(const float x, const float y,
                                          const float z, const float w) {
  std::array<float, 3> calRPY = {(0, 0, 0)};
  // roll = atan2(2(wx+yz),1-2(x*x+y*y))
  calRPY[0] = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  // pitch = arcsin(2(wy-zx))
  calRPY[1] = asin(2 * (w * y - z * x));
  // yaw = atan2(2(wx+yz),1-2(y*y+z*z))
  calRPY[2] = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  return calRPY;
}

vector<double> r_x_;
vector<double> r_y_;

int pointNum = 0;  //保存路径点的个数
int targetIndex = pointNum - 1;
/*方案一*/
// vector<int> bestPoints_ = {pointNum - 1};
/*方案二*/
vector<float> bestPoints_ = {0.0};

//计算发送给模型车的转角
void poseCallback(const nav_msgs::Odometry &currentWaypoint) {
  auto currentPositionX = currentWaypoint.pose.pose.position.x;
  auto currentPositionY = currentWaypoint.pose.pose.position.y;
  auto currentPositionZ = 0.0;

  auto currentQuaternionX = currentWaypoint.pose.pose.orientation.x;
  auto currentQuaternionY = currentWaypoint.pose.pose.orientation.y;
  auto currentQuaternionZ = currentWaypoint.pose.pose.orientation.z;
  auto currentQuaternionW = currentWaypoint.pose.pose.orientation.w;
  auto currentPositionYaw = tf::getYaw(currentWaypoint.pose.pose.orientation);
  //std::array<float, 3> calRPY = calQuaternionToEuler(currentQuaternionX, currentQuaternionY,currentQuaternionZ, currentQuaternionW);

  /*************************************************************************************************
  //  方案一：通过累加路径距离，和预瞄距离进行比较以及夹角方向
  // 寻找匹配目标点
  for (int i = 0; i < pointNum; i++) {
    float lad = 0.0;  //累加前视距离

    float next_x = r_x_[i + 1];
    float next_y = r_y_[i + 1];
    lad += sqrt(pow(next_x - currentPositionX, 2) +
                pow(next_y - currentPositionY, 2));
    // cos(aAngle)判断方向
    float aAngle =
        atan2(next_y - currentPositionY, next_x - currentPositionX) -
        calRPY[2];
    if (lad > preview_dis && cos(aAngle) >= 0) {
      targetIndex = i + 1;
      bestPoints_.push_back(targetIndex);
      break;
    }
  }
  // 取容器中的最大值
  int index = *max_element(bestPoints_.begin(), bestPoints_.end());
  ***************************************************************************************************/

  /**************************************************************************************************/
  // 方案二:通过计算当前坐标和目标轨迹距离，找到一个最小距离的索引号
  int index;
  vector<double> bestPoints_;
  for (int i = 0; i < pointNum; i++) {
    // float lad = 0.0;
    double path_x = r_x_[i];
    double path_y = r_y_[i];
    // 遍历所有路径点和当前位置的距离，保存到数组中
    double lad = sqrt(pow(path_x - currentPositionX, 2) +
                     pow(path_y - currentPositionY, 2));

    bestPoints_.push_back(lad);
  }
  // 找到数组中最小横向距离
  auto smallest = min_element(bestPoints_.begin(), bestPoints_.end());
  // 找到最小横向距离的索引位置
  index = distance(bestPoints_.begin(), smallest);

  int temp_index;
  for (int i = index; i < pointNum; i++) {
    //遍历路径点和预瞄点的距离，从最小横向位置的索引开始
    float dis =
        sqrt(pow(r_y_[index] - r_y_[i], 2) + pow(r_x_[index] - r_x_[i], 2));
    //判断跟预瞄点的距离
    if (dis < preview_dis) {
      temp_index = i;
    } else {
      break;
    }
  }
  index = temp_index;
  /**************************************************************************************************/

  float alpha =
      atan2(r_y_[index] - currentPositionY, r_x_[index] - currentPositionX) -
      currentPositionYaw;

  // 当前点和目标点的距离Id
  float dl = sqrt(pow(r_y_[index] - currentPositionY, 2) +
                  pow(r_x_[index] - currentPositionX, 2));
  // 发布小车运动指令及运动轨迹
  if (dl > 0.5) {
    float theta = atan(2 * Ld * sin(alpha) / dl);
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.3;
    vel_msg.angular.z = theta;
    purepersuit_.publish(vel_msg);
    // 发布小车运动轨迹
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = currentPositionX;
    this_pose_stamped.pose.position.y = currentPositionY;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(theta);
    this_pose_stamped.pose.orientation.x = currentQuaternionX;
    this_pose_stamped.pose.orientation.y = currentQuaternionY;
    this_pose_stamped.pose.orientation.z = currentQuaternionZ;
    this_pose_stamped.pose.orientation.w = currentQuaternionW;

    this_pose_stamped.header.stamp = ros::Time::now();

    this_pose_stamped.header.frame_id = "map";
    path.poses.push_back(this_pose_stamped);
  } else {
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    purepersuit_.publish(vel_msg);
  }
  path_pub_.publish(path);
}

void velocityCall(const carla_msgs::CarlaEgoVehicleStatus& carWaypoint) {
  // carVelocity = carWaypoint.linear.x;
  carVelocity = carWaypoint.velocity;
  preview_dis = k * carVelocity + PREVIEW_DIS;
}

void pointCallback(const nav_msgs::Path &msg) {
  // geometry_msgs/PoseStamped[] poses
  pointNum = msg.poses.size();

  // auto a = msg.poses[0].pose.position.x;
  for (int i = 0; i < pointNum; i++) {
    r_x_.push_back(msg.poses[i].pose.position.x);
    r_y_.push_back(msg.poses[i].pose.position.y);
  }
}

int main(int argc, char **argv) {
  //创建节点
  ros::init(argc, argv, "pure_pursuit");

  //创建节点句柄
  ros::NodeHandle n;
  //创建Publisher，发送经过pure_pursuit计算后的转角及速度
  purepersuit_ = n.advertise<geometry_msgs::Twist>("/smart/cmd_vel", 20);

  path_pub_ = n.advertise<nav_msgs::Path>("rvizpath", 100, true);
  // ros::Rate loop_rate(10);

  path.header.frame_id = "map";
  // 设置时间戳
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  // 设置参考系
  pose.header.frame_id = "map";

  ros::Subscriber splinePath = n.subscribe("/splinepoints", 20, pointCallback);
  ros::Subscriber carVel = n.subscribe("/carla/hero/vehicle_status", 20, velocityCall);
  ros::Subscriber carPose = n.subscribe("/carla/hero/odometry", 20, poseCallback);
  ros::spin();
  return 0;
}
