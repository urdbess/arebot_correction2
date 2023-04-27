#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>


Eigen::Vector2d tmp;
double vel_tmp = 0;
double actual_dist = -1;

void savePose(const nav_msgs::Odometry::ConstPtr& msg) {
    tmp.x() = msg->pose.pose.position.x;
    tmp.y() = msg->pose.pose.position.y;
    vel_tmp = msg->twist.twist.linear.x; 
}

void saveActualDist(const std_msgs::Float64::ConstPtr& msg) {
    actual_dist = msg->data;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "multiplier_radius");
    ros::NodeHandle n;
    ros::Time::init();
    ros::Rate r(2);

    ros::Subscriber ideal_dist_sub = n.subscribe<nav_msgs::Odometry>("/mobile_base_controller/odom", 1, savePose);
    ros::Subscriber actual_dist_sub = n.subscribe<std_msgs::Float64>("/actual_dist", 1, saveActualDist);
    
    Eigen::Vector2d start;
    Eigen::Vector2d end;
    int count = 0;
    double last_v = 0;

    ROS_INFO("please start going straight!");
    //订阅odom距离
    while(n.ok()) {
        ros::spinOnce();
        if (count == 1) {
            start.x() = tmp.x();
            start.y() = tmp.y();
            ROS_INFO("the start pose is:%f, %f", start.x(), start.y());
            count++;
            continue;
        };
        
        //小车停下了
        if(abs(vel_tmp) < 0.01 && last_v > 0.01) {
            end.x() = tmp.x();
            end.y() = tmp.y();
            ROS_INFO("the end pose is:%f, %f", end.x(), end.y());
            break;
        }
        count++;
        last_v = vel_tmp;
        r.sleep();
    }
    //计算理论距离
    Eigen::Vector2d diff = end - start;
    double ideal_dist = diff.norm();
    ROS_INFO("the ideal distance is:%f", ideal_dist);

    // //通过控制台输入实际测量距离
    // double actual_dist = 0;
    // ROS_INFO("please enter the actual distance:");
    // std::cin >> actual_dist;

    //通过话题订阅实际测量距离
    ROS_INFO("please enter the actual distance:");
    while(n.ok()){
        ros::spinOnce();
        if(actual_dist != -1) break;
    }
    

    //计算车轮半径乘数
    double radius_multiplier = actual_dist / ideal_dist;
    ROS_INFO("the wheel's radius multiplier is:%f", radius_multiplier);

    //将乘数写进文件
    //获取乘数文件路径
    std::string filePath;
    n.getParam("multiplier_radius/path", filePath);
    std::ofstream outfile(filePath);
    if(!outfile) {
      ROS_WARN("Failed to open file!");
      return false;
    }
    outfile << radius_multiplier;
    outfile.close();

    return 0;
}