#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

double ideal_angular_v = 0;
double actual_angular_v = 0;

bool only_show_once = true;
void saveIdealAngular(const geometry_msgs::Twist::ConstPtr& msg) {
    ideal_angular_v = msg->angular.z;
    if (only_show_once) {
      ROS_INFO("ideal_angular_v：%f", ideal_angular_v);  
      only_show_once = false; 
    }
}

void saveActualAngular(const std_msgs::Float64::ConstPtr& msg) {
    actual_angular_v = msg->data;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "multiplier_separation");
    ros::NodeHandle n;

    ros::Subscriber ideal_angular_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, saveIdealAngular);
    ros::Subscriber actual_angular_sub = n.subscribe<std_msgs::Float64>("/arebot_correction2/mean_angular_v", 1, saveActualAngular);
    
    ROS_INFO("please start spinning in place!");

    //订阅理论速度和实际速度
    while(n.ok()) {
        ros::spinOnce();
        if (ideal_angular_v != 0 && actual_angular_v != 0) break;
    }

    //获取乘数文件路径
    std::string filePath;
    n.getParam("multiplier_radius/path", filePath);

    //读取车轮半径乘数
    std::ifstream infile;
    infile.open(filePath, std::ifstream::in | std::ifstream::binary);
    if(!infile) {
      ROS_WARN("Failed to open file!");
      return false;
    }
    std::string s;
    getline(infile, s);
    double radius_multiplier = atof(s.c_str());
    infile.close();

    //计算车轮间距乘数
    double separation_multiplier = radius_multiplier * (ideal_angular_v / actual_angular_v);
    ROS_INFO("the wheel's separation multiplier is:%f", separation_multiplier);

    //将间距乘数写进文件
    std::ofstream outfile(filePath, std::ofstream::app);
    if(!outfile) {
      ROS_WARN("Failed to open file!");
      return false;
    }
    outfile << "\n" << separation_multiplier;
    outfile.close();
    return 0;
}