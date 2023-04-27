#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <vector>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

double tmp;

void saveImuAngular_v(const sensor_msgs::Imu::ConstPtr& msg) {
    tmp = msg->angular_velocity.z;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pub_actual_angular");
    ros::NodeHandle n;
    ros::Time::init();
    ros::Rate r(5);
    ros::Publisher pub_actual_angular_v = n.advertise<std_msgs::Float64>("/arebot_correction2/mean_angular_v", 1);
    ros::Subscriber sub_actual_angular = n.subscribe<sensor_msgs::Imu>("/arebot_control/imu_data", 1, saveImuAngular_v);

    double last_v = 0;
    std::vector<double> angular_vec;
    //每0.2秒记录一次，只记录角速度稳定的数据，记录若干次

    int times;
    n.getParam("pub_actual_angular/times", times);
    while(n.ok()) {
        ros::spinOnce();
        r.sleep();
        if(tmp < 0.1 ) {
            continue;
        } else {
            angular_vec.push_back(tmp);
        }
        
        if (angular_vec.size() >= times) {
            break;
        }
        
    }

    double mean_angular_v = 0;
    //剔除部分异常数据
    while (true) {
        //计算平均值
        for(int i = 0; i < angular_vec.size(); i++) {
            mean_angular_v += angular_vec[i];
        }
        mean_angular_v /= times;
        //剔除远离平均值的数据
        bool flag = true;
        for(int i = 0; i < angular_vec.size(); i++) {
            if(abs(angular_vec[i] - mean_angular_v) > 0.2 && angular_vec[i] != 0) {
                angular_vec[i] = 0;
                times--;
                flag = false;
            }
        }
        if(flag) break;
    }

    std_msgs::Float64 msg;
    msg.data = mean_angular_v;
    pub_actual_angular_v.publish(msg);
    ROS_INFO("the mean angular velocity is %f", mean_angular_v);

    return 0;
}