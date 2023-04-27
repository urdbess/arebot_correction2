//测试用话题传递实际测量距离

#include "ros/ros.h"
#include <std_msgs/Float64.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pub_actual_dist");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64>("/actual_dist", 1);
    ros::Time::init();
    ros::Rate r(2);
    std_msgs::Float64 msg;
    msg.data = 1.5;

    while(n.ok()) {
        pub.publish(msg);
        ROS_INFO("publish a dist!");
        r.sleep();
    }
    
    
    
    return 0;
}