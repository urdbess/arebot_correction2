#arebot_correction2
---
##介绍
arebot_correction2为ROS标定车轮半径及间距的功能包；  
必须先标定车轮半径再标定车轮间距！

##运行过程
首先启动小车；  
###标定车轮半径乘数
运行半径标定节点:  
`roslaunch arebot_correction2 radius_multiplier.launch`  
然后控制小车直线走一段距离  
通过话题传递实际测量距离（实测距离在pub_dist_for_test中发布）:  
`rosrun arebot_correction2 pub_dist_for_test`  
###标定车轮间距乘数
运行间距标定节点（其中参数/arebot_correction2/times为收集imu角速度的数据量）:  
`roslaunch arebot_correction2 separation_multiplier.launch`  
然后控制小车原地转圈一段时间后即可。  

运行完毕后，车轮半径和间距的乘数依次保存在arebot_correction2/multiplier.txt中；