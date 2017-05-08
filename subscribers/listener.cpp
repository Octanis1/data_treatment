#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Temperature.h"
#include <iostream>

void cmdRCCallback(const sensor_msgs::Temperature::ConstPtr& cmd_RC){
float temp=cmd_RC->temperature;
float temp_mod = temp + 5;
std::cout << temp_mod << std::endl;
ROS_INFO("%f",cmd_RC->temperature);
    
}

int main(int argc, char **argv)
{
ros::init(argc, argv, "listener");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("imu/temp", 1000, cmdRCCallback);

ros::spin();
return 0;
}
