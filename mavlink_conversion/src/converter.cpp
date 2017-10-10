#include "ros/ros.h"
#include <mavros_msgs/mavlink_convert.h> //requires C++ 11, which needs to be activated in the CMakeLists.txt file in the /src directory of the package
#include <mavconn/interface.h>

#include <iostream>

void mavlinkCallback(const mavros_msgs::Mavlink& msg){
	mavlink::mavlink_message_t mavMsg;
	mavros_msgs::mavlink::convert(msg, mavMsg);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "converter");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("mavlink/from", 1000, mavlinkCallback);
	ros::spin();

	std::cout << "test" << std::endl;



	return 0;
}