#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Temperature.h"
#include <iostream>
#include <iomanip>
#include <fstream>

class Listener{
	public:
		void cmdRCCallback(const sensor_msgs::Temperature::ConstPtr& cmd_RC);
		std::vector<float> & getTemperature();
		std::vector<int> & getSequenceNr();
	protected:
		std::vector<float> temp;
		std::vector<int> seqnr;
};

void Listener::cmdRCCallback(const sensor_msgs::Temperature::ConstPtr& cmd_RC){
	float temperature = cmd_RC->temperature;
	this -> temp.push_back(temperature);
	std_msgs::Header hdr = cmd_RC -> header;
	int sequence = hdr.seq;
	this -> seqnr.push_back(sequence);

	//ROS_INFO("%f",cmd_RC->temperature);
}

std::vector<float> & Listener::getTemperature(){
	return this -> temp;
}

std::vector<int> & Listener::getSequenceNr(){
	return this -> seqnr;
}

/*--------------------------------------------------------------------------*/

class Writer{
	public:
		void writer(std::vector<float> data, int length, std::vector<int> seqnr);
};

void Writer::writer(std::vector<float> data, int length, std::vector<int> seqnr){
	std::string filename = "testfile";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Sequence_nr" << ";" << "Temperature" << std::endl;
	for (int i = 0; i < length; i++){
		file << seqnr[i]  << ";" << data[i]<< std::endl;
	}
}

/*--------------------------------------------------------------------------*/

int main(int argc, char **argv){
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	Listener lstnr;
	Writer wrtr;
	ros::Subscriber sub = n.subscribe("imu/temp", 1000, &Listener::cmdRCCallback, &lstnr);

	ros::spin();

	std::vector<float> tmp = lstnr.getTemperature();
	int length = tmp.size();
	std::vector<int> seqnr = lstnr.getSequenceNr();
		
	wrtr.writer(tmp, length, seqnr);

	return 0;
}
