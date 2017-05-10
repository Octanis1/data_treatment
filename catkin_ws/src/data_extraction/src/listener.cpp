/*add the .h file of any additional topic to this list (compare sensor_msgs/Temperature.h) */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <iomanip>
#include <fstream>

/*--------------------------------------------------------------------------*/

/*Listener class: contains the subscribers */

class Listener{
	public:
		void tempCallback(const sensor_msgs::Temperature::ConstPtr& tmp_clbc); //subscriber temperature
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_clbc); //subscriber IMU
		std::vector<float> & getTemperature();
		std::vector<int> & getSequenceNrTemp();
		std::vector<float> & getAngularVelocityX();
		std::vector<float> & getAngularVelocityY();
		std::vector<float> & getAngularVelocityZ();
		std::vector<int> & getSequenceNrImu();
	protected:
		std::vector<float> temp; //vector of the temperature measurements
		std::vector<int> seqnr_temp; //vector of sequence numbers (temperature)
		std::vector<float> angular_vel_x; //vector of the angular velocity x values
		std::vector<float> angular_vel_y; //vector of the angular velocity y values
		std::vector<float> angular_vel_z; //vector of the angular velocity z values
		std::vector<int> seqnr_imu; //vector of sequence numbers (IMU)
};

void Listener::tempCallback(const sensor_msgs::Temperature::ConstPtr& tmp_clbc){
	float temperature = tmp_clbc->temperature;
	this -> temp.push_back(temperature);
	std_msgs::Header hdr = tmp_clbc -> header;
	int sequence = hdr.seq;
	this -> seqnr_temp.push_back(sequence);
}

void Listener::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_clbc){
	float angularx = imu_clbc -> angular_velocity.x;
	float angulary = imu_clbc -> angular_velocity.y;
	float angularz = imu_clbc -> angular_velocity.z;
	this -> angular_vel_x.push_back(angularx);
	this -> angular_vel_y.push_back(angulary);
	this -> angular_vel_z.push_back(angularz);
	std_msgs::Header hdr = imu_clbc -> header;
	int sequence = hdr.seq;
	this -> seqnr_imu.push_back(sequence);
}

std::vector<float> & Listener::getTemperature(){
	return this -> temp;
}

std::vector<int> & Listener::getSequenceNrTemp(){
	return this -> seqnr_temp;
}

std::vector<float> & Listener::getAngularVelocityX(){
	return this -> angular_vel_x;
}

std::vector<float> & Listener::getAngularVelocityY(){
	return this -> angular_vel_y;
}

std::vector<float> & Listener::getAngularVelocityZ(){
	return this -> angular_vel_z;
}

std::vector<int> & Listener::getSequenceNrImu(){
	return this -> seqnr_imu;
}


/*--------------------------------------------------------------------------*/

/*TempWriter class: writes the temperature data to csv file (includes a header row)*/

class TempWriter{
	public:
		void writer(std::vector<float> data, int length, std::vector<int> seqnr);
};

void TempWriter::writer(std::vector<float> data, int length, std::vector<int> seqnr){
	std::string filename = "tempdata";
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

/*IMUWriter class: writes the IMU data to csv file (includes a header row)*/

class IMUWriter{
	public:
		void writer(std::vector<float> ang_vel_x, std::vector<float> ang_vel_y, std::vector<float> ang_vel_z, int length, std::vector<int> seqnr);
};

void IMUWriter::writer(std::vector<float> ang_vel_x, std::vector<float> ang_vel_y, std::vector<float> ang_vel_z, int length, std::vector<int> seqnr){
	std::string filename = "imudata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Sequence_nr" << ";" << "Angular_velocity_x" << ";" << "Angular_velocity_y" << ";" << "Angular_velocity_z" << std::endl;
	for (int i = 0; i < length; i++){
		file << seqnr[i]  << ";" << ang_vel_x[i] << ";" << ang_vel_y[i] << ";" << ang_vel_z[i] << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*main: Runs the subscribers. Upon exiting the programme, it runs the writers. */

int main(int argc, char **argv){
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	Listener lstnr;
	TempWriter tmpwrtr;
	IMUWriter imuwrtr;
	ros::Subscriber sub1 = n.subscribe("imu/temp", 1000, &Listener::tempCallback, &lstnr);
	ros::Subscriber sub2 = n.subscribe("imu/raw", 1000, &Listener::imuCallback, &lstnr);

	ros::spin();

	std::vector<float> tmp = lstnr.getTemperature();
	int length_temp = tmp.size();
	std::vector<int> seqnr_temp = lstnr.getSequenceNrTemp();
	std::vector<float> ang_vel_x = lstnr.getAngularVelocityX();
	std::vector<float> ang_vel_z = lstnr.getAngularVelocityY();
	std::vector<float> ang_vel_y = lstnr.getAngularVelocityZ();
	int length_angvel = ang_vel_x.size();
	std::vector<int> seqnr_angvel = lstnr.getSequenceNrImu();	
	tmpwrtr.writer(tmp, length_temp, seqnr_temp);
	imuwrtr.writer(ang_vel_x, ang_vel_y, ang_vel_z, length_angvel, seqnr_angvel);
	

	return 0;
}
