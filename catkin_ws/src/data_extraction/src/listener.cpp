/*add the .h file of any additional topic to this list (compare sensor_msgs/Temperature.h) 
Identify the name of the .h file by executing 'rostopic type <topic>'*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/FluidPressure.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/RCIn.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/Mavlink.h"
#include "sensor_msgs/BatteryState.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iterator>
#include <algorithm>

/*NOTES:
#mavros/mission/waypoints: problem with message types; currently unresolved (not compiling if uncommented)
*/

/*--------------------------------------------------------------------------*/

/*Listener class: contains the subscribers, who store the data as members of the Listener class (in arrays). Data is accessible through access member functions. */
//battery status / solar cell input data: subscribe to battery topic, array cell_voltage
class Listener{
	public:
		//subscribers
		void tempCallback(const sensor_msgs::Temperature::ConstPtr& tmp_clbc); //subscriber imu/temp
		void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_clbc); //subscriber imu/raw
		void mavrosAtmPressureCallback(const sensor_msgs::FluidPressure::ConstPtr& mavros_atm_pres_clbc); //subscriber mavros/atm_pressure
		void mavTempCallback(const sensor_msgs::Temperature::ConstPtr& mavtmp_clbc); //subscriber mavros/imu/temp
		//void missionWaypointCallback(const mavros_msgs::WaypointList::ConstPtr& waypntlst_clbc); //subscriber mavros/mission/waypoints NOTE: SEQUENCE NR NOT AVAILABLE
		void mavRCInCallback(const mavros_msgs::RCIn::ConstPtr& mavRCIn_clbc); //subscriber mavros/rc/in
		void mavStateCallback(const mavros_msgs::State::ConstPtr& mavState_clbc); //subscriber mavros/state
		//access member functions
		void mavLinkCallback(const mavros_msgs::Mavlink::ConstPtr& mavlink_clbc); //subscriber mavlink/from
		void battStateCallback(const sensor_msgs::BatteryState::ConstPtr& battstate_clbc); //subscriber mavros/battery
		std::vector<float> & getTemperature();
		std::vector<int> & getSequenceNrTemp();
		std::vector<std::string> & getTimeTemp();
		std::vector<float> & getAngularVelocityX();
		std::vector<float> & getAngularVelocityY();
		std::vector<float> & getAngularVelocityZ();
		std::vector<float> & getOrientationX();
		std::vector<float> & getOrientationY();
		std::vector<float> & getOrientationZ();
		std::vector<float> & getOrientationW();
		std::vector<float> & getLinearAccelerationX();
		std::vector<float> & getLinearAccelerationY();
		std::vector<float> & getLinearAccelerationZ();
		std::vector<int> & getSequenceNrImu();
		std::vector<std::string> & getTimeImu();
		std::vector<float> & getPressure();
		std::vector<int> & getSequenceNrMavPress();
		std::vector<std::string> & getTimePress();
		std::vector<float> & getMavTemperature();
		std::vector<int> & getMavSequenceNrTemp();
		std::vector<std::string> & getTimeMavTemp();
		//std::vector<float> & getXLat();
		//std::vector<float> & getYLong();
		//std::vector<float> & getZAlt();
		std::vector<int> & getMavRCRSSI();
		std::vector<std::vector<short unsigned int> > & getChannels();
		std::vector<int> & getSequenceNrRCIn();
		std::vector<std::string> & getTimeRCIn();
		std::vector<bool> & getConnected();
		std::vector<bool> & getArmed();
		std::vector<bool> & getGuided();
		std::vector<std::string> & getMode();
		std::vector<int> & getSequenceNrState();
		std::vector<std::string> & getTimeState();
		std::vector<bool> & getIsValidMavlink();
		std::vector<int> & getLenMavlink();
		std::vector<int> & getSeqMavlink();
		std::vector<int> & getSysidMavlink();
		std::vector<int> & getCompidMavlink();
		std::vector<int> & getMsgidMavlink();
		std::vector<int> & getChecksumMavlink();
		std::vector<std::vector<long unsigned int> > & getPayloadMavlink();
		std::vector<int> & getSequenceNrMavlink();
		std::vector<std::string> & getTimeMavlink();
		std::vector<std::vector<float> > & getCellVoltage();
		std::vector<float> & getCurrent();
		std::vector<int> & getSequenceNrBatteryState();
		std::vector<std::string> & getTimeBatteryState();
	protected:
		std::vector<float> temp; //vector of the temperature measurements (/imu/temp)
		std::vector<int> seqnr_temp; //vector of sequence numbers (temperature) (/imu/temp)
		std::vector<std::string> time_temp; //vector of the time stamps (imu/temp)
		std::vector<float> angular_vel_x; //vector of the angular velocity x values (/imu/raw)
		std::vector<float> angular_vel_y; //vector of the angular velocity y values (/imu/raw)
		std::vector<float> angular_vel_z; //vector of the angular velocity z values (/imu/raw)
		std::vector<float> orientation_x; //vector of the orientation x values (/imu/raw)
		std::vector<float> orientation_y; //vector of the orientation y values (/imu/raw)
		std::vector<float> orientation_z; //vector of the orientation z values (/imu/raw)
		std::vector<float> orientation_w; //vector of the orientation w values (/imu/raw)
		std::vector<float> linear_acc_x; //vector of the linear acceleration x values (/imu/raw)
		std::vector<float> linear_acc_y; //vector of the linear acceleration y values (/imu/raw)
		std::vector<float> linear_acc_z; //vector of the linear acceleration z values (/imu/raw)
		std::vector<int> seqnr_imu; //vector of sequence numbers (IMU) (/imu/raw)
		std::vector<std::string> time_imu; //vector of timestamps (/imu/raw)
		std::vector<float> pressure; //vector of the fluid pressure values (mavros/atm_pressure)
		std::vector<int> seqnr_mav_pressure; //vector of the sequence numbers (mavros/atm_pressure)
		std::vector<std::string> time_press; //vector of the time stamps (mavros/atm_pressure)
		std::vector<float> mav_temp; //vector of the temperature measurements (mavros/imu/temp)
		std::vector<int> seqnr_mav_temp; //vector of the sequence numbers (mavros/imu/temp)
		std::vector<std::string> time_mav_temp; //vector of the time stamps (mavros/imu/temp)
		//std::vector<float> x_lat; //vector of the latitude values (mavros/mission/waypoints)
		//std::vector<float> y_long; //vector of the longitude values (mavros/mission/waypoints)
		//std::vector<float> z_alt; //vector of the altitude values (mavros/mission/waypoints)
		std::vector<int> rssi; //vector of the RSSI values (mavros/rc/in)
		std::vector<std::vector<short unsigned int> > channels; //array of the channels vectors (mavros/rc/in)
		std::vector<int> seqnr_mav_rc_in; //vector of the RC sequence numbers (mavros/rc/in)
		std::vector<std::string> time_rc_in; //vector of time stamps (mavros/rc/in)
		std::vector<bool> connected; //vector of connected booleans (mavros/State)
		std::vector<bool> armed; //vector of armed booleans (mavros/State)
		std::vector<bool> guided; //vector of guided booleans (mavros/State)
		std::vector<std::string> mode; //vector of mode (mavros/State)
		std::vector<int> seqnr_state; //vector of the sequence numbers (mavros/state)
		std::vector<std::string> time_state; //vector of the time stamps (mavros/state)
		std::vector<bool> is_valid_mavlink; 
		std::vector<int> len_mavlink;
		std::vector<int> seq_mavlink;
		std::vector<int> sysid_mavlink;
		std::vector<int> compid_mavlink;
		std::vector<int> msgid_mavlink;
		std::vector<int> checksum_mavlink;
		std::vector<std::vector<long unsigned int> > payload_mavlink;
		std::vector<int> seqnr_mavlink;
		std::vector<std::string> time_mavlink;
		std::vector<std::vector<float> > cell_voltage;
		std::vector<float> current;
		std::vector<int> seqnr_battstate;
		std::vector<std::string> time_battstate;
};

void Listener::tempCallback(const sensor_msgs::Temperature::ConstPtr& tmp_clbc){
	float temperature = tmp_clbc->temperature;
	this -> temp.push_back(temperature);
	std_msgs::Header hdr = tmp_clbc -> header;
	int sequence = hdr.seq;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_temp.push_back(time);
	this -> seqnr_temp.push_back(sequence);
}

void Listener::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_clbc){
	float angularx = imu_clbc -> angular_velocity.x;
	float angulary = imu_clbc -> angular_velocity.y;
	float angularz = imu_clbc -> angular_velocity.z;
	float orientationx = imu_clbc -> orientation.x;
	float orientationy = imu_clbc -> orientation.y;
	float orientationz = imu_clbc -> orientation.z;
	float orientationw = imu_clbc -> orientation.w;
	float linearx = imu_clbc -> linear_acceleration.x;
	float lineary = imu_clbc -> linear_acceleration.y;
	float linearz = imu_clbc -> linear_acceleration.z;
	this -> angular_vel_x.push_back(angularx);
	this -> angular_vel_y.push_back(angulary);
	this -> angular_vel_z.push_back(angularz);
	this -> orientation_x.push_back(orientationx);
	this -> orientation_y.push_back(orientationy);
	this -> orientation_z.push_back(orientationz);
	this -> orientation_w.push_back(orientationw);
	this -> linear_acc_x.push_back(linearx);
	this -> linear_acc_y.push_back(lineary);
	this -> linear_acc_z.push_back(linearz);
	std_msgs::Header hdr = imu_clbc -> header;
	int sequence = hdr.seq;
	int secs_imu = hdr.stamp.sec;
	int nanosecs_imu = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs_imu << "." << nanosecs_imu;
	std::string time_imu = ss.str();
	this -> seqnr_imu.push_back(sequence);
	this -> time_imu.push_back(time_imu);
}

void Listener::mavrosAtmPressureCallback(const sensor_msgs::FluidPressure::ConstPtr& mavros_atm_pres_clbc){
	float pressure = mavros_atm_pres_clbc -> fluid_pressure;
	this -> pressure.push_back(pressure);
	std_msgs::Header hdr = mavros_atm_pres_clbc -> header;
	int sequence = hdr.seq;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_press.push_back(time);
	this -> seqnr_mav_pressure.push_back(sequence);
}

void Listener::mavTempCallback(const sensor_msgs::Temperature::ConstPtr& mavtmp_clbc){
	float temperature = mavtmp_clbc->temperature;
	this -> mav_temp.push_back(temperature);
	std_msgs::Header hdr = mavtmp_clbc -> header;
	int sequence = hdr.seq;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_mav_temp.push_back(time);
	this -> seqnr_mav_temp.push_back(sequence);
}

/*void Listener::missionWaypointCallback(const mavros_msgs::WaypointList::ConstPtr& waypntlst_clbc){
	mavros_msgs::WaypointList waypoints = waypntlst_clbc -> waypoints;
	float x_lat = waypntlst_clbc-> waypoints.x_lat;
	float y_long = waypntlst_clbc-> waypoints.y_long;
	float z_alt = waypntlst_clbc-> waypoints.z_alt;
	this -> x_lat.push_back(x_lat);
	this -> y_long.push_back(y_long);
	this -> z_alt.push_back(z_alt);
}*/

void Listener::mavRCInCallback(const mavros_msgs::RCIn::ConstPtr& mavRCIn_clbc){
	int rssi = mavRCIn_clbc -> rssi;
	std::vector<short unsigned int, std::allocator<short unsigned int> > channels = mavRCIn_clbc -> channels;
	int nr_channels = channels.size();
	std::vector<short unsigned int> channel;
	for (int i = 0; i < nr_channels; i++){
		channel.push_back(channels[i]);
	}
	this -> rssi.push_back(rssi);
	this -> channels.push_back(channel);
	std_msgs::Header hdr = mavRCIn_clbc -> header;
	int sequence = hdr.seq;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::string frame_id = hdr.frame_id;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_rc_in.push_back(time);
	this -> seqnr_mav_rc_in.push_back(sequence);
}

void Listener::mavStateCallback(const mavros_msgs::State::ConstPtr& mavState_clbc){
	bool connected = mavState_clbc -> connected;
	bool armed = mavState_clbc -> armed;
	bool guided = mavState_clbc -> guided;
	std::string mode = mavState_clbc -> mode;
	this -> connected.push_back(connected);
	this -> armed.push_back(armed);
	this -> guided.push_back(guided);
	this -> mode.push_back(mode);
	std_msgs::Header hdr = mavState_clbc -> header;
	int sequence = hdr.seq;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_state.push_back(time);
	this -> seqnr_state.push_back(sequence);
}

void Listener::mavLinkCallback(const mavros_msgs::Mavlink::ConstPtr& mavlink_clbc){
	bool is_valid = mavlink_clbc -> is_valid;
	int len = mavlink_clbc -> len;
	int seq = mavlink_clbc -> seq;
	int sysid = mavlink_clbc -> sysid;
	int compid = mavlink_clbc -> compid;
	int msgid = mavlink_clbc -> msgid;
	int checksum = mavlink_clbc -> checksum;
	std::vector<long unsigned int> payload = mavlink_clbc -> payload64;
	this -> is_valid_mavlink.push_back(is_valid);
	this -> len_mavlink.push_back(len);
	this -> seq_mavlink.push_back(seq);
	this -> sysid_mavlink.push_back(sysid);
	this -> compid_mavlink.push_back(compid);
	this -> msgid_mavlink.push_back(msgid);
	this -> checksum_mavlink.push_back(checksum);
	this -> payload_mavlink.push_back(payload);
	std_msgs::Header hdr = mavlink_clbc -> header;
	int sequence = hdr.seq;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_mavlink.push_back(time);
	this -> seqnr_mavlink.push_back(sequence);
}

void Listener::battStateCallback(const sensor_msgs::BatteryState::ConstPtr& battstate_clbc){
	std::vector<float> cell_voltage = battstate_clbc -> cell_voltage;
	float current = battstate_clbc -> current;
	this -> cell_voltage.push_back(cell_voltage);
	this -> current.push_back(current);
	std_msgs::Header hdr = battstate_clbc -> header;
	int sequence = hdr.seq;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_battstate.push_back(time);
	this -> seqnr_battstate.push_back(sequence);
}

std::vector<float> & Listener::getTemperature(){
	return this -> temp;
}

std::vector<int> & Listener::getSequenceNrTemp(){
	return this -> seqnr_temp;
}

std::vector<std::string> & Listener::getTimeTemp(){
	return this -> time_temp;
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

std::vector<float> & Listener::getOrientationX(){
	return this -> orientation_x;
}

std::vector<float> & Listener::getOrientationY(){
	return this -> orientation_y;
}

std::vector<float> & Listener::getOrientationZ(){
	return this -> orientation_z;
}

std::vector<float> & Listener::getOrientationW(){
	return this -> orientation_w;
}

std::vector<float> & Listener::getLinearAccelerationX(){
	return this -> linear_acc_x;
}

std::vector<float> & Listener::getLinearAccelerationY(){
	return this -> linear_acc_y;
}

std::vector<float> & Listener::getLinearAccelerationZ(){
	return this -> linear_acc_z;
}

std::vector<int> & Listener::getSequenceNrImu(){
	return this -> seqnr_imu;
}

std::vector<std::string> & Listener::getTimeImu(){
	return this -> time_imu;
}

std::vector<float> & Listener::getPressure(){
	return this -> pressure;
}

std::vector<int> & Listener::getSequenceNrMavPress(){
	return this -> seqnr_mav_pressure;
}

std::vector<std::string> & Listener::getTimePress(){
	return this -> time_press;
}

std::vector<float> & Listener::getMavTemperature(){
	return this -> mav_temp;
}

std::vector<int> & Listener::getMavSequenceNrTemp(){
	return this -> seqnr_mav_temp;
}

std::vector<std::string> & Listener::getTimeMavTemp(){
	return this -> time_mav_temp;
}

/*std::vector<float> & Listener::getXLat(){
	return this -> x_lat;
}

std::vector<float> & Listener::getYLong(){
	return this -> y_long;
}

std::vector<float> & Listener::getZAlt(){
	return this -> z_alt;
}*/

std::vector<int> & Listener::getMavRCRSSI(){
	return this -> rssi;
}

std::vector<std::vector<short unsigned int> > & Listener::getChannels(){
	return this -> channels;
}

std::vector<int> & Listener::getSequenceNrRCIn(){
	return this -> seqnr_mav_rc_in;
}

std::vector<std::string> & Listener::getTimeRCIn(){
	return this -> time_rc_in;
}

std::vector<bool> & Listener::getConnected(){
	return this -> connected;
}

std::vector<bool> & Listener::getArmed(){
	return this -> armed;
}

std::vector<bool> & Listener::getGuided(){
	return this -> guided;
}

std::vector<std::string> & Listener::getMode(){
	return this -> mode;
}

std::vector<int> & Listener::getSequenceNrState(){
	return this -> seqnr_state;
}

std::vector<std::string> & Listener::getTimeState(){
	return this -> time_state;
}

std::vector<bool> & Listener::getIsValidMavlink(){
	return this -> is_valid_mavlink;
}

std::vector<int> & Listener::getLenMavlink(){
	return this -> len_mavlink;
}

std::vector<int> & Listener::getSeqMavlink(){
	return this -> seq_mavlink;
}

std::vector<int> & Listener::getSysidMavlink(){
	return this -> sysid_mavlink;
}

std::vector<int> & Listener::getCompidMavlink(){
	return this -> compid_mavlink;
}

std::vector<int> & Listener::getMsgidMavlink(){
	return this -> msgid_mavlink;
}

std::vector<int> & Listener::getChecksumMavlink(){
	return this -> checksum_mavlink;
}

std::vector<std::vector<long unsigned int> > & Listener::getPayloadMavlink(){
	return this -> payload_mavlink;
}

std::vector<int> & Listener::getSequenceNrMavlink(){
	return this -> seqnr_mavlink;
}

std::vector<std::string> & Listener::getTimeMavlink(){
	return this -> time_mavlink;
}

std::vector<std::vector<float> > & Listener::getCellVoltage(){
	return this -> cell_voltage;
}

std::vector<float> & Listener::getCurrent(){
	return this -> current;
}

std::vector<int> & Listener::getSequenceNrBatteryState(){
	return this -> seqnr_battstate;
}

std::vector<std::string> & Listener::getTimeBatteryState(){
	return this -> time_battstate;
}

/*--------------------------------------------------------------------------*/

/*TempWriter class: writes the temperature data to csv file (includes a header row)*/

class TempWriter{
	public:
		void writer(std::vector<float> data, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void TempWriter::writer(std::vector<float> data, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "tempdata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "Temperature" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << seqnr[i]  << ";" << data[i]<< std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*IMUWriter class: writes the IMU data to csv file (includes a header row)*/

class IMUWriter{
	public:
		void writer(std::vector<float> ang_vel_x, std::vector<float> ang_vel_y, std::vector<float> ang_vel_z, std::vector<float> ornt_x, std::vector<float> ornt_y, std::vector<float> ornt_z, std::vector<float> ornt_w, std::vector<float> linear_acc_x, std::vector<float> linear_acc_y, std::vector<float> linear_acc_z, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void IMUWriter::writer(std::vector<float> ang_vel_x, std::vector<float> ang_vel_y, std::vector<float> ang_vel_z, std::vector<float> ornt_x, std::vector<float> ornt_y, std::vector<float> ornt_z, std::vector<float> ornt_w, std::vector<float> linear_acc_x, std::vector<float> linear_acc_y, std::vector<float> linear_acc_z, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "imudata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "Angular_velocity_x" << ";" << "Angular_velocity_y" << ";" << "Angular_velocity_z" << ";" << "Orientation_x" << ";" << "Orientation_y" << ";" << "Orientation_z" << ";" << "Orientation_w" << ";" << "Linear_acceleration_x" << ";" << "Linear_acceleration_y" << ";" << "Linear_acceleration_z" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i]  << ";" << seqnr[i] << ";" << ang_vel_x[i] << ";" << ang_vel_y[i] << ";" << ang_vel_z[i] << ";" << ornt_x[i] << ";" << ornt_y[i] << ";" << ornt_z[i] << ";" << ornt_w[i] << ";" << linear_acc_x[i] << ";" << linear_acc_y[i] << ";" << linear_acc_z[i] << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavPressWriter class: writes the fluid pressure data to csv file (includes a header row)*/

class MavPressWriter{
	public:
		void writer(std::vector<float> data, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void MavPressWriter::writer(std::vector<float> data, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "mavpressdata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "Fluid_pressure" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << seqnr[i]  << ";" << data[i]<< std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavTempWriter class: writes the temperature data to csv file (includes a header row)*/

class MavTempWriter{
	public:
		void writer(std::vector<float> data, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void MavTempWriter::writer(std::vector<float> data, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "mavtempdata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "Temperature" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << seqnr[i]  << ";" << data[i]<< std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MissionWaypointWriter class: writes the mission waypoint data to csv file (includes a header row)*/

/*class MissionWaypointWriter{
	public:
		void writer(std::vector<float> x_lat, std::vector<float> y_long, std::vector<float> z_alt, int length);
};

void MissionWaypointWriter::writer(std::vector<float> x_lat, std::vector<float> y_long, std::vector<float> z_alt, int length){
	std::string filename = "mavtempdata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Sequence_nr" << ";" << "Latitude" << ";" << "Longitude" << ";" << "Altitude" << std::endl;
	for (int i = 0; i < length; i++){
		file << "N/A"  << ";" << x_lat[i] << ";" << y_long[i] << ";" << z_alt[i] << std::endl;
	}
}*/

/*--------------------------------------------------------------------------*/

/*RCInWriter class: writes the RC In data to csv file (includes a header row) */

class RCInWriter{
	public:
		void writer(std::vector<int> rssi, std::vector<std::vector<short unsigned int> > channels, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void RCInWriter::writer(std::vector<int> rssi, std::vector<std::vector<short unsigned int> > channels, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "RCIndata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	int nr_channels = channels[0].size();
	std::stringstream tt;
	for (int i = 0; i < nr_channels; i++){
		tt << ";";
		tt << "Channel" << " " << i+1 ;
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "RSSI" << tt.str() << std::endl;
	for (int i = 0; i < length; i++){
		std::vector<short unsigned int> channel = channels[i];
		std::stringstream ss;
		std::copy(channel.begin(), channel.end(), std::ostream_iterator<short unsigned int>(ss, ";"));	
		std::string s = ss.str();
		s = s.substr(0, s.length()-1);		
		file << time[i] << ";" << seqnr[i]  << ";" << rssi[i] << ";" << s << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavStateWriter class: writes the state data to csv file (includes a header row)*/

class MavStateWriter{
	public:
		void writer(std::vector<bool> connected, std::vector<bool> armed, std::vector<bool> guided, std::vector<std::string> mode, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void MavStateWriter::writer(std::vector<bool> connected, std::vector<bool> armed, std::vector<bool> guided, std::vector<std::string> mode, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "mavstatedata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "Connected" << ";" << "Armed" << ";" << "Guided" << ";" << "Mode" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << seqnr[i]  << ";" << connected[i] << ";" << armed[i] << ";" << guided[i] << ";" << mode[i] << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavLinkWriter class: writes the mavlink data to csv file (includes a header row)*/

class MavLinkWriter{
	public:
		void writer(std::vector<bool> is_valid, std::vector<int> len, std::vector<int> seq, std::vector<int> sysid, std::vector<int> compid, std::vector<int> msgid, std::vector<int> checksum, std::vector<std::vector<long unsigned int> > payload, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void MavLinkWriter::writer(std::vector<bool> is_valid, std::vector<int> len, std::vector<int> seq, std::vector<int> sysid, std::vector<int> compid, std::vector<int> msgid, std::vector<int> checksum, std::vector<std::vector<long unsigned int> > payload, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "mavlinkdata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	int len_payload = payload[0].size();
	std::stringstream tt;
	for (int i = 0; i < len_payload; i++){
		tt << ";";
		tt << "payload";
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "is_valid" << ";" << "len" << ";" << "seq" << ";" << "sysid" << ";" << "compid" << ";" << "msgid" << ";" << "checksum" << ";" << tt.str() << std::endl;
	for (int i = 0; i < length; i++){
		std::vector<long unsigned int> payloadx = payload[i];
		std::stringstream ss;
		std::copy(payloadx.begin(), payloadx.end(), std::ostream_iterator<long unsigned int>(ss, ";"));	
		std::string s = ss.str();
		s = s.substr(0, s.length()-1);		
		file << time[i] << ";" << seqnr[i]  << ";" << is_valid[i] << ";" << len[i] << ";" << seq[i] << ";" << sysid[i] << ";" << compid[i] << ";" << msgid[i] << ";" << checksum[i] << ";" << s << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*BattStateWriter class: writes the battery and solar panel data to csv file (includes a header row)*/

class BattStateWriter{
	public:
		void writer(std::vector<std::vector<float> > cell_voltage, std::vector<float> current, int length, std::vector<int> seqnr, std::vector<std::string> time);
};

void BattStateWriter::writer(std::vector<std::vector<float> > cell_voltage, std::vector<float> current, int length, std::vector<int> seqnr, std::vector<std::string> time){
	std::string filename = "batterydata";
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Sequence_nr" << ";" << "Current_in" << ";" << "Current_out_maybe" << ";" << "Solar_voltage" << ";" << "Est_solar_power" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << seqnr[i] << ";" << cell_voltage[i][2] << ";" << current[i] << ";" << cell_voltage[i][1] << ";" << cell_voltage[i][0] << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*CleanData class: contains functions with the purpose to clean data prior to extraction */

class CleanData{
	public:
		std::vector<std::vector<float> > calculateMedian(std::vector<float> data, std::vector<std::string> time); //averages the data to the nearest second (averaging by taking the median of the data points between two seconds)
};

std::vector<std::vector<float> > CleanData::calculateMedian(std::vector<float> data, std::vector<std::string> time){

	//convert the vector with the time stamp to integer for further calculations
	std::vector<double> time_double;
	int ln = time.size();
	for (int i = 0; i < ln; i++){
		std::string x = time[i];
		double y = ::atof(x.c_str());
		time_double.push_back(y);
	}

	//initialising
	std::vector<std::vector<float> > medians;
	int startpos = 0;
	int endpos;
	int tag = 0;

	while (tag != 1){
		for (int i = startpos; i < time_double.size(); i++){
			int firstint = floor(time_double[startpos] + 0.5);
			int nearestint = floor(time_double[i] + 0.5);
			//when the time stamp is rounded to the next second, start calculating the median of the data points until the next rounded second is reached
			if (nearestint != firstint){
				endpos = i;
				std::vector<float> partialdata;
				//access only the data points covering one second of data
				for (int j = startpos; j < endpos; j++){
					partialdata.push_back(data[j]);
				}
				int length = partialdata.size();
				//calculate the median
				float median;
				sort(partialdata.begin(), partialdata.end());
				if (length  % 2 == 0){
					median = (partialdata[length / 2 - 1] + partialdata[length / 2]) / 2;
				}
				else {
					median = partialdata[length / 2];
				}
				//store the median in a vector which only contains one data point per second (data averaged (median) to the nearest second)
				std::vector<float> point;
				point.push_back(median);
				point.push_back(firstint);
				medians.push_back(point);
				startpos = endpos;
			}
			//final iteration is different in order not to loose the last data point
			else if (i == (time_double.size() - 1)){
				endpos = i;
				std::vector<float> partialdata;
				//access only the data points covering one second of data
				for (int j = startpos; j <= endpos; j++){
					partialdata.push_back(data[j]);
				}
				int length = partialdata.size();
				//calculate the median
				float median;
				sort(partialdata.begin(), partialdata.end());
				if (length  % 2 == 0){
					median = (partialdata[length / 2 - 1] + partialdata[length / 2]) / 2;
				}
				else {
					median = partialdata[length / 2];
				}
				//store the median in a vector which only contains one data point per second (data averaged (median) to the nearest second)
				std::vector<float> point;
				point.push_back(median);
				point.push_back(firstint);
				medians.push_back(point);
				tag = 1;
			}
		}
	}	

	return medians;
}

/*--------------------------------------------------------------------------*/

/*main: Runs the subscribers. Upon exiting the programme, it runs the writers. */

int main(int argc, char **argv){
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	Listener lstnr;

	TempWriter tmpwrtr;
	IMUWriter imuwrtr;
	MavPressWriter mavpresswrtr;
	MavTempWriter mavtmpwrtr;
	//MissionWaypointWriter mssnwptwrtr;
	RCInWriter rcinwrtr;
	MavStateWriter stwrtr;
	//MavLinkWriter mvlnkwrtr;
	BattStateWriter bttstwrtr;
	CleanData clndata;

	ros::Subscriber sub1 = n.subscribe("imu/temp", 1000, &Listener::tempCallback, &lstnr);
	ros::Subscriber sub2 = n.subscribe("imu/raw", 1000, &Listener::imuCallback, &lstnr);
	ros::Subscriber sub3 = n.subscribe("mavros/imu/atm_pressure", 1000, &Listener::mavrosAtmPressureCallback, &lstnr);
	ros::Subscriber sub4 = n.subscribe("mavros/imu/temperature", 1000, &Listener::mavTempCallback, &lstnr);
	//ros::Subscriber sub5 = n.subscribe("mavros/mission/waypoints", 1000, &Listener::missionWaypointCallback, &lstnr);
	ros::Subscriber sub6 = n.subscribe("mavros/rc/in", 1000, &Listener::mavRCInCallback, &lstnr);
	ros::Subscriber sub7 = n.subscribe("mavros/state", 1000, &Listener::mavStateCallback, &lstnr);
	//ros::Subscriber sub8 = n.subscribe("mavlink/from", 1000, &Listener::mavLinkCallback, &lstnr);
	ros::Subscriber sub9 = n.subscribe("mavros/battery", 1000, &Listener::battStateCallback, &lstnr);

	ros::spin();

	std::vector<float> tmp = lstnr.getTemperature();
	int length_temp = tmp.size();
	std::vector<int> seqnr_temp = lstnr.getSequenceNrTemp();
	std::vector<std::string> time_temp = lstnr.getTimeTemp();
	std::vector<float> ang_vel_x = lstnr.getAngularVelocityX();
	std::vector<float> ang_vel_z = lstnr.getAngularVelocityY();
	std::vector<float> ang_vel_y = lstnr.getAngularVelocityZ();
	std::vector<float> orientation_x = lstnr.getOrientationX();
	std::vector<float> orientation_y = lstnr.getOrientationY();
	std::vector<float> orientation_z = lstnr.getOrientationZ();
	std::vector<float> orientation_w = lstnr.getOrientationW();
	std::vector<float> linear_acc_x = lstnr.getLinearAccelerationX();
	std::vector<float> linear_acc_y = lstnr.getLinearAccelerationY();
	std::vector<float> linear_acc_z = lstnr.getLinearAccelerationZ();
	int length_imu = ang_vel_x.size();
	std::vector<int> seqnr_angvel = lstnr.getSequenceNrImu();
	std::vector<std::string> time_imu = lstnr.getTimeImu();
	std::vector<float> fluid_pressure = lstnr.getPressure();
	int length_mav_press = fluid_pressure.size();
	std::vector<int> seqnr_fluid_pressure = lstnr.getSequenceNrMavPress();
	std::vector<std::string> time_press = lstnr.getTimePress();
	std::vector<float> mavtmp = lstnr.getMavTemperature();
	int length_mav_temp = mavtmp.size();
	std::vector<int> seqnr_mav_temp = lstnr.getMavSequenceNrTemp();
	std::vector<std::string> time_mav_temp = lstnr.getTimeMavTemp();
	//std::vector<float> x_lat = lstnr.getXLat();
	//std::vector<float> y_long = lstnr.getYLong();
	//std::vector<float> z_alt = lstnr.getZAlt();
	//int length_mssnwpt = x_lat.size();
	std::vector<int> rssi = lstnr.getMavRCRSSI();
	std::vector<std::vector<short unsigned int> > channels = lstnr.getChannels();
	int length_rcin = rssi.size();
	std::vector<int> seqnr_rc_in = lstnr.getSequenceNrRCIn();
	std::vector<std::string> time_rc_in = lstnr.getTimeRCIn();
	std::vector<bool> connected = lstnr.getConnected();
	std::vector<bool> armed = lstnr.getArmed();
	std::vector<bool> guided = lstnr.getGuided();
	std::vector<std::string> mode = lstnr.getMode();
	int length_state = connected.size();
	std::vector<int> seqnr_state = lstnr.getSequenceNrState();
	std::vector<std::string> time_state = lstnr.getTimeState();
	std::vector<bool> is_valid_mavlink = lstnr.getIsValidMavlink();
	//std::vector<int> len_mavlink = lstnr.getLenMavlink();
	//std::vector<int> seq_mavlink = lstnr.getSeqMavlink();
	//std::vector<int> sysid_mavlink = lstnr.getSysidMavlink();
	//std::vector<int> compid_mavlink = lstnr.getCompidMavlink();
	//std::vector<int> msgid_mavlink = lstnr.getMsgidMavlink();
	//std::vector<int> checksum_mavlink = lstnr.getChecksumMavlink();
	//std::vector<std::vector<long unsigned int> > payload_mavlink = lstnr.getPayloadMavlink();
	//int length_mavlink = is_valid_mavlink.size();
	//std::vector<int> seqnr_mavlink = lstnr.getSequenceNrMavlink();
	//std::vector<std::string> time_mavlink = lstnr.getTimeMavlink();
	std::vector<std::vector<float> > cell_voltage = lstnr.getCellVoltage();
	std::vector<float> current = lstnr.getCurrent();
	int length_battstate = current.size();
	std::vector<int> seqnr_battstate = lstnr.getSequenceNrBatteryState();
	std::vector<std::string> time_battstate = lstnr.getTimeBatteryState();

	tmpwrtr.writer(tmp, length_temp, seqnr_temp, time_temp);
	imuwrtr.writer(ang_vel_x, ang_vel_y, ang_vel_z, orientation_x, orientation_y, orientation_z, orientation_w, linear_acc_x, linear_acc_y, linear_acc_z, length_imu, seqnr_angvel, time_imu);
	mavpresswrtr.writer(fluid_pressure, length_mav_press, seqnr_fluid_pressure, time_press);
	mavtmpwrtr.writer(mavtmp, length_mav_temp, seqnr_mav_temp, time_mav_temp);
	//mssnwptwrtr.writer(x_lat, y_long, z_alt, length_mssnwpt);
	rcinwrtr.writer(rssi, channels, length_rcin, seqnr_rc_in, time_rc_in);
	stwrtr.writer(connected, armed, guided, mode, length_state, seqnr_state, time_state);
	//mvlnkwrtr.writer(is_valid_mavlink, len_mavlink, seq_mavlink, sysid_mavlink, compid_mavlink, msgid_mavlink, checksum_mavlink, payload_mavlink, length_mavlink, seqnr_mavlink, time_mavlink);
	bttstwrtr.writer(cell_voltage, current, length_battstate, seqnr_battstate, time_battstate);

	return 0;
}
