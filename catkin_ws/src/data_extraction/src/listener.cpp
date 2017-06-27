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
		std::vector<std::string> & getTimeImu();
		std::vector<float> & getPressure();
		std::vector<std::string> & getTimePress();
		std::vector<float> & getMavTemperature();
		std::vector<std::string> & getTimeMavTemp();
		//std::vector<float> & getXLat();
		//std::vector<float> & getYLong();
		//std::vector<float> & getZAlt();
		std::vector<int> & getMavRCRSSI();
		std::vector<std::vector<short unsigned int> > & getChannels();
		std::vector<std::string> & getTimeRCIn();
		std::vector<bool> & getConnected();
		std::vector<bool> & getArmed();
		std::vector<bool> & getGuided();
		std::vector<std::string> & getMode();
		std::vector<std::string> & getTimeState();
		std::vector<bool> & getIsValidMavlink();
		std::vector<int> & getLenMavlink();
		std::vector<int> & getSysidMavlink();
		std::vector<int> & getCompidMavlink();
		std::vector<int> & getMsgidMavlink();
		std::vector<int> & getChecksumMavlink();
		std::vector<std::vector<long unsigned int> > & getPayloadMavlink();
		std::vector<std::string> & getTimeMavlink();
		std::vector<std::vector<float> > & getCellVoltage();
		std::vector<float> & getCurrent();
		std::vector<std::string> & getTimeBatteryState();
	protected:
		std::vector<float> temp; //vector of the temperature measurements (/imu/temp)
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
		std::vector<std::string> time_imu; //vector of timestamps (/imu/raw)
		std::vector<float> pressure; //vector of the fluid pressure values (mavros/atm_pressure)
		std::vector<std::string> time_press; //vector of the time stamps (mavros/atm_pressure)
		std::vector<float> mav_temp; //vector of the temperature measurements (mavros/imu/temp)
		std::vector<std::string> time_mav_temp; //vector of the time stamps (mavros/imu/temp)
		//std::vector<float> x_lat; //vector of the latitude values (mavros/mission/waypoints)
		//std::vector<float> y_long; //vector of the longitude values (mavros/mission/waypoints)
		//std::vector<float> z_alt; //vector of the altitude values (mavros/mission/waypoints)
		std::vector<int> rssi; //vector of the RSSI values (mavros/rc/in)
		std::vector<std::vector<short unsigned int> > channels; //array of the channels vectors (mavros/rc/in)
		std::vector<std::string> time_rc_in; //vector of time stamps (mavros/rc/in)
		std::vector<bool> connected; //vector of connected booleans (mavros/State)
		std::vector<bool> armed; //vector of armed booleans (mavros/State)
		std::vector<bool> guided; //vector of guided booleans (mavros/State)
		std::vector<std::string> mode; //vector of mode (mavros/State)
		std::vector<std::string> time_state; //vector of the time stamps (mavros/state)
		std::vector<bool> is_valid_mavlink; 
		std::vector<int> len_mavlink;
		std::vector<int> sysid_mavlink;
		std::vector<int> compid_mavlink;
		std::vector<int> msgid_mavlink;
		std::vector<int> checksum_mavlink;
		std::vector<std::vector<long unsigned int> > payload_mavlink;
		std::vector<std::string> time_mavlink;
		std::vector<std::vector<float> > cell_voltage;
		std::vector<float> current;
		std::vector<std::string> time_battstate;
};

void Listener::tempCallback(const sensor_msgs::Temperature::ConstPtr& tmp_clbc){
	float temperature = tmp_clbc->temperature;
	this -> temp.push_back(temperature);
	std_msgs::Header hdr = tmp_clbc -> header;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_temp.push_back(time);
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
	int secs_imu = hdr.stamp.sec;
	int nanosecs_imu = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs_imu << "." << nanosecs_imu;
	std::string time_imu = ss.str();
	this -> time_imu.push_back(time_imu);
}

void Listener::mavrosAtmPressureCallback(const sensor_msgs::FluidPressure::ConstPtr& mavros_atm_pres_clbc){
	float pressure = mavros_atm_pres_clbc -> fluid_pressure;
	this -> pressure.push_back(pressure);
	std_msgs::Header hdr = mavros_atm_pres_clbc -> header;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_press.push_back(time);
}

void Listener::mavTempCallback(const sensor_msgs::Temperature::ConstPtr& mavtmp_clbc){
	float temperature = mavtmp_clbc->temperature;
	this -> mav_temp.push_back(temperature);
	std_msgs::Header hdr = mavtmp_clbc -> header;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_mav_temp.push_back(time);
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
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::string frame_id = hdr.frame_id;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_rc_in.push_back(time);
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
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_state.push_back(time);
}

void Listener::mavLinkCallback(const mavros_msgs::Mavlink::ConstPtr& mavlink_clbc){
	bool is_valid = mavlink_clbc -> is_valid;
	int len = mavlink_clbc -> len;
	int sysid = mavlink_clbc -> sysid;
	int compid = mavlink_clbc -> compid;
	int msgid = mavlink_clbc -> msgid;
	int checksum = mavlink_clbc -> checksum;
	std::vector<long unsigned int> payload = mavlink_clbc -> payload64;
	this -> is_valid_mavlink.push_back(is_valid);
	this -> len_mavlink.push_back(len);
	this -> sysid_mavlink.push_back(sysid);
	this -> compid_mavlink.push_back(compid);
	this -> msgid_mavlink.push_back(msgid);
	this -> checksum_mavlink.push_back(checksum);
	this -> payload_mavlink.push_back(payload);
	std_msgs::Header hdr = mavlink_clbc -> header;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_mavlink.push_back(time);
}

void Listener::battStateCallback(const sensor_msgs::BatteryState::ConstPtr& battstate_clbc){
	std::vector<float> cell_voltage = battstate_clbc -> cell_voltage;
	float current = battstate_clbc -> current;
	this -> cell_voltage.push_back(cell_voltage);
	this -> current.push_back(current);
	std_msgs::Header hdr = battstate_clbc -> header;
	int secs = hdr.stamp.sec;
	int nanosecs = hdr.stamp.nsec;
	std::stringstream ss;
	ss << secs << "." << nanosecs;
	std::string time = ss.str();
	this -> time_battstate.push_back(time);
}

std::vector<float> & Listener::getTemperature(){
	return this -> temp;
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

std::vector<std::string> & Listener::getTimeImu(){
	return this -> time_imu;
}

std::vector<float> & Listener::getPressure(){
	return this -> pressure;
}

std::vector<std::string> & Listener::getTimePress(){
	return this -> time_press;
}

std::vector<float> & Listener::getMavTemperature(){
	return this -> mav_temp;
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
std::vector<std::string> & Listener::getTimeState(){
	return this -> time_state;
}

std::vector<bool> & Listener::getIsValidMavlink(){
	return this -> is_valid_mavlink;
}

std::vector<int> & Listener::getLenMavlink(){
	return this -> len_mavlink;
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
std::vector<std::string> & Listener::getTimeMavlink(){
	return this -> time_mavlink;
}

std::vector<std::vector<float> > & Listener::getCellVoltage(){
	return this -> cell_voltage;
}

std::vector<float> & Listener::getCurrent(){
	return this -> current;
}

std::vector<std::string> & Listener::getTimeBatteryState(){
	return this -> time_battstate;
}

/*--------------------------------------------------------------------------*/

/*TempWriter class: writes the temperature data to csv file (includes a header row)*/

class TempWriter{
	public:
		void writer(std::string filename, std::vector<float> data, int length, std::vector<double> time);
};

void TempWriter::writer(std::string filename, std::vector<float> data, int length, std::vector<double> time){
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Temperature" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << data[i]<< std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*IMUWriter class: writes the IMU data to csv file (includes a header row)*/

class IMUWriter{
	public:
		void writer(std::string filename, std::vector<float> ang_vel_x, std::vector<float> ang_vel_y, std::vector<float> ang_vel_z, std::vector<float> ornt_x, std::vector<float> ornt_y, std::vector<float> ornt_z, std::vector<float> ornt_w, std::vector<float> linear_acc_x, std::vector<float> linear_acc_y, std::vector<float> linear_acc_z, int length, std::vector<double> time);
};

void IMUWriter::writer(std::string filename, std::vector<float> ang_vel_x, std::vector<float> ang_vel_y, std::vector<float> ang_vel_z, std::vector<float> ornt_x, std::vector<float> ornt_y, std::vector<float> ornt_z, std::vector<float> ornt_w, std::vector<float> linear_acc_x, std::vector<float> linear_acc_y, std::vector<float> linear_acc_z, int length, std::vector<double> time){
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Angular_velocity_x" << ";" << "Angular_velocity_y" << ";" << "Angular_velocity_z" << ";" << "Orientation_x" << ";" << "Orientation_y" << ";" << "Orientation_z" << ";" << "Orientation_w" << ";" << "Linear_acceleration_x" << ";" << "Linear_acceleration_y" << ";" << "Linear_acceleration_z" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i]  << ";" << ang_vel_x[i] << ";" << ang_vel_y[i] << ";" << ang_vel_z[i] << ";" << ornt_x[i] << ";" << ornt_y[i] << ";" << ornt_z[i] << ";" << ornt_w[i] << ";" << linear_acc_x[i] << ";" << linear_acc_y[i] << ";" << linear_acc_z[i] << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavPressWriter class: writes the fluid pressure data to csv file (includes a header row)*/

class MavPressWriter{
	public:
		void writer(std::string filename, std::vector<float> data, int length, std::vector<double> time);
};

void MavPressWriter::writer(std::string filename, std::vector<float> data, int length, std::vector<double> time){
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Fluid_pressure" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << data[i]<< std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavTempWriter class: writes the temperature data to csv file (includes a header row)*/

class MavTempWriter{
	public:
		void writer(std::string filename, std::vector<float> data, int length, std::vector<double> time);
};

void MavTempWriter::writer(std::string filename, std::vector<float> data, int length, std::vector<double> time){
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Temperature" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << data[i]<< std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MissionWaypointWriter class: writes the mission waypoint data to csv file (includes a header row)*/

/*class MissionWaypointWriter{
	public:
		void writer(std::string filename, std::vector<float> x_lat, std::vector<float> y_long, std::vector<float> z_alt, int length);
};

void MissionWaypointWriter::writer(std::string filename, std::vector<float> x_lat, std::vector<float> y_long, std::vector<float> z_alt, int length){
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
		void writer(std::string filename, std::vector<int> rssi, std::vector<std::vector<short unsigned int> > channels, int length, std::vector<double> time);
};

void RCInWriter::writer(std::string filename, std::vector<int> rssi, std::vector<std::vector<short unsigned int> > channels, int length, std::vector<double> time){
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "RSSI" << ";" << "Internal_humidity_percent" << ";" << "External_humidity_percent" << ";" << "External_temperature" << ";" << "External_temperature_C" << ";" << "UV_light" << ";" << "IR_light" << ";" << "Visual_light_Lux" << ";" << "Deep_UV_light" << ";" << "Radioactivity_CPM" << ";" << "Health_effect" << std::endl;
	for (int i = 0; i < length; i++){
		std::vector<short unsigned int> channel = channels[i];
		std::vector<float> channel_float;
		channel_float.push_back(static_cast< float >(channel[0]) / 100);
		channel_float.push_back(static_cast< float >(channel[1]) / 1000);
		channel_float.push_back(static_cast< float >(channel[2]));
		channel_float.push_back((static_cast< float >(channel[3]) - 273.15) / 100);
		channel_float.push_back((0.00391 * static_cast< float >(channel[4]) * static_cast< float >(channel[4]) / 2.44 + static_cast< float >(channel[4]) / 1.56) * 0.0187);
		channel_float.push_back(static_cast< float >(channel[5]));
		channel_float.push_back(static_cast< float >(channel[6]));
		channel_float.push_back(static_cast< float >(channel[7]));
		channel_float.push_back(static_cast< float >(channel[8]));
		channel_float.push_back(static_cast< float >(channel[9]));
		std::stringstream ss;
		std::copy(channel_float.begin(), channel_float.end(), std::ostream_iterator<float>(ss, ";"));	
		std::string s = ss.str();
		s = s.substr(0, s.length()-1);		
		file << time[i] << ";" << rssi[i] << ";" << s << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavStateWriter class: writes the state data to csv file (includes a header row)*/

class MavStateWriter{
	public:
		void writer(std::string filename, std::vector<bool> connected, std::vector<bool> armed, std::vector<bool> guided, std::vector<std::string> mode, int length, std::vector<double> time);
};

void MavStateWriter::writer(std::string filename, std::vector<bool> connected, std::vector<bool> armed, std::vector<bool> guided, std::vector<std::string> mode, int length, std::vector<double> time){
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Connected" << ";" << "Armed" << ";" << "Guided" << ";" << "Mode" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << connected[i] << ";" << armed[i] << ";" << guided[i] << ";" << mode[i] << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*MavLinkWriter class: writes the mavlink data to csv file (includes a header row)*/

class MavLinkWriter{
	public:
		void writer(std::string filename, std::vector<bool> is_valid, std::vector<int> len, std::vector<int> sysid, std::vector<int> compid, std::vector<int> msgid, std::vector<int> checksum, std::vector<std::vector<long unsigned int> > payload, int length, std::vector<double> time);
};

void MavLinkWriter::writer(std::string filename, std::vector<bool> is_valid, std::vector<int> len, std::vector<int> sysid, std::vector<int> compid, std::vector<int> msgid, std::vector<int> checksum, std::vector<std::vector<long unsigned int> > payload, int length, std::vector<double> time){
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
	file << "Time" << ";" << "is_valid" << ";" << "len" << ";" << "sysid" << ";" << "compid" << ";" << "msgid" << ";" << "checksum" << ";" << tt.str() << std::endl;
	for (int i = 0; i < length; i++){
		std::vector<long unsigned int> payloadx = payload[i];
		std::stringstream ss;
		std::copy(payloadx.begin(), payloadx.end(), std::ostream_iterator<long unsigned int>(ss, ";"));	
		std::string s = ss.str();
		s = s.substr(0, s.length()-1);		
		file << time[i] << ";" << is_valid[i] << ";" << len[i] << ";" << sysid[i] << ";" << compid[i] << ";" << msgid[i] << ";" << checksum[i] << ";" << s << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*BattStateWriter class: writes the battery and solar panel data to csv file (includes a header row)*/

class BattStateWriter{
	public:
		void writer(std::string filename, std::vector<std::vector<float> > cell_voltage, std::vector<float> current, int length, std::vector<double> time);
};

void BattStateWriter::writer(std::string filename, std::vector<std::vector<float> > cell_voltage, std::vector<float> current, int length, std::vector<double> time){
	std::ofstream file(filename.c_str());
	if (file.is_open() == false){
		std::cout << "File could not be opened" << std::endl;
		throw;
	}
	file << "Time" << ";" << "Current_in_Ampere" << ";" << "Current_out_Ampere" << ";" << "Solar_voltage_Volt" << ";" << "Est_solar_power_Watt" << std::endl;
	for (int i = 0; i < length; i++){
		file << time[i] << ";" << cell_voltage[i][2] / 1000 << ";" << current[i] / 100 << ";" << cell_voltage[i][1] / 1000 << ";" << (cell_voltage[i][0] / 1000) * (cell_voltage[i][2] / 1000) / 0.9 << std::endl;
	}
}

/*--------------------------------------------------------------------------*/

/*CleanData class: contains functions with the purpose to clean data prior to extraction */

class CleanData{
	public:
		std::vector<std::vector<float> > calculateMedian(std::vector<float> data, std::vector<double> time_double); //averages the data to the nearest second (averaging by taking the median of the data points between two seconds)
		std::vector<double> convertTimeToDouble(std::vector<std::string> time);
};

std::vector<double> CleanData::convertTimeToDouble(std::vector<std::string> time){
	std::vector<double> time_double;
	for (int i = 0; i < time.size(); i++){
		std::string x = time[i];
		double y = ::atof(x.c_str());
		time_double.push_back(y);
	}
	int start_time = time_double[0];
	std::vector<double> time_double_clean;
	for (int i = 0; i < time_double.size(); i++){
		double time = time_double[i];
		time_double_clean.push_back(time - start_time);
	}

	return time_double_clean;
}

std::vector<std::vector<float> > CleanData::calculateMedian(std::vector<float> data, std::vector<double> time_double){

	//initialising
	std::vector<std::vector<float> > medians;
	int startpos = 0;
	int endpos;
	int tag = 0;

	while (tag != 1){
		for (int i = 0; i < time_double.size(); i++){
			int firstint = floor(time_double[startpos] + 0.5);
			int nearestint = floor(time_double[i] + 0.5);
			//when the time stamp is rounded to the next second, start calculating the median of the data points until the next rounded second is reached
			if (nearestint != firstint && nearestint > firstint){
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

	std::cout << "Subscribing to topics and storing/treating the data contained therein. Upon exiting the programme, the data is written to csv files stored in the folder catkin_ws." << std::endl;
	std::cout << " " << std::endl;
	std::cout << "NOTE: Let the code run for at least 1 min to avoid segmentation fault errors due to missing data streams from topics with low publishing rates." << std::endl;

//declare the writer classes
	TempWriter tmpwrtr;
	IMUWriter imuwrtr;
	MavPressWriter mavpresswrtr;
	MavTempWriter mavtmpwrtr;
	//MissionWaypointWriter mssnwptwrtr;
	RCInWriter rcinwrtr;
	MavStateWriter stwrtr;
	//MavLinkWriter mvlnkwrtr;
	BattStateWriter bttstwrtr;

//declare the clean data class
	CleanData clndata;

//run the subscribers
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

//get all the data stored as members of the listener class
	std::vector<float> tmp = lstnr.getTemperature();
	int length_temp = tmp.size();
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
	std::vector<std::string> time_imu = lstnr.getTimeImu();
	std::vector<float> fluid_pressure = lstnr.getPressure();
	int length_mav_press = fluid_pressure.size();
	std::vector<std::string> time_press = lstnr.getTimePress();
	std::vector<float> mavtmp = lstnr.getMavTemperature();
	int length_mav_temp = mavtmp.size();
	std::vector<std::string> time_mav_temp = lstnr.getTimeMavTemp();
	//std::vector<float> x_lat = lstnr.getXLat();
	//std::vector<float> y_long = lstnr.getYLong();
	//std::vector<float> z_alt = lstnr.getZAlt();
	//int length_mssnwpt = x_lat.size();
	std::vector<int> rssi = lstnr.getMavRCRSSI();
	std::vector<std::vector<short unsigned int> > channels = lstnr.getChannels();
	int length_rcin = rssi.size();
	std::vector<std::string> time_rc_in = lstnr.getTimeRCIn();
	std::vector<bool> connected = lstnr.getConnected();
	std::vector<bool> armed = lstnr.getArmed();
	std::vector<bool> guided = lstnr.getGuided();
	std::vector<std::string> mode = lstnr.getMode();
	int length_state = connected.size();
	std::vector<std::string> time_state = lstnr.getTimeState();
	std::vector<bool> is_valid_mavlink = lstnr.getIsValidMavlink();
	//std::vector<int> len_mavlink = lstnr.getLenMavlink();
	//std::vector<int> sysid_mavlink = lstnr.getSysidMavlink();
	//std::vector<int> compid_mavlink = lstnr.getCompidMavlink();
	//std::vector<int> msgid_mavlink = lstnr.getMsgidMavlink();
	//std::vector<int> checksum_mavlink = lstnr.getChecksumMavlink();
	//std::vector<std::vector<long unsigned int> > payload_mavlink = lstnr.getPayloadMavlink();
	//int length_mavlink = is_valid_mavlink.size();
	//std::vector<std::string> time_mavlink = lstnr.getTimeMavlink();
	std::vector<std::vector<float> > cell_voltage = lstnr.getCellVoltage();
	std::vector<float> current = lstnr.getCurrent();
	int length_battstate = current.size();
	std::vector<std::string> time_battstate = lstnr.getTimeBatteryState();

//write the data to file with the writer functions of the writer classes
	std::string filename_tmp = "tempdata";
	std::vector<double> time_temp_double = clndata.convertTimeToDouble(time_temp);
	tmpwrtr.writer(filename_tmp, tmp, length_temp, time_temp_double);
	std::string filename_imu = "imudata";
	std::vector<double> time_imu_double = clndata.convertTimeToDouble(time_imu);
	imuwrtr.writer(filename_imu, ang_vel_x, ang_vel_y, ang_vel_z, orientation_x, orientation_y, orientation_z, orientation_w, linear_acc_x, linear_acc_y, linear_acc_z, length_imu, time_imu_double);
	std::string filename_mav_press = "mavpressdata";
	std::vector<double> time_press_double = clndata.convertTimeToDouble(time_press);
	mavpresswrtr.writer(filename_mav_press, fluid_pressure, length_mav_press, time_press_double);
	std::string filename_mavtemp = "mavtempdata";
	std::vector<double> time_mav_temp_double = clndata.convertTimeToDouble(time_mav_temp);
	mavtmpwrtr.writer(filename_mavtemp, mavtmp, length_mav_temp, time_mav_temp_double);
	//mssnwptwrtr.writer(x_lat, y_long, z_alt, length_mssnwpt);
	std::string filename_rcchannels = "rcchanneldata";
	std::vector<double> time_rc_in_double = clndata.convertTimeToDouble(time_rc_in);
	rcinwrtr.writer(filename_rcchannels, rssi, channels, length_rcin, time_rc_in_double);
	std::string filename_status = "statusdata";
	std::vector<double> time_state_double = clndata.convertTimeToDouble(time_state);
	stwrtr.writer(filename_status, connected, armed, guided, mode, length_state, time_state_double);
	//mvlnkwrtr.writer(is_valid_mavlink, len_mavlink, sysid_mavlink, compid_mavlink, msgid_mavlink, checksum_mavlink, payload_mavlink, length_mavlink, time_mavlink);
	std::string filename_batterystatus = "batterydata";
	std::vector<double> time_battstate_double = clndata.convertTimeToDouble(time_battstate);
	bttstwrtr.writer(filename_batterystatus, cell_voltage, current, length_battstate, time_battstate_double);

//write data averaged to the nearest second by taking the median to file
	std::string filename_median_temp = "tempdata_median";
	std::vector<std::vector<float> > median_tmp_raw = clndata.calculateMedian(tmp, time_temp_double);
	std::vector<float> median_tmp;
	std::vector<double> median_time_temp;
	for (int i = 0; i < median_tmp_raw.size(); i++){
		median_tmp.push_back(median_tmp_raw[i][0]);
		median_time_temp.push_back(median_tmp_raw[i][1]);
	}
	int length_median_tmp = median_tmp.size();
	tmpwrtr.writer(filename_median_temp, median_tmp, length_median_tmp, median_time_temp);
	std::string filename_median_imu = "imudata_median";
	std::vector<std::vector<float> > median_ang_vel_x_raw = clndata.calculateMedian(ang_vel_x, time_imu_double);
	std::vector<std::vector<float> > median_ang_vel_y_raw = clndata.calculateMedian(ang_vel_y, time_imu_double);
	std::vector<std::vector<float> > median_ang_vel_z_raw = clndata.calculateMedian(ang_vel_z, time_imu_double);
	std::vector<std::vector<float> > median_orientation_x_raw = clndata.calculateMedian(orientation_x, time_imu_double);
	std::vector<std::vector<float> > median_orientation_y_raw = clndata.calculateMedian(orientation_y, time_imu_double);
	std::vector<std::vector<float> > median_orientation_z_raw = clndata.calculateMedian(orientation_z, time_imu_double);
	std::vector<std::vector<float> > median_orientation_w_raw = clndata.calculateMedian(orientation_w, time_imu_double);
	std::vector<std::vector<float> > median_linear_acc_x_raw = clndata.calculateMedian(linear_acc_x, time_imu_double);
	std::vector<std::vector<float> > median_linear_acc_y_raw = clndata.calculateMedian(linear_acc_y, time_imu_double);
	std::vector<std::vector<float> > median_linear_acc_z_raw = clndata.calculateMedian(linear_acc_z, time_imu_double);
	std::vector<float> median_ang_vel_x;
	std::vector<float> median_ang_vel_y;
	std::vector<float> median_ang_vel_z;
	std::vector<float> median_orientation_x;
	std::vector<float> median_orientation_y;
	std::vector<float> median_orientation_z;
	std::vector<float> median_orientation_w;
	std::vector<float> median_linear_acc_x;
	std::vector<float> median_linear_acc_y;
	std::vector<float> median_linear_acc_z;
	std::vector<double> median_time_imu;
	for (int i = 0; i < median_ang_vel_x_raw.size(); i++){
		median_ang_vel_x.push_back(median_ang_vel_x_raw[i][0]);
		median_ang_vel_y.push_back(median_ang_vel_y_raw[i][0]);
		median_ang_vel_z.push_back(median_ang_vel_z_raw[i][0]);
		median_orientation_x.push_back(median_orientation_x_raw[i][0]);
		median_orientation_y.push_back(median_orientation_y_raw[i][0]);
		median_orientation_z.push_back(median_orientation_z_raw[i][0]);
		median_orientation_w.push_back(median_orientation_w_raw[i][0]);
		median_linear_acc_x.push_back(median_linear_acc_x_raw[i][0]);
		median_linear_acc_y.push_back(median_linear_acc_y_raw[i][0]);
		median_linear_acc_z.push_back(median_linear_acc_z_raw[i][0]);
		median_time_imu.push_back(median_ang_vel_x_raw[i][1]);
	}
	int length_median_imu = median_ang_vel_x.size();
	imuwrtr.writer(filename_median_imu, median_ang_vel_x, median_ang_vel_y, median_ang_vel_z, median_orientation_x, median_orientation_y, median_orientation_z, median_orientation_w, median_linear_acc_x, median_linear_acc_y, median_linear_acc_z, length_median_imu, median_time_imu);
	std::string filename_median_mavpress = "median_mavpressdata";
	std::vector<std::vector<float> > median_fluid_pressure_raw = clndata.calculateMedian(fluid_pressure, time_press_double);
	std::vector<float> median_fluid_pressure;
	std::vector<double> median_time_press;
	for (int i = 0; i < median_fluid_pressure_raw.size(); i++){
		median_fluid_pressure.push_back(median_fluid_pressure_raw[i][0]);
		median_time_press.push_back(median_fluid_pressure_raw[i][1]);
	}
	int length_median_press = median_fluid_pressure.size();
	mavpresswrtr.writer(filename_median_mavpress, median_fluid_pressure, length_median_press, median_time_press);
	std::string filename_median_mavtemp = "mavtempdata_median";
	std::vector<std::vector<float> > median_mavtmp_raw = clndata.calculateMedian(mavtmp, time_mav_temp_double);
	std::vector<float> median_mavtmp;
	std::vector<double> median_time_mavtemp;
	for (int i = 0; i < median_mavtmp_raw.size(); i++){
		median_mavtmp.push_back(median_mavtmp_raw[i][0]);
		median_time_mavtemp.push_back(median_mavtmp_raw[i][1]);
	}
	int length_median_mavtmp = median_mavtmp.size();
	mavtmpwrtr.writer(filename_median_mavtemp, median_mavtmp, length_median_mavtmp, median_time_mavtemp);
	std::string filename_rcchannels_median = "rcchanneldata_median";
	std::vector<float> channel0;
	std::vector<float> channel1;
	std::vector<float> channel2;
	std::vector<float> channel3;
	std::vector<float> channel4;
	std::vector<float> channel5;
	std::vector<float> channel6;
	std::vector<float> channel7;
	std::vector<float> channel8;
	std::vector<float> channel9;
	for (int i = 0; i < channels.size(); i++){
		channel0.push_back(static_cast< float >(channels[i][0]));
		channel1.push_back(static_cast< float >(channels[i][1]));
		channel2.push_back(static_cast< float >(channels[i][2]));
		channel3.push_back(static_cast< float >(channels[i][3]));
		channel4.push_back(static_cast< float >(channels[i][4]));
		channel5.push_back(static_cast< float >(channels[i][5]));
		channel6.push_back(static_cast< float >(channels[i][6]));
		channel7.push_back(static_cast< float >(channels[i][7]));
		channel8.push_back(static_cast< float >(channels[i][8]));
		channel9.push_back(static_cast< float >(channels[i][9]));
	}
	std::vector<std::vector<float> > median_channel0_raw = clndata.calculateMedian(channel0, time_rc_in_double);
	std::vector<std::vector<float> > median_channel1_raw = clndata.calculateMedian(channel1, time_rc_in_double);
	std::vector<std::vector<float> > median_channel2_raw = clndata.calculateMedian(channel2, time_rc_in_double);
	std::vector<std::vector<float> > median_channel3_raw = clndata.calculateMedian(channel3, time_rc_in_double);
	std::vector<std::vector<float> > median_channel4_raw = clndata.calculateMedian(channel4, time_rc_in_double);
	std::vector<std::vector<float> > median_channel5_raw = clndata.calculateMedian(channel5, time_rc_in_double);
	std::vector<std::vector<float> > median_channel6_raw = clndata.calculateMedian(channel6, time_rc_in_double);
	std::vector<std::vector<float> > median_channel7_raw = clndata.calculateMedian(channel7, time_rc_in_double);
	std::vector<std::vector<float> > median_channel8_raw = clndata.calculateMedian(channel8, time_rc_in_double);
	std::vector<std::vector<float> > median_channel9_raw = clndata.calculateMedian(channel9, time_rc_in_double);
	std::vector<float> median_channel0;
	std::vector<float> median_channel1;
	std::vector<float> median_channel2;
	std::vector<float> median_channel3;
	std::vector<float> median_channel4;
	std::vector<float> median_channel5;
	std::vector<float> median_channel6;
	std::vector<float> median_channel7;
	std::vector<float> median_channel8;
	std::vector<float> median_channel9;
	std::vector<double> median_channels_time;
	for (int i = 0; i < median_channel0_raw.size(); i++){
		median_channel0.push_back(median_channel0_raw[i][0]);
		median_channel1.push_back(median_channel1_raw[i][0]);
		median_channel2.push_back(median_channel2_raw[i][0]);
		median_channel3.push_back(median_channel3_raw[i][0]);
		median_channel4.push_back(median_channel4_raw[i][0]);
		median_channel5.push_back(median_channel5_raw[i][0]);
		median_channel6.push_back(median_channel6_raw[i][0]);
		median_channel7.push_back(median_channel7_raw[i][0]);
		median_channel8.push_back(median_channel8_raw[i][0]);
		median_channel9.push_back(median_channel9_raw[i][0]);
		median_channels_time.push_back(median_channel0_raw[i][1]);
	}
	std::vector<std::vector<short unsigned int> > median_channels;
	std::vector<int> median_rssi;
	int is_nan = NAN;
	for (int i = 0; i < median_channel0.size(); i++){
		std::vector<short unsigned int> median_channel_point;
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel0[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel1[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel2[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel3[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel4[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel5[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel6[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel7[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel8[i]));
		median_channel_point.push_back(static_cast< short unsigned int >(median_channel9[i]));
		median_channels.push_back(median_channel_point);
		median_rssi.push_back(is_nan);
	}
	int length_median_rc_channels = median_channels.size();
	rcinwrtr.writer(filename_rcchannels_median, median_rssi, median_channels, length_median_rc_channels, median_channels_time);
	std::string filename_batterystatus_median = "batterydata_median";
	std::vector<float> current_in;
	std::vector<float> solar_power;
	std::vector<float> est_solar_power;
	for (int i; i < cell_voltage.size(); i++){
		current_in.push_back(cell_voltage[i][2]);
		solar_power.push_back(cell_voltage[i][1]);
		est_solar_power.push_back(cell_voltage[i][0]);
	}
	std::vector<std::vector<float> > median_current_in_raw = clndata.calculateMedian(current_in, time_battstate_double);
	std::vector<std::vector<float> > median_solar_power_raw = clndata.calculateMedian(solar_power, time_battstate_double);
	std::vector<std::vector<float> > median_est_solar_power_raw = clndata.calculateMedian(est_solar_power, time_battstate_double);
	std::vector<std::vector<float> > median_current_raw = clndata.calculateMedian(current, time_battstate_double);
	std::vector<float> median_current_in;
	std::vector<float> median_solar_power;
	std::vector<float> median_est_solar_power;
	std::vector<float> median_current;
	std::vector<double> median_batterystatus_time;
	for (int i = 0; i < median_current_in_raw.size(); i++){
		median_current_in.push_back(median_current_in_raw[i][0]);
		median_solar_power.push_back(median_solar_power_raw[i][0]);
		median_est_solar_power.push_back(median_est_solar_power_raw[i][0]);
		median_current.push_back(median_current_raw[i][0]);
		median_batterystatus_time.push_back(median_current_in_raw[i][1]);
	}
	std::vector<std::vector<float> > median_cell_voltage;
	for (int i = 0; i < median_current_in.size(); i++){
		std::vector<float> cell_voltage_point;
		cell_voltage_point.push_back(median_est_solar_power[i]);
		cell_voltage_point.push_back(median_solar_power[i]);
		cell_voltage_point.push_back(median_current_in[i]);
		median_cell_voltage.push_back(cell_voltage_point);
	}
	int length_median_battery = median_current.size();
	bttstwrtr.writer(filename_batterystatus_median, median_cell_voltage, median_current, length_median_battery, median_batterystatus_time);

std::cout << "Data has been written to file." << std::endl;
	return 0;
}
