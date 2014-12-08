#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"

#include <sstream>
#include "Serial.hpp"

uint32_t get_reading(Serial* serial, uint8_t addr) {
  std::stringstream ss;
	
  uint8_t start_ranging_cmd[] = {0x55, addr, 0x00 ,0x01, 0x51};
	uint8_t read_ranging_cmd[] = {0x55, addr+1, 0x00 ,0x04};
	uint8_t reading[4]; 
	uint32_t res;
	uint32_t range;

	try {
		// Starts the SRF08 at address "addr" to ranging in cm
		serial->writeString(start_ranging_cmd, 5);
		// Read (n)ack
		res = serial->readChar();

		// Read range from SRF08
		do {
			serial->writeString(read_ranging_cmd, 4);
			reading[0] = serial->readChar();
			reading[1] = serial->readChar();
			reading[2] = serial->readChar();
			reading[3] = serial->readChar();
		} while (reading[3] == 0xFF);

	} catch(boost::system::system_error& e) {
		ss <<"Error: "<<e.what()<<"\n";
		return 0;
	}

	range = (reading[2]<<8)|reading[3];
	return range;
} 

int main(int argc, char **argv)
{	
	// init
	ros::init(argc, argv, "SRF08");

	// Variables
	Serial* sensor_0;
	Serial* sensor_1;
	ros::NodeHandle n;
	ros::Rate loop_rate(5); // 5 Hz

	sensor_msgs::Range msg_sensor_0;
	sensor_msgs::Range msg_sensor_1;

	// Sets the right type of the message
	msg_sensor_0.radiation_type = msg_sensor_0.ULTRASOUND;
	msg_sensor_1.radiation_type = msg_sensor_1.ULTRASOUND;

	ros::Publisher SRF08_sensor_0 = n.advertise<sensor_msgs::Range>("SRF08_sensor_0", 1000);
	ros::Publisher SRF08_sensor_1 = n.advertise<sensor_msgs::Range>("SRF08_sensor_1", 1000);
	
	try {
		sensor_1 = new Serial("/dev/ttyUSB0", 19200);
		sensor_2 = new Serial("/dev/ttyUSB1", 19200);	
	} catch(boost::system::system_error& e) {
		std::stringstream ss;
		ss << "Error, cant open Serial USB!\n" << "Error msg: " << e.what();
		ROS_INFO("%s", ss.str().c_str());
	} 

	while (ros::ok()) {
    // These values are divided by 100 to get the range in meters
		msg_sensor_0.range = get_reading(sensor_0, 0xE2)/100.0;
		msg_sensor_1.range = get_reading(sensor_1, 0xE4)/100.0;
		//msg_sensor_1.range = (float)10; 

		//std::cout << "Distance_0: " << msg_sensor_0.range << "\n";
		//std::cout << "Distance_1: " << msg_sensor_1.range << "\n";

		SRF08_sensor_0.publish(msg_sensor_0);
		SRF08_sensor_1.publish(msg_sensor_1);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	delete sensor_0;
	delete sensor_1;

 	return 0;
}

