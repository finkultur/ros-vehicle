/*
Software License Agreement (BSD License)

Copyright (c) 2014, Viktor Nilsson and Herman Fransson.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
   copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided
   with the distribution.
 * Neither the name of Willow Garage, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Revision $Id$
*/

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

int main(int argc, char **argv) {	
	ros::init(argc, argv, "SRF08");
	ros::NodeHandle n;
	ros::Rate loop_rate(5); // 5 Hz

	Serial* sensor_0;
	Serial* sensor_1;

	sensor_msgs::Range msg_sensor_0;
	sensor_msgs::Range msg_sensor_1;

	// Sets the right type of the message
	msg_sensor_0.radiation_type = msg_sensor_0.ULTRASOUND;
	msg_sensor_1.radiation_type = msg_sensor_1.ULTRASOUND;

	ros::Publisher us_sensor0 = n.advertise<sensor_msgs::Range>("us_sensor0", 1000);
	ros::Publisher us_sensor1 = n.advertise<sensor_msgs::Range>("us_sensor1", 1000);
	
	try {
		sensor_0 = new Serial("/dev/ussE4", 19200);
	} catch(boost::system::system_error& e) {
		std::stringstream ss;
		ss << "Error, can not open /dev/ussE4!\n" << "Error msg: " << e.what();
		ROS_INFO("%s", ss.str().c_str());
	}
	try {
		sensor_1 = new Serial("/dev/ussE6", 19200);	
	} catch(boost::system::system_error& e) {
		std::stringstream ss;
		ss << "Error, can not open /dev/ussE6!\n" << "Error msg: " << e.what();
		ROS_INFO("%s", ss.str().c_str());
	} 

	while (ros::ok()) {
    // These values are divided by 100 to get the range in meters
		msg_sensor_0.range = get_reading(sensor_0, 0xE4)/100.0;
		msg_sensor_1.range = get_reading(sensor_1, 0xE6)/100.0;

		us_sensor0.publish(msg_sensor_0);
		us_sensor1.publish(msg_sensor_1);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	delete sensor_0;
	delete sensor_1;

 	return 0;
}

