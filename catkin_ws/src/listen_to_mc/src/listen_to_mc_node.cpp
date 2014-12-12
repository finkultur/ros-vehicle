/*
  This node should listen for data from the motor controller,
  and either publish it or make use of the data directly.
*/

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/String.h"

#include <sstream>
#include "Serial.hpp"
//#include "buffer.h"

namespace {
const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
  0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
  0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
  0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
  0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
  0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
  0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
  0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
  0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
  0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
  0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
  0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
  0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
  0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
  0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
  0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
  0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
  0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
  0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
  0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
  0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
  0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
  0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
  0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
  0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };
}

using namespace std;
using namespace boost;

uint32_t get_values();
int recv_packet();
int init_mc(void);
void process_data(const unsigned char *data, int len);
void process_packet(const unsigned char *data, int len);
int send_packet(const unsigned char *data, int len); 
uint16_t crc16(const unsigned char *buf, unsigned int len);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);

Serial* mc;

uint32_t get_values() {
  unsigned char cmd[1] = {0x00};
  send_packet(cmd, 1);
  recv_packet();
  return 0;
}

int recv_packet() {
  std::stringstream ss;

  uint8_t status;
  uint8_t len;
  uint8_t crc_high;
  uint8_t crc_low;

  unsigned char recv[256];

	try {
    status = mc->readChar();
    if (status != 2) {
      return -1;
    }
    len = mc->readChar();
    for (int i=0; i<len; i++) {
      recv[i] = mc->readChar();
    }
    crc_high = mc->readChar();
    crc_low = mc->readChar();

    if (mc->readChar() == 3) {
      if (crc16(recv, len) == ((unsigned short)crc_high << 8
          | (unsigned short)crc_low)) {
        // Packet received!
        process_packet(recv, len);
      }
    }
	} catch(boost::system::system_error& e) {
		ss << "Error: " << e.what() << "\n";
		return -1;
  }
	return 0;
} 

/*
  Initializes the motor controller.
  Just opens the serial port.
*/
int init_mc() {
  std::stringstream ss;

  try {
    mc = new Serial("/dev/ttyACM0", 115200);
  } catch(boost::system::system_error e) {
    std::stringstream ss;
    ss << "Error, cant open Serial Port /dev/ttyACM0!\n"
       << "Error msg: " << e.what();
    ROS_INFO("%s", ss.str().c_str());
    return -1;
  }
  return 0;
}

void process_data(const unsigned char *data, int len) {
  // ...
  // process_packet()   
}

void process_packet(const unsigned char *data, int len) {
  if (!len) {
    return;
  }

  uint8_t packet_id;
  int32_t ind = 0; 

  packet_id = data[0];
  data++;
  len--;

  switch (packet_id) {
    case 0: //0 == COMM_GET_VALUES
      ind = 0;
      printf("TEMP MOS1: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);
      printf("TEMP MOS2: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);
      printf("TEMP MOS3: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);
      printf("TEMP MOS4: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);
      printf("TEMP MOS5: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);
      printf("TEMP MOS6: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);
      printf("TEMP PCB: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);

      printf("Current motor: %f\n", ((double)buffer_get_int32(data, &ind)) / 100.0);
      printf("Current in: %f\n", ((double)buffer_get_int32(data, &ind)) / 100.0);
      printf("Duty now: %f\n", ((double)buffer_get_int16(data, &ind)) / 1000.0);
      printf("RPM: %f\n", ((double)buffer_get_int32(data, &ind)));
      printf("V in: %f\n", ((double)buffer_get_int16(data, &ind)) / 10.0);
      printf("Amp hours: %f\n", ((double)buffer_get_int32(data, &ind)) / 10000.0);
      printf("Amp hours charged: %f\n", ((double)buffer_get_int32(data, &ind)) / 10000.0);
      printf("Watt hours: %f\n", ((double)buffer_get_int32(data, &ind)) / 10000.0);
      printf("Watt hours charged: %f\n", ((double)buffer_get_int32(data, &ind)) / 10000.0);
      printf("Tachometer: %f\n", ((double)buffer_get_int32(data, &ind)));
      printf("Tachometer abs: %f\n", ((double)buffer_get_int32(data, &ind)));
      //printf("Fault code: %i\n", data[ind++]);
       
      break;

    default:
      break;
  } 

}

/*
  Sends a command to the motor controller over serial. Appends a checksum.
  From bldc-tool/packetinterface.cpp.
*/
int send_packet(const unsigned char *data, int len) {
  unsigned char buffer[len + 5]; 
  buffer[0] = 2;
  buffer[1] = len;

  memcpy(buffer + 2, data, len);

  unsigned short crc = crc16(data, len);
  buffer[len + 2] = crc >> 8;
  buffer[len + 3] = crc;
  buffer[len + 4] = 3;

  try {
    mc->writeString(buffer, len+5);
  } catch (boost::system::system_error& e) {
    cout << "Error: " << e.what() << endl;
    return 1;
  }
  return 0;
}


uint16_t crc16(const unsigned char *buf, unsigned int len) {
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8); 
    }   
    return cksum;
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res = ((uint16_t) buffer[*index]) << 8 | 
          ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res = ((uint32_t) buffer[*index]) << 24 |
          ((uint32_t) buffer[*index + 1]) << 16 |
          ((uint32_t) buffer[*index + 2]) << 8 |
          ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
}

int main(int argc, char **argv)
{	
	// init
	ros::init(argc, argv, "listen_for_mc_data");

	// Variables
	ros::NodeHandle n;
	ros::Rate loop_rate(5); // 5 Hz

	//sensor_msgs::Range msg_sensor_0;
	// Sets the right type of the message
	//msg_sensor_0.radiation_type = msg_sensor_0.ULTRASOUND;
	//ros::Publisher SRF08_sensor_0 = n.advertise<sensor_msgs::Range>("SRF08_sensor_0", 1000);

  init_mc();	

  get_values();

	while (ros::ok()) {
		//msg_sensor_0.range = get_reading(sensor_0, 0xE2)/100.0;
		//std::cout << "Distance_0: " << msg_sensor_0.range << "\n";
		//SRF08_sensor_0.publish(msg_sensor_0);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	delete mc;

 	return 0;
}
