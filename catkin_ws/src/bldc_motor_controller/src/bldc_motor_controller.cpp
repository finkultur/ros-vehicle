#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDrive.h"

#include <iostream>
#include <sstream>
#include "Serial.hpp"

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

int init_mc();
int set_speed(float speed);
int set_steering(float angle, float angle_velocity);
unsigned short crc16(const unsigned char *buf, unsigned int len);
int send_packet(const unsigned char *data, int len);

Serial* mc;

/*
  When a msg is received, we log the msg and set steering angle and speed.
*/
void callback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
  ROS_INFO("Got a message!: \nSteering angle: %f\nSteering angle velocity:%f\nSpeed: %f\nAcceleration: %f\nJerk: %f\n", msg->steering_angle, msg->steering_angle_velocity, msg->speed,msg->acceleration, msg->jerk);

  set_speed(msg->speed);
  set_steering(msg->steering_angle, msg->steering_angle_velocity); 
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

/*
  Sends a speed command to the motor controller.
  Parameter speed is assumed to be given as rpm/1000.
*/
int set_rpm(float speed) {
  const int len = 5; // SET_RPM commands are 5 bytes long
  uint8_t cmd[len] = {0x04, 0x00, 0x00, 0x00, 0x00};
  int32_t rpm = -int(1000*speed);

  //memcpy(&cmd[1], &speed_in_mA, sizeof(uint32_t)); 
  cmd[1] = rpm >> 24;
  cmd[2] = rpm >> 16;
  cmd[3] = rpm >> 8;
  cmd[4] = rpm;

  cout << "RPM to be set: " << std::dec << speed_in_mA << "mA" << endl;
  printf("SET_RPM command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  return send_packet(cmd, len);
}

/*
  Sends a speed command to the motor controller.
  Parameter speed is given in ampere.
*/
int set_speed(float speed) {
  const int len = 5; // Speed commands are 5 bytes long
  uint8_t cmd[len] = {0x02, 0x00, 0x00, 0x00, 0x00};
  int32_t speed_in_mA = -int(1000*speed);

  //memcpy(&cmd[1], &speed_in_mA, sizeof(uint32_t));  
  cmd[1] = speed_in_mA >> 24;
  cmd[2] = speed_in_mA >> 16;
  cmd[3] = speed_in_mA >> 8;
  cmd[4] = speed_in_mA;
 
  cout << "Speed to be set: " << std::dec << speed_in_mA << "mA" << endl; 
  printf("Speed command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  return send_packet(cmd, len);
}

/*
  Sends a steering command to the motor controller,
  which sets the servo offset. Unfortunately we seem to only be able to turn 
  left by doing this. 
*/
/*
int set_steering(float angle, float angle_velocity) {
  const int len = 2;
  uint8_t cmd[len] = {0x06, 0x00};
  uint8_t offset = uint8_t(angle);
  cmd[1] = offset;

  cout << "Angle to be set: " 
       << int(offset) << " in some unit" << endl; 
  printf("Angle Command to be sent: %02x, %02x\n", cmd[0], cmd[1]);
  
  return send_packet(cmd, len);
}
*/

/*
  Sends a steering command to the motor controller, which in turn calls the
  function servo_move().
  TODO: Figure out the units of the parameters.
*/
int set_steering(float angle, float angle_velocity) {
  const int len = 5; // servo_move command is 5 bytes long
  /*
  cmd[0]: what command (0x15 == 21 == COMM_SERVO_MOVE)
  cmd[1]: what servo (assuming servo 0)
  cmd[2-3]: what position (based on angle)
  cmd[4]: what speed (0 to move instantly)
  */
  uint8_t cmd[len] = {0x15, 0x00, 0x00, 0x00, 0x00};

  // The angle we get in is a flow in range[-55, 55]
  /* We do not know why 112 seems to be the standard position,
     but it will keep the wheels in default position. */
  int16_t position = int16_t(angle)+112;
  cmd[2] = position >> 8;
  cmd[3] = position;

  cout << "Position to be set: " << position << endl; 
  printf("Servo move command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);
 
  return send_packet(cmd, len);
}

/*
  Calculates a checksum of the given data.
  From bldc-tool/packetinterface.cpp.
*/
unsigned short crc16(const unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
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

/*
  No comments.
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "BLDC_Motor_Controller_Actuator");
  ros::NodeHandle n;

  if (init_mc() != 0) {
    // ERROR
    return -1;
  }

  ros::Subscriber sub = n.subscribe("motor_controller", 1000, callback);
  ros::spin();

  return 0;
}

