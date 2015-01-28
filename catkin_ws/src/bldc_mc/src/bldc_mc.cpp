/*
  Copyright 2014-2015 Viktor Nilsson and Herman Fransson
  Copyright 2012-2014 Benjamin Vedder benjamin@vedder.se
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "bldc_mc.hpp"

using namespace std;
using namespace boost;

/*
  When a msg is received, we log the msg and set steering angle and speed.
*/
void callback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
  ROS_INFO("Got a message!: \nSteering angle: %f\nSteering angle velocity:%f\nSpeed: %f\nAcceleration: %f\nJerk: %f\n", msg->steering_angle, msg->steering_angle_velocity, msg->speed,msg->acceleration, msg->jerk);

  set_rpm(msg->speed);
  set_steering(msg->steering_angle, msg->steering_angle_velocity);
}

/*
  Callback function for the ultrasonic sensors.
  If a sensor reports a range less than x, the car is stopped and not
  restarted until both sensors report a range greater than x.
*/
void callback_uss(const sensor_msgs::Range::ConstPtr& msg, 
                  const string& sensor_name) {
  if (sensor_name == "us_sensor0") {
    us_sensor0 = msg->range;
  } else if (sensor_name == "us_sensor1") {
    us_sensor1 = msg->range;
  }

  if (current_speed > 0 && !emergency && 
      (us_sensor0 < USS_RANGE || us_sensor1 < USS_RANGE)) {
    ROS_INFO("EMERGENCY!\n");
    emergency = true;
    prev_speed = current_speed;
    current_speed = 0;
    set_duty(0);
  } else if (emergency && (us_sensor0 >= USS_RANGE && us_sensor1 >= USS_RANGE)) {
    emergency = false;
    set_rpm(prev_speed);
  }
} 

/*
  Initializes the motor controller.
  Opens the serial port.
*/
int init_mc() {
  std::stringstream ss;
  
  try {
    mc = new Serial("/dev/bldc_mc", 115200);
  } catch(boost::system::system_error e) {
    ss << "Error, cant open Serial Port!\n"
       << "Error msg: " << e.what();
    ROS_INFO("%s", ss.str().c_str());
    return -1;
  }
  return 0;
}

/*
  Sends an set rpm command to the motor controller.
*/
int set_rpm(float speed) {
  const int len = 5; // SET_RPM commands are 5 bytes long
  uint8_t cmd[len] = {COMM_SET_RPM, 0x00, 0x00, 0x00, 0x00};
  int32_t rpm = -int(speed_to_rpm(speed));

  // If something is in front of the vehicle, only allow reverse
  if (emergency && speed > 0) {
    rpm = 0;
  }

  cmd[1] = rpm >> 24;
  cmd[2] = rpm >> 16;
  cmd[3] = rpm >> 8;
  cmd[4] = rpm;

  cout << "RPM to be set: " << std::dec << rpm << endl;
  printf("SET_RPM command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  current_speed = speed;
  return send_packet(cmd, len);
}

/*
  Converts a speed value that is between [-1.0,1.0] to an RPM.
*/
int speed_to_rpm(float speed) {
  int rpm;
  rpm = speed*MAX_RPM;
  if (abs(rpm) < MIN_RPM) {
    rpm = 0;
  } else if (abs(rpm) > MAX_RPM) {
    rpm = MAX_RPM;
  }
  return rpm;
}

/*
  Sends a speed command to the motor controller.
  Speed is in the range [-1.0,1.0]. Negative values is reverse.
*/
/*int set_current(float speed) {
  const int len = 5; // Speed commands are 5 bytes long
  uint8_t cmd[len] = {COMM_SET_CURRENT, 0x00, 0x00, 0x00, 0x00};
  int32_t speed_in_mA = -int(1000*speed_to_current(speed));

  // If something is in front of the vehicle, only allow reverse
  if (emergency && speed > 0) {
    speed_in_mA = 0;
  }

  cmd[1] = speed_in_mA >> 24;
  cmd[2] = speed_in_mA >> 16;
  cmd[3] = speed_in_mA >> 8;
  cmd[4] = speed_in_mA;
 
  cout << "Speed to be set: " << std::dec << speed_in_mA << "mA" << endl; 
  printf("Speed command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  current_speed = speed;
  return send_packet(cmd, len);
}*/

/*
  Converts a value that is between [-1.0,1.0] to a current.
*/
float speed_to_current(float speed) {
  float current;
  current = speed*MAX_CURRENT;
  if (fabsf(current) < MIN_CURRENT) {
    current = 0;
  }
  if (fabsf(current) > MAX_CURRENT) {
    current = MAX_CURRENT;
  }
  return current;
}

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
  uint8_t cmd[len] = {COMM_SERVO_MOVE, 0x00, 0x00, 0x00, 0x00};

  /* The angle we get in is a float in range[-45, 45]
     We do not know why 114 seems to be the standard position,
     but it will keep the wheels in (almost) default position. 
     The motor controller wants a number between [114-70, 114+70] */
  int16_t position = int16_t(angle*70/22+115);
  cmd[2] = position >> 8;
  cmd[3] = position;

  cout << "Position to be set: " << position << endl; 
  printf("Servo move command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  current_steering_angle = angle;
  return send_packet(cmd, len);
}

int current_brake(int32_t brake_current) {
  const int len = 5; // cmd is 5 bytes long
  /*
    cmd[0]: what command (0x03 == COMM_SET_CURRENT_BRAKE
    cmd[1-4]: current in mA
  */
  uint8_t cmd[len] = {COMM_SET_CURRENT_BRAKE, 0x00, 0x00, 0x00, 0x00};
  cmd[1] = brake_current >> 24;
  cmd[2] = brake_current >> 16;
  cmd[3] = brake_current >> 8;
  cmd[4] = brake_current;

  cout << "Brake current to be set: " << brake_current << "mA" << endl; 
  printf("Servo move command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  return send_packet(cmd, len);
}

/*
  This puts the motor to a full stop.
  Maybe only use this for emergencies.
*/
int set_duty(int32_t duty) {
  const int len = 5; // cmd is 5 bytes long
  /*
    cmd[0]: what command (0x01 == COMM_SET_DUTY
    cmd[1-4]: duty in ?
  */
  uint8_t cmd[len] = {COMM_SET_DUTY, 0x00, 0x00, 0x00, 0x00};
  cmd[1] = duty >> 24;
  cmd[2] = duty >> 16;
  cmd[3] = duty >> 8;
  cmd[4] = duty;

  cout << "Duty to be set: " << duty << endl; 
  printf("Duty command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  return send_packet(cmd, len);
}

/*
  Requests motor controller data from the car.
*/
void get_values() {
  unsigned char cmd[1] = {0x00}; // 0x00 == COMM_GET_VALUES
  send_packet(cmd, 1);
  recv_packet();
}

/*
  Sends an "I'm alive"-message to the car.
  The car is automatically stopped if it has not received anything for 500ms.
*/
void send_alive() {
  uint8_t cmd[1] = {COMM_ALIVE}; // 0x12 == COMM_ALIVE
  send_packet(cmd, 1);
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
  Receives a packet from the motor controller over serial.
  From bldc-tool/packetinterface.cpp.
*/
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
  Processes a packet from the motor controller.
*/
void process_packet(const unsigned char *data, int len) {
  if (!len) {
    return;
  }

  COMM_PACKET_ID packet_id;
  int32_t ind = 0;

  packet_id = (COMM_PACKET_ID) data[0];
  data++;
  len--;

  bldc_mc::MCValues msg;

  switch (packet_id) {
    case COMM_GET_VALUES: // 0x00 == COMM_GET_VALUES
      ind = 0;
      msg.TEMP_MOS1 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS2 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS3 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS4 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS5 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_MOS6 = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.TEMP_PCB = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.current_motor = ((double)buffer_get_int32(data, &ind)) / 100.0;
      msg.current_in = ((double)buffer_get_int32(data, &ind)) / 100.0;
      msg.duty_now = ((double)buffer_get_int16(data, &ind)) / 1000.0;
      msg.rpm = ((double)buffer_get_int32(data, &ind));
      msg.v_in = ((double)buffer_get_int16(data, &ind)) / 10.0;
      msg.amp_hours = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.amp_hours_charged = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.watt_hours = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.watt_hours_charged = ((double)buffer_get_int32(data, &ind)) / 10000.0;
      msg.tachometer = ((double)buffer_get_int32(data, &ind));
      msg.tachometer_abs = ((double)buffer_get_int32(data, &ind));

      // This data is not from the car
      msg.current_steering_angle = current_steering_angle;
      msg.current_speed = current_speed;
      mc_values_pub.publish(msg);
      break;

    default:
      break;
  }
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

  emergency = false;
  us_sensor0 = 0;
  us_sensor1 = 0;

  ros::Subscriber sub_mc_cmds = n.subscribe("mc_cmds", 1, callback);

  bool enable_uss;
  n.param<bool>("bldc_mc/enable_uss", enable_uss, true);
  if (enable_uss) {
    ROS_INFO("Enabling ultrasonic sensors\n");
    // We pass the topic name to the US-sensor callback function
    ros::Subscriber sub_uss0 = n.subscribe<sensor_msgs::Range>("us_sensor0", 1, 
                               boost::bind(callback_uss, _1, "us_sensor0"));
    ros::Subscriber sub_uss1 = n.subscribe<sensor_msgs::Range>("us_sensor1", 1, 
                               boost::bind(callback_uss, _1, "us_sensor1"));
  } else {
    ROS_INFO("Ultrasonic sensors are not enabled.\n");
  }

  ros::Rate loop_rate(1000);
  mc_values_pub = n.advertise<bldc_mc::MCValues>("mc_values", 1000);
  
  int counter = 0;
  while (ros::ok()) {
    // Get values every x ms
    if (counter % GET_VALUES_INTERVAL == 0) {
      get_values();
    }
    // Send COMM_ALIVE every x ms
    // (might not be needed since we're sending COMM_GET_VALUES)
    if (counter % SEND_ALIVE_INTERVAL == 0) {
      send_alive();
    }
    counter = (counter+1)%1000;

    // But process callbacks every loop
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

