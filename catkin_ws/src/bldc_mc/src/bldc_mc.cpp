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

void callback_uss(const sensor_msgs::Range::ConstPtr& msg, 
                  const string& sensor_name) {
  if (sensor_name == "us_sensor0") {
    us_sensor0 = msg->range;
  } else if (sensor_name == "us_sensor1") {
    us_sensor1 = msg->range;
  }

  if (!emergency && (us_sensor0 < 0.7 || us_sensor1 < 0.7)) {
    emergency = true;
    prev_speed = current_speed;
    //set_rpm(0);
    //set_duty(0);
  } else if (emergency && (us_sensor0 >= 0.7 && us_sensor1 >= 0.7)) {
    emergency = false;
    //set_rpm(prev_speed);
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

  cmd[1] = rpm >> 24;
  cmd[2] = rpm >> 16;
  cmd[3] = rpm >> 8;
  cmd[4] = rpm;

  cout << "RPM to be set: " << std::dec << rpm << "mA" << endl;
  printf("SET_RPM command to be sent: %02x, %02x, %02x, %02x, %02x\n",
          cmd[0], cmd[1], cmd[2], cmd[3], cmd[4]);

  current_speed = speed;
  return send_packet(cmd, len);
}

/*
  Converts a speed to an RPM.
*/
int speed_to_rpm(float speed) {
  float rpm;
  int MAX_RPM = 15000;
  if (fabsf(speed) < 0.15) {
    rpm = 0;
  } else {
    rpm = speed*MAX_RPM;
  }
  return rpm;
}

/*
  Sends a speed command to the motor controller.
  Parameter speed is given in ampere.
*/
int set_speed(float speed) {
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
}

/*
  Converts a value that is between [-1.0,1.0] to a current.
*/
float speed_to_current(float speed) {
  float current;
  int MAX_CURRENT = 6; // Max speed in current
  if (fabsf(speed) < 0.2) {
    current = 0;
  } else {
    current = speed*MAX_CURRENT;
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
  int16_t position = int16_t(angle*70/22+114);
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
  Returns an int16_t from the buffer, increasing the index by 2 bytes.
  Code from bldc/buffer.c.
*/
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
  int16_t res = ((uint16_t) buffer[*index]) << 8 |
          ((uint16_t) buffer[*index + 1]);
  *index += 2;
  return res;
}

/*
  Returns an int32_t from the buffer, increasing the index by 4 bytes.
  Code from bldc/buffer.c.
*/
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
  int32_t res = ((uint32_t) buffer[*index]) << 24 |
          ((uint32_t) buffer[*index + 1]) << 16 |
          ((uint32_t) buffer[*index + 2]) << 8 |
          ((uint32_t) buffer[*index + 3]);
  *index += 4;
  return res;
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
      ROS_INFO("Publishing MCValues!\n");
      mc_values_pub.publish(msg);
      break;

    default:
      break;
  }

}

void get_values() {
  unsigned char cmd[1] = {0x00}; // 0x00 == COMM_GET_VALUES
  send_packet(cmd, 1);
  recv_packet();
}

void send_alive() {
  uint8_t cmd[1] = {COMM_ALIVE}; // 0x12 == COMM_ALIVE
  send_packet(cmd, 1);
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

  // We pass the topic name to the US-sensor callback function
  bool enable_uss;
  n.param<bool>("enable_uss", enable_uss, true);
  if (enable_uss) {
    ros::Subscriber sub_uss0 = n.subscribe<sensor_msgs::Range>("us_sensor0", 1, 
                               boost::bind(callback_uss, _1, "us_sensor0"));
    ros::Subscriber sub_uss1 = n.subscribe<sensor_msgs::Range>("us_sensor1", 1, 
                               boost::bind(callback_uss, _1, "us_sensor1"));
  }

  ros::Rate loop_rate(1000);
  mc_values_pub = n.advertise<bldc_mc::MCValues>("mc_values", 1000);
  
  int counter = 0;
  while (ros::ok()) {
    // Get values every 50ms
    if (counter % 50 == 0) {
      get_values();
    }
    // Send COMM_ALIVE every 50 ms
    if (counter % 50 == 0) {
      send_alive();
    }
    counter = (counter+1)%1000;

    // But process callbacks every loop
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

