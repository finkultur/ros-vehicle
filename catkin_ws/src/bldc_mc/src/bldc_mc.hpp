#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "bldc_mc/MCValues.h"

#include <unistd.h>
#include <iostream>
#include <sstream>
#include "datatypes.h"
#include "Serial.hpp"
#include "utils.hpp"

void callback_uss(const sensor_msgs::Range::ConstPtr& msg, const std::string &sensor_name);
int init_mc();
int set_speed(float speed);
float speed_to_current(float speed);
int set_rpm(float speed);
int speed_to_rpm(float speed);
int set_steering(float angle, float angle_velocity);
int current_brake(int32_t current_in_mA);
int set_duty(int32_t duty);

int send_packet(const unsigned char *data, int len);
int recv_packet();
void process_packet(const unsigned char *data, int len);

Serial *mc;
ros::Publisher mc_values_pub;

float current_steering_angle;
float current_speed;
float prev_speed;
float us_sensor0;
float us_sensor1;
bool emergency;
