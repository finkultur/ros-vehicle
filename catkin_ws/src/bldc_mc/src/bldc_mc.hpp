/*
  Copyright 2014-2015 Viktor Nilsson and Herman Fransson.
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

#define USS_RANGE 0.5
#define MIN_CURRENT 1.2
#define MAX_CURRENT 6
#define MIN_RPM 3500
#define MAX_RPM 15000
#define GET_VALUES_INTERVAL 20
#define SEND_ALIVE_INTERVAL 50

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

