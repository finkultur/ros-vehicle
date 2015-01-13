#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Position.hpp"
#include "Kalman.hpp"
#include <bldc_mc/MCValues.h>
#include "get_imu/IMUData.h"
#include "estimate_position/Position.h"
#include <ctime>

using namespace std;

#define m_per_tick 0.002173913043
#define PI 3.14159265

Kalman* filter;
Position* pos;
ros::Publisher pos_publisher;
ros::Subscriber bldc_listener;
ros::Subscriber imu_listener;

bool recv_first_imu = false;
double start_time;

bool recv_first_bldc = false;
double start_distance;

void pos_callback(const bldc_mc::MCValues::ConstPtr& msg);
