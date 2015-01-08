#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Position.hpp"
#include "Kalman.hpp"
#include <bldc_mc/MCValues.h>
#include "get_imu/IMUData.h"
#include "estimate_position/Position_msg.h"
#include <ctime>

using namespace std;

#define m_per_tick 0.002173913043
#define PI 3.14159265

Kalman* filter;
Position* pos;
ros::Publisher position_publisher;
ros::Subscriber bldc_lister;
ros::Subscriber imu_lister;

bool recv_first_imu = false;
double start_time;

bool recv_first_bldc = false;
double start_distance;

void bldc_callback(const bldc_mc::MCValues::ConstPtr& msg);
