#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "Position.hpp"
#include "Kalman.hpp"
#include <bldc_mc/MCValues.h>
#include "get_imu/IMUData.h"
#include "estimate_position/Position.h"
#include <ctime>

using namespace std;

//#define m_per_tick 0.0022371365   // 447 ticks/m
#define m_per_tick 0.00222222222 // 450 ticks/m
//#define m_per_tick 0.002173913043 // 451 ticks/m
//#define m_per_tick 0.00221238938 // 452 ticks/m
//#define m_per_tick 0.002197802197 // 455 ticks/m
#define PI 3.14159265

Kalman* filter;
Position* pos;
ros::Publisher pos_publisher;
ros::Subscriber bldc_listener;
ros::Subscriber imu_listener;

// Just for rviz
ros::Publisher pose_publisher;

bool recv_first_imu = false;
double start_time;

bool recv_first_bldc = false;
double start_distance;

bool recv_mc_values = false;

void pos_callback(const bldc_mc::MCValues::ConstPtr& msg);
