#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDrive.h"

/*
  No comments.
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "BLDC Motor Controller Actuator");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("motor_controller", 1000, callback);
  ros::spin();

  return 0;
}

/*
  When a msg is received, we log the msg and set steering angle and speed.
*/
void callback(const ackermann_msgs::AckermannDrive::ConstPTr& msg) {
  ROS_INFO("Got a message!: \n
           Steering angle: %f\n
           Steering angle velocity: %f\n
           Speed: %f\n
           Acceleration: %f\n
           Jerk: %f\n",
           msg->steering_angle, msg_steering_angle_velocity, msg->speed,
           msg->acceleration, msg->jerk);

  setSpeed(msg->speed);
  setSteering(msg->steering_angle, msg->steering_angle_velocity); 
}

void setSpeed(float speed) {
  // Not implemented
}

void setSteering(float angle, float angle_velocity) {
  // Not implemented
}
