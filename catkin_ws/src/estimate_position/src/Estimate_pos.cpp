#include "Estimate_pos.hpp"

void pos_callback(const bldc_mc::MCValues::ConstPtr& msg) {
  float distance = (-msg->tachometer) * m_per_tick;
  float steering_angle = msg->current_steering_angle;

  /* 
    Checks if it is the first received message, if so then set 
    start_distance to that value
  */
  if (recv_first_bldc == false) {
    recv_first_bldc = true;
    start_distance = distance;
  }
  
  filter->update_model(pos, distance-start_distance, steering_angle);
}

void imu_callback(const get_imu::IMUData::ConstPtr& msg) {
  double s = msg -> header.stamp.sec;
  double ns = msg -> header.stamp.nsec / 1000000000.0; // convert from ns to s
  double time = (s + ns);

  /* 
    Checks if it is the first recivied message, if so then set 
    start_time to that value
  */
  if (recv_first_imu == false) {
    recv_first_imu = true;
    start_time = time;
  }

  filter->update_gyro(pos, msg->gyroZ, float (time-start_time));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  // Get parameters, set default values (0,0,0) if not found
  double x, y;
  double heading;
  n.param<double>("x", x, 2.015);
  n.param<double>("y", y, -0.585);
  n.param<double>("heading", heading, 0);
  /*n.getParam("x", x);
  n.getParam("y", y);
  n.getParam("heading", heading);*/

  filter = new Kalman();
  pos = new Position((float)x, (float)y, (float)heading);

  // The message to publish
  estimate_position::Position pos_msg;

  // Publishes topic "Position"
  pos_publisher = n.advertise<estimate_position::Position>("Position", 1000);

  // Just for rviz
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "map";
  geometry_msgs::Pose pose;
  geometry_msgs::Point point;
  point.x = point.y = point.z = 0;
  geometry_msgs::Quaternion orientation;
  orientation.x = orientation.y = orientation.z = orientation.w = 0;
  pose.orientation = orientation; 
  // Publishes topic "pose" (just for rviz)
  pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1000);

  // listens to mc_values and imu_data
  bldc_listener = n.subscribe("mc_values", 100, pos_callback);
  imu_listener = n.subscribe("imu_data", 100, imu_callback);
  
  ros::Rate loop_rate(200);

  while (ros::ok()) {
    ros::spinOnce();

    // publishes the position
    pos_msg.x = pos->get_x();
    pos_msg.y = pos->get_y();
    pos_msg.heading = pos->get_heading();
    pos_msg.header.stamp = ros::Time::now();
    pos_publisher.publish(pos_msg);

    // Published the position to rviz
    pose_stamped.header.stamp = pos_msg.header.stamp;
    point.x = pos_msg.x; 
    point.y = pos_msg.y;
    pose.position = point;
    pose_stamped.pose = pose;
    pose_publisher.publish(pose_stamped);

    loop_rate.sleep();
  }
  return 0;
}

