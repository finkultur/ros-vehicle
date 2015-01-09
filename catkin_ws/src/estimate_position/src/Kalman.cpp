#include "Kalman.hpp"

Kalman::Kalman() {
  total_traveled_distance = 0;
  total_time = 0;
}

void Kalman::update_model(Position* pos, float distance, float steering_angle) {
	float new_x, new_y, new_heading;
	float delta_distace;

	delta_distace = distance - total_traveled_distance;
	total_traveled_distance = distance;

	// convert from degree to radians
	steering_angle = (steering_angle / 180.0) * PI; 


	/*
		code for the kinetic version of the bicycle model
	*/
	new_x = pos->get_x() + delta_distace * cos(pos->get_heading());
	new_y = pos->get_y() + delta_distace * sin(pos->get_heading());

	// L = car length
	new_heading = pos->get_heading() + (delta_distace * tan(steering_angle))/L;

	pos->set_x(new_x);
	pos->set_y(new_y);
	pos->set_heading(new_heading);
}

void Kalman::update_gyro(Position* pos, float gyro, float t) {

	float delta_time, delta_heading_radian, new_heading;

	delta_time = t - total_time;
	total_time = t;

	/*
		-1.61 is hardcoded drift for the gyro:s z axis
		and convert it to radians
	*/
	delta_heading_radian = (delta_time * gyro) / 180.0 * PI;

	/* 	
		sums up the old heading with the new heading to be able to
		get a absolute heading 
	*/
	new_heading = pos->get_heading() + delta_heading_radian;
	pos->set_heading(new_heading);
}

