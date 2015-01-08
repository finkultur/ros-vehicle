/*
	A naive and implemention of a "kalman" filter that
	uses the bicycle model and the gyro to estimate a 
	position
*/

#ifndef KALMAN_HPP
#define KALMAN_HPP

#include "Position.hpp"
#include <cmath>
#include <iostream>

#define L 0.3 // Car length
#define PI 3.14159265

class Kalman {

private:
	float total_traveled_distance;
	double total_time;

public:
	Kalman();

	void update_model(Position* pos, float distance, float steering_angle);
	void update_gyro(Position* pos, float gyro, float delta_t);

};
#endif