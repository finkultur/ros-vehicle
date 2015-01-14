#include "Position.hpp"

Position::Position() {
	x = 0.0;
	y = 0.0;
	heading = 0.0;
}

Position::Position(float x_, float y_, float heading_) {
		x = x_;
		y = y_;
		heading = heading_;
}

float Position::get_x() {
	return x;
}

float Position::get_y() {
	return y;
}

float Position::get_heading() {
	return heading;
}
	
void Position::set_x(float x_) {
	x = x_;
}

void Position::set_y(float y_) {
	y = y_;
}

void Position::set_heading(float heading_) {
	heading = heading_;
}
