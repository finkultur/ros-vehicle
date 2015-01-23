#ifndef POSITION_HPP
#define POSITION_HPP

class Position {

private:
	float x, y, heading, gyro_heading;

public:

	Position();
	Position(float, float, float);

	float get_x();
	float get_y();
	float get_heading();
	float get_gyro_heading();

	void set_x(float x);
	void set_y(float y);
	void set_heading(float heading_);
  void set_gyro_heading(float gyro_heading_);
};
#endif

