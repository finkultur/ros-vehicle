/*
 * dead_reckoning.c
 *
 *  Created on: 29 dec 2011
 *      Author: benjamin
 */
// also see https://www.udacity.com/wiki/cs373/unit-2#kalman-filters

#include "dead_reckoning.h"
#include "mc_driver.h"
#include "nine_dof.h"
#include "uart_ft.h"
#include <stdio.h>
#include <math.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/*
 * Variables
 */
double dr_compass_offset = 0;
double tot_dist_traveled = 0;
double abs_dist_traveled = 0;

double dr_get_steering_angle() {
	return ((double) mc_get_current_servo_pos() - (double)MC_SERVO_CENTER)
			* ((2.0*DR_MC_MAX_SERVO_RAD)
					/ ((double)MC_SERVO_LEFT - (double)MC_SERVO_RIGHT));
}

unsigned char dr_get_servo_pos(double steering_angle) {
	return (unsigned char)(steering_angle/((2.0*DR_MC_MAX_SERVO_RAD)
					/ ((double)MC_SERVO_LEFT - (double)MC_SERVO_RIGHT)) + (double)MC_SERVO_CENTER);
}

/*
 * Get distance traveled since last call of this function.
 *
 *
 * Returns:
 * Distance traveled in mm
 */
double dr_get_travel_distance() {
	const double dist = (double)mc_get_tacho(1) * MC_METER_TACHO_PULSE * 1000.0;
	tot_dist_traveled += dist;
	abs_dist_traveled += fabs(dist);
	return dist;
}

double dr_get_total_travel_distance() {
	return tot_dist_traveled;
}

double dr_get_abs_travel_distance() {
	return abs_dist_traveled;
}

void dr_update_position_angle(double *pos_x, double *pos_y, double *angle) {
	const double steering_angle = dr_get_steering_angle();
	const double travel_distance = dr_get_travel_distance();

	if (travel_distance == 0) {
		return;
	}

	// Avoid division by zero
	if (fabs(steering_angle) < 0.001) {
		*pos_x += cos(*angle) * travel_distance;
		*pos_y += sin(*angle) * travel_distance;
	} else {
		const double turn_rad_rear = DR_AXIS_DISTANCE / tan(steering_angle);
		double turn_rad_front = sqrt(
				DR_AXIS_DISTANCE * DR_AXIS_DISTANCE
						+ turn_rad_rear * turn_rad_rear);

		if (turn_rad_rear < 0) {
			turn_rad_front = -turn_rad_front;
		}
		const double angle_diff = (travel_distance * 2.0) / (turn_rad_rear + turn_rad_front);

		*pos_x += turn_rad_rear * (sin(*angle + angle_diff) - sin(*angle));
		*pos_y += turn_rad_rear * (cos(*angle - angle_diff) - cos(*angle));
		*angle += angle_diff;

		while (*angle > 2.0 * M_PI) {
			*angle -= 2.0 * M_PI;
		}

		while (*angle < 0) {
			*angle += 2.0 * M_PI;
		}

//		dr_compass_corr_angle(angle);
	}
}

double deltaDist()
{
	static double dist = 0;
	double deltadist = dr_get_total_travel_distance()-dist;
	dist = dr_get_total_travel_distance();
	return deltadist;
}

/* Rewritten by Karl and Alexander */
void dr_corr_rcm(double *pos_x, double *pos_y, double *angle, double *sigma, Anchor *anch) {

	// Translate to rcm position on car
	const double diff_x = RCM_OFS_DIST * cos(*angle + RCM_OFS_ANGLE);
	const double diff_y = RCM_OFS_DIST * sin(*angle + RCM_OFS_ANGLE);
	double rcm_x = *pos_x + diff_x;
	double rcm_y = *pos_y + diff_y;

	const double a = rcm_x - (double)anch->posx;		// Distance in x from anchor to car
	const double b = rcm_y - (double)anch->posy;		// Distance in y from anchor to car
	static double lastDist = 0;							// mm totally traveled
	double travelDist = dr_get_abs_travel_distance()-lastDist;// mm traveled since last call of this function
	lastDist = dr_get_abs_travel_distance();			// Update lastDist
//	char debug_buffer[64];								// Buffer for debug printing

	/*
	 * This can cause a division by zero if we believe that we are on an anchor.
	 * However, we should not hit the anchors.
	 */
	double d = sqrt(a * a + b * b);						// The car's distance between car and anchor
	double estdist;										// Estimated distance between car and anchor based on both car and anchors standard deviations and their estimated distances

	if(travelDist == 0)
		*sigma = 10;
	else
		*sigma += dev_per_dist * travelDist;					// The new standard deviation is the old one + how it grows over distance * distance travelled

	double const sigmaCar = *sigma;							// The car's standard deviation
	double const sigmaCar2 = pow(sigmaCar,2);					// The car's standard deviation^2
	double const sigmaAnch2 = pow(anch->deviation,2);			// The anchor's standard deviation^2
	*sigma = sqrt((sigmaCar2 * sigmaAnch2) / (sigmaCar2 + sigmaAnch2));
	estdist = (d * sigmaAnch2 + anch->dist * sigmaCar2) / (sigmaCar2 + sigmaAnch2);

	const double comp_factor = estdist / d;		// comp_factor is used to calculate new x and y coordinates based on the cars location

	rcm_x = a * comp_factor + anch->posx;
	rcm_y = b * comp_factor + anch->posy;

	// Translate back to car position
	*pos_x = rcm_x - diff_x;
	*pos_y = rcm_y - diff_y;

	// Save position to last corrected point
	anch->last_point.x = *pos_x;
	anch->last_point.y = *pos_y;
}


/* Written by Benjamin */
void old_dr_corr_rcm(double *pos_x, double *pos_y, double *angle, Anchor *anch) {
	const double a = *pos_x - (double)anch->posx;		// Distance in x from anchor to car
	const double b = *pos_y - (double)anch->posy;		// Distance in y from anchor to car

	/*
	 * This can cause a division by zero if we believe that we are on an anchor.
	 * However, we should not hit the anchors.
	 */
	double d = sqrt(a * a + b * b);
	double err = d - (double)anch->dist;

	if (err < -DR_RCM_MAX_CORR) {
		err = -DR_RCM_MAX_CORR;
	} else if (err > DR_RCM_MAX_CORR) {
		err = DR_RCM_MAX_CORR;
	}

	const double comp_factor = err / d;

	const double xp = *pos_x - a * comp_factor;			// Measured distance from RCM to car in x
	const double yp = *pos_y - b * comp_factor;			// Measured distance from RCM to car in y

//	double travel_dist = sqrt((*pos_x - anch->last_point.x) * (*pos_x - anch->last_point.x)
//							+ (*pos_y - anch->last_point.y) * (*pos_y - anch->last_point.y));

	*pos_x = *pos_x * (1 - DR_RCM_POS_WEIGHT) + xp * DR_RCM_POS_WEIGHT;
	*pos_y = *pos_y * (1 - DR_RCM_POS_WEIGHT) + yp * DR_RCM_POS_WEIGHT;

	// Save position to last corrected point
	anch->last_point.x = *pos_x;
	anch->last_point.y = *pos_y;

//	if (travel_dist > 5) {
//		double angle_car_rcm;
//
//		if (fabs(a) < 1) {
//			if (b > 0) {
//				angle_car_rcm = M_PI_2 - *angle;
//			} else {
//				angle_car_rcm = -M_PI_2 - *angle;
//			}
//		} else {
//			angle_car_rcm = atan(b/a) - *angle;
//		}
//
//		*angle += err * sin(angle_car_rcm) * DR_RCM_ANGLE_WEIGHT;
//	}
}

void dr_compass_reset_offset() {
	unsigned int tmp;

	if (nine_dof_read_compass(&tmp)) {
		dr_compass_offset = (double)tmp * (M_PI / 180.0);
	}
}

void dr_compass_corr_angle(double *angle) {
	unsigned int tmp;
	if (!nine_dof_read_compass(&tmp)) {
		return;
	}

	double comp_angle = (double)tmp * (M_PI / 180.0) - dr_compass_offset;

	if (comp_angle < 0) {
		comp_angle += 2.0 * M_PI;
	}

	if (fabs(*angle - comp_angle) > M_PI) {
		if (*angle < comp_angle) {
			*angle += 2.0 * M_PI;
		} else {
			comp_angle += 2.0 * M_PI;
		}
	}

	*angle = *angle * (1 - DR_COMPASS_WEIGHT) + comp_angle * DR_COMPASS_WEIGHT;

	if (*angle < 0) {
		*angle += 2.0 * M_PI;
	} else if (*angle > 2.0 * M_PI) {
		*angle -= 2.0 * M_PI;
	}
}





