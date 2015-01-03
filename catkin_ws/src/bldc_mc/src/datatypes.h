/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * datatypes.h
 *
 *  Created on: 14 sep 2014
 *      Author: benjamin
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

// Data types
typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum {
	PWM_MODE_NONSYNCHRONOUS_HISW = 0, // This mode is not recommended
	PWM_MODE_SYNCHRONOUS, // The recommended and most tested mode
	PWM_MODE_BIPOLAR // Some glitches occasionally, can kill MOSFETs
} mc_pwm_mode;

typedef enum {
	COMM_MODE_INTEGRATE = 0,
	COMM_MODE_DELAY
} mc_comm_mode;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV8302,
	FAULT_CODE_ABS_OVER_CURRENT
} mc_fault_code;

typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_NONE
} mc_control_mode;

typedef struct {
	float cycle_int_limit;
	float cycle_int_limit_running;
	float cycle_int_limit_max;
	float comm_time_sum;
	float comm_time_sum_min_rpm;
	int32_t comms;
	uint32_t time_at_comm;
} mc_rpm_dep_struct;

typedef struct {
	// Switching and drive
	mc_pwm_mode pwm_mode;
	mc_comm_mode comm_mode;
	// Limits
	float l_current_max;
	float l_current_min;
	float l_in_current_max;
	float l_in_current_min;
	float l_abs_current_max;
	float l_min_erpm;
	float l_max_erpm;
	float l_max_erpm_fbrake;
	float l_min_vin;
	float l_max_vin;
	bool l_slow_abs_current;
	bool l_rpm_lim_neg_torque;
	float l_temp_fet_start;
	float l_temp_fet_end;
	float l_temp_motor_start;
	float l_temp_motor_end;
	// Overridden limits (Computed during runtime)
	float lo_current_max;
	float lo_current_min;
	float lo_in_current_max;
	float lo_in_current_min;
	// Sensorless
	bool sl_is_sensorless;
	float sl_min_erpm;
	float sl_min_erpm_cycle_int_limit;
	float sl_cycle_int_limit;
	float sl_cycle_int_limit_high_fac;
	float sl_cycle_int_rpm_br;
	float sl_bemf_coupling_k;
	// Hall sensor
	int8_t hall_dir;
	int8_t hall_fwd_add;
	int8_t hall_rev_add;
	// Speed PID
	float s_pid_kp;
	float s_pid_ki;
	float s_pid_kd;
	float s_pid_min_rpm;
	// Current controller
	float cc_startup_boost_duty;
	float cc_min_current;
	float cc_gain;
	// Misc
	int32_t m_fault_stop_time_ms;
} mc_configuration;

// Applications to use
typedef enum {
	APP_NONE = 0,
	APP_PPM,
	APP_UART,
	APP_PPM_UART,
	APP_NUNCHUK,
	APP_CUSTOM
} app_use;

// PPM control types
typedef enum {
	PPM_CTRL_TYPE_NONE = 0,
	PPM_CTRL_TYPE_CURRENT,
	PPM_CTRL_TYPE_CURRENT_NOREV,
	PPM_CTRL_TYPE_CURRENT_NOREV_BRAKE,
	PPM_CTRL_TYPE_DUTY,
	PPM_CTRL_TYPE_DUTY_NOREV,
	PPM_CTRL_TYPE_PID,
	PPM_CTRL_TYPE_PID_NOREV
} ppm_control_type;

// Nunchuk control types
typedef enum {
	CHUK_CTRL_TYPE_NONE = 0,
	CHUK_CTRL_TYPE_CURRENT,
	CHUK_CTRL_TYPE_CURRENT_NOREV
} chuk_control_type;

typedef struct {
	// Settings
	uint32_t timeout_msec;
	float timeout_brake_current;

	// Application to use
	app_use app_to_use;

	// PPM application settings
	ppm_control_type app_ppm_ctrl_type;
	float app_ppm_pid_max_erpm;
	float app_ppm_hyst;
	float app_ppm_pulse_start;
	float app_ppm_pulse_width;
	float app_ppm_rpm_lim_start;
	float app_ppm_rpm_lim_end;

	// UART application settings
	uint32_t app_uart_baudrate;

	// Nunchuk
	chuk_control_type app_chuk_ctrl_type;
	float app_chuk_hyst;
	float app_chuk_rpm_lim_start;
	float app_chuk_rpm_lim_end;
} app_configuration;

// Communication commands
typedef enum {
	COMM_GET_VALUES = 0,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_DETECT,
	COMM_SET_SERVO_OFFSET,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_CHUK,
  COMM_SERVO_MOVE,
  COMM_SERVO_MOVE_WITHIN_TIME,
  COMM_SERVO_RESET_POS
} COMM_PACKET_ID;

// CAN commands
typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM
} CAN_PACKET_ID;

// Logged fault data
typedef struct {
	mc_fault_code fault;
	float current;
	float current_filtered;
	float voltage;
	float duty;
	float rpm;
	int tacho;
	int tim_pwm_cnt;
	int tim_samp_cnt;
	int comm_step;
	float temperature;
} fault_data;

// External LED state
typedef enum {
	LED_EXT_OFF = 0,
	LED_EXT_NORMAL,
	LED_EXT_BRAKE,
	LED_EXT_TURN_LEFT,
	LED_EXT_TURN_RIGHT,
	LED_EXT_BRAKE_TURN_LEFT,
	LED_EXT_BRAKE_TURN_RIGHT
} LED_EXT_STATE;

#endif /* DATATYPES_H_ */
