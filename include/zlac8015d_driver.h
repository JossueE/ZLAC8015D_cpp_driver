#pragma once

#include <modbus/modbus.h>
#include <iostream>
#include <cerrno>
#include <cmath>
#include <utility>
#include <string>
#include <algorithm>
#include <optional>
#include <sstream>
#include <iomanip>

#define M_PI 3.14159265358979323846
#define counts_per_rad (4096 / (2 * M_PI)) // Assuming 4096 counts per revolution

// ---------------------------------------- Common ---------------------------------------- 

#define CONTROL_REG 0x200E
#define OPR_MODE  0x200D
#define L_ACL_TIME  0x2080
#define R_ACL_TIME  0x2081
#define L_DCL_TIME  0x2082
#define R_DCL_TIME  0x2083

// ----------------------------------- For Velocity Mode ----------------------------------- 

#define SET_RPM  0x2088
#define GET_RPM  0x20AB

// ----------------------- For Position Mode - Absolute and Relative ----------------------- 

#define SET_POS  0x208A  // 0x208A (high) 0x208B (low) = Left, 0x208C (high) 0x208D (low) = Right
#define L_MAX_RPM_POS  0x208E
#define R_MAX_RPM_POS  0x208F

// ---------------------------------------- Encoder ---------------------------------------- 
#define ENC_LEFT  0x20A7
#define ENC_RIGHT 0x20A9

// ------------------------ CONTROL GAINS for Velocity control -----------------------------
// LEFT
#define VL_KP 0x203C
#define VL_KI 0x203D
#define VL_KF 0x203E

// RIGHT
#define VR_KP 0x206C
#define VR_KI 0x206D
#define VR_KF 0x206E

// ------------------------ CONTROL GAINS for Position control -----------------------------
// LEFT
#define PL_KP 0x203F
#define PL_KF 0x2040

// RIGHT
#define PR_KP 0x206F
#define PR_KF 0x2070

// ------------------------------------ Troubleshooting ------------------------------------ 

#define FAULTS_LEFT 0x20A5 // <- High 16 bits = Left and Low 16 bits = Right
#define FAULTS_RIGHT 0x20A6 // <- High 16 bits = Left and Low 16 bits = Right
#define READ_ACTUAL_RPM  0x20AB // <- Left and Right
#define READ_ACTUAL_CURRENT 0x20AD // <- Unit = A
#define READ_ACTUAL_SOFTWARE_VERSION 0x20A0 // <- Provided by the manufacturer, can be used to check if the driver is correctly reading registers
#define READ_TEMPERATURE 0x20A4 // <- High 8 bits = Left and Low 8 bits = Right --- Unit = °C

// ------------------------------------ Control Command ------------------------------------ 

#define EMER_STOP  0x05
#define ALRM_CLR 0x06
#define DOWN_TIME 0x07
#define ENABLE 0x08
#define POS_SYNC 0x10
#define POS_L_START 0x11
#define POS_R_START 0x12

/*

FAULT CODES:

NO_FAULT = 0x0000       0
OVER_VOLT = 0x0001      1
UNDER_VOLT = 0x0002     2
OVER_CURR = 0x0004      4
OVER_LOAD = 0x0008      8
CURR_OUT_TOL = 0x0010   16
ENCOD_OUT_TOL = 0x0020  32
MOTOR_BAD = 0x0040      64
REF_VOLT_ERROR = 0x0080 128
EEPROM_ERROR = 0x0100   256
WALL_ERROR = 0x0200     512
HIGH_TEMP = 0x0400      1024
READ_MULT_REG = 0x83
WRITE_SINGLE_REG = 0x86
WRITE_MULTI_REG = 0x89
GENERAL_ERROR = -1

*/
// ----------------------- Structs for Position - Velocity Control ------------------------

struct VelocityMotorGains { int16_t kp, ki, kf; };
struct PositionMotorGains { int16_t kp, kf; };

struct VelocityGains { VelocityMotorGains left, right; };
struct PositionGains { PositionMotorGains left, right; };

class ZLAC8015D
{
public:

	ZLAC8015D(const std::string& port = "/dev/ttyUSB0",
				int baudrate = 115200,
				const std::string& mode = "velocity"
			);
    ~ZLAC8015D();


	bool connect();
	void disconnect();
	
	// General
	int emergency_stop();
	int disable_motor();
    int enable_motor();
	int change_mode(std::optional<std::string> mode = std::nullopt);
	int reset_alarm();

	// Encoder
	std::pair<int32_t, int32_t> get_encoder_count();

	// Control Gains Velocity - Position
	std::optional<VelocityGains> get_control_gains_velocity();
	std::optional<PositionGains> get_control_gains_position();
	int set_control_gains_velocity(VelocityGains &gains);
	int set_control_gains_position(PositionGains &gains);
	

	// For Velocity Mode
	int set_rpm(int16_t L_rpm, int16_t R_rpm); 
	std::pair<int, int> get_rpm();

	// For relative_position and absolute_position Mode
	int set_max_speed_position_mode(int16_t L_rpm, int16_t R_rpm);
	std::pair<int, int> get_max_speed_position_mode();
	int set_sync_position(float L_rad, float R_rad);

	// For Torque Mode
	int set_current(int16_t L_mA, int16_t R_mA);
	std::pair<int, int> get_current();

	// Error and troubleshooting
	std::pair<std::string, std::string> get_error();
	std::string decode_error(uint16_t reg, const char* side);
	std::pair<int, int> get_temperature();
	std::string get_software_version();

	

	// ########## Under Evaluation ##########
	int set_decel_time(uint16_t L_ms, uint16_t R_ms);
	int set_accel_time(uint16_t L_ms, uint16_t R_ms);
	
	


private:
	std::string port_;
	std::string mode_;
    int baudrate_;
    modbus_t* client_ = nullptr;

	int set_mode();
	static bool is_valid_mode(const std::string& m);
	std::string hex16(uint16_t v); 
	

};

