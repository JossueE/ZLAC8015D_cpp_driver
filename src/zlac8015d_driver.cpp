#include "zlac8015d_driver.h"

bool ZLAC8015D::is_valid_mode(const std::string& m) {
  return m == "velocity" ||
         m == "relative_position" ||
         m == "absolute_position" ||
         m == "torque";
}

ZLAC8015D::ZLAC8015D(const std::string& port, int baudrate, const std::string& mode): port_(port), baudrate_(baudrate), mode_(mode){

  if (!is_valid_mode(mode_)) {
    std::cerr << "Invalid Mode Selected: " << mode_ << "\n"
              << "Available modes: velocity, relative_position, absolute_position, torque\n";
    return;
  }
}

ZLAC8015D::~ZLAC8015D(){
  disconnect();
}

bool ZLAC8015D::connect(){

  /**
   * @brief Establishes a Modbus RTU connection to the ZLAC8015D driver.
   *
   * Creates and connects a libmodbus RTU context using the configured serial
   * port and baudrate, sets slave ID = 1, then applies the initial operation
   * mode and enables the motor. On failure, cleans up resources and returns false.
   *
   * @return true if already connected or connection/setup succeeds; false otherwise.
   */
  
  if (client_) {
    std::cout << "You have a previous connection, we recommended to reconnect, disconnecting first." << std::endl;
    return true;
  }
  std::cout << "Trying port: " << port_ << " baudrate: " << baudrate_ << "\n";
  client_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
  if (!client_) {
    std::cout << "Error creating context Modbus RTU (modbus_new_rtu fail)." << std::endl;
    return false;
  }
  modbus_set_slave(client_, 1);
  if (modbus_connect(client_) == -1) {
    std::cerr << "modbus_connect failed: " << modbus_strerror(errno) << "\n";
    modbus_free(client_);
    client_ = nullptr;
    return false;
  }

  // Aplica el modo inicial
  if (set_mode() == -1 || enable_motor() == -1) {
    disconnect();
    return false;
  }
  
  std::cout << "Connecting to ZLAC8015D by: port " << port_.c_str() << ", baudrate "<< baudrate_ << std::endl;
  return true;
}

void ZLAC8015D::disconnect(){

  /**
   * @brief Disconnect Modbus RTU connection to the ZLAC8015D driver.
   *
   * If a connection exists, sends an emergency_stop to the driver, closes the Modbus and frees the context. 
   * If no connection exists, simply logs that there was nothing to disconnect.
   *
   * @return --
   * @note After calling disconnect(), the emergency_stop command disable the motor and tu use again the driver you need to call connect() - enable_motor() again.
   */

  if (client_) {
    emergency_stop();
    modbus_close(client_);
    modbus_free(client_);
    client_ = nullptr;
    std::cout << "Successfully disconnected\n";
  } else {
    std::cout << "No active connection to disconnect\n";
  }
}

int ZLAC8015D::emergency_stop(){

  /**
   * @brief Stop the motor immediately by sending the emergency stop command to the driver.
   * @return -1 if no active connection, or if the emergency stop command fails; 0 on success.
   * @note After calling disconnect(), the emergency_stop command disable the motor and tu use again the driver you need to call connect() - enable_motor() again.
   */

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, EMER_STOP);
};

int ZLAC8015D::disable_motor(){

  /**
   * @brief The motor is disable without cutting the power, so it can be enabled calling enable_motor() again.
   * @return -1 if no active connection, or if the emergency stop command fails; 0 on success.
   */

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, DOWN_TIME);
}

int ZLAC8015D::enable_motor(){

  /**
  * @brief Enable the motor if it was disabled by disable_motor() or after an emergency stop.
  * @return -1 if no active connection, or if the emergency stop command fails; 0 on success.
  */

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, ENABLE);
}

int ZLAC8015D::reset_alarm(){

  /**
  * @brief Clear the alarm if there is any active.
  * @return -1 if no active connection, or if the alarm clear command fails; 0 on success.
  */

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, ALRM_CLR);
}

int ZLAC8015D::change_mode(std::optional<std::string> mode) {

  /**
  * @brief It will change to that mode if it's valid, and modify mode_ attribute.
  * @return -1 if no active connection, or if the emergency stop command fails; 0 on success. Print the mode to change.
  */

  if (!client_) return -1;
  if (mode) mode_ = *mode; // Modify The Argument of the class
  if (!is_valid_mode(mode_)) return -1;
  return set_mode();
}

int ZLAC8015D::set_mode() {

  /**
  * @brief Private method that set the mode of the driver according to the mode_ attribute, it will be called by change_mode() and connect() methods.
  * @return -1 if no active connection, or if the emergency stop command fails; 0 on success. Print the mode to change.
  */

  if (!client_) return -1;

  if (mode_ == "velocity") {
    std::cout << "Successfully setted to velocity mode" << std::endl;
    return modbus_write_register(client_, OPR_MODE, 0X0003);
  }
  else if (mode_ == "relative_position") {
    std::cout << "Successfully setted to relative_position mode" << std::endl;
    return modbus_write_register(client_, OPR_MODE, 0X0001);
  }
  else if (mode_ == "absolute_position") {
    std::cout << "Successfully setted to absolute_position mode" << std::endl;
    return modbus_write_register(client_, OPR_MODE, 0X0002);
  }
  else if (mode_ == "torque") {
    std::cout << "Successfully setted to torque mode" << std::endl;
    return modbus_write_register(client_, OPR_MODE, 0X0004);
  }

  return -1;
}


std::pair<int32_t, int32_t> ZLAC8015D::get_encoder_count(){
  
  /**
   * @brief Read left and right motor encoder counts.
   * @return encoder_count_left and encoder_count_right on success, -1 on error (not connected, Modbus failure).
   */

  if (!client_) return {-1, -1};

  uint16_t L[2] = {0, 0};
  uint16_t R[2] = {0, 0};

  int rc_l = modbus_read_registers(client_, ENC_LEFT, 2, L);
  int rc_r = modbus_read_registers(client_, ENC_RIGHT, 2, R);

  if (rc_l == -1 || rc_r == -1) {
    std::cerr << "get_encoder_count failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }

  int32_t left_count  = (static_cast<int32_t>(L[0]) << 16) | L[1];
  int32_t right_count = (static_cast<int32_t>(R[0]) << 16) | R[1];

  std::cout << "Encoder counts: Left: " << left_count << " Right: " << right_count << std::endl;
  return {left_count, right_count};
} 

std::optional<VelocityGains> ZLAC8015D::get_control_gains_velocity(){
  /**
   * @brief Read the current control gains for velocity mode.
   * @return 2 Structs with the control gains for velocity mode for left and right motor on success, std::nullopt on error (not connected, Modbus failure).
   * @note This is a placeholder implementation. You should read the actual registers for the control gains and return them in a suitable format.
   */

  if (!client_) return std::nullopt;

  uint16_t kp_u_l {0}, ki_u_l {0}, kf_u_l {0};
  uint16_t kp_u_r {0}, ki_u_r {0}, kf_u_r {0};

  // Left
  if(modbus_read_registers(client_, VL_KP, 1, &kp_u_l) != 1 ||
     modbus_read_registers(client_, VL_KI, 1, &ki_u_l) != 1 ||
     modbus_read_registers(client_, VL_KF, 1, &kf_u_l) != 1) {
    std::cerr << "Failed to read left velocity gains for left_motor: " << modbus_strerror(errno) << std::endl;
    return std::nullopt;
  }

  // Right
  if(modbus_read_registers(client_, VR_KP, 1, &kp_u_r) != 1 ||
     modbus_read_registers(client_, VR_KI, 1, &ki_u_r) != 1 ||
     modbus_read_registers(client_, VR_KF, 1, &kf_u_r) != 1) {
    std::cerr << "Failed to read left velocity gains for right_motor: " << modbus_strerror(errno) << std::endl;
    return std::nullopt;
  } 

  VelocityGains gains;
  gains.left.kp = static_cast<int16_t>(kp_u_l);
  gains.left.ki = static_cast<int16_t>(ki_u_l);
  gains.left.kf = static_cast<int16_t>(kf_u_l);  
  
  gains.right.kp = static_cast<int16_t>(kp_u_r);
  gains.right.ki = static_cast<int16_t>(ki_u_r);
  gains.right.kf = static_cast<int16_t>(kf_u_r);  

  std::cout << "Velocity Gains - Left: Kp=" << gains.left.kp << " Ki=" << gains.left.ki << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Ki=" << gains.right.ki << " Kf=" << gains.right.kf << std::endl;

  return gains;
}


std::optional<PositionGains> ZLAC8015D::get_control_gains_position(){

  /**
   * @brief Read the current control gains for velocity mode.
   * @return 2 Structs with the control gains for velocity mode for left and right motor on success, std::nullopt on error (not connected, Modbus failure).
   * @note This is a placeholder implementation. You should read the actual registers for the control gains and return them in a suitable format.
   */

  if (!client_) return std::nullopt;
  uint16_t kp_u_l {0}, kf_u_l {0};
  uint16_t kp_u_r {0}, kf_u_r {0};

  // Left
  if(modbus_read_registers(client_, PL_KP, 1, &kp_u_l) != 1 ||
     modbus_read_registers(client_, PL_KF, 1, &kf_u_l) != 1) {
    std::cerr << "Failed to read left position gains for left_motor: " << modbus_strerror(errno) << std::endl;  
    return std::nullopt;
  }

  // Right
  if(modbus_read_registers(client_, PR_KP, 1, &kp_u_r) != 1 ||
     modbus_read_registers(client_, PR_KF, 1, &kf_u_r) != 1) {
    std::cerr << "Failed to read left position gains for right_motor: " << modbus_strerror(errno) << std::endl;
    return std::nullopt;
  }

  PositionGains gains;
  gains.left.kp = static_cast<int16_t>(kp_u_l);
  gains.left.kf = static_cast<int16_t>(kf_u_l);

  gains.right.kp = static_cast<int16_t>(kp_u_r);
  gains.right.kf = static_cast<int16_t>(kf_u_r);

  std::cout << "Position Gains - Left: Kp=" << gains.left.kp << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Kf=" << gains.right.kf << std::endl;

  return gains;
}

int ZLAC8015D::set_control_gains_velocity(VelocityGains &gains){

  /**
   * @brief Set the control gains for velocity mode.
   * @param gains Structs with the control gains for velocity mode for left and right motor.
   * leff.kp, left.ki, left.kf, right.kp, right.ki, right.kf
   * @return 0 on success, -1 on error (not connected, Modbus failure).
   * @note This is a placeholder implementation. You should write the actual registers for the control gains according to the format you choose for the get_control_gains_velocity() method.
   */

  if (!client_) return -1;
  uint16_t regs[6] = {
    static_cast<uint16_t>(gains.left.kp),
    static_cast<uint16_t>(gains.left.ki),
    static_cast<uint16_t>(gains.left.kf),
    static_cast<uint16_t>(gains.right.kp),
    static_cast<uint16_t>(gains.right.ki),
    static_cast<uint16_t>(gains.right.kf)
  };

  if(modbus_write_registers(client_, VL_KP, 1, &regs[0]) == -1 ||
     modbus_write_registers(client_, VL_KI, 1, &regs[1]) == -1 ||
     modbus_write_registers(client_, VL_KF, 1, &regs[2]) == -1 ){
    std::cerr << "set_control_gains_velocity for left side failed: " << modbus_strerror(errno) << std::endl;
    return -1;
    }
  
  if(modbus_write_registers(client_, VR_KP, 1, &regs[3]) == -1 ||
     modbus_write_registers(client_, VR_KI, 1, &regs[4]) == -1 ||
     modbus_write_registers(client_, VR_KF, 1, &regs[5]) == -1 ){
    std::cerr << "set_control_gains_velocity for right side failed: " << modbus_strerror(errno) << std::endl;  
    return -1;
    }
  
  std::cout << "Successfully set velocity gains: " << "Left: Kp=" << gains.left.kp << " Ki=" << gains.left.ki << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Ki=" << gains.right.ki << " Kf=" << gains.right.kf << std::endl;    
  return 0;
  }




int ZLAC8015D::set_control_gains_position(PositionGains &gains){

    /**
   * @brief Set the control gains for position mode.
   * @param gains Structs with the control gains for position mode for left and right motor.
   * left.kp, left.kf, right.kp, right.kf
   * @return 0 on success, -1 on error (not connected, Modbus failure).
   * @note This is a placeholder implementation. You should write the actual registers for the control gains according to the format you choose for the get_control_gains_position() method.
   */

  if (!client_) return -1;
  uint16_t regs[4] = {
    static_cast<uint16_t>(gains.left.kp),
    static_cast<uint16_t>(gains.left.kf),
    static_cast<uint16_t>(gains.right.kp),
    static_cast<uint16_t>(gains.right.kf)
  };

  if(modbus_write_registers(client_, PL_KP, 1, &regs[0]) == -1 ||
     modbus_write_registers(client_, PL_KF, 1, &regs[1]) == -1 ){
    std::cerr << "set_control_gains_position for left side failed: " << modbus_strerror(errno) << std::endl;
    return -1;
    }
  
  if(modbus_write_registers(client_, PR_KP, 1, &regs[2]) == -1 ||
     modbus_write_registers(client_, PR_KF, 1, &regs[3]) == -1 ){
    std::cerr << "set_control_gains_position for right side failed: " << modbus_strerror(errno) << std::endl;  
    return -1;
    }
  
  std::cout << "Successfully set position gains: " << "Left: Kp=" << gains.left.kp << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Kf=" << gains.right.kf << std::endl;    
  return 0;
  }


// For velocity mode
int ZLAC8015D::set_rpm(int16_t L_rpm, int16_t R_rpm){

  /**
   * @brief Sets target wheel speeds (RPM) for left and right motors in velocity mode.
   *
   * Ensures the driver is in "velocity" mode (switches if needed), clamps the input
   * RPM commands to [-max_rpm, max_rpm], then writes both setpoints to the target RPM
   * registers using a single Modbus multi-write transaction.
   *
   * @param L_rpm Target left motor speed in RPM (negative = reverse).
   * @param R_rpm Target right motor speed in RPM (negative = reverse).
   * @return 0 on success, -1 on error (not connected, Modbus failure).
   */

  if (!client_) return -1;
  if(mode_ != "velocity") {
    std::cout << "To set RPM you have to be in velocity mode and you are in" << mode_ << std::endl;
    std::cout << "For security we change your mode to Velocity" << std::endl;
    change_mode("velocity");
  }

  if(L_rpm < -100 || L_rpm > 100 || R_rpm < -100 || R_rpm > 100) {
    std::cerr << "RPM must be between -100 and 100\n";
    L_rpm = std::clamp<int16_t>(L_rpm, -100, 100);
    R_rpm = std::clamp<int16_t>(R_rpm, -100, 100);
    std::cout << "For security we set the max speed for position mode to: " << L_rpm << " for left and " << R_rpm << " for right\n";
  }

  uint16_t regs[2] = {
    static_cast<uint16_t>(L_rpm),
    static_cast<uint16_t>(R_rpm)
  };

  int rc = modbus_write_registers(client_, SET_RPM, 2, regs);
  if (rc == -1) {
    std::cerr << "set_rpm failed: " << modbus_strerror(errno) << std::endl;
    return -1;
  }
  return 0;
}

std::pair<int, int> ZLAC8015D::get_rpm(){

  /**
   * @brief Read left and right motor actual speed
   * @return rpm_left and rmp_right on success, -1 on error (not connected, Modbus failure).
   */

  if (!client_) return {-1, -1};
  uint16_t rpm[2];
  int rc = modbus_read_registers(client_, GET_RPM, 2, rpm);
  if (rc == -1) {
    std::cerr << "get_rpm failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }
  std::cout << "Current RPM: Left: " << rpm[0] << " Right: " << rpm[1] << std::endl;
  return { static_cast<int>(rpm[0]), static_cast<int>(rpm[1]) };
}

// For relative position mode
int ZLAC8015D::set_max_speed_position_mode(int16_t L_rpm, int16_t R_rpm){

  /**
   * @brief Sets the maximum allowed motor speed (RPM) used in position modes.
   *
   * Writes the per-motor max speed limits for position control (left/right) to the
   * corresponding registers. Input values are clamped to [0, 50] RPM for safety
   * before being written.
   *
   * @param L_rpm Maximum speed for the left motor in position mode (0..50 RPM).
   * @param R_rpm Maximum speed for the right motor in position mode (0..50 RPM).
   * @return 0 on success, -1 on error (not connected or Modbus write failure).
   */

  if (!client_) return -1;

  if(L_rpm < 0 || L_rpm > 50 || R_rpm < 0 || R_rpm > 50) {
    std::cerr << "RPM must be between 0 and 50\n";
    L_rpm = std::clamp<int16_t>(L_rpm, 0, 50);
    R_rpm = std::clamp<int16_t>(R_rpm, 0, 50);
    std::cout << "For security we set the max speed for position mode to: " << L_rpm << " for left and " << R_rpm << " for right\n";
  }

  if (modbus_write_register(client_, L_MAX_RPM_POS, L_rpm) == -1 ||
      modbus_write_register(client_, R_MAX_RPM_POS, R_rpm) == -1) {
    std::cerr << "Failed to set pos max speed: " << modbus_strerror(errno) << "\n";
    return -1;
  }
  std::cout << "Successfully set pos max speed: " << "right = " << R_rpm << ", left = " << L_rpm << "\n";
  return 0;
}

std::pair<int, int> ZLAC8015D::get_max_speed_position_mode(){

  /**
 * @brief Reads the configured maximum speed limits (RPM) used in position modes.
 *
 * Retrieves the left and right "max RPM in position mode" values from the driver
 * registers and returns them as a pair {left, right}. On read failure or if not
 * connected, returns {-1, -1}.
 *
 * @return std::pair<int,int> {left_max_rpm, right_max_rpm} on success, or {-1, -1} on error.
 */


  if (!client_) return {-1, -1};

  uint16_t max_rpm_L = 0;
  uint16_t max_rpm_R = 0;
  if (modbus_read_registers(client_, L_MAX_RPM_POS, 1, &max_rpm_L) == -1 ||
      modbus_read_registers(client_, R_MAX_RPM_POS, 1, &max_rpm_R) == -1) {
    std::cerr << "Failed to get pos max speed: " << modbus_strerror(errno) << "\n";
    return {-1, -1};
  }
  std::cout << "Max motor speed limit for position mode: " << "right = " << max_rpm_R << ", left = " << max_rpm_L << "\n";
  return { static_cast<int>(max_rpm_L) , static_cast<int>(max_rpm_R)};
}


int ZLAC8015D::set_sync_position(float L_rad, float R_rad){

/**
 * @brief Sends a synchronized position command (relative or absolute) to both motors.
 *
 * Requires the driver to be in a position mode ("relative_position" or
 * "absolute_position"); if not, it switches to relative position mode for safety.
 * Reads the current per-motor max speed limits for position mode and clamps them
 * to 50 RPM if needed. Converts input angles (radians) to encoder counts using
 * the fixed counts_per_rad factor, writes the 32-bit target positions for left
 * and right motors (4 registers starting at SET_POS), then triggers a synchronized
 * position start via CONTROL_REG (POS_SYNC).
 *
 * @param L_rad Target left position in radians (interpreted as relative/absolute depending on current mode).
 * @param R_rad Target right position in radians (interpreted as relative/absolute depending on current mode).
 * @return 0 on success, -1 on error (not connected, Modbus failure, or partial write).
 */


  if (!client_) return -1;
  if(mode_ != "relative_position" && mode_ != "absolute_position") {
    std::cout << "To set RPM you have to be in relative_position or absolute_position mode and you are in " << mode_ << std::endl;
    std::cout << "For security we change your mode to relative_position" << std::endl;
    change_mode("relative_position");
  }

  auto [max_rpm_L, max_rpm_R]= get_max_speed_position_mode();

  if(max_rpm_L == -1 || max_rpm_R == -1) {
    std::cerr << "Failed to get max speed for position mode, cannot set relative position\n";
  }

  if (max_rpm_L > 50 || max_rpm_R > 50) {
    if(set_max_speed_position_mode(50, 50) == 0){
      std::cout << "FOR SECURITY: Set default max speed for position mode to 50 RPM for both motors\n";
    }
    else {
      std::cerr << "Failed to set default max speed for position mode, cannot set relative position\n";
      return -1;
    }
  }

  int32_t L_pulses = static_cast<int32_t>(std::lround(L_rad * counts_per_rad)); // Assuming 4096 counts per revolution
  int32_t R_pulses = static_cast<int32_t>(std::lround(R_rad * counts_per_rad));

  std::cout << "Set succesfully:" << " Left: " << L_rad << " rad (" << L_pulses << " pulses), "
            << "Right: " << R_rad << " rad (" << R_pulses << " pulses)\n";

  uint16_t regs[4] = {
    static_cast<uint16_t>((L_pulses >> 16) & 0xFFFF),
    static_cast<uint16_t>( L_pulses        & 0xFFFF),
    static_cast<uint16_t>((R_pulses >> 16) & 0xFFFF),
    static_cast<uint16_t>( R_pulses        & 0xFFFF),
  };

  int rc = modbus_write_registers(client_, SET_POS, 4, regs);
  modbus_write_register(client_, CONTROL_REG, POS_SYNC);
  return (rc == 4) ? 0 : -1;
}


std::pair<std::string, std::string> ZLAC8015D::get_error() {
  if (!client_) return {"Not connected", "Not connected"};

  uint16_t error_left{0};
  uint16_t error_right{0};

  if (modbus_read_registers(client_, FAULTS_LEFT, 1, &error_left) == -1) {
    return {std::string("Failed to read left error register: ") + modbus_strerror(errno),
            "Right: not read"};
  }

  if (modbus_read_registers(client_, FAULTS_RIGHT, 1, &error_right) == -1) {
    return {"Left: read ok",
            std::string("Failed to read right error register: ") + modbus_strerror(errno)};
  }

  return {hex16(error_left), hex16(error_right)};
}

std::string ZLAC8015D::decode_error(uint16_t reg, const char* side){

  if (!client_) return "Not connected";
  if (reg == 0) {
    return std::string(side) + ": No error. Driver is working properly.";
  }

  std::ostringstream out;
  out << side << ": Active faults (0x"
      << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << reg
      << std::dec << ")\n";

  uint16_t known_mask = 0;

  for (uint16_t bit = 1; bit != 0; bit <<= 1) {
    if (!(reg & bit)) continue;

    known_mask |= bit;

    switch (bit) {
      case 0x0001:
        out << "\n- Over Voltage Error:\n"
                "  - Power supply voltage is too high.\n"
                "  - Excessive back EMF (consider bleeder circuit)\n";
        break;

      case 0x0002:
        out << "\n- Under Voltage Error:\n"
                "  - Power supply voltage is too low.\n"
                "  - Check wiring connector.\n"
                "  - Check motor parameters.\n";
        break;

      case 0x0004:
        out << "\n- Motor Overcurrent Fault:\n"
                "  - Instantaneous current is too high.\n"
                "  - Motor power cable is loose.\n";
        break;

      case 0x0008:
        out << "\n- Motor Overload Fault:\n"
                "  - Check if the motor cable is loose.\n"
                "  - Check wiring and motor parameters.\n"
                "  - Motor is stall.\n"
                "  - Motor or driver problem.\n";
        break;

      case 0x0020:
        out << "\n- Encoder Value out of Tolerance:\n"
                "  - Motor is stall.\n"
                "  - Encoder problem.\n";
        break;

      case 0x0080:
        out << "\n- Reference Voltage Error:\n"
                "  - Reference voltage circuit issue.\n";
        break;

      case 0x0100:
        out << "\n- EEPROM Read/Write Error:\n"
                "  - Firmware upgraded (needs factory settings).\n"
                "  - EEPROM circuit damaged.\n";
        break;

      case 0x0200:
        out << "\n- Hall Error:\n"
                "  - Check motor cable.\n"
                "  - Motor problem.\n"
                "  - Driver problem.\n";
        break;

      case 0x0400:
        out << "\n- Temperature too High:\n"
                "  - Motor current too high (monitor and reduce current).\n"
                "  - Motor thermistor damaged.\n"
                "  - Driver circuit damaged.\n";
        break;

      case 0x0800:
        out << "\n- Encoder Error:\n"
                "  - Check encoder cable.\n"
                "  - Check if encoder cable is disconnected.\n";
        break;

      case 0x2000:
        out << "\n- Speed Setting Error:\n"
                "  - Given speed exceeds rated speed.\n";
        break;

      default:
        // Si hay bits que tu manual no documenta o aún no mapeaste
        out << "\n- Unknown fault bit set: 0x"
            << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << bit
            << std::dec << "\n";
        break;
    }
  }

  uint16_t unknown = reg & ~known_mask;
  if (unknown) {
    out << "\n- Unmapped bits present: 0x"
        << std::hex << std::uppercase << std::setw(4) << std::setfill('0') << unknown
        << std::dec << "\n";
  }

  return out.str();
};

std::string ZLAC8015D::hex16(uint16_t v) {
  std::ostringstream oss;
  oss << "0x"
      << std::hex << std::uppercase
      << std::setw(4) << std::setfill('0')
      << v;
  return oss.str();
}


std::pair<int, int> ZLAC8015D::get_temperature() {

  /**
   * @brief Read left and right motor temperature in Celsius.
   * @return temp_left and temp_right in °C on success, -1 on error (not connected, Modbus failure).
   */

  if (!client_) return {-1, -1};
  uint16_t temp;
  int rc = modbus_read_registers(client_, READ_TEMPERATURE, 1, &temp);
  if (rc == -1) {
    std::cerr << "get_temperature failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }
  std::cout << "Current Temperature: Left: " << (temp >> 8) << " °C Right: " << (temp & 0xFF) << " °C" << std::endl;
  return { static_cast<int>(temp >> 8), static_cast<int>(temp & 0xFF) };
}
  
std::string ZLAC8015D::get_software_version(){
    /**
   * @brief Read the driver software version.
   * @return version number on success, -1 on error (not connected, Modbus failure).
   */

  if (!client_) return "Not connected";
  uint16_t version;
  int rc = modbus_read_registers(client_, READ_ACTUAL_SOFTWARE_VERSION, 1, &version);
  if (rc == -1) {
    std::cerr << "get_software_version failed: " << modbus_strerror(errno) << std::endl;
    return "Not connected";
  }
  std::cout << "Driver Software Version: " << hex16(version) << std::endl;
  return hex16(version);
}



                
        