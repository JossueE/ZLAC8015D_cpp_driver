#include "zlac8015d_driver.h"

ZLAC8015D::ZLAC8015D(const std::string& port, int baudrate, const OperationMode& mode): port_(port), baudrate_(baudrate), mode_(mode){
  std::cout << "ZLAC8015D driver created with port: " << port_ << ", baudrate: " << baudrate_ << ", mode: " << mode_ << std::endl;
}

ZLAC8015D::~ZLAC8015D(){
  disconnect();
}

bool ZLAC8015D::connect(){
 
  if (client_) {
    DBG("You have a previous connection, we recommended to reconnect, disconnecting first.");
    return true;
  }
  DBG("Trying port: " << port_ << " baudrate: " << baudrate_);
  client_ = modbus_new_rtu(port_.c_str(), baudrate_, 'N', 8, 1);
  if (!client_) {
    std::cerr << "Error creating context Modbus RTU (modbus_new_rtu fail)." << std::endl;
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

  if (client_) {
    emergency_stop();
    modbus_close(client_);
    modbus_free(client_);
    client_ = nullptr;
    DBG("Successfully disconnected\n");
  } else {
    DBG("No active connection to disconnect\n");
  }
}

int ZLAC8015D::emergency_stop(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, EMER_STOP);
};

int ZLAC8015D::disable_motor(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, DOWN_TIME);
}

int ZLAC8015D::enable_motor(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, ENABLE);
}

int ZLAC8015D::reset_alarm(){

  if (!client_) return -1;
  return modbus_write_register(client_, CONTROL_REG, ALRM_CLR);
}

int ZLAC8015D::change_mode(OperationMode mode) {

  if (!client_) return -1;
  mode_ = mode; // Modify The Argument of the class
  return set_mode();
}

int ZLAC8015D::set_mode() {

  if (!client_) return -1;
  if (mode_ == OperationMode::VELOCITY) {
    DBG("Successfully setted to velocity mode");
    return modbus_write_register(client_, OPR_MODE, 0X0003);
  }
  else if (mode_ == OperationMode::RELATIVE_POSITION) {
    DBG("Successfully setted to relative_position mode");
    return modbus_write_register(client_, OPR_MODE, 0X0001);
  }
  else if (mode_ == OperationMode::ABSOLUTE_POSITION) {
    DBG("Successfully setted to absolute_position mode");
    return modbus_write_register(client_, OPR_MODE, 0X0002);
  }
  else if (mode_ == OperationMode::TORQUE) {
    DBG("Successfully setted to torque mode");
    return modbus_write_register(client_, OPR_MODE, 0X0004);
  }
  return -1;
}


std::pair<int32_t, int32_t> ZLAC8015D::get_encoder_count(){

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

  DBG("Encoder counts: Left: " << left_count << " Right: " << right_count);
  return {left_count, right_count};
} 

std::optional<VelocityGains> ZLAC8015D::get_control_gains_velocity(){

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

  DBG("Velocity Gains - Left: Kp=" << gains.left.kp << " Ki=" << gains.left.ki << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Ki=" << gains.right.ki << " Kf=" << gains.right.kf);

  return gains;
}


std::optional<PositionGains> ZLAC8015D::get_control_gains_position(){

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

  DBG("Position Gains - Left: Kp=" << gains.left.kp << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Kf=" << gains.right.kf);

  return gains;
}

int ZLAC8015D::set_control_gains_velocity(VelocityGains &gains){

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
  
  DBG("Successfully set velocity gains: " << "Left: Kp=" << gains.left.kp << " Ki=" << gains.left.ki << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Ki=" << gains.right.ki << " Kf=" << gains.right.kf);
  return 0;
  }




int ZLAC8015D::set_control_gains_position(PositionGains &gains){

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
  
  DBG("Successfully set position gains: " << "Left: Kp=" << gains.left.kp << " Kf=" << gains.left.kf
            << " | Right: Kp=" << gains.right.kp << " Kf=" << gains.right.kf);    
  return 0;
  }


int ZLAC8015D::set_speed_resolution() {

  if (!client_) return -1;
  if (modbus_write_register(client_, SET_SPEED_RESOLUTION, RPM_RES_CERO_POINT_ONE) == -1) return -1; 
  std::cerr << "Remember to Restart the system to make effective the changes\n";
  return 0;
}

std::string ZLAC8015D::get_speed_resolution() {

  if (!client_) return "Unknown";
  uint16_t val;
  if (modbus_read_registers(client_, SET_SPEED_RESOLUTION, 1, &val) != 1) {
    std::cerr << "get_speed_resolution failed: " << modbus_strerror(errno) << std::endl;
    return "Unknown";
  }

  std::string res_str = (val == RPM_RES_CERO_POINT_ONE) ? "0.1 RPM" : (val == RPM_RES_ONE) ? "1 RPM" : "Unknown";
  DBG("Current speed resolution: " << res_str);
  return res_str;
}


int ZLAC8015D::set_sync_rpm(float L_rpm, float R_rpm) {

  if (!client_) return -1;

  if (mode_ != OperationMode::VELOCITY) {
    DBG("To set RPM you have to be in velocity mode and you are in " << mode_ << "\n");
    DBG("For security we change your mode to Velocity\n");
    if (change_mode(OperationMode::VELOCITY) != 0) return -1; 
  }

  std::string res = get_speed_resolution();
  if (res == "Unknown") {
    std::cerr
      << "Failed to get speed resolution\n"
      << " - Remember to set the speed resolution with set_speed_resolution() before setting the RPM\n"
      << " - And restart the system to make effective the changes\n";
    return -1;
  }

  if (res == "1 RPM") {
    std::cerr
      << "It looks like you have setted 1 RPM in config\n"
      << " - Remember to set the speed resolution with set_speed_resolution() before setting the RPM\n"
      << " - And restart the system to make effective the changes\n";
    return -1;
  }

  L_rpm = std::round(L_rpm * 10.0f);
  R_rpm = std::round(R_rpm * 10.0f);

  if (L_rpm < -1000.0f || L_rpm > 1000.0f || R_rpm < -1000.0f || R_rpm > 1000.0f) {
    std::cerr << "RPM must be between -1000.0 and 1000.0\n";
    L_rpm = std::clamp<float>(L_rpm, -1000.0f, 1000.0f);
    R_rpm = std::clamp<float>(R_rpm, -1000.0f, 1000.0f);
  }

  uint16_t regs[2]{0, 0};           
  int32_t l_units = static_cast<int32_t>(L_rpm);
  int32_t r_units = static_cast<int32_t>(R_rpm);

  l_units = std::clamp<int32_t>(l_units, -32768, 32767);
  r_units = std::clamp<int32_t>(r_units, -32768, 32767);

  regs[0] = static_cast<uint16_t>(static_cast<int16_t>(l_units));
  regs[1] = static_cast<uint16_t>(static_cast<int16_t>(r_units));


  int rc = modbus_write_registers(client_, SET_RPM, 2, regs); // SET_RPM debe ser 0x2088
  if (rc == -1) {
    std::cerr << "set_sync_rpm failed: " << modbus_strerror(errno) << "\n";
    return -1;
  }

  DBG("Set RPM: Left: " << L_rpm * 0.1f << " Right: " << R_rpm * 0.1f);

  return 0;
}


std::pair<float, float> ZLAC8015D::get_rpm(){

  if (!client_) return {-1, -1};
  uint16_t rpm[2];
  int rc = modbus_read_registers(client_, GET_RPM, 2, rpm);
  if (rc == -1) {
    std::cerr << "get_rpm failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }
  DBG("Current RPM: Left: " << rpm[0] * 0.1f << " Right: " << rpm[1] * 0.1f);
  return { static_cast<float>(rpm[0]) * 0.1f, static_cast<float>(rpm[1]) * 0.1f };
}

int ZLAC8015D::set_max_speed_position_mode(int16_t L_rpm, int16_t R_rpm){

  if (!client_) return -1;

  if(mode_ != OperationMode::RELATIVE_POSITION && mode_ != OperationMode::ABSOLUTE_POSITION) {
    DBG("To set RPM you have to be in relative_position or absolute_position mode and you are in " << mode_ << std::endl);
    DBG("For security we change your mode to relative_position" << std::endl);
    change_mode(OperationMode::RELATIVE_POSITION);
  }

  if(L_rpm < 0 || L_rpm > 50 || R_rpm < 0 || R_rpm > 50) {
    std::cerr << "RPM must be between 0 and 50\n";
    L_rpm = std::clamp<int16_t>(L_rpm, 0, 50);
    R_rpm = std::clamp<int16_t>(R_rpm, 0, 50);
    DBG("For security we set the max speed for position mode to: " << L_rpm << " for left and " << R_rpm << " for right\n");
  }

  if (modbus_write_register(client_, L_MAX_RPM_POS, L_rpm) == -1 ||
      modbus_write_register(client_, R_MAX_RPM_POS, R_rpm) == -1) {
    std::cerr << "Failed to set pos max speed: " << modbus_strerror(errno) << "\n";
    return -1;
  }
  DBG("Successfully set pos max speed: " << "right = " << R_rpm << ", left = " << L_rpm);
  return 0;
}

std::pair<int, int> ZLAC8015D::get_max_speed_position_mode(){

  if (!client_) return {-1, -1};

  uint16_t max_rpm_L = 0;
  uint16_t max_rpm_R = 0;
  if (modbus_read_registers(client_, L_MAX_RPM_POS, 1, &max_rpm_L) == -1 ||
      modbus_read_registers(client_, R_MAX_RPM_POS, 1, &max_rpm_R) == -1) {
    std::cerr << "Failed to get pos max speed: " << modbus_strerror(errno) << "\n";
    return {-1, -1};
  }
  DBG("Max motor speed limit for position mode: " << "right = " << max_rpm_R << ", left = " << max_rpm_L);
  return { static_cast<int>(max_rpm_L) , static_cast<int>(max_rpm_R)};
}


int ZLAC8015D::set_sync_position(float L_rad, float R_rad){

  if (!client_) return -1;
  if(mode_ != OperationMode::RELATIVE_POSITION && mode_ != OperationMode::ABSOLUTE_POSITION) {
    DBG("To set RPM you have to be in relative_position or absolute_position mode and you are in " << mode_ << std::endl);
    DBG("For security we change your mode to relative_position" << std::endl);
    change_mode(OperationMode::RELATIVE_POSITION);
  }

  auto [max_rpm_L, max_rpm_R]= get_max_speed_position_mode();

  if(max_rpm_L == -1 || max_rpm_R == -1) {
    DBG("Failed to get max speed for position mode, cannot set relative position\n");
  }

  if (max_rpm_L > 50 || max_rpm_R > 50 || max_rpm_L <= 0 || max_rpm_R <= 0) {
    if(set_max_speed_position_mode(50, 50) == 0){
      DBG("FOR SECURITY: Set default max speed for position mode to 50 RPM for both motors\n");
    }
    else {
      std::cerr << "Failed to set default max speed for position mode, cannot set relative position\n";
      return -1;
    }
  }

  int32_t L_pulses = static_cast<int32_t>(std::lround(L_rad * counts_per_rad)); // Assuming 4096 counts per revolution
  int32_t R_pulses = static_cast<int32_t>(std::lround(R_rad * counts_per_rad));

  DBG("Set succesfully:" << " Left: " << L_rad << " rad (" << L_pulses << " pulses), "
            << "Right: " << R_rad << " rad (" << R_pulses << " pulses)\n");

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

int ZLAC8015D::set_sync_current(int16_t L_mA, int16_t R_mA){

  if (!client_) return -1;
  if(mode_ != OperationMode::TORQUE) {
    DBG("To set torque you have to be in torque mode and you are in " << mode_ << std::endl);
    DBG("For security we change your mode to torque" << std::endl);
    change_mode(OperationMode::TORQUE);
  }

  if (L_mA < -2000 || L_mA > 2000 || R_mA < -2000 || R_mA > 2000) {
    std::cerr << "Current must be between -2000 mA and 2000 mA\n";
    L_mA = std::clamp<int16_t>(L_mA, -2000, 2000);
    R_mA = std::clamp<int16_t>(R_mA, -2000, 2000);
    DBG("For security we set the max current for torque mode to: " << L_mA << " mA for left and " << R_mA << " mA for right\n");
  }

  uint16_t regs[2] = {
    static_cast<uint16_t>(L_mA),
    static_cast<uint16_t>(R_mA)
  };

  int rc = modbus_write_registers(client_, SET_TORQUE, 2, regs);
  return (rc == 2) ? 0 : -1;
}

std::pair<float, float> ZLAC8015D::get_current(){

  if (!client_) return {-1, -1};
  uint16_t current[2];
  int rc = modbus_read_registers(client_, READ_ACTUAL_CURRENT, 2, current);
  if (rc == -1) {
    std::cerr << "get_current failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }
  DBG("Current actual mA: Left: " << current[0] * 0.1f << " Right: " << current[1] * 0.1f << std::endl);
  return { static_cast<float>(current[0]) * 0.1f, static_cast<float>(current[1]) * 0.1f };
}


int ZLAC8015D::enable_parking_mode(){

  if (!client_) return -1;
  return modbus_write_register(client_, PARKING_MODE_REG, 0x0001);
}

int ZLAC8015D::disable_parking_mode(){

  if (!client_) return -1;
  return modbus_write_register(client_, PARKING_MODE_REG, 0x0000);
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

  if (!client_) return {-1, -1};
  uint16_t temp;
  int rc = modbus_read_registers(client_, READ_TEMPERATURE, 1, &temp);
  if (rc == -1) {
    std::cerr << "get_temperature failed: " << modbus_strerror(errno) << std::endl;
    return {-1, -1};
  }
  DBG("Current Temperature: Left: " << (temp >> 8) << " °C Right: " << (temp & 0xFF) << " °C" << std::endl);
  return { static_cast<int>(temp >> 8), static_cast<int>(temp & 0xFF) };
}
  
std::string ZLAC8015D::get_software_version(){

  if (!client_) return "Not connected";
  uint16_t version;
  int rc = modbus_read_registers(client_, READ_ACTUAL_SOFTWARE_VERSION, 1, &version);
  if (rc == -1) {
    std::cerr << "get_software_version failed: " << modbus_strerror(errno) << std::endl;
    return "Not connected";
  }
  DBG("Driver Software Version: " << hex16(version));
  return hex16(version);
}

int ZLAC8015D::set_decel_time(uint16_t decel_time_ms) {

  if (!client_) return -1;

  if (decel_time_ms > 32767) {
    DBG("Deceleration time must be between 0 and 32767 ms\n");
    decel_time_ms = std::clamp<uint16_t>(decel_time_ms, 0u, 32767u);
    DBG("For security we set the deceleration time to: " << decel_time_ms << " ms\n");
  }

  int rc = modbus_write_register(client_, L_DCL_TIME, decel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set deceleration time in Left motor - Right motor is not setted: " << modbus_strerror(errno) << std::endl;
    return -1;
  }

  rc = modbus_write_register(client_, R_DCL_TIME, decel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set deceleration time in Right motor - Left motor was set: " << modbus_strerror(errno) << std::endl;
    return -1;
  }
  return 0;
}

int ZLAC8015D::set_accel_time(uint16_t accel_time_ms) {

  if (!client_) return -1;
  
  if (accel_time_ms > 32767) {
    DBG( "Acceleration time must be between 0 and 32767 ms\n");
    accel_time_ms = std::clamp<uint16_t>(accel_time_ms, 0u, 32767u);
    DBG("For security we set the acceleration time to: " << accel_time_ms << " ms\n");
  }

  int rc = modbus_write_register(client_, L_ACL_TIME, accel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set acceleration time in Left motor - Right motor is not setted: " << modbus_strerror(errno) << std::endl;
    return -1;
  }

  rc = modbus_write_register(client_, R_ACL_TIME, accel_time_ms);
  if (rc == -1) {
    std::cerr << "Failed to set acceleration time in Right motor - Left motor was set: " << modbus_strerror(errno) << std::endl;
    return -1;
  }
  return 0;
} 

                
        