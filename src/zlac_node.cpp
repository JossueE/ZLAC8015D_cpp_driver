#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "zlac8015d_driver.h"

struct VelocityMotorGains { int16_t kp, ki, kf; };
struct VelocityGains { VelocityMotorGains left, right; };

using namespace std::chrono_literals;
class ZlacNode : public rclcpp::Node {
public:
  ZlacNode(const std::string &port) : Node("zlac_node"), port_(port) {

    // Wheels configuration parameters
    this->declare_parameter<bool>("wheelR_is_backward", false);
    this->declare_parameter<bool>("wheelL_is_backward", false);
    this->declare_parameter<double>("wheels_separation", 0.4f);
    this->declare_parameter<double>("wheel_radius", 0.1f);

    // Acceleration and deceleration time for velocity control mode
    this->declare_parameter<int>("accel_time_ms", 3000);  // The driver can receive accel for left and right wheel separately, but for simplicity we use the same value for both.
    this->declare_parameter<int>("decel_time_ms", 3000);  // The driver can receive decel for left and right wheel separately, but for simplicity we use the same value for both.

    //PID gains for velocity control mode
    // Left
    this->declare_parameter<int>(base + ".left.kp", 60);
    this->declare_parameter<int>(base + ".left.ki", 20);
    this->declare_parameter<int>(base + ".left.kf", 10);

    // Right
    this->declare_parameter<int>(base + ".right.kp", 60);
    this->declare_parameter<int>(base + ".right.ki", 20);
    this->declare_parameter<int>(base + ".right.kf", 10);

    // Wheels configuration parameters
    wheelR_is_backward_ = this->get_parameter("wheelR_is_backward").as_bool();
    wheelL_is_backward_ = this->get_parameter("wheelL_is_backward").as_bool();
    wheels_separation_ = this->get_parameter("wheels_separation").as_double();
    wheels_radius_ = this->get_parameter("wheel_radius").as_double();

    // Acceleration and deceleration time for velocity control mode
    accel_time_ms_ = this->get_parameter("accel_time_ms").as_int();
    decel_time_ms_ = this->get_parameter("decel_time_ms").as_int();

    // PID gains for velocity control mode
    velocity_gains_.left.kp = this->get_parameter(base + ".left.kp").as_int();
    velocity_gains_.left.ki = this->get_parameter(base + ".left.ki").as_int();
    velocity_gains_.left.kf = this->get_parameter(base + ".left.kf").as_int();
    velocity_gains_.right.kp = this->get_parameter(base + ".right.kp").as_int();
    velocity_gains_.right.ki = this->get_parameter(base + ".right.ki").as_int();
    velocity_gains_.right.kf = this->get_parameter(base + ".right.kf").as_int();

    // -------------------------------------- Timers to Manage the Driver Control ----------------------------------------
    error_timer_ = this->create_wall_timer(1000ms, std::bind(&ZlacNode::error_handler, this));
    warning_timer_ = this->create_wall_timer(1000ms, std::bind(&ZlacNode::warning_handler, this));
    

    // ----------------------------------------- Inizialization of Motor Control -----------------------------------------

    motors_ = ZLAC8015D(port_, 115200, OperationMode::VELOCITY);
    if (!motors_.connect()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to ZLAC8015D driver on port %s", port_.c_str());
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Successfully connected to ZLAC8015D driver on port %s", port_.c_str());
    }
    motors_.disable_motor();  // Disable the motor on startup for safety, the user can enable it later by calling connect() again.
  
    // Set the acceleration and deceleration time for velocity control mode
    motors_.set_accel_time(accel_time_ms_);
    motors_.set_decel_time(decel_time_ms_);
    
    // Set the PID gains for velocity control mode
    motors_.set_control_gains_velocity(&velocity_gains_);

    // We set the RPM before enabling the motor, after disable motor, the wheels are going to be Free. 
    // But the motor isn't start the movement until enable, we set before to make sure that the value is properly setted.
    motors_.set_sync_rpm(0.0, 0.0);


  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  // Error and troubleshooting
  rclcpp::TimerBase::SharedPtr error_timer_;
  int error_count_{0};

  size_t count_;
  bool wheelR_is_backward_;
  bool wheelL_is_backward_;
  double wheels_separation_;
  double wheels_radius_;
  int accel_time_ms_;
  int decel_time_ms_;
  VelocityGains velocity_gains_;  
  const std::string port_
  ZLAC8015D motors_;


  void warning_handler(){

    auto[] = motors_.get_encoder_count();
    auto[] = motors_.get_rpm();
    auto[] = motors_.get_current();
    auto[] = motors_.get_temperature();
  }

  void error_handler() {

    static bool warned_overheat{false};
    static bool warned_speedfail{false};

    auto [error_msg_left, error_msg_right] = motors_.get_error();

    const bool has_error = (error_msg_left != "0x0000") || (error_msg_right != "0x0000");

    if (!has_error) {
      error_count_ = 0;

      if (warned_speedfail){
        RCLCPP_INFO(this->get_logger(),"Speed back to normal. Resuming normal operation.");
        motors_.set_decel_time(decel_time_ms_);
        motors_.set_accel_time(accel_time_ms_);
        warned_speedfail = false;
      }

      if (warned_overheat){
        RCLCPP_INFO(this->get_logger(),"Temperature back to normal. Resuming normal operation.");
        motors_.disable_parking_mode();
        warned_overheat = false;
      }
      return;
    }

    RCLCPP_ERROR(this->get_logger(), "%s", motors_.decode_error(error_msg_left, "Left").c_str());
    RCLCPP_ERROR(this->get_logger(), "%s", motors_.decode_error(error_msg_right, "Right").c_str());

  // ------------------------- CRITICS: Immediately Stop -------------------------
    if (error_msg_left == "0x0800" || error_msg_right == "0x0800" ||   // Encoder Error
        error_msg_left == "0x0020" || error_msg_right == "0x0020" ||   // Encoder Value out of Tolerance
        error_msg_left == "0x0080" || error_msg_right == "0x0080" ||   // Reference Voltage Error
        error_msg_left == "0x0200" || error_msg_right == "0x0200")     // Hall Error
    {
      RCLCPP_FATAL(this->get_logger(), "Critical error detected in ZLAC8015D driver. Emergency stop + shutdown.");
      motors_.emergency_stop();
      rclcpp::shutdown();
      return;
    }

  // ------------------------- RECOVERABLES: Try to Recover -------------------------
    if (error_msg_left == "0x0001" || error_msg_right == "0x0001" ||   // Over Voltage Error
        error_msg_left == "0x0002" || error_msg_right == "0x0002" ||   // Under Voltage Error
        error_msg_left == "0x0004" || error_msg_right == "0x0004" ||   // Overcurrent Fault
        error_msg_left == "0x0008" || error_msg_right == "0x0008" ||   // Motor Overload Fault
        error_msg_left == "0x0100" || error_msg_right == "0x0100" )    // EEPROM Read and Write Error    
    {
      error_count_++;

      // Si persiste muchas veces, apagamos para proteger hardware
      if (error_count_ >= 5) {
        RCLCPP_ERROR(this->get_logger(),"ZLAC error detected %d times, shutting down to prevent damage.", error_count_);
        motors_.emergency_stop(); // The wheels are going to be locked
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(), "Recoverable ZLAC error detected. Attempt %d/5.", error_count_);
    }

  // ------------------------- No Emergency_Stop Needed -------------------------

    if (error_msg_left == "0x2000" || error_msg_right == "0x2000") {   // Speed Setting Error 
      warned_speedfail = true;
      auto[rpm_left, temp_right] = motors_.get_rpm();
      if (rpm_left > 900 || rpm_right > 900){
        RCLCPP_WARN(this->get_logger(),"Too high speed detected in ZLAC8015D driver. RPM Left: %d °C, RPM Right: %d °C", rpm_left, rpm_right);
        motors_.set_decel_time(16000);
        motors_.set_accel_time(16000);
        motors_.set_sync_rpm(100.0, 100.0);
        RCLCPP_WARN(this->get_logger(), "For security the speed is set to 100 RPM, Acceleration and deceleration set to 16 seconds");
      }
    }

    if (error_msg_left == "0x0400" || error_msg_right == "0x0400") {  // Over Temperature Fault
      warned_overheat = true;
      auto[temp_left, temp_right] = motors_.get_temperature();
      if (temp_left > 100 || temp_right > 100) {
        RCLCPP_WARN(this->get_logger(), "Temperature too high detected in ZLAC8015D driver. Temp Left: %d °C, Temp Right: %d °C", temp_left, temp_right);
        motors_.enable_parking_mode();
        RCLCPP_WARN(this->get_logger(), "For security the current is limited to 3A, but the motor can still be used. Monitor the temperature and reduce the load to prevent overheating.");
      }
    }

    // Intento de recuperación
    motors_.reset_alarm();
    motors_.enable_motor();
};

};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZlacNode>());
  rclcpp::shutdown();
  return 0;
}
