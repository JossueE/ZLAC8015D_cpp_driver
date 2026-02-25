#include <chrono>
#include <memory>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "zlac8015d_driver.h"

static constexpr double PI{3.14159265358979323846};

using namespace std::chrono_literals;

std::string node_name = "wheels_driver";


class ZlacNode : public rclcpp::Node {
public:
  ZlacNode(const std::string &port) : Node(node_name), port_(port) {

    // Wheels configuration parameters
    this->declare_parameter<bool>("wheelR_is_backward", false);
    this->declare_parameter<bool>("wheelL_is_backward", true);
    this->declare_parameter<double>("wheels_separation", 0.4f);
    this->declare_parameter<double>("wheel_radius", 0.1f);

    // Acceleration and deceleration time for velocity control mode. 
    // The driver can receive accel for left and right wheel separately.
    this->declare_parameter<int>("accel_time_ms", 3000);
    this->declare_parameter<int>("decel_time_ms", 3000);  

    //Driver Movement Lock
    this->declare_parameter<bool>("unlock_driver", true); 
    this->declare_parameter<double>("time_disabled_driver_s", 3.0f);

    // PID gains for velocity control mode
    this->declare_parameter<int>("VelocityGains.left.kp", 60);
    this->declare_parameter<int>("VelocityGains.left.ki", 20);
    this->declare_parameter<int>("VelocityGains.left.kf", 10);
    this->declare_parameter<int>("VelocityGains.right.kp", 60);
    this->declare_parameter<int>("VelocityGains.right.ki", 20);
    this->declare_parameter<int>("VelocityGains.right.kf", 10);

    // Wheels configuration parameters
    wheelR_is_backward_ = this->get_parameter("wheelR_is_backward").as_bool();
    wheelL_is_backward_ = this->get_parameter("wheelL_is_backward").as_bool();
    wheels_separation_ = this->get_parameter("wheels_separation").as_double();
    wheels_radius_ = this->get_parameter("wheel_radius").as_double();

    // Acceleration and deceleration time for velocity control mode
    accel_time_ms_ = this->get_parameter("accel_time_ms").as_int();
    decel_time_ms_ = this->get_parameter("decel_time_ms").as_int();

    //Driver Movement Lock
    unlock_driver_ = this->get_parameter("unlock_driver").as_bool();
    time_disabled_driver_s_ = get_parameter("time_disabled_driver_s").as_double();

    // PID gains for velocity control mode
    velocity_gains_.left.kp = this->get_parameter("VelocityGains.left.kp").as_int();
    velocity_gains_.left.ki = this->get_parameter("VelocityGains.left.ki").as_int();
    velocity_gains_.left.kf = this->get_parameter("VelocityGains.left.kf").as_int();
    velocity_gains_.right.kp = this->get_parameter("VelocityGains.right.kp").as_int();
    velocity_gains_.right.ki = this->get_parameter("VelocityGains.right.ki").as_int();
    velocity_gains_.right.kf = this->get_parameter("VelocityGains.right.kf").as_int();

    // -------------------------------------- Timers to Manage the Driver Control ----------------------------------------
    warning_timer_ = this->create_wall_timer(1000ms, std::bind(&ZlacNode::warning_handler, this));
    wheel_ticks_timer_ = this->create_wall_timer(20ms, std::bind(&ZlacNode::wheel_ticks_timer, this));

    movement_lock_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(time_disabled_driver_s_)), std::bind(&ZlacNode::movement_lock_timer, this));
    movement_lock_timer_->cancel();

    // ------------------------------------------- Publishers and Subscribers --------------------------------------------
    pub_left_data_ = this->create_publisher<std_msgs::msg::Float64>("wheel/left_data", 10); //Recommendation: Send both in one message
    pub_right_data_ = this->create_publisher<std_msgs::msg::Float64>("wheel/right_data", 10);
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_safe", 10, std::bind(&ZlacNode::command_vel_CB, this, std::placeholders::_1));

    // ----------------------------------------- Initialization of Motor Control -----------------------------------------

    motors_ = ZLAC8015D(port_, 115200, OperationMode::VELOCITY);

    if (!motors_.connect()) {
      RCLCPP_ERROR(rclcpp::get_logger(node_name), "Failed to connect to ZLAC8015D driver on port %s", port_.c_str());
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Successfully connected to ZLAC8015D driver on port %s", port_.c_str());
    }

    motors_.disable_motor();  // Disable the motor on startup for safety, the user can enable it later by calling connect() again.
  
    // Set the acceleration and deceleration time for velocity control mode
    motors_.set_accel_time(accel_time_ms_);
    motors_.set_decel_time(decel_time_ms_);
    
    // Set the PID gains for velocity control mode
    motors_.set_control_gains_velocity(velocity_gains_);

    // We set the RPM before enabling the motor, after disable motor, the wheels are going to be Free. 
    // But the motor isn't start the movement until enable, we set before to make sure that the value is properly setted.
    motors_.set_sync_rpm(0.0, 0.0);

    // Enable the motor after set initial speed
    motors_.enable_motor();

    RCLCPP_INFO(get_logger(), "Node initialized");
  }

private:
  // Error and troubleshooting
  rclcpp::TimerBase::SharedPtr warning_timer_;

  // Encoder - Wheels Ticks
  rclcpp::TimerBase::SharedPtr wheel_ticks_timer_;
  rclcpp::TimerBase::SharedPtr movement_lock_timer_;

  // Publishers and Subscribers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_data_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_data_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  std_msgs::msg::Float64 msg_left_, msg_right_;

  bool wheelR_is_backward_;
  bool wheelL_is_backward_;
  double wheels_separation_;
  double wheels_radius_;
  int accel_time_ms_;
  int decel_time_ms_;
  bool unlock_driver_;
  double time_disabled_driver_s_;
  VelocityGains velocity_gains_;  
  const std::string port_;
  ZLAC8015D motors_;


  void warning_handler(){

    static bool warned_overheat{false};
    static bool warned_speedfail{false};
    static int error_count{0};

    // For Future Improvements, all this information can be send with a personalized ROS2 message and publish in a Topic.
    auto[count_left, count_right] = motors_.get_encoder_count();
    auto[rpm_left, rpm_right] = motors_.get_rpm();
    auto[current_left, current_right] = motors_.get_current(); 
    auto[temp_left, temp_right] = motors_.get_temperature();
    auto[error_msg_left, error_msg_right] = motors_.get_error();

    if ((error_msg_left != "0x0000") || (error_msg_right != "0x0000")){
      RCLCPP_DEBUG(this->get_logger(), "[WARN_MON] The Handler for Errors is Activate");
      error_handler(warned_overheat, warned_speedfail, error_count);
    };

    if (error_msg_left == "0x0000" && error_msg_right == "0x0000"){ 
      if (warned_speedfail && std::abs((double)rpm_left) < 2500 && std::abs((double)rpm_right) < 2500){
        RCLCPP_INFO(this->get_logger(),"Speed back to normal. Resuming normal operation.");
        motors_.set_decel_time(decel_time_ms_);
        motors_.set_accel_time(accel_time_ms_);
        warned_speedfail = false;
      }

      if (warned_overheat && temp_left < 85 && temp_right < 85 ){
        RCLCPP_INFO(this->get_logger(),"Temperature back to normal. Resuming normal operation.");
        motors_.disable_parking_mode();
        warned_overheat = false;
      }
      error_count = 0;
    }

    // Debug message
    RCLCPP_DEBUG(get_logger(), "[WARN_MON] enc(L=%ld R=%ld) rpm(L=%.1f R=%.1f) I(L=%.2fA R=%.2fA) T(L=%.1fC R=%.1fC)",
      static_cast<long>(count_left), static_cast<long>(count_right),
      static_cast<double>(rpm_left), static_cast<double>(rpm_right),
      static_cast<double>(current_left), static_cast<double>(current_right),
      static_cast<double>(temp_left), static_cast<double>(temp_right)
    );

    // Warnings
    if (temp_left > 70 || temp_right > 70) { // 55 - 120 °C
      RCLCPP_WARN(rclcpp::get_logger(node_name), "[WARN_MON] High or Low temperature in wheels: L=%.1fC R=%.1fC", (double)temp_left, (double)temp_right);
      RCLCPP_WARN(rclcpp::get_logger(node_name), "[WARN_MON] High or Low temperature: The normal operation range is 55 - 120 °C");
      
    }

    if (std::abs((double)rpm_left) > 2500 || std::abs((double)rpm_right) > 2500) {
      RCLCPP_WARN(rclcpp::get_logger(node_name),
        "[WARN_MON] High RPM: L=%.1f R=%.1f \n", (double)rpm_left, (double)rpm_right);
      RCLCPP_WARN(rclcpp::get_logger(node_name), 
        "[WARN_MON] High RPM: Max value for Position Mode is 1000 RPM, for Velocity 3000 RPM");
    }

    if (std::abs((double)current_left) > 25000.0 || std::abs((double)current_right) > 25000.0) { // Max value = 30A
      RCLCPP_WARN(rclcpp::get_logger(node_name),
        "[WARN_MON] High current: L=%.2fmA R=%.2fmA \n", (double)current_left, (double)current_right);
      RCLCPP_WARN(rclcpp::get_logger(node_name), "[WARN_MON] High current: Max value for current is 30A");
    }    

    // If more than one publisher   
    const size_t pubs = count_publishers("cmd_vel_safe");
    if (pubs > 1 || pubs == 0) {
      RCLCPP_FATAL(rclcpp::get_logger(node_name), "More than one publisher or No publishers on cmd_vel_safe (%zu). Stopping robot and shutting down.", pubs);
      motors_.set_decel_time(3000);
      motors_.set_sync_rpm(0,0);
      // Destroy node
      rclcpp::shutdown();
    }

  }

  void error_handler(bool &warned_overheat, bool &warned_speedfail, int &error_count){

    auto [error_msg_left, error_msg_right] = motors_.get_error();    

    std::string decoded_left = error_msg_left;
    std::string decoded_right = error_msg_right;
    try {
      int code_left = std::stoi(error_msg_left, nullptr, 16);
      decoded_left = motors_.decode_error(code_left, "left");
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "We cant recognize left error '%s': %s", error_msg_left.c_str(), e.what());
    }

    try {
      int code_right = std::stoi(error_msg_right, nullptr, 16);
      decoded_right = motors_.decode_error(code_right, "right");
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger(node_name), "We cant recognize right error '%s': %s", error_msg_right.c_str(), e.what());
    }

    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Left motor error: %s", decoded_left.c_str());
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Right motor error: %s", decoded_right.c_str());

    // ------------------------- CRITICS: Immediately Stop -------------------------
    if (error_msg_left == "0x0800" || error_msg_right == "0x0800" ||   // Encoder Error
        error_msg_left == "0x0020" || error_msg_right == "0x0020" ||   // Encoder Value out of Tolerance
        error_msg_left == "0x0080" || error_msg_right == "0x0080" ||   // Reference Voltage Error
        error_msg_left == "0x0200" || error_msg_right == "0x0200")     // Hall Error
    {
      RCLCPP_FATAL(rclcpp::get_logger(node_name), "Critical error detected in ZLAC8015D driver. Emergency stop + shutdown.");
      motors_.emergency_stop();
      rclcpp::shutdown();
    }

    // ------------------------- RECOVERABLE: Try to Recover -------------------------
    if (error_msg_left == "0x0001" || error_msg_right == "0x0001" ||   // Over Voltage Error
        error_msg_left == "0x0002" || error_msg_right == "0x0002" ||   // Under Voltage Error
        error_msg_left == "0x0004" || error_msg_right == "0x0004" ||   // Overcurrent Fault 
        error_msg_left == "0x0008" || error_msg_right == "0x0008" ||   // Motor Overload Fault
        error_msg_left == "0x0100" || error_msg_right == "0x0100" )    // EEPROM Read and Write Error    
    {
      error_count++;

      if (error_count >= 5) {
        RCLCPP_ERROR(rclcpp::get_logger(node_name),"ZLAC error detected %d times, shutting down to prevent damage.", error_count);
        motors_.emergency_stop(); // The wheels are going to be locked
        rclcpp::shutdown();
      }
      RCLCPP_WARN(rclcpp::get_logger(node_name), "Recoverable ZLAC error detected. Attempt %d/5.", error_count);
    }

    // ------------------------- No Emergency_Stop Needed -------------------------

    if (error_msg_left == "0x2000" || error_msg_right == "0x2000") {   // Speed Setting Error 
      warned_speedfail = true;
      auto[rpm_left, rpm_right] = motors_.get_rpm();
      if (rpm_left > 2500 || rpm_right > 2500){
        RCLCPP_WARN(this->get_logger(),"Too high speed detected in ZLAC8015D driver. RPM Left:  %.1f, RPM Right:  %.1f", rpm_left, rpm_right);
        motors_.set_decel_time(3000);
        motors_.set_accel_time(3000);
        motors_.set_sync_rpm(1000.0, 1000.0); // Position 1000 RPM // Velocity 3000 RPM
        RCLCPP_WARN(this->get_logger(), "For security the speed is set to 100 RPM, Acceleration and deceleration set to 16 seconds");
      }
    }

    if (error_msg_left == "0x0400" || error_msg_right == "0x0400") {  // Over Temperature Fault
      warned_overheat = true;
      auto[temp_left, temp_right] = motors_.get_temperature();
      if (temp_left > 100 || temp_right > 100) {
        RCLCPP_WARN(rclcpp::get_logger(node_name), "Temperature too high detected in ZLAC8015D driver. Temp Left: %d °C, Temp Right: %d °C", temp_left, temp_right);
        motors_.enable_parking_mode();
        RCLCPP_WARN(rclcpp::get_logger(node_name), "For security the current is limited to 3A, but the motor can still be used. Monitor the temperature and reduce the load to prevent overheating.");
      }
    }

    // Try of recuperation
    motors_.reset_alarm();
    motors_.enable_motor();
  }

  void wheel_ticks_timer(){
    auto [count_left, count_right] = motors_.get_encoder_count();

    msg_left_.data  = static_cast<double>(count_left);
    msg_right_.data = static_cast<double>(count_right);

    pub_left_data_->publish(msg_left_);
    pub_right_data_->publish(msg_right_);
  }
  
  void command_vel_CB(const geometry_msgs::msg::Twist::SharedPtr msg){
    // Twist to Rpm 
    static bool flag = false;
    const double vx = msg->linear.x;
    const double wz = msg->angular.z;

    // RCLCPP_DEBUG(this->get_logger(), "Angular Speed:  %.1f, Linear Speed:  %.1f", wz, vx);

    if(vx == 0.0 && wz == 0.0){
      if(!flag){
        flag = true;
        movement_lock_timer_->reset();
      }
    }
    else{
      if(flag){
        movement_lock_timer_->cancel();
        motors_.disable_parking_mode();
        motors_.enable_motor();
        flag=false;
      }
    }

    auto [rpm_l, rpm_r] = twist_to_rpm(vx, wz);
    // RCLCPP_DEBUG(this->get_logger(), "Publish Speed RPM LEFT:  %.1f, RIGHT:  %.1f", rpm_l, rpm_r);

    motors_.set_sync_rpm(rpm_l, rpm_r);
  }

  void movement_lock_timer(){
    
    if(!unlock_driver_){
      RCLCPP_DEBUG(this->get_logger(), "Driver locked, not free wheels enable");
    }
    else{
      motors_.disable_motor();
      RCLCPP_DEBUG(this->get_logger(), "Driver unlocked");
    }
    motors_.enable_parking_mode();
    movement_lock_timer_->cancel();
  }

  std::pair<float,float> twist_to_rpm(double vx, double wz){
    const double w_l = (vx - (wz * wheels_separation_ / 2.0)) / wheels_radius_;
    const double w_r = (vx + (wz * wheels_separation_ / 2.0)) / wheels_radius_;

    double rpm_l = w_l * 60.0 / (2.0 * PI);
    double rpm_r = w_r * 60.0 / (2.0 * PI);

    if (wheelL_is_backward_) rpm_l = -rpm_l;
    if (wheelR_is_backward_) rpm_r = -rpm_r;

    rpm_l = static_cast<float>(rpm_l);
    rpm_r = static_cast<float>(rpm_r);

    return {rpm_l, rpm_r};
  }
};


int main(int argc, char * argv[]) {
  auto logger = rclcpp::get_logger(node_name);

  std::string port = "";

  if (argc >= 2) {
    port = argv[1];
    // Validate if port exists
    if (access(port.c_str(), F_OK) != 0) {
      RCLCPP_ERROR(logger, "Port %s does not exist", port.c_str());
      return 1;
    }
    RCLCPP_INFO(logger, "Port: %s", port.c_str());
  }else{
    RCLCPP_ERROR(logger, "No argument given. Usage: \n$ ros2 run <package_name> <node_name> <port>");
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZlacNode>(port));
  rclcpp::shutdown();
  return 0;
}
