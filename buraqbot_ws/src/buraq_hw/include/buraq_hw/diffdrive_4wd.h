#pragma once

#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "buraq_hw/hw_driver.h"
#include "buraq_hw/wheel.h"

namespace buraq_hw {

/// @brief Hardware interface for buraq robot.
/// This class is a hardware interface implementation for the buraq robot. It is responsible for
/// abstracting away the specifics of the hardware and exposing interfaces that are easy to work with.
class DiffDriveBuraq : public hardware_interface::SystemInterface {
 public:
  /// @brief Default constructor for the DiffDriveBuraq class.
  DiffDriveBuraq() = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  const std::string kFrontLeftWheelNameParam{"front_left_wheel_name"};
  const std::string kFrontRightWheelNameParam{"front_right_wheel_name"};
  const std::string kRearLeftWheelNameParam{"rear_left_wheel_name"};
  const std::string kRearRightWheelNameParam{"rear_right_wheel_name"};
  const std::string kSerialDeviceParam{"serial_device"};
  const std::string kBaudRateParam{"baud_rate"};
  const std::string kTimeoutParam{"timeout"};
  const std::string kEncTicksPerRevParam{"enc_ticks_per_rev"};
  const std::string kLoopRateParam{"loop_rate"};
  const std::string kPidPParam{"pid_p"};
  const std::string kPidIParam{"pid_i"};
  const std::string kPidDParam{"pid_d"};
  const std::string kPidOParam{"pid_o"};

  // Configuration parameters for the DiffDriveBuraq class.
  struct Config {
    // Names of the four wheels.
    std::string front_left_wheel_name = "";
    std::string front_right_wheel_name = "";
    std::string rear_left_wheel_name = "";
    std::string rear_right_wheel_name = "";
    // Encoder parameters.
    int enc_ticks_per_rev = 0;
    // Communication parameters.
    std::string serial_device = "";
    int baud_rate = 0;
    int timeout = 0;
    float loop_rate = 0.0;
    // pid parameters
    int pid_p = 0;
    int pid_d = 0;
    int pid_i = 0;
    int pid_o = 0;
  };

  // Configuration parameters.
  Config config_;
  // Communication with the firmware in charge of controlling the motors.
  HWDriver hw_driver_;
  // Four wheels of the robot.
  Wheel front_left_wheel_;
  Wheel front_right_wheel_;
  Wheel rear_left_wheel_;
  Wheel rear_right_wheel_;
  
  // IMU data storage
  double imu_orientation_[4] = {0.0, 0.0, 0.0, 1.0};  // qx, qy, qz, qw (stored in ROS order)
  double imu_angular_velocity_[3] = {0.0, 0.0, 0.0};    // gx, gy, gz
  double imu_linear_acceleration_[3] = {0.0, 0.0, 0.0};  // ax, ay, az
  
  // Logger.
  rclcpp::Logger logger_{rclcpp::get_logger("DiffDriveBuraq")};
};

}  // namespace buraq_hw