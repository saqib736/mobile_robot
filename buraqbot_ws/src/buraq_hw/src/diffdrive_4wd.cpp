#include "buraq_hw/diffdrive_4wd.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace buraq_hw {

hardware_interface::CallbackReturn DiffDriveBuraq::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from the URDF
  config_.front_left_wheel_name = info_.hardware_parameters[kFrontLeftWheelNameParam];
  config_.front_right_wheel_name = info_.hardware_parameters[kFrontRightWheelNameParam];
  config_.rear_left_wheel_name = info_.hardware_parameters[kRearLeftWheelNameParam];
  config_.rear_right_wheel_name = info_.hardware_parameters[kRearRightWheelNameParam];
  config_.serial_device = info_.hardware_parameters[kSerialDeviceParam];
  config_.baud_rate = std::stoi(info_.hardware_parameters[kBaudRateParam]);
  config_.timeout = std::stoi(info_.hardware_parameters[kTimeoutParam]);
  config_.front_left_enc_counts_per_rev = std::stoi(info_.hardware_parameters[kFrontLeftEncTicksPerRevParam]);
  config_.front_right_enc_counts_per_rev = std::stoi(info_.hardware_parameters[kFrontRightEncTicksPerRevParam]);
  config_.rear_left_enc_counts_per_rev = std::stoi(info_.hardware_parameters[kRearLeftEncTicksPerRevParam]);
  config_.rear_right_enc_counts_per_rev = std::stoi(info_.hardware_parameters[kRearRightEncTicksPerRevParam]);
  config_.loop_rate = std::stof(info_.hardware_parameters[kLoopRateParam]);
  config_.pid_p = std::stof(info_.hardware_parameters[kPidPParam]);
  config_.pid_d = std::stof(info_.hardware_parameters[kPidDParam]);
  config_.pid_i = std::stof(info_.hardware_parameters[kPidIParam]);
  config_.pid_o = std::stof(info_.hardware_parameters[kPidOParam]);

  // Set up the wheels
  front_left_wheel_.Setup(config_.front_left_wheel_name, config_.front_left_enc_counts_per_rev);
  front_right_wheel_.Setup(config_.front_right_wheel_name, config_.front_right_enc_counts_per_rev);
  rear_left_wheel_.Setup(config_.rear_left_wheel_name, config_.rear_left_enc_counts_per_rev);
  rear_right_wheel_.Setup(config_.rear_right_wheel_name, config_.rear_right_enc_counts_per_rev);
  
  RCLCPP_INFO(
      logger_,
      "DiffDriveBuraq hardware interface initialized with:"
      " front_left_wheel: %s, front_right_wheel: %s"
      " rear_left_wheel: %s, rear_right_wheel: %s"
      " serial_device: %s, baud_rate: %d, timeout: %d"
      " front_left_enc_counts_per_rev: %d, front_right_enc_counts_per_rev: %d"
      " rear_left_enc_counts_per_rev: %d, rear_right_enc_counts_per_rev: %d"
      " loop_rate: %f, pid_p: %d, pid_i: %d, pid_d: %d, pid_o: %d",
      config_.front_left_wheel_name.c_str(), config_.front_right_wheel_name.c_str(),
      config_.rear_left_wheel_name.c_str(), config_.rear_right_wheel_name.c_str(),
      config_.serial_device.c_str(), config_.baud_rate, config_.timeout,
      config_.front_left_enc_counts_per_rev, config_.front_right_enc_counts_per_rev,
      config_.rear_left_enc_counts_per_rev, config_.rear_right_enc_counts_per_rev,
      config_.loop_rate, config_.pid_p, config_.pid_i, config_.pid_d, config_.pid_o);
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBuraq::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Connect to the hw driver
  hw_driver_.Setup(config_.serial_device, config_.baud_rate, config_.timeout);
  
  if (!hw_driver_.is_connected()) {
    RCLCPP_ERROR(logger_, "Failed to connect to hw driver on %s", config_.serial_device.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Send an empty message to ensure the hw driver is ready
  hw_driver_.SendEmptyMsg();
  
  // Set initial PID values (can be adjusted later)
  hw_driver_.SetPidValues(config_.pid_p, config_.pid_i, config_.pid_d, config_.pid_o);

  RCLCPP_INFO(logger_, "DiffDriveBuraq hardware interface configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveBuraq::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add state interfaces for the front left wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      front_left_wheel_.name_, hardware_interface::HW_IF_POSITION, &front_left_wheel_.pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      front_left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &front_left_wheel_.vel_));

  // Add state interfaces for the front right wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      front_right_wheel_.name_, hardware_interface::HW_IF_POSITION, &front_right_wheel_.pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      front_right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &front_right_wheel_.vel_));
      
  // Add state interfaces for the rear left wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      rear_left_wheel_.name_, hardware_interface::HW_IF_POSITION, &rear_left_wheel_.pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      rear_left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_.vel_));

  // Add state interfaces for the rear right wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      rear_right_wheel_.name_, hardware_interface::HW_IF_POSITION, &rear_right_wheel_.pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      rear_right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_.vel_));

  // Add IMU state interfaces
  // Orientation (quaternion)
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.x", &imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.y", &imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.z", &imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "orientation.w", &imu_orientation_[3]));

  // Angular velocity
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &imu_angular_velocity_[2]));

  // Linear acceleration
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &imu_linear_acceleration_[2]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveBuraq::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Add command interfaces for the front left wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      front_left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &front_left_wheel_.vel_set_pt_));

  // Add command interfaces for the front right wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      front_right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &front_right_wheel_.vel_set_pt_));
      
  // Add command interfaces for the rear left wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      rear_left_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &rear_left_wheel_.vel_set_pt_));

  // Add command interfaces for the rear right wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      rear_right_wheel_.name_, hardware_interface::HW_IF_VELOCITY, &rear_right_wheel_.vel_set_pt_));

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffDriveBuraq::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Reset encoders to zero on activation
  hw_driver_.SendMsg("r");
  
  // Read initial encoder values
  auto encoder_values = hw_driver_.ReadEncoderValues();
  
  // Set initial wheel positions
  front_left_wheel_.enc_ = encoder_values[0];  // Front left
  front_right_wheel_.enc_ = encoder_values[1]; // Front right
  rear_left_wheel_.enc_ = encoder_values[2];   // Rear left
  rear_right_wheel_.enc_ = encoder_values[3];  // Rear right
  
  front_left_wheel_.pos_ = front_left_wheel_.Angle();
  front_right_wheel_.pos_ = front_right_wheel_.Angle();
  rear_left_wheel_.pos_ = rear_left_wheel_.Angle();
  rear_right_wheel_.pos_ = rear_right_wheel_.Angle();
  
  // Set initial velocities to zero
  front_left_wheel_.vel_ = 0.0;
  front_right_wheel_.vel_ = 0.0;
  rear_left_wheel_.vel_ = 0.0;
  rear_right_wheel_.vel_ = 0.0;
  
  front_left_wheel_.vel_set_pt_ = 0.0;
  front_right_wheel_.vel_set_pt_ = 0.0;
  rear_left_wheel_.vel_set_pt_ = 0.0;
  rear_right_wheel_.vel_set_pt_ = 0.0;
  
  RCLCPP_INFO(logger_, "DiffDriveBuraq hardware interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveBuraq::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Stop all motors on deactivation
  hw_driver_.SetMotorValues(0, 0, 0, 0);
  
  RCLCPP_INFO(logger_, "DiffDriveBuraq hardware interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveBuraq::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  // Read encoder values from the hw driver
  auto encoder_values = hw_driver_.ReadEncoderValues();
  
  // Read IMU data from the hw driver
  auto imu_data = hw_driver_.ReadIMUData();
  
  // Update IMU data storage (convert to ROS coordinate system if needed)
  // Note: Quaternion order in ROS is (x, y, z, w) but our firmware sends (w, x, y, z)
  imu_orientation_[0] = imu_data.qx;  // x
  imu_orientation_[1] = imu_data.qy;  // y
  imu_orientation_[2] = imu_data.qz;  // z
  imu_orientation_[3] = imu_data.qw;  // w
  
  // Angular velocity (rad/s)
  imu_angular_velocity_[0] = imu_data.gx;  // x
  imu_angular_velocity_[1] = imu_data.gy;  // y
  imu_angular_velocity_[2] = imu_data.gz;  // z
  
  // Linear acceleration (m/s^2)
  imu_linear_acceleration_[0] = imu_data.ax;  // x
  imu_linear_acceleration_[1] = imu_data.ay;  // y
  imu_linear_acceleration_[2] = imu_data.az;  // z
  
  // Update wheel positions and velocities for all four wheels
  int32_t new_front_left_enc = encoder_values[0];  // Front left
  int32_t new_front_right_enc = encoder_values[1]; // Front right
  int32_t new_rear_left_enc = encoder_values[2];   // Rear left
  int32_t new_rear_right_enc = encoder_values[3];  // Rear right
  
  // Calculate change in encoder ticks
  int32_t delta_front_left_enc = new_front_left_enc - front_left_wheel_.enc_;
  int32_t delta_front_right_enc = new_front_right_enc - front_right_wheel_.enc_;
  int32_t delta_rear_left_enc = new_rear_left_enc - rear_left_wheel_.enc_;
  int32_t delta_rear_right_enc = new_rear_right_enc - rear_right_wheel_.enc_;
  
  // Update stored encoder values
  front_left_wheel_.enc_ = new_front_left_enc;
  front_right_wheel_.enc_ = new_front_right_enc;
  rear_left_wheel_.enc_ = new_rear_left_enc;
  rear_right_wheel_.enc_ = new_rear_right_enc;
  
  // Update wheel positions (in radians)
  front_left_wheel_.pos_ = front_left_wheel_.Angle();
  front_right_wheel_.pos_ = front_right_wheel_.Angle();
  rear_left_wheel_.pos_ = rear_left_wheel_.Angle();
  rear_right_wheel_.pos_ = rear_right_wheel_.Angle();
  
  // Calculate wheel velocities (rad/s)
  // Note: This is a simple calculation and might need filtering in a real system
  const double dt = period.seconds();
  
  // Calculate velocities by multiplying delta ticks by radians per tick and dividing by time period
  // The formula is: velocity = (delta_position) / time_period
  // Where delta_position = delta_ticks * radians_per_tick
  front_left_wheel_.vel_ = (delta_front_left_enc * front_left_wheel_.rads_per_tick_) / dt;
  front_right_wheel_.vel_ = (delta_front_right_enc * front_right_wheel_.rads_per_tick_) / dt;
  rear_left_wheel_.vel_ = (delta_rear_left_enc * rear_left_wheel_.rads_per_tick_) / dt;
  rear_right_wheel_.vel_ = (delta_rear_right_enc * rear_right_wheel_.rads_per_tick_) / dt;
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffDriveBuraq::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Convert velocity commands from rad/s to ticks/s for all four wheels
  int front_left_ticks_per_sec = static_cast<int>(front_left_wheel_.vel_set_pt_ / front_left_wheel_.rads_per_tick_ / config_.loop_rate);
  int front_right_ticks_per_sec = static_cast<int>(front_right_wheel_.vel_set_pt_ / front_right_wheel_.rads_per_tick_ / config_.loop_rate);
  int rear_left_ticks_per_sec = static_cast<int>(rear_left_wheel_.vel_set_pt_ / rear_left_wheel_.rads_per_tick_ / config_.loop_rate);
  int rear_right_ticks_per_sec = static_cast<int>(rear_right_wheel_.vel_set_pt_ / rear_right_wheel_.rads_per_tick_ / config_.loop_rate);
  
  // Send commands to all four motors
  // Order: Front left, Front right, Rear left, Rear right
  hw_driver_.SetMotorValues(front_left_ticks_per_sec, front_right_ticks_per_sec, rear_left_ticks_per_sec, rear_right_ticks_per_sec);
  
  return hardware_interface::return_type::OK;
}

}  // namespace buraq_hw

// Register the hardware interface as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    buraq_hw::DiffDriveBuraq, hardware_interface::SystemInterface)