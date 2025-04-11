#include "buraq_hw/hw_driver.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace buraq_hw {

void HWDriver::Setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms) {
  timeout_ms_ = timeout_ms;
  
  try {
    // Open serial port
    serial_port_.Open(serial_device);
    
    // Set serial port parameters
    serial_port_.SetBaudRate(LibSerial::BaudRate(baud_rate));
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    
    // Wait for the serial connection to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    // Flush any existing data
    serial_port_.FlushIOBuffers();
  } catch (const LibSerial::OpenFailed& e) {
    std::cerr << "Error opening serial port: " << e.what() << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error setting up serial port: " << e.what() << std::endl;
  }
}

bool HWDriver::is_connected() const {
  return serial_port_.IsOpen();
}

void HWDriver::SendEmptyMsg() {
  if (!is_connected()) {
    return;
  }
  
  // Send a newline character to flush any pending commands
  try {
    serial_port_.Write("\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    serial_port_.FlushIOBuffers();
  } catch (const std::exception& e) {
    std::cerr << "Error sending empty message: " << e.what() << std::endl;
  }
}

std::string HWDriver::SendMsg(const std::string& msg_to_send) {
  if (!is_connected()) {
    return "";
  }
  
  std::string response;
  try {
    // Clear any existing data
    serial_port_.FlushIOBuffers();
    
    // Send the message with a newline character
    serial_port_.Write(msg_to_send + "\n");
    
    // Wait for response with timeout
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time).count() < timeout_ms_) {
      // Check if data is available to read
      if (serial_port_.IsDataAvailable()) {
        std::string line;
        serial_port_.ReadLine(line, '\n', timeout_ms_);
        if (!line.empty()) {
          response = line;
          break;
        }
      }
      
      // Short sleep to prevent CPU hogging
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  } catch (const std::exception& e) {
    std::cerr << "Error in serial communication: " << e.what() << std::endl;
  }
  
  return response;
}

HWDriver::Encoders HWDriver::ReadEncoderValues() {
  Encoders encoder_values = {0, 0, 0, 0};
  
  // Send 'e' command to read encoder values
  std::string response = SendMsg("e");
  
  // Parse response (format: e:value1:value2:value3:value4)
  if (!response.empty() && response[0] == 'e' && response[1] == ':') {
    std::string values_str = response.substr(2);  // Skip 'e:'
    std::stringstream ss(values_str);
    std::string token;
    
    // Parse the four encoder values
    for (int i = 0; i < 4; ++i) {
      if (std::getline(ss, token, ':')) {
        try {
          encoder_values[i] = std::stoi(token);
        } catch (const std::exception& e) {
          std::cerr << "Error parsing encoder value: " << e.what() << std::endl;
        }
      }
    }
  }
  
  return encoder_values;
}

void HWDriver::SetMotorValues(int val_1, int val_2, int val_3, int val_4) {
  // Format command for closed-loop control (m:val1:val2:val3:val4)
  std::stringstream cmd;
  cmd << "m" << val_1 << ":" << val_2 << ":" << val_3 << ":" << val_4;
  
  // Send command to set motor values
  SendMsg(cmd.str());
}

void HWDriver::SetPidValues(float k_p, float k_d, float k_i, float k_o) {
  // Format command for PID tuning (u:kp:ki:kd:ko)
  std::stringstream cmd;
  cmd << "u" << k_p << ":" << k_i << ":" << k_d << ":" << k_o;
  
  // Send command to set PID values
  SendMsg(cmd.str());
}

HWDriver::IMUData HWDriver::ReadIMUData() {
  IMUData imu_data;
  
  // Send 'i' command to read IMU data
  std::string response = SendMsg("i");
  
  // Parse response (format: i:qw:qx:qy:qz:gx:gy:gz:ax:ay:az)
  if (!response.empty() && response[0] == 'i' && response[1] == ':') {
    std::string values_str = response.substr(2);  // Skip 'i:'
    std::stringstream ss(values_str);
    std::string token;
    
    // Parse quaternion orientation
    if (std::getline(ss, token, ':')) imu_data.qw = std::stof(token);
    if (std::getline(ss, token, ':')) imu_data.qx = std::stof(token);
    if (std::getline(ss, token, ':')) imu_data.qy = std::stof(token);
    if (std::getline(ss, token, ':')) imu_data.qz = std::stof(token);
    
    // Parse angular velocity (gyroscope data)
    if (std::getline(ss, token, ':')) imu_data.gx = std::stof(token);
    if (std::getline(ss, token, ':')) imu_data.gy = std::stof(token);
    if (std::getline(ss, token, ':')) imu_data.gz = std::stof(token);
    
    // Parse linear acceleration
    if (std::getline(ss, token, ':')) imu_data.ax = std::stof(token);
    if (std::getline(ss, token, ':')) imu_data.ay = std::stof(token);
    if (std::getline(ss, token, ':')) imu_data.az = std::stof(token);
  } else {
    std::cerr << "Error: Invalid IMU data response" << std::endl;
  }
  
  return imu_data;
}

}  // namespace buraq_hw