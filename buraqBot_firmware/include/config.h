/**
 * @file config.h
 * @brief Configuration file for 4WD Mobile Robot with Teensy 4.1
 * @details Contains pin definitions and constants for the robot
 */

#ifndef CONFIG_H
#define CONFIG_H

// Motor Driver Pins (L298N)
// Motor 1 - Front Left
#define MOTOR_FL_PWM 2      // PWM pin for speed control
#define MOTOR_FL_DIR1 3     // Direction pin 1
#define MOTOR_FL_DIR2 4     // Direction pin 2
#define ENCODER_FL_A 5      // Encoder A pin
#define ENCODER_FL_B 6      // Encoder B pin

// Motor 2 - Front Right
#define MOTOR_FR_PWM 7      // PWM pin for speed control
#define MOTOR_FR_DIR1 8     // Direction pin 1
#define MOTOR_FR_DIR2 9     // Direction pin 2
#define ENCODER_FR_A 10     // Encoder A pin
#define ENCODER_FR_B 11     // Encoder B pin

// Motor 3 - Rear Left
#define MOTOR_RL_PWM 12     // PWM pin for speed control
#define MOTOR_RL_DIR1 24    // Direction pin 1
#define MOTOR_RL_DIR2 25    // Direction pin 2
#define ENCODER_RL_A 26     // Encoder A pin
#define ENCODER_RL_B 27     // Encoder B pin

// Motor 4 - Rear Right
#define MOTOR_RR_PWM 28     // PWM pin for speed control
#define MOTOR_RR_DIR1 29    // Direction pin 1
#define MOTOR_RR_DIR2 30    // Direction pin 2
#define ENCODER_RR_A 31     // Encoder A pin
#define ENCODER_RR_B 32     // Encoder B pin

// IMU - MPU6050 (I2C)
#define IMU_SDA 18          // I2C SDA pin
#define IMU_SCL 19          // I2C SCL pin

// Serial Communication
#define SERIAL_BAUD_RATE 115200  // Baud rate for communication with ROS2

// Motor Control Constants
#define PWM_FREQUENCY 20000      // PWM frequency in Hz
#define PWM_RESOLUTION 8         // PWM resolution in bits (0-255)
#define MOTOR_MAX_RPM 200        // Maximum motor RPM
#define ENCODER_TICKS_PER_REV 1440 // Encoder ticks per revolution (adjust based on your encoder)

// PID Default Parameters
#define PID_KP 1.0               // Proportional gain
#define PID_KI 0.1               // Integral gain
#define PID_KD 0.01              // Derivative gain
#define PID_KO 1.0               // Output coefficient
#define PID_SAMPLE_TIME 10       // PID calculation interval in milliseconds

// Communication Protocol Constants
#define COMMAND_READ_ENCODERS 'e'    // Read encoder values
#define COMMAND_RESET_ENCODERS 'r'   // Reset encoder values
#define COMMAND_SET_CLOSED_LOOP 'm'  // Set motors speed with PID (ticks/s)
#define COMMAND_SET_OPEN_LOOP 'o'    // Set motors speed directly (PWM duty)
#define COMMAND_SET_PID_GAINS 'u'    // Set PID gains
#define COMMAND_READ_IMU 'i'         // Read IMU data

// System Constants
#define NUM_MOTORS 4                 // Number of motors
#define LOOP_FREQUENCY 100           // Main loop frequency in Hz
#define SERIAL_TIMEOUT 5             // Serial timeout in milliseconds

#endif // CONFIG_H
