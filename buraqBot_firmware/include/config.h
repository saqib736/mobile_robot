/**
 * @file config.h
 * @brief Configuration file for 4WD Mobile Robot with Teensy 4.1
 * @details Contains pin definitions and constants for the robot
 */

#ifndef CONFIG_H
#define CONFIG_H

// Motor Driver Pins (L298N)
// Motor 1 - Front Left
#define MOTOR_FL_PWM 1     // PWM pin for speed control
#define MOTOR_FL_DIR1 20     // Direction pin 1
#define MOTOR_FL_DIR2 21     // Direction pin 2
#define ENCODER_FL_A 15      // Encoder A pin
#define ENCODER_FL_B 14      // Encoder B pin

// Motor 2 - Front Right
#define MOTOR_FR_PWM 5      // PWM pin for speed control
#define MOTOR_FR_DIR1 8     // Direction pin 1
#define MOTOR_FR_DIR2 6     // Direction pin 2
#define ENCODER_FR_A 11     // Encoder A pin
#define ENCODER_FR_B 12     // Encoder B pin

// Motor 3 - Rear Left
#define MOTOR_RL_PWM 22     // PWM pin for speed control
#define MOTOR_RL_DIR1 23    // Direction pin 1
#define MOTOR_RL_DIR2 0    // Direction pin 2
#define ENCODER_RL_A 16     // Encoder A pin
#define ENCODER_RL_B 17     // Encoder B pin

// Motor 4 - Rear Right
#define MOTOR_RR_PWM 4     // PWM pin for speed control
#define MOTOR_RR_DIR1 2    // Direction pin 1
#define MOTOR_RR_DIR2 3    // Direction pin 2
#define ENCODER_RR_A 9     // Encoder A pin
#define ENCODER_RR_B 10    // Encoder B pin

// IMU - MPU6050 (I2C)
#define IMU_SDA 18          // I2C SDA pin
#define IMU_SCL 19          // I2C SCL pin

// Serial Communication
#define SERIAL_BAUD_RATE 115200  // Baud rate for communication with ROS2

// Motor Control Constants
#define PWM_FREQUENCY 20000      // PWM frequency in Hz
#define PWM_RESOLUTION 10         // PWM resolution in bits (0-1023)
#define MOTOR_MAX_RPM 200        // Maximum motor RPM

// Individual encoder ticks per revolution for each motor
#define ENCODER_FL_TICKS_PER_REV 2410 // Front Left encoder ticks per revolution
#define ENCODER_FR_TICKS_PER_REV 2452 // Front Right encoder ticks per revolution
#define ENCODER_RL_TICKS_PER_REV 2369 // Rear Left encoder ticks per revolution
#define ENCODER_RR_TICKS_PER_REV 2327 // Rear Right encoder ticks per revolution

// PID Default Parameters
#define PID_KP 0.3               // Proportional gain - reduced to prevent oscillation
#define PID_KI 0.05              // Integral gain - reduced to prevent windup
#define PID_KD 0.005             // Derivative gain - reduced to prevent noise amplification
#define PID_KO 0.8               // Output coefficient - reduced to soften response
#define PID_SAMPLE_TIME 20       // PID calculation interval in milliseconds - increased for stability

// Communication Protocol Constants
#define COMMAND_READ_ENCODERS 'e'    // Read encoder values
#define COMMAND_RESET_ENCODERS 'r'   // Reset encoder values
#define COMMAND_SET_CLOSED_LOOP 'm'  // Set motors speed with PID (ticks/s)
#define COMMAND_SET_OPEN_LOOP 'o'    // Set motors speed directly (PWM duty)
#define COMMAND_SET_PID_GAINS 'u'    // Set PID gains
#define COMMAND_READ_IMU 'i'         // Read IMU data

// System Constants
#define NUM_MOTORS 4                 // Number of motors
#define LOOP_FREQUENCY 30           // Main loop frequency in Hz
#define SERIAL_TIMEOUT 5             // Serial timeout in milliseconds

#endif // CONFIG_H
