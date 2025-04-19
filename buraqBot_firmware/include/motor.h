/**
 * @file motor.h
 * @brief Motor control interface with custom PID for the 4WD Mobile Robot
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "custom_pid.h" // Using the custom PID implementation
#include "config.h"

// Define motor indices for the custom PID controller
#define LEFT 0
#define RIGHT 1
#define LEFT2 2
#define RIGHT2 3

// Define maximum PWM value (for 10-bit resolution)
#define MAX_PWM 1023

// Forward declaration
class EncoderManager;

// Forward declaration of encoder reading function for custom PID
long readEncoder(int motor);

// Forward declaration of motor speed setting function for custom PID
void setMotorSpeeds(long leftSpeed, long rightSpeed, long left2Speed, long right2Speed);

class MotorController {
private:
    // Reference to the encoder manager
    EncoderManager* encoderManager;
    
    // Motor pins
    uint8_t pwmPins[NUM_MOTORS];
    uint8_t dir1Pins[NUM_MOTORS];
    uint8_t dir2Pins[NUM_MOTORS];
    
    // Motor state
    double targetSpeeds[NUM_MOTORS]; // Target speed in ticks/s for closed loop or PWM for open loop
    double currentSpeeds[NUM_MOTORS]; // Current speed in ticks/s
    double outputPwm[NUM_MOTORS];     // PWM output (0-1023)
    bool closedLoopMode[NUM_MOTORS];  // Whether motor is in closed-loop control mode
    
    // PID gains (used for both custom PID and direct calculation)
    int pidKp, pidKi, pidKd, pidKo; // PID gains
    
    // Last encoder values (for direct PID calculation)
    long lastEncoderValues[NUM_MOTORS];
    
    // Helper function to apply motor output
    void applyMotorOutput(uint8_t motorIndex, long output);
    
public:
    /**
     * @brief Constructor
     * @param encManager Pointer to the EncoderManager instance
     */
    MotorController(EncoderManager* encManager = nullptr);
    
    /**
     * @brief Destructor
     */
    ~MotorController();
    
    /**
     * @brief Initialize the motor controller
     */
    void init();
    
    /**
     * @brief Set motor speed in open-loop mode (direct PWM)
     * @param motorIndex Motor index (0-3)
     * @param speed Speed value (-1023 to 1023), negative for reverse
     */
    void setOpenLoopSpeed(uint8_t motorIndex, double speed);
    
    /**
     * @brief Set motor speed in closed-loop mode (PID controlled)
     * @param motorIndex Motor index (0-3)
     * @param targetSpeed Target speed in ticks per second, negative for reverse
     */
    void setClosedLoopSpeed(uint8_t motorIndex, double targetSpeed);
    
    /**
     * @brief Set all motors to open-loop mode with specified speeds
     * @param speeds Array of speed values (-1023 to 1023)
     */
    void setAllOpenLoop(double speeds[NUM_MOTORS]);
    
    /**
     * @brief Set all motors to closed-loop mode with specified speeds
     * @param speeds Array of target speeds in ticks per second
     */
    void setAllClosedLoop(double speeds[NUM_MOTORS]);
    
    /**
     * @brief Update motor control (should be called in main loop)
     * @param currentEncoderSpeeds Current encoder speeds in ticks per second
     */
    void update(float currentEncoderSpeeds[NUM_MOTORS]);
    
    /**
     * @brief Set PID gains for all motors
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param ko Output coefficient
     */
    void setPIDGains(int kp, int ki, int kd, int ko);
    
    /**
     * @brief Get current PID gains
     * @param kp Pointer to store proportional gain
     * @param ki Pointer to store integral gain
     * @param kd Pointer to store derivative gain
     * @param ko Pointer to store output coefficient
     */
    void getPIDGains(int* kp, int* ki, int* kd, int* ko);
    
    /**
     * @brief Stop all motors
     */
    void stopAll();
    
    /**
     * @brief Get formatted motor data for debugging
     * @return String containing motor data
     */
    String getFormattedData();
    
    /**
     * @brief Get encoder value for the custom PID controller
     * @param motorIndex Motor index (0-3)
     * @return Current encoder value
     */
    long getEncoderValue(uint8_t motorIndex);
};

// Global pointer to the motor controller for the custom PID implementation
extern MotorController* g_motorController;

#endif // MOTOR_H
