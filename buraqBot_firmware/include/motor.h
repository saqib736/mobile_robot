/**
 * @file motor.h
 * @brief Motor control interface with PID for the 4WD Mobile Robot
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h> // Using the PID library for control
#include "config.h"

class MotorController {
private:
    // Motor pins
    uint8_t pwmPins[NUM_MOTORS];
    uint8_t dir1Pins[NUM_MOTORS];
    uint8_t dir2Pins[NUM_MOTORS];
    
    // Motor state
    double targetSpeeds[NUM_MOTORS]; // Target speed in ticks/s for closed loop or PWM for open loop
    double currentSpeeds[NUM_MOTORS]; // Current speed in ticks/s
    double outputPwm[NUM_MOTORS];     // PWM output (0-255)
    bool closedLoopMode[NUM_MOTORS];  // Whether motor is in closed-loop control mode
    
    // PID controllers
    PID* pidControllers[NUM_MOTORS];
    double pidKp, pidKi, pidKd, pidKo; // PID gains
    
public:
    /**
     * @brief Constructor
     */
    MotorController();
    
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
     * @param speed Speed value (-255 to 255), negative for reverse
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
     * @param speeds Array of speed values (-255 to 255)
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
    void setPIDGains(double kp, double ki, double kd, double ko);
    
    /**
     * @brief Get current PID gains
     * @param kp Pointer to store proportional gain
     * @param ki Pointer to store integral gain
     * @param kd Pointer to store derivative gain
     * @param ko Pointer to store output coefficient
     */
    void getPIDGains(double* kp, double* ki, double* kd, double* ko);
    
    /**
     * @brief Stop all motors
     */
    void stopAll();
    
    /**
     * @brief Get formatted motor data for debugging
     * @return String containing motor data
     */
    String getFormattedData();
};

#endif // MOTOR_H
