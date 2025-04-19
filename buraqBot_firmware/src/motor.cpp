/**
 * @file motor.cpp
 * @brief Implementation of the MotorController class with custom PID controller
 */

#include "motor.h"
#include "encoder.h"

// Global pointer to the motor controller for the custom PID implementation
MotorController* g_motorController = nullptr;

// Implementation of encoder reading function for custom PID
long readEncoder(int motor) {
    if (g_motorController != nullptr) {
        return g_motorController->getEncoderValue(motor);
    }
    return 0;
}

// Implementation of motor speed setting function for custom PID
void setMotorSpeeds(long leftSpeed, long rightSpeed, long left2Speed, long right2Speed) {
    if (g_motorController != nullptr) {
        g_motorController->applyMotorOutput(LEFT, leftSpeed);
        g_motorController->applyMotorOutput(RIGHT, rightSpeed);
        g_motorController->applyMotorOutput(LEFT2, left2Speed);
        g_motorController->applyMotorOutput(RIGHT2, right2Speed);
    }
}

MotorController::MotorController(EncoderManager* encManager) {
    // Store encoder manager reference
    encoderManager = encManager;
    
    // Set global pointer for custom PID
    g_motorController = this;
    
    // Initialize arrays
    for (int i = 0; i < NUM_MOTORS; i++) {
        targetSpeeds[i] = 0;
        currentSpeeds[i] = 0.0;
        outputPwm[i] = 0.0;
        closedLoopMode[i] = false;
        lastEncoderValues[i] = 0;
    }
    
    // Use PID values from config.h
    pidKp = PID_KP * 1000;  // Convert to int format for custom PID
    pidKi = PID_KI * 1000;  // Convert to int format for custom PID
    pidKd = PID_KD * 1000;  // Convert to int format for custom PID
    pidKo = PID_KO;
    
    // Update the global PID parameters in custom_pid.h
    Kp = pidKp;
    Ki = pidKi;
    Kd = pidKd;
    Ko = pidKo;
}

MotorController::~MotorController() {
    // Clear global pointer
    if (g_motorController == this) {
        g_motorController = nullptr;
    }
}

void MotorController::init() {
    // Set up motor pins
    pwmPins[0] = MOTOR_FL_PWM;
    dir1Pins[0] = MOTOR_FL_DIR1;
    dir2Pins[0] = MOTOR_FL_DIR2;
    
    pwmPins[1] = MOTOR_FR_PWM;
    dir1Pins[1] = MOTOR_FR_DIR1;
    dir2Pins[1] = MOTOR_FR_DIR2;
    
    pwmPins[2] = MOTOR_RL_PWM;
    dir1Pins[2] = MOTOR_RL_DIR1;
    dir2Pins[2] = MOTOR_RL_DIR2;
    
    pwmPins[3] = MOTOR_RR_PWM;
    dir1Pins[3] = MOTOR_RR_DIR1;
    dir2Pins[3] = MOTOR_RR_DIR2;
    
    // Configure pins
    for (int i = 0; i < NUM_MOTORS; i++) {
        pinMode(pwmPins[i], OUTPUT);
        pinMode(dir1Pins[i], OUTPUT);
        pinMode(dir2Pins[i], OUTPUT);
        
        // Initialize motors to stopped state
        analogWrite(pwmPins[i], 0);
        digitalWrite(dir1Pins[i], LOW);
        digitalWrite(dir2Pins[i], LOW);
    }
    
    // Initialize custom PID
    resetPID();
}

void MotorController::applyMotorOutput(uint8_t motorIndex, long output) {
    if (motorIndex >= NUM_MOTORS) return;
    
    // Constrain output to valid PWM range (10-bit resolution)
    output = constrain(output, -MAX_PWM, MAX_PWM);
    
    // Store output for telemetry
    outputPwm[motorIndex] = output;
    
    // Apply to motor
    int pwmValue = abs(output);
    bool forward = (output >= 0);
    
    analogWrite(pwmPins[motorIndex], pwmValue);
    digitalWrite(dir1Pins[motorIndex], forward ? HIGH : LOW);
    digitalWrite(dir2Pins[motorIndex], forward ? LOW : HIGH);
}

void MotorController::setOpenLoopSpeed(uint8_t motorIndex, double speed) {
    if (motorIndex >= NUM_MOTORS) return;
    
    // Constrain speed to valid PWM range (10-bit resolution)
    speed = constrain(speed, -MAX_PWM, MAX_PWM);
    
    // Set mode and target
    closedLoopMode[motorIndex] = false;
    targetSpeeds[motorIndex] = speed;
    
    // Apply immediately
    applyMotorOutput(motorIndex, speed);
}

void MotorController::setClosedLoopSpeed(uint8_t motorIndex, double targetSpeed) {
    if (motorIndex >= NUM_MOTORS) return;
    
    // Set mode and target
    closedLoopMode[motorIndex] = true;
    targetSpeeds[motorIndex] = targetSpeed;
    
    // Update the corresponding PID target in the custom PID controller
    switch (motorIndex) {
        case LEFT:
            leftPID.TargetTicksPerFrame = targetSpeed;
            break;
        case RIGHT:
            rightPID.TargetTicksPerFrame = targetSpeed;
            break;
        case LEFT2:
            left2PID.TargetTicksPerFrame = targetSpeed;
            break;
        case RIGHT2:
            right2PID.TargetTicksPerFrame = targetSpeed;
            break;
    }
    
    // Set the moving flag to enable PID calculations
    moving = (targetSpeed != 0);
    
    // Print debug info about the target speed and encoder ticks
    if (encoderManager != nullptr) {
        uint16_t tpr = encoderManager->getTicksPerRev(motorIndex);
        Serial.print("Motor ");
        Serial.print(motorIndex);
        Serial.print(": Target Speed = ");
        Serial.print(targetSpeed);
        Serial.print(" ticks/s, TPR = ");
        Serial.println(tpr);
    }
}

void MotorController::setAllOpenLoop(double speeds[NUM_MOTORS]) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        setOpenLoopSpeed(i, speeds[i]);
    }
}

void MotorController::setAllClosedLoop(double speeds[NUM_MOTORS]) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        setClosedLoopSpeed(i, speeds[i]);
    }
}

void MotorController::update(float currentEncoderSpeeds[NUM_MOTORS]) {
    // Update current speeds from encoder readings
    for (int i = 0; i < NUM_MOTORS; i++) {
        currentSpeeds[i] = currentEncoderSpeeds[i];
    }
    
    // If any motor is in closed-loop mode, use the custom PID controller
    bool anyClosedLoop = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (closedLoopMode[i]) {
            anyClosedLoop = true;
            break;
        }
    }
    
    if (anyClosedLoop) {
        // Update the custom PID controller
        updatePID();
    }
}

void MotorController::setPIDGains(int kp, int ki, int kd, int ko) {
    pidKp = kp;
    pidKi = ki;
    pidKd = kd;
    pidKo = ko;
    
    // Update the global PID parameters in custom_pid.h
    Kp = kp;
    Ki = ki;
    Kd = kd;
    Ko = ko;
}

void MotorController::getPIDGains(int* kp, int* ki, int* kd, int* ko) {
    *kp = pidKp;
    *ki = pidKi;
    *kd = pidKd;
    *ko = pidKo;
}

void MotorController::stopAll() {
    // Set the moving flag to disable PID calculations
    moving = false;
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Set targets to zero
        targetSpeeds[i] = 0;
        
        // Update the corresponding PID target in the custom PID controller
        switch (i) {
            case LEFT:
                leftPID.TargetTicksPerFrame = 0;
                break;
            case RIGHT:
                rightPID.TargetTicksPerFrame = 0;
                break;
            case LEFT2:
                left2PID.TargetTicksPerFrame = 0;
                break;
            case RIGHT2:
                right2PID.TargetTicksPerFrame = 0;
                break;
        }
        
        // Stop motors immediately
        analogWrite(pwmPins[i], 0);
        digitalWrite(dir1Pins[i], LOW);
        digitalWrite(dir2Pins[i], LOW);
    }
}

String MotorController::getFormattedData() {
    // Format: target1:current1:pwm1:mode1:target2:current2:pwm2:mode2:...
    String data = "";
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        data += String(targetSpeeds[i]) + ":" +
                String(currentSpeeds[i]) + ":" +
                String(outputPwm[i]) + ":" +
                String(closedLoopMode[i] ? 1 : 0);
        
        if (i < NUM_MOTORS - 1) {
            data += ":";
        }
    }
    
    return data;
}

long MotorController::getEncoderValue(uint8_t motorIndex) {
    if (motorIndex >= NUM_MOTORS || encoderManager == nullptr) {
        return 0;
    }
    
    // Get the current encoder value from the encoder manager
    return encoderManager->getCount(motorIndex);
}
