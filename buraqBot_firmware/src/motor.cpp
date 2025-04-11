/**
 * @file motor.cpp
 * @brief Implementation of the MotorController class
 */

#include "motor.h"
#include "encoder.h"

MotorController::MotorController(EncoderManager* encManager) {
    // Store encoder manager reference
    encoderManager = encManager;
    
    // Initialize arrays
    for (int i = 0; i < NUM_MOTORS; i++) {
        targetSpeeds[i] = 0;
        currentSpeeds[i] = 0.0;
        outputPwm[i] = 0.0;
        closedLoopMode[i] = false;
        pidControllers[i] = nullptr;
    }
    
    // Default PID values
    pidKp = PID_KP;
    pidKi = PID_KI;
    pidKd = PID_KD;
    pidKo = PID_KO;
}

MotorController::~MotorController() {
    // Clean up PID controllers
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (pidControllers[i] != nullptr) {
            delete pidControllers[i];
            pidControllers[i] = nullptr;
        }
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
    
    // Set up PID controllers
    for (int i = 0; i < NUM_MOTORS; i++) {
        pidControllers[i] = new PID(&currentSpeeds[i], &outputPwm[i], &targetSpeeds[i], 
                                   pidKp, pidKi, pidKd, DIRECT);
        pidControllers[i]->SetMode(AUTOMATIC);
        pidControllers[i]->SetSampleTime(PID_SAMPLE_TIME);
        pidControllers[i]->SetOutputLimits(-1023, 1023); // PWM range for 10-bit resolution
    }
}

void MotorController::setOpenLoopSpeed(uint8_t motorIndex, double speed) {
    if (motorIndex >= NUM_MOTORS) return;
    
    // Constrain speed to valid PWM range (10-bit resolution)
    speed = constrain(speed, -1023, 1023);
    
    // Set mode and target
    closedLoopMode[motorIndex] = false;
    targetSpeeds[motorIndex] = speed;
    
    // Apply immediately
    int pwmValue = abs(speed);
    bool forward = (speed >= 0);
    
    analogWrite(pwmPins[motorIndex], pwmValue);
    digitalWrite(dir1Pins[motorIndex], forward ? HIGH : LOW);
    digitalWrite(dir2Pins[motorIndex], forward ? LOW : HIGH);
}

void MotorController::setClosedLoopSpeed(uint8_t motorIndex, double targetSpeed) {
    if (motorIndex >= NUM_MOTORS) return;
    
    // Set mode and target
    closedLoopMode[motorIndex] = true;
    targetSpeeds[motorIndex] = targetSpeed;
    
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
        
        // If in closed-loop mode, compute PID
        if (closedLoopMode[i]) {
            // Compute new PID output
            pidControllers[i]->Compute();
            
            // Apply PID output to motor
            int pwmValue = abs(outputPwm[i] * pidKo); // Apply output coefficient
            pwmValue = constrain(pwmValue, 0, 1023); // 10-bit PWM resolution
            bool forward = (outputPwm[i] >= 0);
            
            analogWrite(pwmPins[i], pwmValue);
            digitalWrite(dir1Pins[i], forward ? HIGH : LOW);
            digitalWrite(dir2Pins[i], forward ? LOW : HIGH);
        }
    }
}

void MotorController::setPIDGains(double kp, double ki, double kd, double ko) {
    pidKp = kp;
    pidKi = ki;
    pidKd = kd;
    pidKo = ko;
    
    // Update all PID controllers
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (pidControllers[i] != nullptr) {
            pidControllers[i]->SetTunings(kp, ki, kd);
        }
    }
}

void MotorController::getPIDGains(double* kp, double* ki, double* kd, double* ko) {
    *kp = pidKp;
    *ki = pidKi;
    *kd = pidKd;
    *ko = pidKo;
}

void MotorController::stopAll() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Set targets to zero
        targetSpeeds[i] = 0;
        
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
