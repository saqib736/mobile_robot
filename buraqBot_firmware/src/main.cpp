/**
 * @file main.cpp
 * @brief Main firmware for 4WD Mobile Robot with Teensy 4.1
 * @details Integrates motor control, encoder reading, IMU, and serial communication
 */

#include <Arduino.h>
#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include "communication.h"

// Module instances
EncoderManager* encoderManager = nullptr;
MotorController* motorController = nullptr;
IMU* imu = nullptr;
Communication* communication = nullptr;

// Timing variables
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 1000 / LOOP_FREQUENCY; // Convert Hz to ms

// Status LED
const int ledPin = LED_BUILTIN;
bool ledState = false;
unsigned long lastLedToggle = 0;

void setup() {
  // Initialize LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // Turn on during initialization
  
  // Initialize modules
  encoderManager = new EncoderManager();
  motorController = new MotorController();
  imu = new IMU();
  communication = new Communication();
  
  // Initialize each module
  encoderManager->init();
  motorController->init();
  imu->init();
  communication->init();
  
  // Setup complete, turn off LED
  digitalWrite(ledPin, LOW);
  lastLoopTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Check for incoming commands
  communication->update();
  
  // Main control loop at specified frequency
  if (currentTime - lastLoopTime >= loopInterval) {
    // Update sensor readings
    encoderManager->update();
    imu->update();
    
    // Get current encoder speeds for motor control
    float speeds[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      speeds[i] = encoderManager->getSpeed(i);
    }
    
    // Update motor controllers with current speeds
    motorController->update(speeds);
    
    // Toggle LED for heartbeat
    if (currentTime - lastLedToggle >= 500) { // 2Hz blink
      ledState = !ledState;
      digitalWrite(ledPin, ledState ? HIGH : LOW);
      lastLedToggle = currentTime;
    }
    
    lastLoopTime = currentTime;
  }
}
