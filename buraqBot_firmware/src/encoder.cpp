/**
 * @file encoder.cpp
 * @brief Implementation of the EncoderManager class
 */

#include "encoder.h"

EncoderManager::EncoderManager() {
    // Initialize arrays
    for (int i = 0; i < NUM_MOTORS; i++) {
        encoders[i] = nullptr;
        encoderCounts[i] = 0;
        prevEncoderCounts[i] = 0;
        speeds[i] = 0.0;
    }
    lastSpeedUpdateTime = 0;
}

EncoderManager::~EncoderManager() {
    // Clean up encoder objects
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (encoders[i] != nullptr) {
            delete encoders[i];
            encoders[i] = nullptr;
        }
    }
}

void EncoderManager::init() {
    // Create encoder objects for each motor
    encoders[0] = new Encoder(ENCODER_FL_A, ENCODER_FL_B); // Front Left
    encoders[1] = new Encoder(ENCODER_FR_A, ENCODER_FR_B); // Front Right
    encoders[2] = new Encoder(ENCODER_RL_A, ENCODER_RL_B); // Rear Left
    encoders[3] = new Encoder(ENCODER_RR_A, ENCODER_RR_B); // Rear Right
    
    // Initialize timestamps and counts
    lastSpeedUpdateTime = millis();
    reset();
}

void EncoderManager::update() {
    // Read current encoder values
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (encoders[i] != nullptr) {
            encoderCounts[i] = encoders[i]->read();
        }
    }
    
    // Calculate speeds (ticks per second)
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastSpeedUpdateTime;
    
    if (deltaTime >= 50) { // Update speed every 50ms for stability
        float timeSeconds = deltaTime / 1000.0;
        
        for (int i = 0; i < NUM_MOTORS; i++) {
            long deltaTicks = encoderCounts[i] - prevEncoderCounts[i];
            speeds[i] = deltaTicks / timeSeconds;
            prevEncoderCounts[i] = encoderCounts[i];
        }
        
        lastSpeedUpdateTime = currentTime;
    }
}

void EncoderManager::reset() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (encoders[i] != nullptr) {
            encoders[i]->write(0);
            encoderCounts[i] = 0;
            prevEncoderCounts[i] = 0;
            speeds[i] = 0.0;
        }
    }
}

long EncoderManager::getCount(uint8_t motorIndex) {
    if (motorIndex < NUM_MOTORS) {
        return encoderCounts[motorIndex];
    }
    return 0;
}

float EncoderManager::getSpeed(uint8_t motorIndex) {
    if (motorIndex < NUM_MOTORS) {
        return speeds[motorIndex];
    }
    return 0.0;
}

String EncoderManager::getFormattedCounts() {
    // Format: count1:count2:count3:count4
    String data = String(encoderCounts[0]) + ":" +
                 String(encoderCounts[1]) + ":" +
                 String(encoderCounts[2]) + ":" +
                 String(encoderCounts[3]);
    return data;
}

String EncoderManager::getFormattedSpeeds() {
    // Format: speed1:speed2:speed3:speed4
    String data = String(speeds[0]) + ":" +
                 String(speeds[1]) + ":" +
                 String(speeds[2]) + ":" +
                 String(speeds[3]);
    return data;
}
