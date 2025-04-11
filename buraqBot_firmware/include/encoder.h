/**
 * @file encoder.h
 * @brief Encoder interface for the 4WD Mobile Robot
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

// Include the Encoder library
#include "../.pio/libdeps/teensy41/Encoder/Encoder.h"

class EncoderManager {
private:
    Encoder* encoders[NUM_MOTORS]; // Array of encoder objects
    long encoderCounts[NUM_MOTORS]; // Current encoder counts
    long prevEncoderCounts[NUM_MOTORS]; // Previous encoder counts for speed calculation
    float speeds[NUM_MOTORS]; // Calculated speeds in ticks per second
    unsigned long lastSpeedUpdateTime; // Timestamp for speed calculation
    uint16_t ticksPerRev[NUM_MOTORS]; // Ticks per revolution for each encoder
    
public:
    /**
     * @brief Constructor
     */
    EncoderManager();
    
    /**
     * @brief Destructor
     */
    ~EncoderManager();
    
    /**
     * @brief Initialize all encoders
     */
    void init();
    
    /**
     * @brief Update encoder readings and calculate speeds
     */
    void update();
    
    /**
     * @brief Reset all encoder counts to zero
     */
    void reset();
    
    /**
     * @brief Get the current encoder count for a specific motor
     * @param motorIndex Motor index (0-3)
     * @return Current encoder count
     */
    long getCount(uint8_t motorIndex);
    
    /**
     * @brief Get the current speed for a specific motor
     * @param motorIndex Motor index (0-3)
     * @return Current speed in ticks per second
     */
    float getSpeed(uint8_t motorIndex);
    
    /**
     * @brief Get the ticks per revolution for a specific motor
     * @param motorIndex Motor index (0-3)
     * @return Ticks per revolution for the specified motor
     */
    uint16_t getTicksPerRev(uint8_t motorIndex);
    
    /**
     * @brief Get formatted encoder data for serial communication
     * @return String containing all encoder counts
     */
    String getFormattedCounts();
    
    /**
     * @brief Get formatted speed data for serial communication
     * @return String containing all encoder speeds
     */
    String getFormattedSpeeds();
};

#endif // ENCODER_H
