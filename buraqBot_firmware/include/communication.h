/**
 * @file communication.h
 * @brief Serial communication interface for the 4WD Mobile Robot
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include "config.h"

class Communication {
private:
    char buffer[128]; // Buffer for incoming commands
    int bufferIndex;  // Current position in buffer
    bool commandReady; // Flag indicating a complete command is ready
    
    /**
     * @brief Parse a command from the buffer
     * @param command The command character
     * @param params String containing parameters
     */
    void parseCommand(char command, String params);
    
    /**
     * @brief Process a complete command
     */
    void processCommand();
    
public:
    /**
     * @brief Constructor
     */
    Communication();
    
    /**
     * @brief Initialize serial communication
     */
    void init();
    
    /**
     * @brief Update communication (check for incoming data)
     * @return true if a command was processed, false otherwise
     */
    bool update();
    
    /**
     * @brief Send encoder data to ROS2
     * @param encoderData Formatted encoder data string
     */
    void sendEncoderData(const String& encoderData);
    
    /**
     * @brief Send IMU data to ROS2
     * @param imuData Formatted IMU data string
     */
    void sendIMUData(const String& imuData);
    
    /**
     * @brief Send a response to ROS2
     * @param command The command character being responded to
     * @param data The response data
     */
    void sendResponse(char command, const String& data);
    
    /**
     * @brief Send an error message to ROS2
     * @param errorMsg Error message
     */
    void sendError(const String& errorMsg);
};

#endif // COMMUNICATION_H
