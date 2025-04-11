/**
 * @file communication.cpp
 * @brief Implementation of the Communication class
 */

#include "communication.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"

// External references to other modules (will be defined in main)
extern EncoderManager* encoderManager;
extern MotorController* motorController;
extern IMU* imu;

Communication::Communication() {
    bufferIndex = 0;
    commandReady = false;
    memset(buffer, 0, sizeof(buffer));
}

void Communication::init() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setTimeout(SERIAL_TIMEOUT);
}

bool Communication::update() {
    // Check for incoming data
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // Handle end of command (newline or carriage return)
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) { // Only process non-empty commands
                buffer[bufferIndex] = '\0'; // Null-terminate the string
                commandReady = true;
                break;
            }
        } else {
            // Add character to buffer if there's space
            if (bufferIndex < static_cast<int>(sizeof(buffer) - 1)) {
                buffer[bufferIndex++] = c;
            }
        }
    }
    
    // Process command if ready
    if (commandReady) {
        processCommand();
        // Reset buffer for next command
        bufferIndex = 0;
        commandReady = false;
        memset(buffer, 0, sizeof(buffer));
        return true;
    }
    
    return false;
}

void Communication::processCommand() {
    if (bufferIndex < 1) return; // Need at least a command character
    
    char command = buffer[0];
    String params = "";
    
    // Extract parameters if any (everything after the command character)
    if (bufferIndex > 1) {
        params = String(buffer + 1);
    }
    
    // Parse and execute the command
    parseCommand(command, params);
}

void Communication::parseCommand(char command, String params) {
    switch (command) {
        case COMMAND_READ_ENCODERS: // 'e' - Read encoder values
            if (encoderManager != nullptr) {
                sendResponse(command, encoderManager->getFormattedCounts());
            } else {
                sendError("Encoder manager not initialized");
            }
            break;
            
        case COMMAND_RESET_ENCODERS: // 'r' - Reset encoder values
            if (encoderManager != nullptr) {
                encoderManager->reset();
                sendResponse(command, "OK");
            } else {
                sendError("Encoder manager not initialized");
            }
            break;
            
        case COMMAND_SET_CLOSED_LOOP: // 'm' - Set closed-loop motor speeds
            if (motorController != nullptr && params.length() > 0) {
                // Parse speeds: m<speed1>:<speed2>:<speed3>:<speed4>
                double speeds[NUM_MOTORS] = {0.0};
                int index = 0;
                int lastIndex = 0;
                
                for (int i = 0; i < NUM_MOTORS; i++) {
                    index = params.indexOf(':', lastIndex);
                    if (index == -1 && i < NUM_MOTORS - 1) {
                        // Not enough parameters
                        sendError("Invalid number of parameters");
                        return;
                    }
                    
                    if (i == NUM_MOTORS - 1) {
                        // Last parameter
                        speeds[i] = params.substring(lastIndex).toInt();
                    } else {
                        speeds[i] = params.substring(lastIndex, index).toInt();
                        lastIndex = index + 1;
                    }
                }
                
                motorController->setAllClosedLoop(speeds);
                sendResponse(command, "OK");
            } else {
                sendError("Motor controller not initialized or invalid parameters");
            }
            break;
            
        case COMMAND_SET_OPEN_LOOP: // 'o' - Set open-loop motor speeds
            if (motorController != nullptr && params.length() > 0) {
                // Parse speeds: o<speed1>:<speed2>:<speed3>:<speed4>
                double speeds[NUM_MOTORS] = {0.0};
                int index = 0;
                int lastIndex = 0;
                
                for (int i = 0; i < NUM_MOTORS; i++) {
                    index = params.indexOf(':', lastIndex);
                    if (index == -1 && i < NUM_MOTORS - 1) {
                        // Not enough parameters
                        sendError("Invalid number of parameters");
                        return;
                    }
                    
                    if (i == NUM_MOTORS - 1) {
                        // Last parameter
                        speeds[i] = params.substring(lastIndex).toInt();
                    } else {
                        speeds[i] = params.substring(lastIndex, index).toInt();
                        lastIndex = index + 1;
                    }
                }
                
                motorController->setAllOpenLoop(speeds);
                sendResponse(command, "OK");
            } else {
                sendError("Motor controller not initialized or invalid parameters");
            }
            break;
            
        case COMMAND_SET_PID_GAINS: // 'u' - Set PID gains
            if (motorController != nullptr && params.length() > 0) {
                // Parse PID gains: u<kp>:<ki>:<kd>:<ko>
                double kp = 0.0, ki = 0.0, kd = 0.0, ko = 1.0;
                int index1 = params.indexOf(':');
                int index2 = params.indexOf(':', index1 + 1);
                int index3 = params.indexOf(':', index2 + 1);
                
                if (index1 != -1 && index2 != -1 && index3 != -1) {
                    kp = params.substring(0, index1).toFloat();
                    ki = params.substring(index1 + 1, index2).toFloat();
                    kd = params.substring(index2 + 1, index3).toFloat();
                    ko = params.substring(index3 + 1).toFloat();
                    
                    motorController->setPIDGains(kp, ki, kd, ko);
                    sendResponse(command, "OK");
                } else {
                    sendError("Invalid PID parameters format");
                }
            } else {
                sendError("Motor controller not initialized or invalid parameters");
            }
            break;
            
        case COMMAND_READ_IMU: // 'i' - Read IMU data
            if (imu != nullptr) {
                sendResponse(command, imu->getFormattedData());
            } else {
                sendError("IMU not initialized");
            }
            break;
            
        default:
            sendError("Unknown command");
            break;
    }
}

void Communication::sendEncoderData(const String& encoderData) {
    sendResponse(COMMAND_READ_ENCODERS, encoderData);
}

void Communication::sendIMUData(const String& imuData) {
    sendResponse(COMMAND_READ_IMU, imuData);
}

void Communication::sendResponse(char command, const String& data) {
    // Format: <command>:<data>\n
    Serial.print(command);
    Serial.print(':');
    Serial.println(data);
}

void Communication::sendError(const String& errorMsg) {
    // Format: ERROR:<message>\n
    Serial.print("ERROR:");
    Serial.println(errorMsg);
}
