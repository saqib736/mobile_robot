/**
 * @file imu.h
 * @brief IMU (MPU6050) interface for the 4WD Mobile Robot with ROS2 compatibility
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h> // Using MPU6050_tockn library for simplicity
#include "config.h"

class IMU {
private:
    MPU6050 mpu;
    
    // Linear acceleration in m/s²
    float accelX, accelY, accelZ;
    
    // Angular velocity in rad/s (converted from degrees/s)
    float gyroX, gyroY, gyroZ;
    
    // Quaternion orientation
    float quat[4]; // [w, x, y, z]
    
    // Euler angles in degrees (for internal calculations)
    float angleX, angleY, angleZ;
    
    float temperature; // Temperature in °C
    unsigned long lastUpdateTime; // Last update timestamp
    
    /**
     * @brief Convert Euler angles to quaternion
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     */
    void eulerToQuaternion(float roll, float pitch, float yaw);
    
public:
    /**
     * @brief Constructor for IMU class
     */
    IMU() : mpu(Wire) {}
    
    /**
     * @brief Initialize the IMU
     * @return true if initialization was successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Update IMU readings
     */
    void update();
    
    /**
     * @brief Get linear acceleration vector
     * @param ax Pointer to store X acceleration (m/s²)
     * @param ay Pointer to store Y acceleration (m/s²)
     * @param az Pointer to store Z acceleration (m/s²)
     */
    void getLinearAcceleration(float* ax, float* ay, float* az);
    
    /**
     * @brief Get angular velocity vector
     * @param gx Pointer to store X angular velocity (rad/s)
     * @param gy Pointer to store Y angular velocity (rad/s)
     * @param gz Pointer to store Z angular velocity (rad/s)
     */
    void getAngularVelocity(float* gx, float* gy, float* gz);
    
    /**
     * @brief Get quaternion orientation
     * @param q Array to store quaternion [w, x, y, z]
     */
    void getQuaternion(float q[4]);
    
    /**
     * @brief Get Euler angles (roll, pitch, yaw)
     * @param roll Pointer to store roll angle (degrees)
     * @param pitch Pointer to store pitch angle (degrees)
     * @param yaw Pointer to store yaw angle (degrees)
     */
    void getEulerAngles(float* roll, float* pitch, float* yaw);
    
    /**
     * @brief Get IMU temperature
     * @return Temperature in degrees Celsius
     */
    float getTemperature();
    
    /**
     * @brief Format IMU data as a string for serial communication
     * @return String containing formatted IMU data for ROS2 compatibility
     */
    String getFormattedData();
};

#endif // IMU_H
