/**
 * @file imu.h
 * @brief IMU (MPU6050) interface for the 4WD Mobile Robot with ROS2 compatibility
 * @details Using ElectronicCats/mpu6050 library with DMP support
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" // Using ElectronicCats/mpu6050 library with DMP support
#include "config.h"

// Conversion constants
#define EARTH_GRAVITY_MS2 9.80665  // m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

class IMU {
private:
    MPU6050 mpu;
    
    // DMP status variables
    bool dmpReady;          // Set true if DMP init was successful
    uint8_t mpuIntStatus;   // Holds actual interrupt status byte from MPU
    uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    
    // Motion processing variables
    Quaternion q;           // [w, x, y, z]         Quaternion container
    VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
    VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
    VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
    VectorFloat gravity;    // [x, y, z]            Gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container
    
    // Processed data for external use
    float accelX, accelY, accelZ;  // Linear acceleration in m/s²
    float gyroX, gyroY, gyroZ;     // Angular velocity in rad/s
    float quatW, quatX, quatY, quatZ; // Quaternion orientation
    float roll, pitch, yaw;        // Euler angles in degrees
    float temperature;             // Temperature in °C
    
    unsigned long lastUpdateTime;   // Last update timestamp
    
public:
    /**
     * @brief Constructor for IMU class
     */
    IMU() : dmpReady(false) {}
    
    /**
     * @brief Initialize the IMU with DMP
     * @return true if initialization was successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Update IMU readings using DMP
     */
    void update();
    
    /**
     * @brief Get linear acceleration vector in world frame
     * @param ax Pointer to store X acceleration (m/s²)
     * @param ay Pointer to store Y acceleration (m/s²)
     * @param az Pointer to store Z acceleration (m/s²)
     */
    void getLinearAcceleration(float* ax, float* ay, float* az);
    
    /**
     * @brief Get angular velocity vector in world frame
     * @param gx Pointer to store X angular velocity (rad/s)
     * @param gy Pointer to store Y angular velocity (rad/s)
     * @param gz Pointer to store Z angular velocity (rad/s)
     */
    void getAngularVelocity(float* gx, float* gy, float* gz);
    
    /**
     * @brief Get quaternion orientation
     * @param qw Pointer to store W component
     * @param qx Pointer to store X component
     * @param qy Pointer to store Y component
     * @param qz Pointer to store Z component
     */
    void getQuaternion(float* qw, float* qx, float* qy, float* qz);
    
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
     * @brief Check if DMP is ready
     * @return true if DMP is ready, false otherwise
     */
    bool isDmpReady() { return dmpReady; }
    
    /**
     * @brief Format IMU data as a string for serial communication
     * @return String containing formatted IMU data for ROS2 compatibility
     */
    String getFormattedData();
};

#endif // IMU_H
