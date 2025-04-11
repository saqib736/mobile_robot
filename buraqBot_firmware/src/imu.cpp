/**
 * @file imu.cpp
 * @brief Implementation of the IMU class for MPU6050 with ROS2 compatibility
 */

#include "imu.h"
#include <math.h>

bool IMU::init() {
    Wire.begin();
    Wire.setSCL(IMU_SCL);
    Wire.setSDA(IMU_SDA);
    
    mpu.begin();
    mpu.calcGyroOffsets(true); // Calibrate gyroscope with output enabled
    
    // Initialize quaternion to identity [w=1, x=0, y=0, z=0]
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
    
    lastUpdateTime = millis();
    return true;
}

void IMU::update() {
    mpu.update();
    
    // Get acceleration values (in m/s²)
    accelX = mpu.getAccX() * 9.81; // Convert g to m/s²
    accelY = mpu.getAccY() * 9.81;
    accelZ = mpu.getAccZ() * 9.81;
    
    // Get gyroscope values (in degrees/s) and convert to rad/s
    gyroX = mpu.getGyroX() * DEG_TO_RAD;
    gyroY = mpu.getGyroY() * DEG_TO_RAD;
    gyroZ = mpu.getGyroZ() * DEG_TO_RAD;
    
    // Get calculated angles in degrees
    angleX = mpu.getAngleX();
    angleY = mpu.getAngleY();
    angleZ = mpu.getAngleZ();
    
    // Convert Euler angles to quaternion
    // Convert degrees to radians for quaternion calculation
    float roll = angleX * DEG_TO_RAD;
    float pitch = angleY * DEG_TO_RAD;
    float yaw = angleZ * DEG_TO_RAD;
    eulerToQuaternion(roll, pitch, yaw);
    
    lastUpdateTime = millis();
}

void IMU::eulerToQuaternion(float roll, float pitch, float yaw) {
    // Calculate quaternion from Euler angles using the ZYX convention
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    // Quaternion: q = [w, x, y, z]
    quat[0] = cr * cp * cy + sr * sp * sy; // w
    quat[1] = sr * cp * cy - cr * sp * sy; // x
    quat[2] = cr * sp * cy + sr * cp * sy; // y
    quat[3] = cr * cp * sy - sr * sp * cy; // z
    
    // Normalize quaternion
    float norm = sqrt(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
    if (norm > 0.0f) {
        quat[0] /= norm;
        quat[1] /= norm;
        quat[2] /= norm;
        quat[3] /= norm;
    }
}

void IMU::getLinearAcceleration(float* ax, float* ay, float* az) {
    *ax = accelX;
    *ay = accelY;
    *az = accelZ;
}

void IMU::getAngularVelocity(float* gx, float* gy, float* gz) {
    *gx = gyroX;
    *gy = gyroY;
    *gz = gyroZ;
}

void IMU::getQuaternion(float q[4]) {
    q[0] = quat[0]; // w
    q[1] = quat[1]; // x
    q[2] = quat[2]; // y
    q[3] = quat[3]; // z
}

void IMU::getEulerAngles(float* roll, float* pitch, float* yaw) {
    *roll = angleX;   // Roll (X)
    *pitch = angleY;  // Pitch (Y)
    *yaw = angleZ;    // Yaw (Z)
}

String IMU::getFormattedData() {
    // Format for ROS2 compatibility:
    // qw:qx:qy:qz:gx:gy:gz:ax:ay:az
    // Quaternion, Angular velocity (rad/s), Linear acceleration (m/s²)
    String data = String(quat[0]) + ":" + String(quat[1]) + ":" + String(quat[2]) + ":" + String(quat[3]) + ":" +
                 String(gyroX) + ":" + String(gyroY) + ":" + String(gyroZ) + ":" +
                 String(accelX) + ":" + String(accelY) + ":" + String(accelZ);
    return data;
}
