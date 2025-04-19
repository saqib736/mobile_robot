/**
 * @file imu.cpp
 * @brief Implementation of the IMU class for MPU6050 with ROS2 compatibility
 * @details Using ElectronicCats/mpu6050 library with DMP support
 */

#include "imu.h"

bool IMU::init() {
    // Initialize I2C communication
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setSCL(IMU_SCL);
        Wire.setSDA(IMU_SDA);
        Wire.setClock(400000); // 400kHz I2C clock
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // Initialize MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    // Verify connection
    Serial.println(F("Testing MPU6050 connection..."));
    if (!mpu.testConnection()) {
        Serial.println(F("MPU6050 connection failed"));
        return false;
    }
    Serial.println(F("MPU6050 connection successful"));
    
    // Initialize DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // Supply your gyro offsets here, scaled for min sensitivity
    // These can be calibrated later
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    
    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        Serial.println(F("Calibrating accelerometer and gyroscope..."));
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println(F("Calibration complete!"));
        mpu.PrintActiveOffsets();
        
        // Turn on the DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
        
        // Initialize data values
        accelX = accelY = accelZ = 0.0f;
        gyroX = gyroY = gyroZ = 0.0f;
        quatW = 1.0f;
        quatX = quatY = quatZ = 0.0f;
        roll = pitch = yaw = 0.0f;
        temperature = 0.0f;
        
        Serial.println(F("DMP ready!"));
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false;
    }
}

void IMU::update() {
    // If DMP initialization failed, don't try to do anything
    if (!dmpReady) return;
    
    // Read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // Get quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        quatW = q.w;
        quatX = q.x;
        quatY = q.y;
        quatZ = q.z;
        
        // Get gravity vector
        mpu.dmpGetGravity(&gravity, &q);
        
        // Get Euler angles (in radians)
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Convert to degrees and store
        yaw = ypr[0] * RAD_TO_DEG;
        pitch = ypr[1] * RAD_TO_DEG;
        roll = ypr[2] * RAD_TO_DEG;
        
        // Get accelerometer values (raw)
        mpu.dmpGetAccel(&aa, fifoBuffer);
        
        // Convert to world frame and apply gravity compensation
        mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
        
        // Convert to m/s² and store
        accelX = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
        accelY = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
        accelZ = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
        
        // Get gyroscope values (raw)
        mpu.dmpGetGyro(&gg, fifoBuffer);
        
        // Convert to world frame
        mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
        
        // Convert to rad/s and store
        gyroX = ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
        gyroY = ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
        gyroZ = ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD;
        
        // Update temperature (not part of DMP, read separately)
        temperature = mpu.getTemperature() / 340.0 + 36.53; // Formula from datasheet
        
        // Update timestamp
        lastUpdateTime = millis();
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

void IMU::getQuaternion(float* qw, float* qx, float* qy, float* qz) {
    *qw = quatW;
    *qx = quatX;
    *qy = quatY;
    *qz = quatZ;
}

void IMU::getEulerAngles(float* r, float* p, float* y) {
    *r = roll;   // Roll
    *p = pitch;  // Pitch
    *y = yaw;    // Yaw
}

float IMU::getTemperature() {
    return temperature;
}

String IMU::getFormattedData() {
    // Format for ROS2 compatibility:
    // qw:qx:qy:qz:gx:gy:gz:ax:ay:az
    // Quaternion, Angular velocity (rad/s), Linear acceleration (m/s²)
    String data = String(quatW) + ":" + String(quatX) + ":" + String(quatY) + ":" + String(quatZ) + ":" +
                 String(gyroX) + ":" + String(gyroY) + ":" + String(gyroZ) + ":" +
                 String(accelX) + ":" + String(accelY) + ":" + String(accelZ);
    return data;
}
