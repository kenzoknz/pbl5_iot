#include "MPUSensor.h"
#include <Arduino.h>

// Static member definitions
MPU6050 MPUSensor::mpu(Wire);
float MPUSensor::currentAngleX = 0;
float MPUSensor::currentAngleY = 0;
float MPUSensor::currentAccelY = 0;
float MPUSensor::lastAccelMagnitude = 0;

bool MPUSensor::begin() {
    // Khởi tạo I2C cho MPU6050
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Khởi tạo MPU6050
    byte status = mpu.begin();
    Serial.print("MPU6050 status: ");
    Serial.println(status);
    
    if (status != 0) {
        Serial.println("!!! LOI: Khong ket noi duoc MPU6050 !!!");
        Serial.println("Kiem tra day noi SDA/SCL va nguon cap 3.3V");
        return false;
    }
    
    Serial.println("Dang hieu chuan MPU6050... Giu xe yen!");
    delay(1000);
    mpu.calcOffsets();  // Hiệu chuẩn (xe phải đứng yên)
    Serial.println("Hieu chuan hoan tat!");
    
    return true;
}

void MPUSensor::update() {
    mpu.update();
    
    // Đọc gia tốc (đơn vị: g)
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    
    // Tính độ lớn gia tốc tổng
    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    
    // Cập nhật dữ liệu
    currentAngleX = mpu.getAngleX();  // Nghiêng trái/phải
    currentAngleY = mpu.getAngleY();  // Nghiêng trước/sau
    currentAccelY = ay;               // Gia tốc ngang (ly tâm)
    
    lastAccelMagnitude = accelMagnitude;
}

bool MPUSensor::checkCollision() {
    mpu.update();
    
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    
    float accelChange = abs(accelMagnitude - lastAccelMagnitude);
    if (accelChange > COLLISION_THRESHOLD) {
        Serial.println("!!! VA CHAM PHAT HIEN !!!");
        lastAccelMagnitude = accelMagnitude;
        return true;
    }
    
    lastAccelMagnitude = accelMagnitude;
    return false;
}

bool MPUSensor::checkTilt() {
    if (abs(currentAngleX) > TILT_THRESHOLD || abs(currentAngleY) > TILT_THRESHOLD) {
        Serial.print("!!! NGHIENG NGUY HIEM: X=");
        Serial.print(currentAngleX);
        Serial.print("° Y=");
        Serial.print(currentAngleY);
        Serial.println("° !!!");
        return true;
    }
    return false;
}