#include "MPUSensor.h"
#include <Arduino.h>

// Static member definitions
MPU6050 MPUSensor::mpu(Wire);
float MPUSensor::currentAngleX = 0;
float MPUSensor::currentAngleY = 0;
float MPUSensor::currentAccelY = 0;
float MPUSensor::lastAccelMagnitude = 0;
bool  MPUSensor::collisionFlag = false;
bool  MPUSensor::tiltFlag = false;
TaskHandle_t MPUSensor::mpuTaskHandle = NULL;

SemaphoreHandle_t mpuMutex;


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
    mpuMutex = xSemaphoreCreateMutex();
    if (mpuMutex == NULL) {
        Serial.println("[MPU][ERROR] Khong tao duoc mpuMutex!");
        return false;   // Báo lỗi lên caller, không để chạy tiếp
    }
    
    Serial.println("Dang hieu chuan MPU6050... Giu xe yen!");
    vTaskDelay(pdMS_TO_TICKS(1000));   // Chờ cảm biến ổn định
    mpu.calcOffsets();
    Serial.println("Hieu chuan hoan tat!");

    xTaskCreatePinnedToCore(
        mpuTask, "MPUTask", 4096, NULL,
        PRIORITY_SENSORS+1,                  // Priority cao hơn sensor siêu âm (3 > 2)
        &mpuTaskHandle,     // ← Lưu handle
        1                   // Core 1 — cùng core với Decision Task
    );
        
    return true;
}

void MPUSensor::mpuTask(void *pvParameters) {
    const TickType_t sampleRate = pdMS_TO_TICKS(10); // 100Hz

    while (true) {

        mpu.update();

        float ax = mpu.getAccX();
        float ay = mpu.getAccY();
        float az = mpu.getAccZ();

        float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
        float accelChange = abs(accelMagnitude - lastAccelMagnitude);

        bool collision = (accelChange > COLLISION_THRESHOLD);
        bool tilt = (abs(mpu.getAngleX()) > TILT_THRESHOLD ||
                     abs(mpu.getAngleY()) > TILT_THRESHOLD);

        xSemaphoreTake(mpuMutex, portMAX_DELAY);

        currentAngleX = mpu.getAngleX();
        currentAngleY = mpu.getAngleY();
        currentAccelY = ay;
        lastAccelMagnitude = accelMagnitude;

        collisionFlag = collision;
        tiltFlag = tilt;
        if (collision) {
            Serial.printf("[MPU] VA CHAM! delta=%.3f (nguong=%.3f)\n",
                        accelChange, (float)COLLISION_THRESHOLD);
        }
        if (tilt) {
            Serial.printf("[MPU] NGHIENG NGUY HIEM: X=%.1f° Y=%.1f°\n",
                        mpu.getAngleX(), mpu.getAngleY());
        }

        xSemaphoreGive(mpuMutex);

        vTaskDelay(sampleRate);
    }
}

// DEPRECATED: Không dùng khi mpuTask đang chạy.
    // Giữ lại chỉ để tương thích ngược nếu cần test không có FreeRTOS.
// void MPUSensor::update() {
//     mpu.update();
    
//     // Đọc gia tốc (đơn vị: g)
//     float ax = mpu.getAccX();
//     float ay = mpu.getAccY();
//     float az = mpu.getAccZ();
    
//     // Tính độ lớn gia tốc tổng
//     float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    
//     // Cập nhật dữ liệu
//     currentAngleX = mpu.getAngleX();  // Nghiêng trái/phải
//     currentAngleY = mpu.getAngleY();  // Nghiêng trước/sau
//     currentAccelY = ay;               // Gia tốc ngang (ly tâm)
    
//     lastAccelMagnitude = accelMagnitude;
// }

bool MPUSensor::checkCollision() {
    bool flag;
    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    flag = collisionFlag;
    collisionFlag = false;  // Reset sau khi đọc
    xSemaphoreGive(mpuMutex);
    return flag;
}

// bool MPUSensor::checkCollision() {
//     mpu.update();
    
//     float ax = mpu.getAccX();
//     float ay = mpu.getAccY();
//     float az = mpu.getAccZ();
//     float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
    
//     float accelChange = abs(accelMagnitude - lastAccelMagnitude);
//     if (accelChange > COLLISION_THRESHOLD) {
//         Serial.println("!!! VA CHAM PHAT HIEN !!!");
//         lastAccelMagnitude = accelMagnitude;
//         return true;
//     }
    
//     lastAccelMagnitude = accelMagnitude;
//     return false;
// }

bool MPUSensor::checkTilt() {
    bool flag;
    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    flag = tiltFlag;
    xSemaphoreGive(mpuMutex);
    return flag;
}

// bool MPUSensor::checkTilt() {
//     if (abs(currentAngleX) > TILT_THRESHOLD || abs(currentAngleY) > TILT_THRESHOLD) {
//         Serial.print("!!! NGHIENG NGUY HIEM: X=");
//         Serial.print(currentAngleX);
//         Serial.print("° Y=");
//         Serial.print(currentAngleY);
//         Serial.println("° !!!");
//         return true;
//     }
//     return false;
// }