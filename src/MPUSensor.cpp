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
static uint8_t collisionHitCount = 0;
static uint32_t lastCollisionLogMs = 0;
static uint32_t lastTiltLogMs = 0;

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

        float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
        float accelChange = fabsf(accelMagnitude - lastAccelMagnitude);

        bool rawCollision = (accelChange > COLLISION_THRESHOLD);

        // Debounce: phải vượt ngưỡng ít nhất 3 mẫu liên tiếp
        if (rawCollision) {
            if (collisionHitCount < 3) collisionHitCount++;
        } else {
            collisionHitCount = 0;
        }

        bool collision = (collisionHitCount >= 3);

        float angleX = mpu.getAngleX();
        float angleY = mpu.getAngleY();

        bool tilt = (fabsf(angleX) > TILT_THRESHOLD || fabsf(angleY) > TILT_THRESHOLD);

        xSemaphoreTake(mpuMutex, portMAX_DELAY);

        currentAngleX = angleX;
        currentAngleY = angleY;
        currentAccelY = ay;
        lastAccelMagnitude = accelMagnitude;

        if (collision) collisionFlag = true;
        tiltFlag = tilt;

        xSemaphoreGive(mpuMutex);

        // Không Serial.print trong mutex
        uint32_t now = millis();

        if (collision && now - lastCollisionLogMs > 500) {
            lastCollisionLogMs = now;
            Serial.printf("[MPU] VA CHAM! delta=%.3f threshold=%.3f\n",
                        accelChange, (float)COLLISION_THRESHOLD);
        }

        if (tilt && now - lastTiltLogMs > 500) {
            lastTiltLogMs = now;
            Serial.printf("[MPU] NGHIENG NGUY HIEM: X=%.1f Y=%.1f\n", angleX, angleY);
        }
        vTaskDelay(sampleRate);
    }
}

bool MPUSensor::checkCollision() {
    bool flag;
    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    flag = collisionFlag;
    collisionFlag = false;  // Reset sau khi đọc
    xSemaphoreGive(mpuMutex);
    return flag;
}


bool MPUSensor::checkTilt() {
    bool flag;
    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    flag = tiltFlag;
    xSemaphoreGive(mpuMutex);
    return flag;
}

MpuSnapshot MPUSensor::getSnapshot() {
    MpuSnapshot s;

    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    s.angleX = currentAngleX;
    s.angleY = currentAngleY;
    s.accelY = currentAccelY;
    s.collision = collisionFlag;
    s.tilt = tiltFlag;
    xSemaphoreGive(mpuMutex);

    return s;
}

float MPUSensor::getCurrentAngleX() {
    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    float value = currentAngleX;
    xSemaphoreGive(mpuMutex);
    return value;
}

float MPUSensor::getCurrentAngleY() {
    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    float value = currentAngleY;
    xSemaphoreGive(mpuMutex);
    return value;
}

float MPUSensor::getCurrentAccelY() {
    xSemaphoreTake(mpuMutex, portMAX_DELAY);
    float value = currentAccelY;
    xSemaphoreGive(mpuMutex);
    return value;
}