#ifndef ENCODER_SENSOR_H
#define ENCODER_SENSOR_H

#include "core/Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class EncoderSensor {
public:
    static bool begin();

    // ── API Thread-safe ──
    // Tốc độ hiện tại (đã qua lọc)
    static float getRPM();              // Vòng/phút
    static float getSpeedCmPerSec();    // cm/giây (cần cấu hình đường kính bánh xe)

    // Quãng đường
    static float getDistanceCm();       // Tổng quãng đường đã đi (cm)
    static void  resetDistance();       // Reset bộ đếm quãng đường

    // Chẩn đoán
    static bool  isStalled();           // Motor chạy nhưng bánh không quay
    static int   getDirection();        // 1 = tiến, -1 = lùi, 0 = dừng
    static long  getTotalPulses();      // Tổng số xung (debug)

private:
    static TaskHandle_t encoderTaskHandle;

    // Dữ liệu được bảo vệ bởi mutex
    static float currentRPM;
    static float filteredRPM;       // RPM sau EMA filter
    static float totalDistanceCm;
    static int   currentDirection;  // 1, -1, 0
    static long  totalPulses;       // Tổng tích lũy (có dấu)
    static bool  stallDetected;

    // Task chạy trên Core 1 (cùng Logic Task)
    static void encoderTask(void *pvParameters);

    // Đọc giá trị từ hardware PCNT
    static int16_t readAndResetPCNT();
};

extern SemaphoreHandle_t encoderMutex;

#endif
