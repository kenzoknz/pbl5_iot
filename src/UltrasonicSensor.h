#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H
#define DEBUG_SENSOR

#include "Config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <algorithm>

enum OperationMode {
    AUTONOMOUS,
    MANUAL
};

// // ── Buffer & timing ──────────────────────────────────────────────────────────
// #define BUFFER_SIZE     3        // Cũ 5 (phản ứng hơi chậm, test 3) Số mẫu trong circular buffer
// #define SENSOR_PERIOD   60      // ms — HC-SR04 yêu cầu >= 60ms giữa 2 trigger

class UltrasonicSensor {
public:
    static void begin();

    // Chuyển chế độ (suspend/resume tất cả sensor tasks)
    static void setMode(OperationMode mode);
    static OperationMode getMode();

    // Public API — Thread-safe, trả về median từ buffer
    static long getFrontDistance();
    static long getRightDistance();
    static long getLeftDistance();
    static long getBackDistance();

   // Raw read (public để có thể test)
    static long readDistanceRaw(int trig, int echo);

private:

    // Task handles
    static TaskHandle_t frontTaskHandle;
    static TaskHandle_t rightTaskHandle;
    static TaskHandle_t leftTaskHandle;
    static TaskHandle_t backTaskHandle;

    static OperationMode currentMode;

    // FreeRTOS Tasks — chạy trên Core 0
    static void frontSensorTask(void *pvParameters);
    static void rightSensorTask(void *pvParameters);
    static void leftSensorTask(void *pvParameters);
    static void backSensorTask(void *pvParameters);

    // Buffer updates
    static void updateFrontBuffer();
    static void updateRightBuffer();
    static void updateLeftBuffer();
    static void updateBackBuffer();

    // Median filter
    static long getMedian(long* buffer);

};

// ── [+] MỚI: Mutex khai báo extern — định nghĩa 1 lần trong .cpp ─────────────
//    V3 gốc khai báo global không có extern → multiple definition khi include
//    nhiều file. Sửa bằng cách khai báo extern ở .h, định nghĩa ở .cpp.
extern SemaphoreHandle_t sensorMutex;

#endif 