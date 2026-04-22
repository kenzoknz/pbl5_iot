#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

// #define DEBUG_SENSOR

#include "core/Config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <algorithm>

enum OperationMode {
    AUTONOMOUS,
    MANUAL
};

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

    // 2 Tasks: 1 cho 3 cảm biến trước (sequential), 1 cho sau
    static TaskHandle_t frontGroupTaskHandle;
    static TaskHandle_t backTaskHandle;
    static OperationMode currentMode;
    static volatile bool tasksEnabled;  // Cờ để enable/disable sensor reading an toàn

    // FreeRTOS Tasks — chạy trên Core 0
    static void frontGroupSensorTask(void *pvParameters);
    static void backSensorTask(void *pvParameters);

    // Buffer updates
    static void updateFrontBuffer();
    static void updateRightBuffer();
    static void updateLeftBuffer();
    static void updateBackBuffer();

    // Median filter
    static long getMedian(long* buffer);
    static long applyAdaptiveFilter(long rawDist, long *buffer, int &index, float &emaValue);
};

// ── [+] MỚI: Mutex khai báo extern — định nghĩa 1 lần trong .cpp ─────────────
//    V3 gốc khai báo global không có extern → multiple definition khi include
//    nhiều file. Sửa bằng cách khai báo extern ở .h, định nghĩa ở .cpp.
// 2 Mutex: front group + back (không cần 4)
extern SemaphoreHandle_t frontGroupMutex;
extern SemaphoreHandle_t backMutex;

#endif 
