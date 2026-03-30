#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

// #define DEBUG_SENSOR

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

extern SemaphoreHandle_t frontGroupMutex;
extern SemaphoreHandle_t backMutex;

#endif 