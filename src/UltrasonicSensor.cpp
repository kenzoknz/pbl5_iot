#include "UltrasonicSensor.h"
#include "MotorController.h"
#include <Arduino.h>
#include <algorithm>

// ================== STATIC MEMBERS ==================
TaskHandle_t UltrasonicSensor::sensorTaskHandle = NULL;
OperationMode UltrasonicSensor::currentMode = OperationMode::AUTONOMOUS;
volatile bool UltrasonicSensor::tasksEnabled = true;

// ================== BUFFERS ==================
static long frontBuffer[BUFFER_SIZE] = {999, 999, 999};
static long rightBuffer[BUFFER_SIZE] = {999, 999, 999};
static long leftBuffer[BUFFER_SIZE]  = {999, 999, 999};
static long backBuffer[BUFFER_SIZE]  = {999, 999, 999};

static int frontIndex = 0;
static int rightIndex = 0;
static int leftIndex  = 0;
static int backIndex  = 0;

// // Timestamp để đảm bảo >= 60ms giữa 2 lần trigger cùng cảm biến
// static unsigned long lastFrontTrigger = 0;
// static unsigned long lastRightTrigger = 0;
// static unsigned long lastLeftTrigger  = 0;
// static unsigned long lastBackTrigger  = 0;

// SemaphoreHandle_t sensorMutex;

// ================== ADAPTIVE EMA ==================
// Giá trị EMA hiện tại cho mỗi cảm biến
static float emaFront = 999.0f;
static float emaRight = 999.0f;
static float emaLeft  = 999.0f;
static float emaBack  = 999.0f;

// ── Adaptive Alpha ──
// Thay vì alpha cố định (0.6) gây chậm phản ứng:
//   - Khi thay đổi lớn (>15cm): alpha = 0.85 → phản ứng gần như tức thì
//   - Khi thay đổi nhỏ (<5cm):  alpha = 0.3  → rất mượt
//   - Khoảng giữa:              alpha nội suy tuyến tính
static const float EMA_ALPHA_FAST = 0.85f;  // Phản ứng nhanh (vật cản bất ngờ)
static const float EMA_ALPHA_SLOW = 0.3f;   // Mượt mà (ổn định)
static const float CHANGE_THRESHOLD_BIG   = 15.0f; // cm - thay đổi lớn
static const float CHANGE_THRESHOLD_SMALL = 5.0f;  // cm - thay đổi nhỏ

// 2 Mutex
SemaphoreHandle_t frontGroupMutex;
SemaphoreHandle_t backMutex;

// ================== INIT ============================== 
void UltrasonicSensor::begin() {
    // Khởi tạo pins — 4 cảm biến
    pinMode(TRIG_FRONT, OUTPUT);  pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);  pinMode(ECHO_RIGHT, INPUT);
    pinMode(TRIG_LEFT,  OUTPUT);  pinMode(ECHO_LEFT,  INPUT);
    pinMode(TRIG_BACK,  OUTPUT);  pinMode(ECHO_BACK,  INPUT);

    frontGroupMutex = xSemaphoreCreateMutex();
    backMutex       = xSemaphoreCreateMutex();

    if (frontGroupMutex == NULL || backMutex == NULL) {
        Serial.println("[SENSOR][FATAL] Mutex creation failed!");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    xTaskCreatePinnedToCore(
        sensorTask,
        "UltrasonicTask",
        4096,
        NULL,
        PRIORITY_SENSORS,
        &sensorTaskHandle,
        0
    );

    Serial.println("[SENSOR] 1 task for 4 sensors (Front Group + Back)");
}


// ================== MODE CONTROL ==================
void UltrasonicSensor::setMode(OperationMode mode) {
    if (mode == currentMode) return;
    currentMode = mode;

    if (mode == OperationMode::MANUAL) {
        tasksEnabled = false;  // Tasks sẽ tự skip sensor reading
        Serial.println("[SENSOR] MANUAL — sensor reading disabled");
    } else {
        tasksEnabled = true;   // Tasks tiếp tục đọc sensors
        Serial.println("[SENSOR] AUTONOMOUS — sensor reading enabled");
    }
}

OperationMode UltrasonicSensor::getMode() {
    return currentMode;
}

// // ================== SENSOR TASKS ==================
// // Stagger: mỗi task delay khởi đầu khác nhau để tránh trigger đồng thời
// // Front: 0ms, Right: 15ms, Left: 30ms, Back: 45ms → tổng chu kỳ 60ms
// void UltrasonicSensor::frontSensorTask(void *pvParameters) {
//     const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
//     vTaskDelay(pdMS_TO_TICKS(0)); // Stagger offset
//     for (;;) {
//         updateFrontBuffer();
//         vTaskDelay(period);
//     }
// }

// void UltrasonicSensor::rightSensorTask(void *pvParameters) {
//     const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
//     vTaskDelay(pdMS_TO_TICKS(15)); // Stagger 15ms
//     for (;;) {
//         updateRightBuffer();
//         vTaskDelay(period);
//     }
// }

// void UltrasonicSensor::leftSensorTask(void *pvParameters) {
//     const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
//     vTaskDelay(pdMS_TO_TICKS(30)); // Stagger 30ms
//     for (;;) {
//         updateLeftBuffer();
//         vTaskDelay(period);
//     }
// }

// void UltrasonicSensor::backSensorTask(void *pvParameters) {
//     const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
//     vTaskDelay(pdMS_TO_TICKS(45)); // Stagger 45ms
//     for (;;) {
//         updateBackBuffer();
//         vTaskDelay(period);
//     }
// }

// ================= RAW READ ===========================
long UltrasonicSensor::readDistanceRaw(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, MAX_DIST_TIMEOUT);

    // Không có echo: giữ 999 nhưng phải hiểu là UNKNOWN.
    // State machine nên thận trọng với cảm biến trước nếu liên tục 999.
    if (duration == 0) return 999;

    long dist = duration * 0.034f / 2.0f;

    if (dist < 2 || dist > 300) {
        return 999;
    }

    return dist;
}

// ================= MEDIAN FILTER ======================
long UltrasonicSensor::getMedian(long *buffer) {
    long temp[BUFFER_SIZE];

    memcpy(temp, buffer, sizeof(long) * BUFFER_SIZE);
    std::sort(temp, temp + BUFFER_SIZE);

    return temp[BUFFER_SIZE/2];
}

// ================== ADAPTIVE FILTER ==================
// Median loại outlier → Adaptive EMA làm mượt nhưng phản ứng nhanh khi cần
long UltrasonicSensor::applyAdaptiveFilter(long rawDist, long *buffer, int &index, float &emaValue) {
    // 1. Ghi vào circular buffer
    buffer[index] = rawDist;
    index = (index + 1) % BUFFER_SIZE;

    // 2. Median filter — loại bỏ giá trị nhiễu đột biến
    long medianVal = getMedian(buffer);

    // 3. Khởi tạo EMA lần đầu
    if (emaValue >= 998.0f) {
        emaValue = (float)medianVal;
        return medianVal;
    }

    // 4. Tính alpha dựa trên mức độ thay đổi
    float change = fabsf((float)medianVal - emaValue);
    float alpha;

    if (change >= CHANGE_THRESHOLD_BIG) {
        // Thay đổi lớn (vật cản bất ngờ) → phản ứng gần tức thì
        alpha = EMA_ALPHA_FAST;
    } else if (change <= CHANGE_THRESHOLD_SMALL) {
        // Thay đổi nhỏ (dao động nhẹ) → làm mượt mạnh
        alpha = EMA_ALPHA_SLOW;
    } else {
        // Nội suy tuyến tính giữa SLOW và FAST
        float ratio = (change - CHANGE_THRESHOLD_SMALL)
                    / (CHANGE_THRESHOLD_BIG - CHANGE_THRESHOLD_SMALL);
        alpha = EMA_ALPHA_SLOW + ratio * (EMA_ALPHA_FAST - EMA_ALPHA_SLOW);
    }

    // 5. Áp dụng EMA
    emaValue = alpha * (float)medianVal + (1.0f - alpha) * emaValue;
    return (long)(emaValue + 0.5f); // Làm tròn
}

void UltrasonicSensor::sensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // 4 cảm biến, mỗi cảm biến cách nhau 25ms.
    // Một vòng ~100ms => mỗi cảm biến ~10Hz, đủ cho robot tự hành.
    const TickType_t xPeriod = pdMS_TO_TICKS(100);

    for (;;) {
        if (!tasksEnabled) {
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            continue;
        }

        long rawFront = readDistanceRaw(TRIG_FRONT, ECHO_FRONT);
        vTaskDelay(pdMS_TO_TICKS(25));

        long rawLeft = readDistanceRaw(TRIG_LEFT, ECHO_LEFT);
        vTaskDelay(pdMS_TO_TICKS(25));

        long rawRight = readDistanceRaw(TRIG_RIGHT, ECHO_RIGHT);
        vTaskDelay(pdMS_TO_TICKS(25));

        long rawBack = readDistanceRaw(TRIG_BACK, ECHO_BACK);

        xSemaphoreTake(frontGroupMutex, portMAX_DELAY);
        applyAdaptiveFilter(rawFront, frontBuffer, frontIndex, emaFront);
        applyAdaptiveFilter(rawLeft,  leftBuffer,  leftIndex,  emaLeft);
        applyAdaptiveFilter(rawRight, rightBuffer, rightIndex, emaRight);
        xSemaphoreGive(frontGroupMutex);

        xSemaphoreTake(backMutex, portMAX_DELAY);
        applyAdaptiveFilter(rawBack, backBuffer, backIndex, emaBack);
        xSemaphoreGive(backMutex);

        #ifdef DEBUG_SENSOR
        Serial.printf("[US] F:%ld L:%ld R:%ld B:%ld | EMA F:%.0f L:%.0f R:%.0f B:%.0f\n",
                      rawFront, rawLeft, rawRight, rawBack,
                      emaFront, emaLeft, emaRight, emaBack);
        #endif

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}


// ================== PUBLIC API ==================
long UltrasonicSensor::getFrontDistance() {
    long val;
    xSemaphoreTake(frontGroupMutex, portMAX_DELAY);
    val = (long)(emaFront + 0.5f);
    xSemaphoreGive(frontGroupMutex);
    return val;
}

long UltrasonicSensor::getRightDistance() {
    long val;
    xSemaphoreTake(frontGroupMutex, portMAX_DELAY);
    val = (long)(emaRight + 0.5f);
    xSemaphoreGive(frontGroupMutex);
    return val;
}

long UltrasonicSensor::getLeftDistance() {
    long val;
    xSemaphoreTake(frontGroupMutex, portMAX_DELAY);
    val = (long)(emaLeft + 0.5f);
    xSemaphoreGive(frontGroupMutex);
    return val;
}

long UltrasonicSensor::getBackDistance() {
    long val;
    xSemaphoreTake(backMutex, portMAX_DELAY);
    val = (long)(emaBack + 0.5f);
    xSemaphoreGive(backMutex);
    return val;
}