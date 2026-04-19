#include "UltrasonicSensor.h"
#include "MotorController.h"
#include <Arduino.h>
#include <algorithm>

// ================== STATIC MEMBERS ==================
TaskHandle_t UltrasonicSensor::frontGroupTaskHandle = NULL;
TaskHandle_t UltrasonicSensor::backTaskHandle       = NULL;
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
    
    // ── 1 Task cho 3 cảm biến trước (Sequential Polling) ──
    // Stack: 3072 bytes đủ cho 3x pulseIn + median sort + EMA
    // (giảm từ 4096*3 = 12KB xuống 3072 = 3KB → tiết kiệm 9KB RAM)
    xTaskCreatePinnedToCore(
        frontGroupSensorTask, "FrontGroup",
        3072,                       // Đủ cho sequential 3 cảm biến
        NULL, PRIORITY_SENSORS,
        &frontGroupTaskHandle, 0    // Core 0
    );

    // ── 1 Task cho cảm biến sau ──
    // Stack: 2048 bytes cho 1 cảm biến
    xTaskCreatePinnedToCore(
        backSensorTask, "BackSensor",
        2048,
        NULL, PRIORITY_SENSORS,
        &backTaskHandle, 0          // Core 0
    );

    Serial.println("[SENSOR] 2 tasks for 4 sensors (Front Group + Back)");
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
    if (duration == 0) return 999;

    long dist = duration * 0.034 / 2;
    return (dist < 2 || dist > 400) ? 999 : dist;
}

// // ================== BUFFER UPDATES ==================
// void UltrasonicSensor::updateFrontBuffer() {
//     if (millis() - lastFrontTrigger < US_UPDATE_RATE_MS) return;
//     lastFrontTrigger = millis();

//     long dist = readDistanceRaw(TRIG_FRONT, ECHO_FRONT);

//     #ifdef DEBUG_SENSOR
//         Serial.printf("[US][FRONT] raw=%ld cm\n", dist);
//     #endif

//     xSemaphoreTake(sensorMutex, portMAX_DELAY);
//     frontBuffer[frontIndex] = dist;
//     frontIndex = (frontIndex + 1) % BUFFER_SIZE;
//     xSemaphoreGive(sensorMutex);
// }

// void UltrasonicSensor::updateRightBuffer() {
//     if (millis() - lastRightTrigger < US_UPDATE_RATE_MS) return;
//     lastRightTrigger = millis();

//     long dist = readDistanceRaw(TRIG_RIGHT, ECHO_RIGHT);

//     #ifdef DEBUG_SENSOR
//         Serial.printf("[US][RIGHT] raw=%ld cm\n", dist);
//     #endif

//     xSemaphoreTake(sensorMutex, portMAX_DELAY);
//     rightBuffer[rightIndex] = dist;
//     rightIndex = (rightIndex + 1) % BUFFER_SIZE;
//     xSemaphoreGive(sensorMutex);
// }

// void UltrasonicSensor::updateLeftBuffer() {
//     if (millis() - lastLeftTrigger < US_UPDATE_RATE_MS) return;
//     lastLeftTrigger = millis();

//     long dist = readDistanceRaw(TRIG_LEFT, ECHO_LEFT);

//     #ifdef DEBUG_SENSOR
//         Serial.printf("[US][LEFT] raw=%ld cm\n", dist);
//     #endif

//     xSemaphoreTake(sensorMutex, portMAX_DELAY);
//     leftBuffer[leftIndex] = dist;
//     leftIndex = (leftIndex + 1) % BUFFER_SIZE;
//     xSemaphoreGive(sensorMutex);
// }

// void UltrasonicSensor::updateBackBuffer() {
//     if (millis() - lastBackTrigger < US_UPDATE_RATE_MS) return;
//     lastBackTrigger = millis();

//     long dist = readDistanceRaw(TRIG_BACK, ECHO_BACK);

//     #ifdef DEBUG_SENSOR
//         Serial.printf("[US][BACK] raw=%ld cm\n", dist);
//     #endif

//     xSemaphoreTake(sensorMutex, portMAX_DELAY);
//     backBuffer[backIndex] = dist;
//     backIndex = (backIndex + 1) % BUFFER_SIZE;
//     xSemaphoreGive(sensorMutex);
// }

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

// ================== FRONT GROUP TASK ==================
// Sequential Polling: LEFT → FRONT → RIGHT
// Đảm bảo KHÔNG BAO GIỜ 2 cảm biến trước bắn sóng cùng lúc
//
// Timing analysis:
//   pulseIn worst case: ~15ms (MAX_DIST_TIMEOUT / 1000)
//   3 sensors × 15ms = 45ms
//   + 2 × 3ms gaps = 6ms
//   Total max: ~51ms < 60ms → vừa khít 1 chu kỳ
//   Best case (vật cản gần): 3 × 2ms + 6ms = 12ms
void UltrasonicSensor::frontGroupSensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(US_UPDATE_RATE_MS);

    for (;;) {
        // Kiểm tra cờ enable trước khi đọc sensors (tránh deadlock khi chuyển mode)
        if (!tasksEnabled) {
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            continue;
        }

        // ── 1. Quét LEFT ──
        long rawLeft = readDistanceRaw(TRIG_LEFT, ECHO_LEFT);

        // Chờ sóng âm tiêu tán (~3ms đủ, sóng âm 340m/s × 3ms = 1m)
        // Không dùng vTaskDelay(5) vì 5ms tick resolution có thể thành 10ms
        delayMicroseconds(3000);

        // ── 2. Quét FRONT (ưu tiên cao nhất — cảm biến quan trọng nhất) ──
        long rawFront = readDistanceRaw(TRIG_FRONT, ECHO_FRONT);
        delayMicroseconds(3000);

        // ── 3. Quét RIGHT ──
        long rawRight = readDistanceRaw(TRIG_RIGHT, ECHO_RIGHT);

        // ── 4. Ghi tất cả vào buffer + filter (1 lần lock duy nhất) ──
        xSemaphoreTake(frontGroupMutex, portMAX_DELAY);
        applyAdaptiveFilter(rawLeft,  leftBuffer,  leftIndex, emaLeft);
        applyAdaptiveFilter(rawFront, frontBuffer, frontIndex, emaFront);
        applyAdaptiveFilter(rawRight, rightBuffer, rightIndex, emaRight);
        xSemaphoreGive(frontGroupMutex);

        #ifdef DEBUG_SENSOR
        Serial.printf("[US] L:%ld F:%ld R:%ld | EMA L:%.0f F:%.0f R:%.0f\n",
                      rawLeft, rawFront, rawRight, emaLeft, emaFront, emaRight);
        #endif

        // Đợi chu kỳ tiếp theo (60ms tính từ đầu chu kỳ)
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ================== BACK SENSOR TASK ==================
void UltrasonicSensor::backSensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(US_UPDATE_RATE_MS);

    for (;;) {
        // Kiểm tra cờ enable trước khi đọc sensors
        if (!tasksEnabled) {
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
            continue;
        }

        long rawBack = readDistanceRaw(TRIG_BACK, ECHO_BACK);

        xSemaphoreTake(backMutex, portMAX_DELAY);
        applyAdaptiveFilter(rawBack, backBuffer, backIndex, emaBack);
        xSemaphoreGive(backMutex);

        #ifdef DEBUG_SENSOR
        Serial.printf("[US] B:%ld | EMA B:%.0f\n", rawBack, emaBack);
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