#include "UltrasonicSensor.h"
#include "MotorController.h"
#include <Arduino.h>
#include <algorithm>

// ================== STATIC MEMBERS ==================
TaskHandle_t UltrasonicSensor::frontTaskHandle = NULL;
TaskHandle_t UltrasonicSensor::rightTaskHandle = NULL;
TaskHandle_t UltrasonicSensor::leftTaskHandle  = NULL;
TaskHandle_t UltrasonicSensor::backTaskHandle  = NULL;
OperationMode UltrasonicSensor::currentMode    = OperationMode::AUTONOMOUS;


// ================== BUFFERS (median filter) ==================
static long frontBuffer[BUFFER_SIZE];
static long rightBuffer[BUFFER_SIZE];
static long leftBuffer[BUFFER_SIZE];
static long backBuffer[BUFFER_SIZE];

static int frontIndex = 0;
static int rightIndex = 0;
static int leftIndex  = 0;
static int backIndex  = 0;

// Timestamp để đảm bảo >= 60ms giữa 2 lần trigger cùng cảm biến
static unsigned long lastFrontTrigger = 0;
static unsigned long lastRightTrigger = 0;
static unsigned long lastLeftTrigger  = 0;
static unsigned long lastBackTrigger  = 0;

SemaphoreHandle_t sensorMutex;

// ================== INIT ============================== 
void UltrasonicSensor::begin() {
    // Khởi tạo pins — 4 cảm biến
    pinMode(TRIG_FRONT, OUTPUT);  pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);  pinMode(ECHO_RIGHT, INPUT);
    pinMode(TRIG_LEFT,  OUTPUT);  pinMode(ECHO_LEFT,  INPUT);
    pinMode(TRIG_BACK,  OUTPUT);  pinMode(ECHO_BACK,  INPUT);

    // Khởi tạo buffer = 999 (chưa có dữ liệu)
    for (int i = 0; i < BUFFER_SIZE; i++) {
        frontBuffer[i] = 999;
        rightBuffer[i] = 999;
        leftBuffer[i]  = 999;
        backBuffer[i]  = 999;
    }

    sensorMutex = xSemaphoreCreateMutex();

    if (sensorMutex == NULL) {
        Serial.println("[SENSOR][ERROR] Khong the tao sensorMutex! Dung chuong trinh.");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Halt an toàn
    }
    // ── TẠO 4 TASKS, TẤT CẢ TRÊN CORE 0 ──
    // Mỗi cảm biến 1 task riêng, stagger thời gian để không trigger đồng thời
    // (tránh nhiễu xuyên âm giữa các HC-SR04)

    xTaskCreatePinnedToCore(
        frontSensorTask, "FrontSensor",
        STACK_SIZE_SENSORS, NULL, PRIORITY_SENSORS,
        &frontTaskHandle, 0
    );

    xTaskCreatePinnedToCore(
        rightSensorTask, "RightSensor",
        STACK_SIZE_SENSORS, NULL, PRIORITY_SENSORS,
        &rightTaskHandle, 0
    );

    xTaskCreatePinnedToCore(
        leftSensorTask, "LeftSensor",
        STACK_SIZE_SENSORS, NULL, PRIORITY_SENSORS,
        &leftTaskHandle, 0
    );

    xTaskCreatePinnedToCore(
        backSensorTask, "BackSensor",
        STACK_SIZE_SENSORS, NULL, PRIORITY_SENSORS,
        &backTaskHandle, 0
    );

    Serial.println("[SENSOR] 4 ultrasonic sensors initialized (Front/Right/Left/Back)");
}


// ================== MODE CONTROL ==================
void UltrasonicSensor::setMode(OperationMode mode) {
    if (mode == currentMode) return;
    currentMode = mode;

    TaskHandle_t handles[] = {frontTaskHandle, rightTaskHandle, leftTaskHandle, backTaskHandle};

    if (mode == OperationMode::MANUAL) {
        for (auto h : handles) { if (h) vTaskSuspend(h); }
        Serial.println("[SENSOR] Mode: MANUAL — all sensor tasks suspended.");
    } else {
        for (auto h : handles) { if (h) vTaskResume(h); }
        Serial.println("[SENSOR] Mode: AUTONOMOUS — all sensor tasks resumed.");
    }
}

OperationMode UltrasonicSensor::getMode() {
    return currentMode;
}

// ================== SENSOR TASKS ==================
// Stagger: mỗi task delay khởi đầu khác nhau để tránh trigger đồng thời
// Front: 0ms, Right: 15ms, Left: 30ms, Back: 45ms → tổng chu kỳ 60ms
void UltrasonicSensor::frontSensorTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
    vTaskDelay(pdMS_TO_TICKS(0)); // Stagger offset
    for (;;) {
        updateFrontBuffer();
        vTaskDelay(period);
    }
}

void UltrasonicSensor::rightSensorTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
    vTaskDelay(pdMS_TO_TICKS(15)); // Stagger 15ms
    for (;;) {
        updateRightBuffer();
        vTaskDelay(period);
    }
}

void UltrasonicSensor::leftSensorTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
    vTaskDelay(pdMS_TO_TICKS(30)); // Stagger 30ms
    for (;;) {
        updateLeftBuffer();
        vTaskDelay(period);
    }
}

void UltrasonicSensor::backSensorTask(void *pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(US_UPDATE_RATE_MS);
    vTaskDelay(pdMS_TO_TICKS(45)); // Stagger 45ms
    for (;;) {
        updateBackBuffer();
        vTaskDelay(period);
    }
}

// ================= RAW READ ===========================
long UltrasonicSensor::readDistanceRaw(int trig, int echo) {
    // Giới hạn 60ms giữa 2 lần trigger
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

// ================== BUFFER UPDATES ==================
void UltrasonicSensor::updateFrontBuffer() {
    if (millis() - lastFrontTrigger < US_UPDATE_RATE_MS) return;
    lastFrontTrigger = millis();

    long dist = readDistanceRaw(TRIG_FRONT, ECHO_FRONT);

    #ifdef DEBUG_SENSOR
        Serial.printf("[US][FRONT] raw=%ld cm\n", dist);
    #endif

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    frontBuffer[frontIndex] = dist;
    frontIndex = (frontIndex + 1) % BUFFER_SIZE;
    xSemaphoreGive(sensorMutex);
}

void UltrasonicSensor::updateRightBuffer() {
    if (millis() - lastRightTrigger < US_UPDATE_RATE_MS) return;
    lastRightTrigger = millis();

    long dist = readDistanceRaw(TRIG_RIGHT, ECHO_RIGHT);

    #ifdef DEBUG_SENSOR
        Serial.printf("[US][RIGHT] raw=%ld cm\n", dist);
    #endif

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    rightBuffer[rightIndex] = dist;
    rightIndex = (rightIndex + 1) % BUFFER_SIZE;
    xSemaphoreGive(sensorMutex);
}

void UltrasonicSensor::updateLeftBuffer() {
    if (millis() - lastLeftTrigger < US_UPDATE_RATE_MS) return;
    lastLeftTrigger = millis();

    long dist = readDistanceRaw(TRIG_LEFT, ECHO_LEFT);

    #ifdef DEBUG_SENSOR
        Serial.printf("[US][LEFT] raw=%ld cm\n", dist);
    #endif

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    leftBuffer[leftIndex] = dist;
    leftIndex = (leftIndex + 1) % BUFFER_SIZE;
    xSemaphoreGive(sensorMutex);
}

void UltrasonicSensor::updateBackBuffer() {
    if (millis() - lastBackTrigger < US_UPDATE_RATE_MS) return;
    lastBackTrigger = millis();

    long dist = readDistanceRaw(TRIG_BACK, ECHO_BACK);

    #ifdef DEBUG_SENSOR
        Serial.printf("[US][BACK] raw=%ld cm\n", dist);
    #endif

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    backBuffer[backIndex] = dist;
    backIndex = (backIndex + 1) % BUFFER_SIZE;
    xSemaphoreGive(sensorMutex);
}

// ================= MEDIAN FILTER ======================
long UltrasonicSensor::getMedian(long *buffer) {
    long temp[BUFFER_SIZE];

    memcpy(temp, buffer, sizeof(long) * BUFFER_SIZE);
    std::sort(temp, temp + BUFFER_SIZE);

    return temp[BUFFER_SIZE/2];
}

// ================== PUBLIC API ==================
long UltrasonicSensor::getFrontDistance() {
    long value;
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    value = getMedian(frontBuffer);
    xSemaphoreGive(sensorMutex);
    return value;
}

long UltrasonicSensor::getRightDistance() {
    long value;
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    value = getMedian(rightBuffer);
    xSemaphoreGive(sensorMutex);
    return value;
}

long UltrasonicSensor::getLeftDistance() {
    long value;
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    value = getMedian(leftBuffer);
    xSemaphoreGive(sensorMutex);
    return value;
}

long UltrasonicSensor::getBackDistance() {
    long value;
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    value = getMedian(backBuffer);
    xSemaphoreGive(sensorMutex);
    return value;
}