#include "UltrasonicSensor.h"
#include "MotorController.h"
#include <Arduino.h>
#include <algorithm>

// STATIC: 
TaskHandle_t UltrasonicSensor::frontTaskHandle = NULL;
TaskHandle_t UltrasonicSensor::backTaskHandle  = NULL;
OperationMode UltrasonicSensor::currentMode    = OperationMode::AUTONOMOUS;

// Khởi tạo bộ đệm để lọc nhiễu mà không gây block
// ================== BUFFER ==================
static long frontBuffer[5] = {999,999,999,999,999};
static long backBuffer[5]  = {999,999,999,999,999};

static int frontIndex = 0;
static int backIndex  = 0;

// Timestamp trigger gần nhất
static unsigned long lastFrontTrigger = 0;
static unsigned long lastBackTrigger  = 0;

// [+] Định nghĩa mutex (khai báo extern ở .h)
SemaphoreHandle_t sensorMutex;

// ================== INIT ============================== 
void UltrasonicSensor::begin() {

    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_BACK, OUTPUT);
    pinMode(ECHO_BACK, INPUT);

    sensorMutex = xSemaphoreCreateMutex();

    if (sensorMutex == NULL) {
        Serial.println("[SENSOR][ERROR] Khong the tao sensorMutex! Dung chuong trinh.");
        while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Halt an toàn
    }

    // ─────────────────────────────────────────────────────────────────────────
    // [3] TẠO TASK VÀ LƯU HANDLE
    //     Thay đổi: V3 gốc truyền NULL cho task handle → không thể suspend/resume
    //     Ý nghĩa: Lưu handle để setMode() có thể suspend khi chuyển MANUAL,
    //              resume khi quay lại AUTONOMOUS
    //
    //     Core 0 — ưu tiên 2 (sensor chạy ổn định, không tranh với quyết định)
    // ─────────────────────────────────────────────────────────────────────────
    xTaskCreatePinnedToCore(
        frontSensorTask,        // Hàm task
        "FrontSensor",          // Tên (debug)
        2048,                   // Stack (byte) — đủ cho pulseIn + sort
        NULL,                   // Tham số
        2,                      // Priority
        &frontTaskHandle,       // [+] Lưu handle
        0                       // Core 0
    );

    xTaskCreatePinnedToCore(
        backSensorTask,
        "BackSensor",
        2048,
        NULL,
        2,
        &backTaskHandle,        // [+] Lưu handle
        0
    );

    Serial.println("[SENSOR] UltrasonicSensor initialized.");
}


// ─────────────────────────────────────────────────────────────────────────────
// [4] setMode() / getMode() — HOÀN TOÀN MỚI
//     Ý nghĩa:
//       - AUTONOMOUS → resume cả 2 task sensor (chạy bình thường)
//       - MANUAL     → suspend cả 2 task sensor (nhường CPU/stack cho
//                      task xử lý lệnh Bluetooth/WiFi từ app người dùng)
//     Gọi từ: main loop hoặc task nhận lệnh chuyển chế độ
// ─────────────────────────────────────────────────────────────────────────────
void UltrasonicSensor::setMode(OperationMode mode) {
    if (mode == currentMode) return; // Không làm gì nếu không đổi
    currentMode = mode;

    if (mode == OperationMode::MANUAL) {
        // Suspend: task vẫn tồn tại trong bộ nhớ nhưng không được lên lịch
        if (frontTaskHandle) vTaskSuspend(frontTaskHandle);
        if (backTaskHandle)  vTaskSuspend(backTaskHandle);
        Serial.println("[SENSOR] Mode: MANUAL — sensor tasks suspended.");

    } else { // AUTONOMOUS
        if (frontTaskHandle) vTaskResume(frontTaskHandle);
        if (backTaskHandle)  vTaskResume(backTaskHandle);
        Serial.println("[SENSOR] Mode: AUTONOMOUS — sensor tasks resumed.");
    }
}

OperationMode UltrasonicSensor::getMode() {
    return currentMode;
}

/////////////////////////////////////////////////////////
/*
CORE 0 - TASK: FRONT SENSOR
Priority: 2 (Medium)
Chu kỳ: 60ms (HC-SR04 yêu cầu >= 60ms)
Chức năng:
- Đọc cảm biến trước
- Cập nhật buffer xoay vòng
- Không block toàn hệ thống
*/
void UltrasonicSensor::frontSensorTask(void *pvParameters) {
    const TickType_t delayTime = pdMS_TO_TICKS(60);

    while (true) {
        updateFrontBuffer();
        vTaskDelay(delayTime);
    }
}

/////////////////////////////////////////////////////////
/*
CORE 0 - TASK: BACK SENSOR
Priority: 2 (Medium)
Chu kỳ: 60ms
Chức năng:
- Đọc cảm biến sau
- Cập nhật buffer
*/
void UltrasonicSensor::backSensorTask(void *pvParameters) {
    const TickType_t delayTime = pdMS_TO_TICKS(60);

    while (true) {
        updateBackBuffer();
        vTaskDelay(delayTime);
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

    long duration = pulseIn(echo, HIGH, 15000); // timeout tối ưu

    if (duration == 0) return 999;

    long dist = duration * 0.034 / 2;
    return (dist < 2 || dist > 400) ? 999 : dist;
}

// ================= BUFFER UPDATE ======================
void UltrasonicSensor::updateFrontBuffer() {

    if (millis() - lastFrontTrigger < 60) return;
    lastFrontTrigger = millis();

    long dist = readDistanceRaw(TRIG_FRONT, ECHO_FRONT);

    //debug log
    #ifdef DEBUG_SENSOR
        Serial.printf("[SENSOR][FRONT] raw = %ld cm\n", dist);
    #endif

    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    frontBuffer[frontIndex] = dist;
    frontIndex = (frontIndex + 1) % BUFFER_SIZE;
    xSemaphoreGive(sensorMutex);
}

void UltrasonicSensor::updateBackBuffer() {

    if (millis() - lastBackTrigger < 60) return;
    lastBackTrigger = millis();

    long dist = readDistanceRaw(TRIG_BACK, ECHO_BACK);

    #ifdef DEBUG_SENSOR
        Serial.printf("[SENSOR][BACK] raw = %ld cm\n", dist);
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

// ================= PUBLIC API =========================
long UltrasonicSensor::getFrontDistance() {

    long value;
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    value = getMedian(frontBuffer);
    xSemaphoreGive(sensorMutex);

    #ifdef DEBUG_SENSOR
        Serial.printf("[SENSOR][FRONT] filtered = %ld cm\n", value);
    #endif

    return value;
}

long UltrasonicSensor::getBackDistance() {

    long value;
    xSemaphoreTake(sensorMutex, portMAX_DELAY);
    value = getMedian(backBuffer);
    xSemaphoreGive(sensorMutex);

    #ifdef DEBUG_SENSOR
        Serial.printf("[SENSOR][BACK] filtered = %ld cm\n", value);
    #endif

    return value;
}


// ================= SERVO SCAN =========================
/*
CORE 1 - Decision Task gọi khi cần rẽ
Priority: 3 (Cao hơn sensor)
Chức năng:
- Quay servo sang phải
- Đọc khoảng cách
- Quay servo sang trái
- Đọc khoảng cách
- Trả về giữa
*/
// void UltrasonicSensor::scanAllDirections(long &rightDist, long &leftDist) {

//     MotorController::setUSSensorServoAngle(US_SCAN_RIGHT);
//     vTaskDelay(pdMS_TO_TICKS(200));
//     rightDist = readDistanceRaw(TRIG_FRONT, ECHO_FRONT);

//     MotorController::setUSSensorServoAngle(US_SCAN_LEFT);
//     vTaskDelay(pdMS_TO_TICKS(200));
//     leftDist = readDistanceRaw(TRIG_FRONT, ECHO_FRONT);

//     MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
// }
// ─────────────────────────────────────────────────────────────────────────────
// [7] scanAllDirections() — CẢI TIẾN: đọc nhiều mẫu thay vì 1 lần
//     Thay đổi: V3 gốc đọc rawDistanceRaw() 1 lần duy nhất tại mỗi góc
//               → dễ bị nhiễu vì chỉ có 1 mẫu
//     Ý nghĩa: Đọc 3 lần lấy median tại chỗ, không cần background buffer
//              vì servo đang giữ cố định. Thêm log rõ ràng để debug khi cần.
//
//     Gọi từ: Decision Task (Core 1) khi phát hiện vật cản phía trước
// ─────────────────────────────────────────────────────────────────────────────
static long quickMedian3(int trig, int echo) {
    // Đọc 3 mẫu liên tiếp, lấy trung vị
    // Dùng vTaskDelay thay delay() để không block toàn core
    long samples[3];
    for (int i = 0; i < 3; i++) {
        samples[i] = UltrasonicSensor::readDistanceRaw(trig, echo);
        // Gọi hàm private qua friend không được → dùng lại logic raw ở đây
        // (hoặc tách thành helper public nếu cần)
        vTaskDelay(pdMS_TO_TICKS(60));
    }
    // Sort 3 phần tử đơn giản
    if (samples[0] > samples[1]) std::swap(samples[0], samples[1]);
    if (samples[1] > samples[2]) std::swap(samples[1], samples[2]);
    if (samples[0] > samples[1]) std::swap(samples[0], samples[1]);
    return samples[1]; // median
}

void UltrasonicSensor::scanAllDirections(long& rightDist, long& leftDist) {
    Serial.println("[SENSOR] >>> Bat dau quet servo...");

    // Quét phải
    MotorController::setUSSensorServoAngle(US_SCAN_RIGHT);
    vTaskDelay(pdMS_TO_TICKS(200)); // Chờ servo ổn định

    rightDist = quickMedian3(TRIG_FRONT, ECHO_FRONT);
    Serial.printf("[SENSOR]   Phai (%d°): %ld cm\n", US_SCAN_RIGHT, rightDist);

    // Quét trái
    MotorController::setUSSensorServoAngle(US_SCAN_LEFT);
    vTaskDelay(pdMS_TO_TICKS(200));

    leftDist = quickMedian3(TRIG_FRONT, ECHO_FRONT);
    Serial.printf("[SENSOR]   Trai (%d°): %ld cm\n", US_SCAN_LEFT, leftDist);

    // Trả về giữa
    MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
    Serial.println("[SENSOR]   Servo tra ve trung tam (90°)");
}


// ─────────────────────────────────────────────────────────────────────────────
// suspendFrontTask() / resumeFrontTask()
//
// Tại sao cần 2 hàm này thay vì chỉ dùng setMode(MANUAL)?
//
//   setMode(MANUAL) suspend CẢ frontTask VÀ backTask — dùng khi chuyển hẳn
//   sang chế độ người điều khiển, không cần theo dõi cảm biến nào cả.
//
//   suspendFrontTask() chỉ dừng frontTask — dùng trong lúc quét servo:
//   backTask vẫn chạy liên tục để phát hiện xe đang lùi vào vật cản.
//
// Race condition khi resume:
//   Sau resumeFrontTask(), frontBuffer còn chứa các mẫu cũ từ góc quét.
//   frontSensorTask sẽ ghi đè dần trong ~5 × 60ms = 300ms tiếp theo.
//   Trong thời gian đó getFrontDistance() vẫn trả về median — nếu buffer
//   có 3 mẫu cũ + 2 mẫu mới thì median vẫn hợp lý. Chấp nhận được.
//   Nếu cần chính xác tuyệt đối: flush buffer sau resume (xem TODO dưới).
// ─────────────────────────────────────────────────────────────────────────────
void UltrasonicSensor::suspendFrontTask() {
    if (frontTaskHandle == NULL) return;
    vTaskSuspend(frontTaskHandle);

    #ifdef DEBUG_SENSOR
        Serial.println("[SENSOR] frontSensorTask SUSPENDED (scan mode)");
    #endif
}

void UltrasonicSensor::resumeFrontTask() {
    if (frontTaskHandle == NULL) return;

    // TODO (nếu cần): Flush buffer về 999 để xóa dữ liệu cũ từ góc quét
    // xSemaphoreTake(sensorMutex, portMAX_DELAY);
    // for (int i = 0; i < BUFFER_SIZE; i++) frontBuffer[i] = 999;
    // frontIndex = 0;
    // xSemaphoreGive(sensorMutex);

    vTaskResume(frontTaskHandle);

    #ifdef DEBUG_SENSOR
        Serial.println("[SENSOR] frontSensorTask RESUMED");
    #endif
}