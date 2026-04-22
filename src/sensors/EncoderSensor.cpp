#include "sensors/EncoderSensor.h"
#include "control/MotorController.h"
#include <Arduino.h>
#include <driver/pcnt.h>  // ESP32 Hardware Pulse Counter

// ================== STATIC MEMBERS ==================
TaskHandle_t EncoderSensor::encoderTaskHandle = NULL;
float EncoderSensor::currentRPM       = 0;
float EncoderSensor::filteredRPM      = 0;
float EncoderSensor::totalDistanceCm  = 0;
int   EncoderSensor::currentDirection = 0;
long  EncoderSensor::totalPulses      = 0;
bool  EncoderSensor::stallDetected    = false;

SemaphoreHandle_t encoderMutex;

// ================== CẤU HÌNH PCNT ==================
// ESP32 PCNT Unit 0 — đếm xung phần cứng, KHÔNG tốn CPU
// Kênh A = GPIO 34 (pulse input)
// Kênh B = GPIO 35 (control/direction input)
//
// Chế độ Quadrature x4:
//   - Đếm cả cạnh lên + cạnh xuống của cả 2 kênh
//   - 600 PPR × 4 = 2400 counts/vòng
//   - Giá trị dương = tiến, âm = lùi
#define PCNT_UNIT       PCNT_UNIT_0
#define PCNT_CHANNEL_A  PCNT_CHANNEL_0
#define PCNT_CHANNEL_B  PCNT_CHANNEL_1

// ================== INIT ==================
bool EncoderSensor::begin() {
    encoderMutex = xSemaphoreCreateMutex();
    if (encoderMutex == NULL) {
        Serial.println("[ENCODER][FATAL] Mutex creation failed!");
        return false;
    }

    // ── Cấu hình PCNT Channel 0 (Kênh A = pulse, Kênh B = control) ──
    pcnt_config_t pcnt_config_a = {};
    pcnt_config_a.pulse_gpio_num = ENCODER_PIN_A;   // GPIO 34
    pcnt_config_a.ctrl_gpio_num  = ENCODER_PIN_B;   // GPIO 35
    pcnt_config_a.channel        = PCNT_CHANNEL_A;
    pcnt_config_a.unit           = PCNT_UNIT;
    // Khi control (B) = HIGH: cạnh lên A → đếm tăng
    pcnt_config_a.pos_mode       = PCNT_COUNT_INC;
    pcnt_config_a.neg_mode       = PCNT_COUNT_DEC;
    // Khi control (B) = LOW: cạnh lên A → đếm giảm (quadrature)
    pcnt_config_a.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_config_a.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config_a.counter_h_lim  = 30000;  // Giới hạn tránh overflow
    pcnt_config_a.counter_l_lim  = -30000;

    esp_err_t err = pcnt_unit_config(&pcnt_config_a);
    if (err != ESP_OK) {
        Serial.printf("[ENCODER][ERROR] PCNT Channel A config failed: %d\n", err);
        return false;
    }

    // ── Cấu hình PCNT Channel 1 (Kênh B = pulse, Kênh A = control) ──
    // Kết hợp 2 channel = chế độ Quadrature x4 (đếm cả 4 cạnh)
    pcnt_config_t pcnt_config_b = {};
    pcnt_config_b.pulse_gpio_num = ENCODER_PIN_B;   // GPIO 35
    pcnt_config_b.ctrl_gpio_num  = ENCODER_PIN_A;   // GPIO 34
    pcnt_config_b.channel        = PCNT_CHANNEL_B;
    pcnt_config_b.unit           = PCNT_UNIT;
    pcnt_config_b.pos_mode       = PCNT_COUNT_DEC;  // Ngược lại Channel A
    pcnt_config_b.neg_mode       = PCNT_COUNT_INC;
    pcnt_config_b.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_config_b.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_config_b.counter_h_lim  = 30000;
    pcnt_config_b.counter_l_lim  = -30000;

    err = pcnt_unit_config(&pcnt_config_b);
    if (err != ESP_OK) {
        Serial.printf("[ENCODER][ERROR] PCNT Channel B config failed: %d\n", err);
        return false;
    }

    // ── Bộ lọc nhiễu phần cứng ──
    // Bỏ qua xung < 100 APB clock cycles (~1.25µs ở 80MHz)
    // Loại bỏ bounce/noise mà KHÔNG tốn CPU
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);

    // ── Khởi động counter ──
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);

    Serial.println("[ENCODER] PCNT Quadrature x4 initialized (2400 counts/rev)");

    // ── Tạo Task đọc encoder — Core 1 ──
    // Tần số 50Hz (20ms) — đủ nhanh cho PID loop
    // Core 1 cùng với LogicTask để dữ liệu được dùng ngay
    xTaskCreatePinnedToCore(
        encoderTask, "EncoderTask",
        2048,                   // Stack nhỏ — chỉ đọc PCNT + tính toán đơn giản
        NULL,
        PRIORITY_SENSORS,       // Cùng priority với sensor
        &encoderTaskHandle,
        1                       // Core 1
    );

    return true;
}

// ================== PCNT READ & RESET ==================
// Đọc giá trị counter hiện tại rồi reset về 0
// Atomic: pause → read → clear → resume
int16_t EncoderSensor::readAndResetPCNT() {
    int16_t count = 0;
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_get_counter_value(PCNT_UNIT, &count);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
    return count;
}

// ================== ENCODER TASK ==================
// Chạy 50Hz trên Core 1
// Tính: RPM, direction, distance, stall detection
void EncoderSensor::encoderTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(ENCODER_SAMPLE_MS); // 20ms

    // Biến cục bộ (không cần mutex)
    float localFilteredRPM = 0;
    unsigned long stallCounter = 0;  // Đếm số chu kỳ liên tiếp không có xung

    for (;;) {
        // 1. Đọc số xung trong chu kỳ vừa rồi (20ms)
        int16_t deltaPulses = readAndResetPCNT();

        // 2. Tính RPM
        //    deltaPulses = số counts trong ENCODER_SAMPLE_MS (ms)
        //    1 vòng = ENCODER_CPR counts (2400 ở chế độ x4)
        //    RPM = (deltaPulses / ENCODER_CPR) × (60000 / ENCODER_SAMPLE_MS)
        float instantRPM = ((float)deltaPulses / (float)ENCODER_CPR)
                         * (60000.0f / (float)ENCODER_SAMPLE_MS);

        // 3. Adaptive EMA filter cho RPM
        //    - Thay đổi lớn → alpha cao (phản ứng nhanh)
        //    - Ổn định → alpha thấp (mượt)
        float change = fabsf(instantRPM - localFilteredRPM);
        float alpha;
        if (change > 50.0f) {
            alpha = 0.8f;       // Tăng/giảm tốc đột ngột
        } else if (change > 10.0f) {
            alpha = 0.5f;       // Thay đổi vừa
        } else {
            alpha = 0.2f;       // Ổn định → mượt mà
        }
        localFilteredRPM = alpha * instantRPM + (1.0f - alpha) * localFilteredRPM;

        // 4. Xác định hướng quay
        int dir = 0;
        if (deltaPulses > 2) dir = 1;        // Tiến (ngưỡng 2 để lọc noise)
        else if (deltaPulses < -2) dir = -1;  // Lùi
        // else dir = 0 (dừng hoặc quá chậm)

        // 5. Tính quãng đường tăng thêm
        //    distance = (|deltaPulses| / ENCODER_CPR) × chu_vi_bánh_xe
        float deltaDistCm = (fabsf((float)deltaPulses) / (float)ENCODER_CPR)
                           * WHEEL_CIRCUMFERENCE_CM;

        // 6. Phát hiện xe bị kẹt (Stall Detection)
        //    Điều kiện: Motor đang chạy (targetSpeed != 0) nhưng encoder không quay
        int currentMotorSpeed = MotorController::getCurrentSpeed();
        bool motorRunning = (abs(currentMotorSpeed) > 30);
        bool wheelStopped = (abs(deltaPulses) <= 1);  // <= 1 count coi như dừng

        if (motorRunning && wheelStopped) {
            stallCounter++;
        } else {
            stallCounter = 0;
        }
        // Kẹt nếu liên tục > STALL_TIMEOUT_MS / ENCODER_SAMPLE_MS chu kỳ
        bool stall = (stallCounter > (STALL_TIMEOUT_MS / ENCODER_SAMPLE_MS));

        // 7. Cập nhật dữ liệu chia sẻ (thread-safe)
        xSemaphoreTake(encoderMutex, portMAX_DELAY);
        currentRPM       = instantRPM;
        filteredRPM      = localFilteredRPM;
        currentDirection = dir;
        totalPulses     += deltaPulses;
        totalDistanceCm += deltaDistCm;
        stallDetected    = stall;
        xSemaphoreGive(encoderMutex);

        #ifdef DEBUG_SENSOR
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 500) {
            lastPrint = millis();
            Serial.printf("[ENC] delta:%d RPM:%.0f filtered:%.0f dir:%d dist:%.1fcm stall:%d\n",
                          deltaPulses, instantRPM, localFilteredRPM, dir,
                          totalDistanceCm, stall);
        }
        #endif

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ================== PUBLIC API (Thread-safe) ==================
float EncoderSensor::getRPM() {
    float val;
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    val = filteredRPM;
    xSemaphoreGive(encoderMutex);
    return val;
}

float EncoderSensor::getSpeedCmPerSec() {
    float rpm;
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    rpm = filteredRPM;
    xSemaphoreGive(encoderMutex);
    // v = RPM × chu_vi / 60
    return (rpm * WHEEL_CIRCUMFERENCE_CM) / 60.0f;
}

float EncoderSensor::getDistanceCm() {
    float val;
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    val = totalDistanceCm;
    xSemaphoreGive(encoderMutex);
    return val;
}

void EncoderSensor::resetDistance() {
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    totalDistanceCm = 0;
    totalPulses = 0;
    xSemaphoreGive(encoderMutex);
}

bool EncoderSensor::isStalled() {
    bool val;
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    val = stallDetected;
    xSemaphoreGive(encoderMutex);
    return val;
}

int EncoderSensor::getDirection() {
    int val;
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    val = currentDirection;
    xSemaphoreGive(encoderMutex);
    return val;
}

long EncoderSensor::getTotalPulses() {
    long val;
    xSemaphoreTake(encoderMutex, portMAX_DELAY);
    val = totalPulses;
    xSemaphoreGive(encoderMutex);
    return val;
}
