#include "Config.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include "EncoderSensor.h"
#include "GPSSensor.h"
#include "GpsQueue.h"
#include "VehicleStateMachine.h"
#include "NetworkManager.h"
#include "CommandProcessor.h"

// Task Handles để quản lý (Suspend/Resume/Monitor)
TaskHandle_t xLogicTaskHandle = NULL;
TaskHandle_t xAppTaskHandle   = NULL;

// Hàm định nghĩa các Task
void vLogicTask(void *pvParameters);
void vAppTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 ROBOT ===");

    // 1. phần cứng
    UltrasonicSensor::begin(); // 4 cam bien -> 4 tasks Core 0
    
    if (!MPUSensor::begin()) { // mpuTask trên Core 1
        Serial.println("!!! DUNG LAI: Loi MPU6050 !!!");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    MotorController::begin();
    // Encoder (PCNT hardware + task Core 1) 
    if (!EncoderSensor::begin()) {
        Serial.println("!!! ENCODER FAILED !!!");
        // Không halt — encoder là optional, robot vẫn chạy được không có PID
    } else {
        MotorController::enablePID(true);  // Bật PID khi encoder hoạt động
    }
    GPSSensor::begin();
    VehicleStateMachine::begin();
    
    // Init GPS queue
    GpsQueue::begin();

    // 2. Task Xử lý Logic (State Machine) -  Core 1, 20Hz
    // Nhiệm vụ: Ra quyết định điều hướng dựa trên dữ liệu từ SensorTask
    xTaskCreatePinnedToCore(
        vLogicTask,             // Hàm thực thi
        "LogicTask",            // Tên Task
        STACK_SIZE_LOGIC,       // Độ lớn Stack (từ Config.h)
        NULL,                   // Tham số truyền vào
        PRIORITY_LOGIC,         // Ưu tiên (Trung bình)
        &xLogicTaskHandle,      // Handle để quản lý
        1                       // Chạy trên Core 1 (Xử lý tính toán)
    );

    // 3. Task App - Core 0, 20Hz
    // Nhiệm vụ: Nhận lệnh từ WiFi/Bluetooth mà không block hệ thống
    xTaskCreatePinnedToCore(
        vAppTask,
        "AppTask",
        STACK_SIZE_APP,         // AppTask xử lý WS + HTTP + JSON, cần stack lớn hơn
        NULL,
        PRIORITY_APP,           // Ưu tiên cao nhất để phản hồi realtime
        &xAppTaskHandle,
        0                       // Core 0 (Chuyên trách kết nối & I/O)
    );

    Serial.println("=== TAT CA TASK DA KHOI CHAY ===");
    
    // Xóa Task setup() để giải phóng bộ nhớ (FreeRTOS sẽ quản lý hoàn toàn)
    vTaskDelete(NULL); 
}

/**
 * Task Logic: Xử lý State Machine và Điều khiển Motor
 * Chạy trên Core 1 để tách biệt với việc đọc cảm biến ở Core 0
 */
void vLogicTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz

    for (;;) {
        // Chỉ chạy logic tự hành nếu đang ở chế độ AUTONOMOUS
        if (UltrasonicSensor::getMode() == OperationMode::AUTONOMOUS) {
            VehicleStateMachine::update();
        } else {
            // MANUAL mode: Vẫn cần cập nhật servo và motor để phản hồi lệnh
            // Chỉ chạy output pipeline, không chạy state machine logic
            MotorController::smoothSteerServoTransition();
            MotorController::limitSpeedBySteering();
            MotorController::updatePID();
            MotorController::smoothSpeedTransition();
        }

        // Luôn cập nhật Output cho Motor (Smooth transition)        
        #ifdef DEBUG_SENSOR
        VehicleStateMachine::debugOutput();
        #endif

        // Đợi cho đến chu kỳ tiếp theo
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * Task App: Kết nối WiFi, WebSocket, nhận lệnh từ Web Server.
 * Chạy trên Core 0 — non-blocking.
 *
 * Luồng:
 *   1. Kết nối WiFi
 *   2. Khởi tạo WebSocket → server push lệnh realtime
 *   3. Vòng lặp:
 *      - Kiểm tra WiFi, reconnect nếu mất
 *      - Gọi ws.loop() để xử lý frame WebSocket đến
 *      - Fallback: HTTP polling khi WS ngắt
 *      - Gửi status heartbeat mỗi 5s
 */
void vAppTask(void *pvParameters) {
    // ── Bước 1: WiFi ──
    bool wifiOk = NetworkManager::initWiFi();
    if (!wifiOk) {
        Serial.println("[APP] Không có WiFi — tiếp tục ở chế độ offline (AUTONOMOUS)");
    }

    // ── Bước 2: WebSocket ──
    // Callback: chuyển mọi message WS sang CommandProcessor
    NetworkManager::initWebSocket([](const String& msg) {
        CommandProcessor::handleWsMessage(msg);
    });

    // ── Bước 3: Khởi tạo CommandProcessor ──
    CommandProcessor::begin();

    Serial.println("[APP] AppTask ready.");

    for (;;) {
        // Kiểm tra WiFi, tự reconnect với exponential backoff
        NetworkManager::reconnectIfNeeded();

        // Tick WebSocket (xử lý ping/pong, nhận frame, gửi pending)
        NetworkManager::wsLoop();

        // Tick GPS parser (Neo-7N UART2)
        GPSSensor::update();

        // Tick portal cấu hình WiFi khi ESP đang ở SoftAP fallback
        NetworkManager::portalLoop();

        // Fallback HTTP polling CHỈ KHI WebSocket KHÔNG kết nối
        // Khi WS hoạt động, MODE_CHANGE sẽ được push realtime
        if (!NetworkManager::wsConnected()) {
            CommandProcessor::pollCommands();
            CommandProcessor::pollMode();  // Sync mode với database
        }

        // Flush GPS queue khi mạng lại
        CommandProcessor::tickQueueFlush();

        // Gửi status robot lên server qua WebSocket (throttled 5s)
        CommandProcessor::sendStatusUpdate();

        // Safety watchdog: nếu joystick ngừng gửi realtime command thì tự dừng xe
        CommandProcessor::tickSafety();

        // Nhường CPU — 50ms (20Hz), đủ nhạy cho MANUAL control
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void loop() {
}