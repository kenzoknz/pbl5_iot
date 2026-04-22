#include "core/Config.h"
#include "sensors/UltrasonicSensor.h"
#include "sensors/MPUSensor.h"
#include "control/MotorController.h"
#include "sensors/EncoderSensor.h"
#include "sensors/GPSSensor.h"
#include "sensors/GpsQueue.h"
#include "control/VehicleStateMachine.h"
#include "communication/NetworkManager.h"
#include "communication/CommandProcessor.h"
#include "communication/JetsonUART.h"
#include "core/ConfigStorage.h"

TaskHandle_t xLogicTaskHandle  = NULL;
TaskHandle_t xAppTaskHandle    = NULL;
TaskHandle_t xJetsonTaskHandle = NULL;
 
void vLogicTask (void *pvParameters);
void vAppTask   (void *pvParameters);
void vJetsonTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 ROBOT ===");

    // 1. phần cứng
    UltrasonicSensor::begin();
    
    if (!MPUSensor::begin()) { // mpuTask trên Core 1
        Serial.println("!!! MPU6050 init failed !!!");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    MotorController::begin();

    // Encoder (PCNT hardware + task Core 1) 
    if (!EncoderSensor::begin()) {
        Serial.println("!!! ENCODER FAILED -> PID disabled !!!"); // robot vẫn chạy được không có PID
    } else {
        MotorController::enablePID(true);  // Bật PID khi encoder hoạt động
    }

    GPSSensor::begin();
    VehicleStateMachine::begin();
    GpsQueue::begin();

    xTaskCreatePinnedToCore(
        vJetsonTask,
        "JetsonTask",
        STACK_SIZE_JETSON,      // 3072 bytes
        NULL,
        PRIORITY_JETSON,        // 6
        &xJetsonTaskHandle,
        0
    );

    // 2. Task Xử lý Logic (State Machine) -  Core 1, 20Hz
    // Nhiệm vụ: Ra quyết định điều hướng dựa trên dữ liệu từ SensorTask
    xTaskCreatePinnedToCore(
        vLogicTask,             
        "LogicTask",           
        STACK_SIZE_LOGIC,       // Độ lớn Stack (từ Config.h)
        NULL,                   
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

    Serial.println("=== ALL TASKS STARTED ===");
    
    vTaskDelete(NULL); 
}

 
/* ══════════════════════════════════════════════════════
 *  vJetsonTask — Core 0, Priority 6, 100Hz
 *
 *  Hoạt động như một sensor task:
 *  - Độc lập hoàn toàn với WiFi
 *  - Polling Serial1 mỗi 10ms (5x nhanh hơn LogicTask)
 *  - Đảm bảo lệnh STOP được enqueue TRƯỚC khi LogicTask
 *    chạy chu kỳ tiếp theo (50ms)
 *
 *  Chỉ làm:  đọc RX + parse + enqueue + gửi TX status
 *  Không làm: WiFi, WebSocket, MotorController
 * ══════════════════════════════════════════════════════ */
void vJetsonTask(void *pvParameters) {
    JetsonUART::begin();
 
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(JETSON_TASK_RATE_MS); // 10ms
 
    for (;;) {
        /* ── RX: đọc byte → khi có line hoàn chỉnh thì parse → enqueue ── */
        if (JetsonUART::checkForCommands()) {
            JetsonUART::handleJetsonCommand();
        }
 
        /* Giữ nhịp 100Hz chính xác — preempt nếu vừa xử lý xong sớm */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
 

/**
 * Task Logic: Xử lý State Machine và Điều khiển Motor
 * Chạy trên Core 1 để tách biệt với việc đọc cảm biến ở Core 0
 */
void vLogicTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz

    for (;;) {
        JetsonUART::processQueuedCommand();
        JetsonUART::checkWatchdog();
        if (JetsonUART::isStopHoldActive()) {
            MotorController::setTargetSpeed(0);
            MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
            MotorController::stopMotor();
            MotorController::smoothSteerServoTransition();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

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
 * *  ── WiFi Guard ──
 *  Khi không có WiFi:
 *    • Chỉ chạy GPS update (Serial2, độc lập WiFi)
 *    • Chỉ chạy tickSafety (joystick watchdog)
 *    • reconnectIfNeeded() với backoff
 *    • Delay 500ms → task chạy ở 2Hz thay vì 20Hz
 *    • SKIP toàn bộ WebSocket / HTTP / GPS queue
 *
 *  Khi có WiFi:
 *    • Chạy đầy đủ 20Hz
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
    if (wifiOk) {
        NetworkManager::initWebSocket([](const String& msg) {
            CommandProcessor::handleWsMessage(msg);
        });
        Serial.println("[APP] WiFi + WebSocket ready");
    } else {
        Serial.println("[APP] No WiFi — offline mode (Jetson + AUTONOMOUS unaffected)");
    }

    // ── Bước 3: Khởi tạo CommandProcessor ──
    CommandProcessor::begin();
    // JetsonUART::begin();

    Serial.println("[APP] AppTask ready.");

    // static uint32_t lastStatusSend = 0;
    // bool jetsonTimeoutLogged = false;

    for (;;) {
        // ----------luôn chạy, độc lập với Wifi

        // Tick GPS parser (Neo-7N UART2)
        GPSSensor::update();
        // Safety watchdog: nếu joystick ngừng gửi realtime command thì tự dừng xe
        CommandProcessor::tickSafety();

        // --------wifi (skip if offline)
        if (!NetworkManager::isWiFiConnected()) {
            NetworkManager::reconnectIfNeeded();
 
            /* Chạy chậm lại khi offline: 2Hz thay vì 20Hz, không tranh với JetsonTask (priority 6) */
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;  
        }

        // ---------Online mode: chạy đầy đủ 20Hz, xử lý WebSocket realtime

        // Tick WebSocket (xử lý ping/pong, nhận frame, gửi pending)
        NetworkManager::wsLoop();

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

        // Nhường CPU — 50ms (20Hz), đủ nhạy cho MANUAL control
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void loop() {
}
