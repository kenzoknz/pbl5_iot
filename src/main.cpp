#include "Config.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include "EncoderSensor.h" 
#include "VehicleStateMachine.h"

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
    VehicleStateMachine::begin();

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
        4096,                   // Stack dành cho kết nối mạng thường lớn
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
 * Task App: Nơi xử lý dữ liệu từ người dùng (WiFi/Bluetooth/Serial)
 * Hiện tại để trống, sẵn sàng cho việc tích hợp App sau này
 */
void vAppTask(void *pvParameters) {
    for (;;) {
        // Ví dụ: Kiểm tra dữ liệu từ Serial/App
        // Nếu nhận lệnh MANUAL: 
        // 1. UltrasonicSensor::setMode(OperationMode::MANUAL); -> Tự động Suspend 4 Sensor Tasks
        // 2. Xử lý lệnh di chuyển trực tiếp từ người dùng.
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Kiểm tra mỗi 100ms
    }
}

void loop() {
}