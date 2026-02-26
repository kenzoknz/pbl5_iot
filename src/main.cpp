#include "Config.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include "VehicleStateMachine.h"

// Task Handles để quản lý (Suspend/Resume/Monitor)
TaskHandle_t xLogicTaskHandle = NULL;
TaskHandle_t xAppTaskHandle   = NULL;

// Hàm định nghĩa các Task
void vLogicTask(void *pvParameters);
void vAppTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    Serial.println("=== KHOI TAO HE THONG ESP32 MULTI-TASKING ===");

    // 1. Khởi tạo phần cứng (Đã bao gồm tạo Task Sensor bên trong các module)
    UltrasonicSensor::begin(); // Đã tạo 2 task đọc cảm biến trên Core 0
    
    if (!MPUSensor::begin()) { // Đã tạo mpuTask trên Core 1
        Serial.println("!!! DUNG LAI: Loi MPU6050 !!!");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
    
    MotorController::begin();
    VehicleStateMachine::begin();

    // 2. Tạo Task Xử lý Logic (State Machine) - Chạy trên Core 1
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

    // 3. Tạo Task App (Dự phòng cho chế độ điều khiển qua App) - Chạy trên Core 0
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
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Chạy ổn định ở 20Hz

    for (;;) {
        // Chỉ chạy logic tự hành nếu đang ở chế độ AUTONOMOUS
        if (UltrasonicSensor::getMode() == OperationMode::AUTONOMOUS) {
            VehicleStateMachine::update();
        }

        // Luôn cập nhật Output cho Motor (Smooth transition)
        // Lưu ý: Các hàm smooth này đã được tối ưu trong MotorController
        
        #ifdef DEBUG_SENSOR
        VehicleStateMachine::debugOutput();
        #endif

        // Đợi cho đến chu kỳ tiếp theo (Tối ưu hơn delay() thường)
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
        // 1. UltrasonicSensor::setMode(OperationMode::MANUAL); -> Tự động Suspend Sensor Tasks
        // 2. Xử lý lệnh di chuyển trực tiếp từ người dùng.
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Kiểm tra mỗi 100ms
    }
}

void loop() {
    // Để trống hoàn toàn vì hệ thống đã chạy đa nhiệm Task
}