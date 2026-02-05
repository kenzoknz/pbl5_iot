

#include "Config.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include "VehicleStateMachine.h"

void setup() {
    Serial.begin(115200);
    Serial.println("=== KHOI TAO HE THONG ESP32 PBL5 MODULAR ===");
    
    // Khởi tạo các module theo thứ tự
    Serial.println("1. Khoi tao Ultrasonic Sensors...");
    UltrasonicSensor::begin();
    
    Serial.println("2. Khoi tao MPU6050...");
    if (!MPUSensor::begin()) {
        Serial.println("!!! DUNG LAI: Loi MPU6050 !!!");
        while (1) { delay(1000); }
    }
    
    Serial.println("3. Khoi tao Motor Controller...");
    MotorController::begin();
    
    Serial.println("4. Khoi tao State Machine...");
    VehicleStateMachine::begin();
    
    Serial.println("=== HE THONG SAN SANG ===");
    Serial.println("Modules:");
    Serial.println("- UltrasonicSensor: Front/Back distance measurement");
    Serial.println("- MPUSensor: Collision & tilt detection");
    Serial.println("- MotorController: Servo + differential steering");
    Serial.println("- VehicleStateMachine: Navigation logic");
    Serial.println("=====================================");
}

void loop() {
    // Cập nhật state machine (chứa toàn bộ logic điều khiển)
    VehicleStateMachine::update();
    
    // Hiển thị debug info
    VehicleStateMachine::debugOutput();
    
    // Delay nhỏ cho ổn định
    delay(60);
}