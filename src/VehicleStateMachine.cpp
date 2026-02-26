#include "VehicleStateMachine.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include <Arduino.h>

// Static member definitions
State VehicleStateMachine::currentState = INIT;
unsigned long VehicleStateMachine::stateStartTime = 0;
unsigned long VehicleStateMachine::lastDebugTime = 0;
long VehicleStateMachine::scannedRightDist = 0;
long VehicleStateMachine::scannedLeftDist = 0;
bool VehicleStateMachine::turnRight = true;
bool VehicleStateMachine::scanCompleted = false;

// Biến nội bộ quản lý tiến trình quét Servo không chặn (Non-blocking scan)
static ScanPhase currentScanPhase = SCAN_IDLE;
static unsigned long lastScanStepTime = 0;

void VehicleStateMachine::begin() {
    // Khởi tạo random seed
    randomSeed(analogRead(0) + micros());
    
    // Đặt servo siêu âm về giữa
    MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
    
    currentState = NORMAL;
    currentScanPhase = SCAN_IDLE;
    Serial.println(">>> State Machine khoi tao: NORMAL");
}

void VehicleStateMachine::update() {
    // 1. ƯU TIÊN CAO NHẤT: Kiểm tra an toàn khẩn cấp (Thread-safe)
    if (MPUSensor::checkCollision() || MPUSensor::checkTilt()) {
        if (currentState != STOP) {
            MotorController::stopMotor();
            currentState = STOP;
            Serial.println("[SAFETY] EMERGENCY STOP ACTIVATED");
        }
        return;
    }
    // Cập nhật dữ liệu cảm biến
    // MPUSensor::update();
    
    // ⚠️ KIỂM TRA VA CHẠM / NGHIÊNG - DỪNG NGAY LẬP TỨC
    // if (MPUSensor::checkCollision() || MPUSensor::checkTilt()) {
    //     MotorController::setTargetSpeed(STOP_SPEED);
    //     MotorController::setTargetSteerServoAngle(90);
    //     Serial.println("!!! VA CHAM / NGHIENG - DUNG XE !!!");
    //     delay(2000);
    //     return;
    // }

    // 2. Đo khoảng cách Lấy dữ liệu cảm biến từ Buffer (Không block vì đã có Task riêng đọc)
    long frontDist = UltrasonicSensor::getFrontDistance();
    long backDist = UltrasonicSensor::getBackDistance();
    unsigned long now = millis();
    
    // 3. Xử lý State Machine
    switch (currentState) {
        case NORMAL:   handleNormalState(frontDist); break;
        case SLOW:     handleSlowState(frontDist); break;
        case TURN:     handleTurnState(frontDist, backDist); break; 
        case BACKING:  handleBackingState(backDist, now); break;
        case TURNING:  handleTurningState(now); break;
        case RESUMING: handleResumingState(frontDist, now); break;
        case STOP:     handleStopState(frontDist); break;
        default:       currentState = NORMAL; break;
    }

    // 4. Cập nhật Output (Chỉ thực thi khi có thay đổi thực sự nhờ Dirty Bit trong MotorController)
    MotorController::smoothSteerServoTransition();
    MotorController::limitSpeedBySteering();
    MotorController::smoothSpeedTransition();
}

// // ========== 1️⃣ NORMAL ==========
// void VehicleStateMachine::handleNormalState(long frontDist) {
//     // Đảm bảo servo siêu âm ở giữa
//     MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
    
//     if (frontDist > TURN_DISTANCE) {
//         // An toàn - chạy cruise speed
//         MotorController::setTargetSpeed(CRUISE_SPEED);
//         MotorController::setTargetSteerServoAngle(90);
//     } 
//     else if (frontDist > SLOW_DISTANCE && frontDist <= TURN_DISTANCE) {
//         // Gần vật cản - chuyển sang SLOW
//         currentState = SLOW;
//         Serial.println(">>> NORMAL -> SLOW");
//     } 
//     else if (frontDist <= SLOW_DISTANCE) {
//         // Rất gần - chuyển sang TURN (tránh vật cản)
//         currentState = TURN;
//         stateStartTime = millis();
//         scanCompleted = false;  // Reset flag để quét lại
//         Serial.println(">>> NORMAL -> TURN (quet servo)");
//     }
// }

// ========== 2️⃣ SLOW ==========
void VehicleStateMachine::handleSlowState(long frontDist) {
    // Chạy chậm
    MotorController::setTargetSpeed(MIN_RUN_SPEED);
    MotorController::setTargetSteerServoAngle(90);
    
    if (frontDist > TURN_DISTANCE) {
        // Đường rộng lại - về NORMAL
        currentState = NORMAL;
        Serial.println(">>> SLOW -> NORMAL");
    } 
    else if (frontDist <= STOP_DISTANCE) {
        // Quá gần - phải tránh
        currentState = TURN;
        currentScanPhase = SCAN_IDLE;
        Serial.println(">>> SLOW -> TURN (quet servo)");
    }
}

// // ========== 3️⃣ TURN (QUÉT SERVO & QUYẾT ĐỊNH) ==========
// void VehicleStateMachine::handleTurnState(long frontDist, long backDist) {
//     // Dừng xe để quét
//     MotorController::setTargetSpeed(STOP_SPEED);
//     MotorController::setTargetSteerServoAngle(90);

//     unsigned long now = millis();
    
//     // ⚠️ BƯỚC 2: QUÉT BẰNG SERVO SIÊU ÂM - CHỈ 1 LẦN!
//     if (!scanCompleted) {
//         UltrasonicSensor::scanAllDirections(scannedRightDist, scannedLeftDist);
//         scanCompleted = true;  // Đánh dấu đã quét xong
//     }
    
//     // ⚠️ BƯỚC 3: RA QUYẾT ĐỊNH
//     Serial.print(">>> Phan tich: Phai=");
//     Serial.print(scannedRightDist);
//     Serial.print(" Trai=");
//     Serial.println(scannedLeftDist);
    
//     // Nếu phải rộng hơn và > TURN_DISTANCE
//     if (scannedRightDist > TURN_DISTANCE && scannedRightDist > scannedLeftDist) {
//         turnRight = true;
//         currentState = TURNING;
//         stateStartTime = millis();
//         Serial.println(">>> Quyet dinh: QUAY PHAI");
//     }
//     // Nếu trái rộng hơn và > TURN_DISTANCE
//     else if (scannedLeftDist > TURN_DISTANCE) {
//         turnRight = false;
//         currentState = TURNING;
//         stateStartTime = millis();
//         Serial.println(">>> Quyet dinh: QUAY TRAI");
//     }
//     // Cả 2 hướng đều hẹp - phải lùi
//     else {
//         if (backDist > BACK_DANGER_DISTANCE) {
//             currentState = BACKING;
//             stateStartTime = millis();
//             Serial.println(">>> Quyet dinh: LUI LAI");
//         } else {
//             currentState = STOP;
//             Serial.println(">>> Quyet dinh: BI KET - DUNG LAI");
//         }
//     }
// }

void VehicleStateMachine::handleTurnState(long frontDist, long backDist) {
    // Dừng xe để quét
    MotorController::setTargetSpeed(STOP_SPEED);
    
    unsigned long now = millis();

    // Thay vì gọi hàm scanAllDirections tập trung (gây block), ta chia nhỏ giai đoạn
    switch (currentScanPhase) {
        case SCAN_IDLE:
            Serial.println("[SCAN] Step 1: Quay phai...");
            MotorController::setUSSensorServoAngle(US_SCAN_RIGHT);
            lastScanStepTime = now;
            currentScanPhase = SCAN_RIGHT;
            break;

        case SCAN_RIGHT:
            if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
                scannedRightDist = UltrasonicSensor::readDistanceRaw(TRIG_FRONT, ECHO_FRONT);
                Serial.printf("[SCAN] Ket qua Phai: %ld cm. Quay trai...\n", scannedRightDist);
                MotorController::setUSSensorServoAngle(US_SCAN_LEFT);
                lastScanStepTime = now;
                currentScanPhase = SCAN_LEFT;
            }
            break;

        case SCAN_LEFT:
            if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
                scannedLeftDist = UltrasonicSensor::readDistanceRaw(TRIG_FRONT, ECHO_FRONT);
                Serial.printf("[SCAN] Ket qua Trai: %ld cm. Ve trung tam...\n", scannedLeftDist);
                MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
                lastScanStepTime = now;
                currentScanPhase = SCAN_CENTER_RETURN;
            }
            break;

        case SCAN_CENTER_RETURN:
            if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
                currentScanPhase = SCAN_COMPLETE;
            }
            break;

        case SCAN_COMPLETE:
            // Đưa ra quyết định dựa trên dữ liệu đã quét
            if (scannedRightDist > TURN_DISTANCE && scannedRightDist >= scannedLeftDist) {
                turnRight = true;
                currentState = TURNING;
            } else if (scannedLeftDist > TURN_DISTANCE) {
                turnRight = false;
                currentState = TURNING;
            } else {
                currentState = (backDist > BACK_DANGER_DISTANCE) ? BACKING : STOP;
            }
            
            stateStartTime = now;
            currentScanPhase = SCAN_IDLE; // Reset cho lần sau
            break;
    }
}

// ========== CÁC HÀM XỬ LÝ KHÁC (Giữ nguyên logic cũ nhưng tối ưu Task) ==========
void VehicleStateMachine::handleNormalState(long frontDist) {
    if (frontDist <= STOP_DISTANCE) {
        currentState = TURN;
        currentScanPhase = SCAN_IDLE;
    } else if (frontDist <= TURN_DISTANCE) {
        currentState = SLOW;
    } else {
        MotorController::setTargetSpeed(CRUISE_SPEED);
        MotorController::setTargetSteerServoAngle(90);
    }
}
// ========== 4️⃣ BACKING ==========
void VehicleStateMachine::handleBackingState(long backDist, unsigned long now) {
    // Kiểm tra an toàn phía sau
    if (backDist <= BACK_DANGER_DISTANCE) {
        // Không an toàn - dừng
        MotorController::stopMotor();
        currentState = STOP;
        Serial.println(">>> BACKING -> STOP (sau bi chan)");
        return;
    }
    
    // Lùi với servo giữa
    MotorController::setTargetSpeed(-BACK_SPEED);
    MotorController::setTargetSteerServoAngle(90);
    
    // Lùi đủ thời gian?
    if (now - stateStartTime >= BACK_TIME) {
        // Sau khi lùi xong, quét lại
        currentState = TURN;
        scanCompleted = false;  // Reset flag để quét lại
        Serial.println(">>> BACKING -> TURN (quet lai)");
    }
}

// ========== 5️⃣ TURNING (THỰC HIỆN Rẽ) ==========
void VehicleStateMachine::handleTurningState(unsigned long now) {
    // Servo siêu âm về giữa (không quét nữa)
    // MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
    
    // Rẽ với tốc độ cao để thắng ma sát
    MotorController::setTargetSpeed(TURN_BOOST);
    MotorController::setTargetSteerServoAngle(turnRight ? 45 : 135);
    
    // if (turnRight) {
    //     MotorController::setTargetSteerServoAngle(45);  // Rẽ phải
    // } else {
    //     MotorController::setTargetSteerServoAngle(135); // Rẽ trái
    // }
    
    // Giữ thời gian rẽ
    if (now - stateStartTime >= TURN_TIME) {
        currentState = RESUMING;
        stateStartTime = now;
        Serial.println(">>> TURNING -> RESUMING");
    }
}

// ========== 6️⃣ RESUMING ==========
void VehicleStateMachine::handleResumingState(long frontDist, unsigned long now) {
    // Trả servo lái về giữa dần
    MotorController::setTargetSteerServoAngle(90);
    
    // Chạy chậm tiếp
    MotorController::setTargetSpeed(CRUISE_SPEED);
    
    // Giữ một chút rồi về NORMAL
    if (now - stateStartTime >= RESUME_TIME) {
        currentState = (frontDist > TURN_DISTANCE) ? NORMAL : TURN;
        // Nếu trong khoảng STOP_DISTANCE..TURN_DISTANCE, tiếp tục RESUMING
    }
}

// ========== 7️⃣ STOP ==========
void VehicleStateMachine::handleStopState(long frontDist) {
    // Dừng hoàn toàn
    MotorController::setTargetSpeed(STOP_SPEED);
    MotorController::setTargetSteerServoAngle(90);
    
    // Nếu đường trước rộng lại → về NORMAL
    if (frontDist > TURN_DISTANCE) {
        currentState = NORMAL;
        Serial.println(">>> STOP -> NORMAL (duong rong)");
    }
}

// ========== DEBUG OUTPUT ==========
void VehicleStateMachine::debugOutput() {
    if (millis() - lastDebugTime > 500) {
        long frontDist = UltrasonicSensor::getFrontDistance();
        long backDist = UltrasonicSensor::getBackDistance();
        
        const char* stateNames[] = {"INIT", "NORMAL", "SLOW", "TURN", "STOP", "BACKING", "TURNING", "RESUMING"};
        
        Serial.print("[");
        Serial.print(stateNames[currentState]);
        Serial.print("] F:");
        Serial.print(frontDist);
        Serial.print(" B:");
        Serial.print(backDist);
        Serial.print(" | Spd:");
        Serial.print(MotorController::getCurrentSpeed());
        Serial.print(" L:");
        Serial.print(MotorController::getLeftMotorSpeed());
        Serial.print(" R:");
        Serial.print(MotorController::getRightMotorSpeed());
        Serial.print(" | Steer:");
        Serial.print(MotorController::getSteerServoAngle());
        Serial.print(" US:");
        Serial.print(MotorController::getUSSensorServoAngle());
        Serial.print(" [");
        Serial.print(turnRight ? "R" : "L");
        Serial.print("]");
        
        if (scannedRightDist > 0 || scannedLeftDist > 0) {
            Serial.print(" | Scan R:");
            Serial.print(scannedRightDist);
            Serial.print(" L:");
            Serial.print(scannedLeftDist);
        }
        
        Serial.println("");
        
        lastDebugTime = millis();
    }
}