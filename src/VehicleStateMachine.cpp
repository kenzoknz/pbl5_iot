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
ScanPhase VehicleStateMachine::currentScanPhase = SCAN_IDLE; 
unsigned long VehicleStateMachine::lastScanStepTime = 0;

static const uint32_t SERVO_SETTLE_MS = 200;

// ── [FIX BUG-1] Hàm đọc median 3 mẫu tại chỗ ────────────────────────────────
// Gọi từ handleTurnState() khi servo đã đứng yên ở góc cố định.
// Không dùng getFrontDistance() vì lúc này background task bị suspend ([FIX BUG-7])
// nên buffer có thể không được cập nhật.
// vTaskDelay(60ms) giữa các lần đọc = đúng yêu cầu HC-SR04.
static long scanMedian3(int trig, int echo) {
    long s[3];
    for (int i = 0; i < 3; i++) {
        s[i] = UltrasonicSensor::readDistanceRaw(trig, echo);
        vTaskDelay(pdMS_TO_TICKS(65)); // 65ms > 60ms min của HC-SR04
    }
    // Bubble sort 3 phần tử
    if (s[0] > s[1]) { long t = s[0]; s[0] = s[1]; s[1] = t; }
    if (s[1] > s[2]) { long t = s[1]; s[1] = s[2]; s[2] = t; }
    if (s[0] > s[1]) { long t = s[0]; s[0] = s[1]; s[1] = t; }
    return s[1];
}

void VehicleStateMachine::begin() {
    // Khởi tạo random seed
    randomSeed(analogRead(0) + micros());
    
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

    // 4. Cập nhật Output (Chỉ thực thi khi có thay đổi -> Dirty Bit trong MotorController)
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

void VehicleStateMachine::handleNormalState(long frontDist) {
    MotorController::setUSSensorServoAngle(US_SCAN_CENTER); 
    
    if (frontDist > TURN_DISTANCE) {
        // Đường rộng — chạy tốc độ cruise
        MotorController::setTargetSpeed(CRUISE_SPEED);
        MotorController::setTargetSteerServoAngle(90);
    } else if (frontDist > SLOW_DISTANCE) {
        // Vùng đệm — giảm tốc, chưa cần dừng
        currentState = SLOW;
        Serial.println(">>> NORMAL -> SLOW");
    }
    else {
        // Gần hoặc rất gần — quét ngay
        currentState     = TURN;
        currentScanPhase = SCAN_IDLE; // [FIX BUG-5]
        stateStartTime   = millis();
        Serial.println(">>> NORMAL -> TURN");
    }
}

// ========== 2️⃣ SLOW ==========
void VehicleStateMachine::handleSlowState(long frontDist) {
    MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
    MotorController::setTargetSpeed(MIN_RUN_SPEED);
    
    if (frontDist > TURN_DISTANCE) {
        // Đường rộng lại - về NORMAL
        currentState = NORMAL;
        Serial.println(">>> SLOW -> NORMAL");
    } 
    else if (frontDist <= STOP_DISTANCE) {
        // Quá gần - phải tránh
        currentState = TURN;
        currentScanPhase = SCAN_IDLE;
        stateStartTime   = millis();
        Serial.println(">>> SLOW -> TURN (quet servo)");
        // Nếu SLOW_DISTANCE >= frontDist > STOP_DISTANCE → giữ SLOW, tiếp tục giảm tốc
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

// void VehicleStateMachine::handleTurnState(long frontDist, long backDist) {
//     // Dừng xe để quét
//     MotorController::setTargetSpeed(STOP_SPEED);
//     unsigned long now = millis();

//     // Thay vì gọi hàm scanAllDirections tập trung (gây block), ta chia nhỏ giai đoạn
//     switch (currentScanPhase) {
//         case SCAN_IDLE:
//             Serial.println("[SCAN] Step 1: Quay phai...");
//             MotorController::setUSSensorServoAngle(US_SCAN_RIGHT);
//             lastScanStepTime = now;
//             currentScanPhase = SCAN_RIGHT;
//             break;

//         case SCAN_RIGHT:
//             if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
//                 scannedRightDist = UltrasonicSensor::readDistanceRaw(TRIG_FRONT, ECHO_FRONT);
//                 Serial.printf("[SCAN] Ket qua Phai: %ld cm. Quay trai...\n", scannedRightDist);
//                 MotorController::setUSSensorServoAngle(US_SCAN_LEFT);
//                 lastScanStepTime = now;
//                 currentScanPhase = SCAN_LEFT;
//             }
//             break;

//         case SCAN_LEFT:
//             if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
//                 scannedLeftDist = UltrasonicSensor::readDistanceRaw(TRIG_FRONT, ECHO_FRONT);
//                 Serial.printf("[SCAN] Ket qua Trai: %ld cm. Ve trung tam...\n", scannedLeftDist);
//                 MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
//                 lastScanStepTime = now;
//                 currentScanPhase = SCAN_CENTER_RETURN;
//             }
//             break;

//         case SCAN_CENTER_RETURN:
//             if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
//                 currentScanPhase = SCAN_COMPLETE;
//             }
//             break;

//         case SCAN_COMPLETE:
//             // Đưa ra quyết định dựa trên dữ liệu đã quét
//             if (scannedRightDist > TURN_DISTANCE && scannedRightDist >= scannedLeftDist) {
//                 turnRight = true;
//                 currentState = TURNING;
//             } else if (scannedLeftDist > TURN_DISTANCE) {
//                 turnRight = false;
//                 currentState = TURNING;
//             } else {
//                 currentState = (backDist > BACK_DANGER_DISTANCE) ? BACKING : STOP;
//             }
            
//             stateStartTime = now;
//             currentScanPhase = SCAN_IDLE; // Reset cho lần sau
//             break;
//     }
// }

// ════════════════════════════════════════════════════════════════════════════
// handleTurnState — QUÉT SERVO THEO STATE MACHINE KHÔNG BLOCKING
//
// Luồng đúng:
//   SCAN_IDLE
//     → Ra lệnh quay servo sang phải
//   SCAN_MOVING_RIGHT  (chờ SERVO_SETTLE_MS)
//     → Servo đã đứng yên: đọc 3 mẫu median [FIX BUG-1,3]
//   SCAN_READING_RIGHT
//     → Ra lệnh quay servo sang trái
//   SCAN_MOVING_LEFT   (chờ SERVO_SETTLE_MS)
//     → Servo đã đứng yên: đọc 3 mẫu median
//   SCAN_READING_LEFT
//     → Ra lệnh quay về giữa, resume background task [FIX BUG-7]
//   SCAN_RETURNING_CENTER (chờ SERVO_SETTLE_MS)
//   SCAN_COMPLETED
//     → Ra quyết định hướng đi
// ════════════════════════════════════════════════════════════════════════════

// Trong VehicleStateMachine.cpp
void VehicleStateMachine::handleTurnState(long frontDist, long backDist) {
    MotorController::setTargetSpeed(STOP_SPEED); // Dừng xe để quét
    unsigned long now = millis();

    switch (currentScanPhase) {
        case SCAN_IDLE:
            UltrasonicSensor::suspendFrontTask();
            MotorController::setUSSensorServoAngle(US_SCAN_RIGHT);
            lastScanStepTime = now;
            currentScanPhase = SCAN_MOVING_RIGHT;
            break;

        case SCAN_MOVING_RIGHT:
            if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) { // Đợi servo quay xong
                // scannedRightDist = UltrasonicSensor::readDistanceRaw(TRIG_FRONT, ECHO_FRONT); // Đo thực tế
                // currentScanPhase = SCAN_MOVING_LEFT;
                currentScanPhase = SCAN_READING_RIGHT;
                // MotorController::setUSSensorServoAngle(US_SCAN_LEFT); // Ra lệnh quay trái
                // lastScanStepTime = now;
            }
            break;
        
         case SCAN_READING_RIGHT:
            // [FIX BUG-1] Đọc median 3 mẫu thay vì 1 mẫu
            scannedRightDist = scanMedian3(TRIG_FRONT, ECHO_FRONT);
            Serial.printf("[SCAN] Phai (%d°): %ld cm\n", US_SCAN_RIGHT, scannedRightDist);
            MotorController::setUSSensorServoAngle(US_SCAN_LEFT);
            lastScanStepTime = now;
            currentScanPhase = SCAN_MOVING_LEFT;
            break;

        case SCAN_MOVING_LEFT:
            if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
                // scannedLeftDist = UltrasonicSensor::readDistanceRaw(TRIG_FRONT, ECHO_FRONT);
                // currentScanPhase = SCAN_RETURNING_CENTER;
                // MotorController::setUSSensorServoAngle(US_SCAN_CENTER); // Quay về thẳng
                // lastScanStepTime = now;
                currentScanPhase = SCAN_READING_LEFT;
            }
            break;
        
        case SCAN_READING_LEFT:
            // [FIX BUG-1] Đọc median 3 mẫu
            scannedLeftDist = scanMedian3(TRIG_FRONT, ECHO_FRONT);
            Serial.printf("[SCAN] Trai (%d°): %ld cm\n", US_SCAN_LEFT, scannedLeftDist);
            // Quay về giữa và resume background task
            MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
            UltrasonicSensor::resumeFrontTask(); // [FIX BUG-7]
            lastScanStepTime = now;
            currentScanPhase = SCAN_RETURNING_CENTER;
            break;

        case SCAN_RETURNING_CENTER:
            if (now - lastScanStepTime >= SERVO_SCAN_DELAY_MS) {
                currentScanPhase = SCAN_COMPLETED;
            }
            break;

        case SCAN_COMPLETED:
            Serial.printf("[SCAN] Phan tich: Phai=%ld Trai=%ld (nguong=%d)\n",
                          scannedRightDist, scannedLeftDist, TURN_DISTANCE); 
            
            bool rightOk = (scannedRightDist > TURN_DISTANCE);
            bool leftOk  = (scannedLeftDist  > TURN_DISTANCE);
            // RA QUYẾT ĐỊNH HƯỚNG ĐI cũ 
            // if (scannedRightDist > TURN_DISTANCE && scannedRightDist >= scannedLeftDist) {
            //     turnRight = true;
            //     currentState = TURNING; // Rẽ phải
            // } else if (scannedLeftDist > TURN_DISTANCE) {
            //     turnRight = false;
            //     currentState = TURNING; // Rẽ trái
            // } else {
            //     // Nếu cả 2 bên đều vướng, kiểm tra phía sau để lùi
            //     currentState = (backDist > BACK_DANGER_DISTANCE) ? BACKING : STOP;
            // }
            // currentScanPhase = SCAN_IDLE; // Reset phase cho lần sau
            // stateStartTime = now;

             if (rightOk && scannedRightDist >= scannedLeftDist) {
                turnRight    = true;
                currentState = TURNING;
                Serial.println("[SCAN] Quyet dinh: QUAY PHAI");
            }
            else if (leftOk) {
                turnRight    = false;
                currentState = TURNING;
                Serial.println("[SCAN] Quyet dinh: QUAY TRAI");
            }
            else if (backDist > BACK_DANGER_DISTANCE) {
                currentState = BACKING;
                Serial.println("[SCAN] Quyet dinh: LUI LAI");
            }
            else {
                currentState = STOP;
                Serial.println("[SCAN] Quyet dinh: BI KET - DUNG");
            }

            currentScanPhase = SCAN_IDLE; // [FIX BUG-5] Reset cho lần sau
            stateStartTime   = now;
            break;
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
    MotorController::setUSSensorServoAngle(90);
    
    // Lùi đủ thời gian?
    if (now - stateStartTime >= BACK_TIME) {
        // Sau khi lùi xong, quét lại
        currentState = TURN;
        currentScanPhase = SCAN_IDLE;
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
    MotorController::setUSSensorServoAngle(turnRight ? 45 : 135);
    
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

// ════════════════════════════════════════════════════════════════════════════
// [FIX BUG-6] Thêm nhánh SLOW cho vùng trung gian STOP_DISTANCE..TURN_DISTANCE
// Code cũ không xử lý vùng này → robot chạy thẳng vào vật cản
// ════════════════════════════════════════════════════════════════════════════

// ========== 6️⃣ RESUMING ==========
void VehicleStateMachine::handleResumingState(long frontDist, unsigned long now) {
    MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
    MotorController::setTargetSpeed(CRUISE_SPEED);
    
    // Giữ một chút rồi về NORMAL
    // if (now - stateStartTime >= RESUME_TIME) {
    //     currentState = (frontDist > TURN_DISTANCE) ? NORMAL : TURN;
    //     // Nếu trong khoảng STOP_DISTANCE..TURN_DISTANCE, tiếp tục RESUMING
    // }
    if (now - stateStartTime >= RESUME_TIME) {
        if (frontDist > TURN_DISTANCE) {
            currentState = NORMAL;
            Serial.println(">>> RESUMING -> NORMAL");
        }
        else if (frontDist > SLOW_DISTANCE) {
            currentState = SLOW;  // [FIX BUG-6]
            Serial.println(">>> RESUMING -> SLOW");
        }
        else {
            // Vẫn còn vật cản gần → quét lại
            currentState     = TURN;
            currentScanPhase = SCAN_IDLE;
            stateStartTime   = now;
            Serial.println(">>> RESUMING -> TURN (van con vat can)");
        }
    }
}

// ========== 7️⃣ STOP ==========
void VehicleStateMachine::handleStopState(long frontDist) {
    MotorController::setTargetSpeed(STOP_SPEED);
    MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
    currentScanPhase = SCAN_IDLE;
    
    // Nếu đường trước rộng lại → về NORMAL
    if (frontDist > TURN_DISTANCE) {
        currentState = NORMAL;
        Serial.println(">>> STOP -> NORMAL (duong rong)");
    }
        currentScanPhase = SCAN_IDLE; // Reset cho lần sau
    }

// void VehicleStateMachine::handleScanningProcess() {
//     unsigned long now = millis();

//     switch (currentScanPhase) {
//         case SCAN_IDLE:
//             scannedRightDist = 0;
//             scannedLeftDist = 0;
//             MotorController::setUSSensorServoAngle(US_SCAN_RIGHT);
//             lastScanStepTime = now;
//             currentScanPhase = SCAN_MOVING_RIGHT;
//             break;

//         case SCAN_MOVING_RIGHT:
//             if (now - lastScanStepTime > SERVO_SETTLE_TIME) {
//                 scannedRightDist = UltrasonicSensor::getFrontDistance();
//                 MotorController::setUSSensorServoAngle(US_SCAN_LEFT);
//                 lastScanStepTime = now;
//                 currentScanPhase = SCAN_MOVING_LEFT;
//             }
//             break;

//         case SCAN_MOVING_LEFT:
//             if (now - lastScanStepTime > SERVO_SETTLE_TIME) {
//                 scannedLeftDist = UltrasonicSensor::getFrontDistance();
//                 MotorController::setUSSensorServoAngle(US_SCAN_CENTER);
//                 lastScanStepTime = now;
//                 currentScanPhase = SCAN_RETURNING_CENTER;
//             }
//             break;

//         case SCAN_RETURNING_CENTER:
//             if (now - lastScanStepTime > SERVO_SETTLE_TIME) {
//                 currentScanPhase = SCAN_COMPLETED;
//             }
//             break;
//     }
// }

// // ========== DEBUG OUTPUT ==========
// void VehicleStateMachine::debugOutput() {
//     if (millis() - lastDebugTime > 500) {
//         long frontDist = UltrasonicSensor::getFrontDistance();
//         long backDist = UltrasonicSensor::getBackDistance();
        
//         const char* stateNames[] = {"INIT", "NORMAL", "SLOW", "TURN", "STOP", "BACKING", "TURNING", "RESUMING"};
        
//         Serial.print("[");
//         Serial.print(stateNames[currentState]);
//         Serial.print("] F:");
//         Serial.print(frontDist);
//         Serial.print(" B:");
//         Serial.print(backDist);
//         Serial.print(" | Spd:");
//         Serial.print(MotorController::getCurrentSpeed());
//         Serial.print(" L:");
//         Serial.print(MotorController::getLeftMotorSpeed());
//         Serial.print(" R:");
//         Serial.print(MotorController::getRightMotorSpeed());
//         Serial.print(" | Steer:");
//         Serial.print(MotorController::getSteerServoAngle());
//         Serial.print(" US:");
//         Serial.print(MotorController::getUSSensorServoAngle());
//         Serial.print(" [");
//         Serial.print(turnRight ? "R" : "L");
//         Serial.print("]");
        
//         if (scannedRightDist > 0 || scannedLeftDist > 0) {
//             Serial.print(" | Scan R:");
//             Serial.print(scannedRightDist);
//             Serial.print(" L:");
//             Serial.print(scannedLeftDist);
//         }
        
//         Serial.println("");
        
//         lastDebugTime = millis();
//     }
// }

void VehicleStateMachine::debugOutput() {
    if (millis() - lastDebugTime < 500) return;
    lastDebugTime = millis();

    long frontDist = UltrasonicSensor::getFrontDistance();
    long backDist  = UltrasonicSensor::getBackDistance();

    const char* stateNames[] = {
        "INIT", "NORMAL", "SLOW", "TURN", "STOP",
        "BACKING", "TURNING", "RESUMING", "MANUAL"
    };
    const char* scanNames[] = {
        "IDLE", "MOV_R", "READ_R", "MOV_L", "READ_L", "RET_C", "DONE"
    };

    Serial.printf("[%s|%s] F:%ld B:%ld | Spd:%d L:%d R:%d | Steer:%d US:%d [%s]",
        stateNames[currentState],
        scanNames[currentScanPhase],
        frontDist, backDist,
        MotorController::getCurrentSpeed(),
        MotorController::getLeftMotorSpeed(),
        MotorController::getRightMotorSpeed(),
        MotorController::getSteerServoAngle(),
        MotorController::getUSSensorServoAngle(),
        turnRight ? "R" : "L"
    );

    if (scannedRightDist > 0 || scannedLeftDist > 0) {
        Serial.printf(" | Scan R:%ld L:%ld", scannedRightDist, scannedLeftDist);
    }
    Serial.println();
}