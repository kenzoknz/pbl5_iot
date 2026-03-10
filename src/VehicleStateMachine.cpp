#include "VehicleStateMachine.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include "EncoderSensor.h"
#include <Arduino.h>

// Static member definitions
State VehicleStateMachine::currentState = INIT;
unsigned long VehicleStateMachine::stateStartTime = 0;
unsigned long VehicleStateMachine::lastDebugTime = 0;
bool VehicleStateMachine::turnRight = true;

// Anti-Trap
int   VehicleStateMachine::oscillationCount    = 0;
int   VehicleStateMachine::backingCount         = 0;
State VehicleStateMachine::lastTurnDirection    = NORMAL;
unsigned long VehicleStateMachine::trapStartTime  = 0;
unsigned long VehicleStateMachine::lastNormalTime = 0;
bool  VehicleStateMachine::escapeMode           = false;

// ══════════════════════════════════════════════════════════════
// TÍNH TỐC ĐỘ THÔNG MINH DỰA TRÊN CẢM BIẾN + ENCODER
//
// tính tốc độ phù hợp dựa trên:
//   1. Khoảng cách phía trước (càng gần → càng chậm)
//   2. Khoảng cách 2 bên (hẹp → chậm)
//   3. Tốc độ thực tế từ encoder (đang nhanh quá → giảm)
// ══════════════════════════════════════════════════════════════
static int calculateAdaptiveSpeed(long front, long right, long left) {
    // ── 1. Tốc độ base theo khoảng cách phía trước ──
    int baseSpeed;
    if (front > PREPARE_DISTANCE) {
        baseSpeed = CRUISE_SPEED;  // Đường rộng
    } else if (front > TURN_DISTANCE) {
        // Nội suy tuyến tính: PREPARE → TURN distance
        // CRUISE_SPEED → MIN_RUN_SPEED
        float ratio = (float)(front - TURN_DISTANCE)
                    / (float)(PREPARE_DISTANCE - TURN_DISTANCE);
        baseSpeed = MIN_RUN_SPEED + (int)(ratio * (CRUISE_SPEED - MIN_RUN_SPEED));
    } else {
        baseSpeed = MIN_RUN_SPEED;
    }

    // ── 2. Giảm tốc nếu 2 bên hẹp (hành lang chật) ──
    int minSide = min(right, left);
    if (minSide < SIDE_DANGER_DIST) {
        // Hành lang hẹp → giảm thêm 20%
        baseSpeed = baseSpeed * 80 / 100;
        if (baseSpeed < MIN_RUN_SPEED) baseSpeed = MIN_RUN_SPEED;
    }

    return baseSpeed;
}

// ══════════════════════════════════════════════════════════════
// 1. CHỌN HƯỚNG TỐI ƯU — TẬP TRUNG 1 HÀM DUY NHẤT
//
// Quy tắc ưu tiên (Priority-based + Quantitative):
//   P1: Front rộng → NORMAL/SLOW
//   P2: So sánh Left vs Right (có hysteresis) → TURN_LEFT/TURN_RIGHT
//   P3: Cả hai hẹp → BACKING
//   P4: Sau bị chặn → STOP
// ══════════════════════════════════════════════════════════════
State VehicleStateMachine::decideDirection(long front, long right, long left, long back) {
    // ── P1: Phía trước rộng → tiếp tục đi ──
    if (front > TURN_DISTANCE) {
        if (front > PREPARE_DISTANCE) return NORMAL;
        return SLOW;
    }

    // ── P2: So sánh Left vs Right với hysteresis ──
    // Hysteresis tránh dao động khi Left ≈ Right
    long diff = left - right;  // dương = trái rộng hơn

    if (diff > DIRECTION_HYSTERESIS && left > SIDE_DANGER_DIST) {
        // Trái rộng hơn rõ ràng
        return TURN_LEFT;
    }
    if (diff < -DIRECTION_HYSTERESIS && right > SIDE_DANGER_DIST) {
        // Phải rộng hơn rõ ràng
        return TURN_RIGHT;
    }

    // Chênh lệch nhỏ (|diff| <= HYSTERESIS) — chọn bên nào rộng hơn
    if (left > SIDE_DANGER_DIST && right > SIDE_DANGER_DIST) {
        // Cả hai đều đủ rộng → chọn bên xa hơn, ưu tiên hướng cũ nếu bằng nhau
        if (left >= right) return TURN_LEFT;
        return TURN_RIGHT;
    }
    if (left > SIDE_DANGER_DIST) return TURN_LEFT;
    if (right > SIDE_DANGER_DIST) return TURN_RIGHT;

    // ── P3: Cả hai bên đều hẹp → LÙI ──
    if (back > BACK_DANGER_DISTANCE) {
        return BACKING;
    }

    // ── P4: Bị chặn hoàn toàn ──
    return STOP;
}

// ══════════════════════════════════════════════════════════════
// 2. ANTI-TRAP DETECTION — Phát hiện robot bị kẹt
// ══════════════════════════════════════════════════════════════

// Gọi mỗi khi chuyển state để theo dõi oscillation
void VehicleStateMachine::updateTrapDetection(State newState) {
    // ── Đếm oscillation: LEFT ↔ RIGHT ──
    if ((newState == TURN_LEFT && lastTurnDirection == TURN_RIGHT) ||
        (newState == TURN_RIGHT && lastTurnDirection == TURN_LEFT)) {
        oscillationCount++;
        Serial.printf("[TRAP] Oscillation #%d (L↔R)\n", oscillationCount);
    }

    // Cập nhật hướng rẽ cuối
    if (newState == TURN_LEFT || newState == TURN_RIGHT) {
        lastTurnDirection = newState;
    }

    // ── Đếm BACKING liên tục ──
    if (newState == BACKING) {
        backingCount++;
        Serial.printf("[TRAP] Backing #%d\n", backingCount);
    }

    // ── Bắt đầu đếm thời gian trap nếu chưa ──
    if (trapStartTime == 0 && newState != NORMAL && newState != SLOW) {
        trapStartTime = millis();
    }
}

bool VehicleStateMachine::shouldEscape() {
    // Điều kiện 1: Oscillation quá nhiều (góc chết chữ V)
    if (oscillationCount >= MAX_OSCILLATIONS) {
        Serial.printf("[TRAP] ESCAPE: %d oscillations >= %d\n",
                      oscillationCount, MAX_OSCILLATIONS);
        return true;
    }

    // Điều kiện 2: Lùi quá nhiều lần (tiến-lùi không thoát)
    if (backingCount >= MAX_BACKING_RETRIES) {
        Serial.printf("[TRAP] ESCAPE: %d backings >= %d\n",
                      backingCount, MAX_BACKING_RETRIES);
        return true;
    }

    // Điều kiện 3: Quá lâu không về NORMAL (kẹt tổng thể)
    if (trapStartTime > 0 && (millis() - trapStartTime) >= TRAP_TIMEOUT_MS) {
        Serial.printf("[TRAP] ESCAPE: %lums without NORMAL >= %lu\n",
                      millis() - trapStartTime, TRAP_TIMEOUT_MS);
        return true;
    }

    return false;
}

void VehicleStateMachine::resetTrapCounters() {
    oscillationCount = 0;
    backingCount     = 0;
    lastTurnDirection = NORMAL;
    trapStartTime    = 0;
    escapeMode       = false;
}

// Quyết định hướng xoay khi ESCAPE
State VehicleStateMachine::decideEscapeDirection(long right, long left, long back) {
    // Ưu tiên: bên nào rộng hơn → xoay về bên đó
    // Nếu bằng nhau → xoay ngược hướng rẽ cuối (phá vòng lặp)
    if (left > right + DIRECTION_HYSTERESIS) return TURN_LEFT;
    if (right > left + DIRECTION_HYSTERESIS) return TURN_RIGHT;

    // Bằng nhau → ngược hướng cũ
    if (lastTurnDirection == TURN_RIGHT) return TURN_LEFT;
    return TURN_RIGHT;
}



void VehicleStateMachine::begin() {
    // Khởi tạo random seed
    randomSeed(analogRead(0) + micros()); 
    currentState = NORMAL;
    lastNormalTime = millis();
    resetTrapCounters();
    Serial.println("[FSM] State Machine initialized: NORMAL");
}

void VehicleStateMachine::update() {
    // 1. AN TOÀN KHẨN CẤP — MPU6050 (Thread-safe)
    if (MPUSensor::checkCollision() || MPUSensor::checkTilt()) {
        if (currentState != EMERGENCY) {
            MotorController::stopMotor();
            currentState = EMERGENCY;
            stateStartTime = millis();
            Serial.println("[SAFETY] EMERGENCY STOP");
        }
        return;
    }

    // Phát hiện xe bị kẹt bằng encoder ──
    if (EncoderSensor::isStalled() && currentState != STOP
        && currentState != EMERGENCY && currentState != BACKING
        && currentState != ESCAPE ) {
        Serial.println("[ENCODER] XE BI KET! Motor chay nhung banh khong quay");
        MotorController::stopMotor();
        // Thử lùi thay vì dừng hẳn
        long back = UltrasonicSensor::getBackDistance();
        if (back > BACK_DANGER_DISTANCE) {
            currentState = BACKING;
            stateStartTime = millis();
             EncoderSensor::resetDistance();  // [FIX] Reset trước khi lùi
            updateTrapDetection(BACKING);
            Serial.println("[FSM] STALL->BACKING");
        } else {
            currentState = STOP;
            Serial.println("[FSM] STALL->STOP");
        }
        return;
    }
    
    // 2. ĐỌC 4 CẢM BIẾN (thread-safe, non-blocking)
    long front = UltrasonicSensor::getFrontDistance();
    long right = UltrasonicSensor::getRightDistance();
    long left  = UltrasonicSensor::getLeftDistance();
    long back  = UltrasonicSensor::getBackDistance();
    unsigned long now = millis();

    // ══════ TRAP DETECTION — Kiểm tra trước khi xử lý state ══════
    // [FIX] Chỉ cho phép ESCAPE nếu dữ liệu cảm biến hợp lệ (> 0)
    if (currentState != ESCAPE && currentState != EMERGENCY && shouldEscape()) {
        // Validate sensor data trước khi vào ESCAPE
        if (front > 0 && right > 0 && left > 0 && back > 0) {
            MotorController::stopMotor();
            escapeMode = true;
            currentState = ESCAPE;
            stateStartTime = now;
            // Reset encoder để đo quãng đường xoay
            EncoderSensor::resetDistance();
            Serial.println("[FSM] >>> ENTERING ESCAPE MODE <<<");
            // Không return — để handleEscapeState xử lý ngay
        } else {
            Serial.println("[FSM] ⚠ ESCAPE postponed: waiting for valid sensor data");
            // Reset trap counters để không spam check
            resetTrapCounters();
        }
    }

    // 3. DỪNG KHẨN CẤP NẾU PHÍA TRƯỚC QUÁ GẦN
    if (front <= EMERGENCY_DIST && currentState != BACKING &&
        currentState != STOP && currentState != EMERGENCY &&
        currentState != ESCAPE ) {
        MotorController::stopMotor();
        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = now;
        Serial.printf("[FSM] EMERGENCY FRONT=%ld -> %d\n", front, currentState);
        return;
    }
    
    // 4. XỬ LÝ STATE MACHINE
    switch (currentState) {
        case NORMAL:      handleNormalState(front, right, left);                break;
        case SLOW:        handleSlowState(front, right, left);                  break;
        case AVOID_LEFT:  handleAvoidLeftState(front, right, left);             break;
        case AVOID_RIGHT: handleAvoidRightState(front, right, left);            break;
        case TURN_LEFT:   handleTurnLeftState(front, right, left, back, now);   break;
        case TURN_RIGHT:  handleTurnRightState(front, right, left, back, now);  break;
        case BACKING:     handleBackingState(back, now);                        break;
        case STOP:        handleStopState(front, right, left, back);            break;
        case EMERGENCY:   handleEmergencyState(front);                          break;
        case ESCAPE:      handleEscapeState(front, right, left, back, now);     break;
        default:          currentState = NORMAL; break;
    }

    // 6. OUTPUT PIPELINE (thứ tự quan trọng!)
    //    a. Servo lái → smooth transition
    //    b. Limit speed theo góc lái
    //    c. PID đọc encoder → tính pidOutput (KHÔNG sửa targetSpeed)
    //    d. Smooth speed → regulatedSpeed = currentSpeed + pidOutput → motor
    MotorController::smoothSteerServoTransition();
    MotorController::limitSpeedBySteering();
    MotorController::updatePID();
    MotorController::smoothSpeedTransition();
}

// ══════════════════════════════════════════════════════════════
// STATE HANDLERS — Dùng decideDirection() 
// ══════════════════════════════════════════════════════════════

// void VehicleStateMachine::handleNormalState(long front, long right, long left) {
//     // Tốc độ adaptive thay vì cố định
//     int speed = calculateAdaptiveSpeed(front, right, left);
//     MotorController::setTargetSpeed(speed);
//     MotorController::setTargetSteerServoAngle(90);

//     // Cảm biến bên
//     if (right < SIDE_DANGER_DIST && left >= SIDE_DANGER_DIST) {
//         currentState = AVOID_LEFT;
//         Serial.printf("[FSM] NORMAL->AVOID_LEFT (R=%ld)\n", right);
//         return;
//     }
//     if (left < SIDE_DANGER_DIST && right >= SIDE_DANGER_DIST) {
//         currentState = AVOID_RIGHT;
//         Serial.printf("[FSM] NORMAL->AVOID_RIGHT (L=%ld)\n", left);
//         return;
//     }

//     // Phía trước
//     if (front <= SLOW_DISTANCE) {
//         if (right > TURN_DISTANCE && right >= left) {
//             currentState = TURN_RIGHT;
//             stateStartTime = millis();
//             Serial.println("[FSM] NORMAL->TURN_RIGHT");
//         } else if (left > TURN_DISTANCE) {
//             currentState = TURN_LEFT;
//             stateStartTime = millis();
//             Serial.println("[FSM] NORMAL->TURN_LEFT");
//         } else {
//             currentState = SLOW;
//             Serial.println("[FSM] NORMAL->SLOW");
//         }
//     }
//     else if (front <= PREPARE_DISTANCE) {
//         currentState = SLOW;
//         Serial.println("[FSM] NORMAL->SLOW");
//     }
// }

// void VehicleStateMachine::handleSlowState(long front, long right, long left) {
//     int speed = calculateAdaptiveSpeed(front, right, left);
//     MotorController::setTargetSpeed(speed);
//     MotorController::setTargetSteerServoAngle(90);

//     if (front > PREPARE_DISTANCE && right > SIDE_DANGER_DIST && left > SIDE_DANGER_DIST) {
//         currentState = NORMAL;
//         Serial.println("[FSM] SLOW->NORMAL");
//         return;
//     }

//     if (front <= STOP_DISTANCE) {
//         if (right > TURN_DISTANCE && right >= left) {
//             currentState = TURN_RIGHT;
//             stateStartTime = millis();
//             Serial.printf("[FSM] SLOW->TURN_RIGHT (R=%ld L=%ld)\n", right, left);
//         } else if (left > TURN_DISTANCE) {
//             currentState = TURN_LEFT;
//             stateStartTime = millis();
//             Serial.printf("[FSM] SLOW->TURN_LEFT (R=%ld L=%ld)\n", right, left);
//         } else {
//             currentState = BACKING;
//             stateStartTime = millis();
//             Serial.println("[FSM] SLOW->BACKING");
//         }
//         return;
//     }

//     if (right < SIDE_DANGER_DIST) {
//         currentState = AVOID_LEFT;
//         Serial.println("[FSM] SLOW->AVOID_LEFT");
//     } else if (left < SIDE_DANGER_DIST) {
//         currentState = AVOID_RIGHT;
//         Serial.println("[FSM] SLOW->AVOID_RIGHT");
//     }
// }

// void VehicleStateMachine::handleAvoidLeftState(long front, long right, long left) {
//     int speed = calculateAdaptiveSpeed(front, right, left);
//     MotorController::setTargetSpeed(speed);
//     MotorController::setTargetSteerServoAngle(110);

//     if (right > TURN_DISTANCE) {
//         if (front > PREPARE_DISTANCE) {
//             currentState = NORMAL;
//             Serial.println("[FSM] AVOID_LEFT->NORMAL");
//         } else {
//             currentState = SLOW;
//             Serial.println("[FSM] AVOID_LEFT->SLOW");
//         }
//         return;
//     }

//     if (front <= STOP_DISTANCE) {
//         if (left > TURN_DISTANCE) {
//             currentState = TURN_LEFT;
//             stateStartTime = millis();
//             Serial.println("[FSM] AVOID_LEFT->TURN_LEFT");
//         } else {
//             currentState = BACKING;
//             stateStartTime = millis();
//             Serial.println("[FSM] AVOID_LEFT->BACKING");
//         }
//         return;
//     }

//     if (left < SIDE_DANGER_DIST) {
//         if (front > SLOW_DISTANCE) {
//             MotorController::setTargetSteerServoAngle(90);
//             currentState = SLOW;
//         } else {
//             currentState = BACKING;
//             stateStartTime = millis();
//         }
//     }
// }

// void VehicleStateMachine::handleAvoidRightState(long front, long right, long left) {
//     int speed = calculateAdaptiveSpeed(front, right, left);
//     MotorController::setTargetSpeed(speed);
//     MotorController::setTargetSteerServoAngle(70);

//     if (left > TURN_DISTANCE) {
//         if (front > PREPARE_DISTANCE) {
//             currentState = NORMAL;
//             Serial.println("[FSM] AVOID_RIGHT->NORMAL");
//         } else {
//             currentState = SLOW;
//             Serial.println("[FSM] AVOID_RIGHT->SLOW");
//         }
//         return;
//     }

//     if (front <= STOP_DISTANCE) {
//         if (right > TURN_DISTANCE) {
//             currentState = TURN_RIGHT;
//             stateStartTime = millis();
//             Serial.println("[FSM] AVOID_RIGHT->TURN_RIGHT");
//         } else {
//             currentState = BACKING;
//             stateStartTime = millis();
//             Serial.println("[FSM] AVOID_RIGHT->BACKING");
//         }
//         return;
//     }

//     if (right < SIDE_DANGER_DIST) {
//         if (front > SLOW_DISTANCE) {
//             MotorController::setTargetSteerServoAngle(90);
//             currentState = SLOW;
//         } else {
//             currentState = BACKING;
//             stateStartTime = millis();
//         }
//     }
// }

// void VehicleStateMachine::handleTurnLeftState(long front, long right, long left, long back, unsigned long now) {
//     MotorController::setTargetSpeed(TURN_BOOST);
//     MotorController::setTargetSteerServoAngle(135);
//     turnRight = false;

//     if (front > TURN_DISTANCE && left > SIDE_DANGER_DIST) {
//         currentState = NORMAL;
//         Serial.println("[FSM] TURN_LEFT->NORMAL");
//         return;
//     }

//     if (now - stateStartTime >= TURN_TIME) {
//         if (front > SLOW_DISTANCE) {
//             currentState = SLOW;
//             Serial.println("[FSM] TURN_LEFT->SLOW");
//         } else if (right > TURN_DISTANCE) {
//             currentState = TURN_RIGHT;
//             stateStartTime = now;
//             Serial.println("[FSM] TURN_LEFT->TURN_RIGHT");
//         } else if (back > BACK_DANGER_DISTANCE) {
//             currentState = BACKING;
//             stateStartTime = now;
//             Serial.println("[FSM] TURN_LEFT->BACKING");
//         } else {
//             currentState = STOP;
//             Serial.println("[FSM] TURN_LEFT->STOP");
//         }
//     }
// }

// void VehicleStateMachine::handleTurnRightState(long front, long right, long left, long back, unsigned long now) {
//     MotorController::setTargetSpeed(TURN_BOOST);
//     MotorController::setTargetSteerServoAngle(45);
//     turnRight = true;

//     if (front > TURN_DISTANCE && right > SIDE_DANGER_DIST) {
//         currentState = NORMAL;
//         Serial.println("[FSM] TURN_RIGHT->NORMAL");
//         return;
//     }

//     if (now - stateStartTime >= TURN_TIME) {
//         if (front > SLOW_DISTANCE) {
//             currentState = SLOW;
//             Serial.println("[FSM] TURN_RIGHT->SLOW");
//         } else if (left > TURN_DISTANCE) {
//             currentState = TURN_LEFT;
//             stateStartTime = now;
//             Serial.println("[FSM] TURN_RIGHT->TURN_LEFT");
//         } else if (back > BACK_DANGER_DISTANCE) {
//             currentState = BACKING;
//             stateStartTime = now;
//             Serial.println("[FSM] TURN_RIGHT->BACKING");
//         } else {
//             currentState = STOP;
//             Serial.println("[FSM] TURN_RIGHT->STOP");
//         }
//     }
// }

// void VehicleStateMachine::handleBackingState(long back, unsigned long now) {
//     if (back <= BACK_DANGER_DISTANCE) {
//         MotorController::stopMotor();
//         currentState = STOP;
//         Serial.println("[FSM] BACKING->STOP (sau bi chan)");
//         return;
//     }

//     MotorController::setTargetSpeed(-BACK_SPEED);
//     MotorController::setTargetSteerServoAngle(90);

//     // Dùng encoder để xác nhận đã lùi đủ xa thay vì chỉ đếm thời gian
//     // Nếu encoder hoạt động: lùi ít nhất 15cm HOẶC hết BACK_TIME
//     float distSinceBack = EncoderSensor::getDistanceCm();

//     if (now - stateStartTime >= BACK_TIME) {
//         long front = UltrasonicSensor::getFrontDistance();
//         long right = UltrasonicSensor::getRightDistance();
//         long left  = UltrasonicSensor::getLeftDistance();

//         if (right > TURN_DISTANCE && right >= left) {
//             currentState = TURN_RIGHT;
//             Serial.println("[FSM] BACKING->TURN_RIGHT");
//         } else if (left > TURN_DISTANCE) {
//             currentState = TURN_LEFT;
//             Serial.println("[FSM] BACKING->TURN_LEFT");
//         } else if (front > SLOW_DISTANCE) {
//             currentState = SLOW;
//             Serial.println("[FSM] BACKING->SLOW");
//         } else {
//             currentState = STOP;
//             Serial.println("[FSM] BACKING->STOP");
//         }
//         stateStartTime = now;
//     }
// }

// void VehicleStateMachine::handleStopState(long front, long right, long left, long back) {
//     MotorController::setTargetSpeed(STOP_SPEED);
//     MotorController::setTargetSteerServoAngle(90);

//     if (front > TURN_DISTANCE) {
//         currentState = NORMAL;
//         Serial.println("[FSM] STOP->NORMAL");
//     } else if (right > TURN_DISTANCE) {
//         currentState = TURN_RIGHT;
//         stateStartTime = millis();
//         Serial.println("[FSM] STOP->TURN_RIGHT");
//     } else if (left > TURN_DISTANCE) {
//         currentState = TURN_LEFT;
//         stateStartTime = millis();
//         Serial.println("[FSM] STOP->TURN_LEFT");
//     } else if (back > BACK_DANGER_DISTANCE) {
//         currentState = BACKING;
//         stateStartTime = millis();
//         Serial.println("[FSM] STOP->BACKING");
//     }
// }

// void VehicleStateMachine::handleEmergencyState(long front) {
//     MotorController::stopMotor();

//     if (millis() - stateStartTime >= 3000) {
//         if (!MPUSensor::checkTilt() && !MPUSensor::checkCollision()) {
//             currentState = STOP;
//             stateStartTime = millis();
//             Serial.println("[FSM] EMERGENCY->STOP (phuc hoi)");
//         }
//     }
// }

void VehicleStateMachine::handleNormalState(long front, long right, long left) {
    // ── Về NORMAL → reset trap counters ──
    lastNormalTime = millis();
    resetTrapCounters();

    int speed = calculateAdaptiveSpeed(front, right, left);
    MotorController::setTargetSpeed(speed);
    MotorController::setTargetSteerServoAngle(90);

    // Cảm biến bên — né nhẹ
    if (right < SIDE_DANGER_DIST && left >= SIDE_DANGER_DIST) {
        currentState = AVOID_LEFT;
        Serial.printf("[FSM] NORMAL->AVOID_LEFT (R=%ld)\n", right);
        return;
    }
    if (left < SIDE_DANGER_DIST && right >= SIDE_DANGER_DIST) {
        currentState = AVOID_RIGHT;
        Serial.printf("[FSM] NORMAL->AVOID_RIGHT (L=%ld)\n", left);
        return;
    }

    // Phía trước có vật cản
    if (front <= SLOW_DISTANCE) {
        long back = UltrasonicSensor::getBackDistance();
        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = millis();
        Serial.printf("[FSM] NORMAL->%d (F=%ld R=%ld L=%ld)\n",
                      nextState, front, right, left);
    }
    else if (front <= PREPARE_DISTANCE) {
        currentState = SLOW;
    }
}

void VehicleStateMachine::handleSlowState(long front, long right, long left) {
    int speed = calculateAdaptiveSpeed(front, right, left);
    MotorController::setTargetSpeed(speed);
    MotorController::setTargetSteerServoAngle(90);

    if (front > PREPARE_DISTANCE && right > SIDE_DANGER_DIST && left > SIDE_DANGER_DIST) {
        currentState = NORMAL;
        Serial.println("[FSM] SLOW->NORMAL");
        return;
    }

    if (front <= STOP_DISTANCE) {
        long back = UltrasonicSensor::getBackDistance();
        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = millis();
        Serial.printf("[FSM] SLOW->%d (F=%ld R=%ld L=%ld)\n",
                      nextState, front, right, left);
        return;
    }

    // Cảm biến bên
    if (right < SIDE_DANGER_DIST) {
        currentState = AVOID_LEFT;
    } else if (left < SIDE_DANGER_DIST) {
        currentState = AVOID_RIGHT;
    }
}

void VehicleStateMachine::handleAvoidLeftState(long front, long right, long left) {
    int speed = calculateAdaptiveSpeed(front, right, left);
    MotorController::setTargetSpeed(speed);
    MotorController::setTargetSteerServoAngle(110);

    if (right > TURN_DISTANCE) {
        currentState = (front > PREPARE_DISTANCE) ? NORMAL : SLOW;
        Serial.printf("[FSM] AVOID_LEFT->%d\n", currentState);
        return;
    }

    if (front <= STOP_DISTANCE) {
        long back = UltrasonicSensor::getBackDistance();
        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = millis();
        return;
    }

    if (left < SIDE_DANGER_DIST) {
        if (front > SLOW_DISTANCE) {
            MotorController::setTargetSteerServoAngle(90);
            currentState = SLOW;
        } else {
            long back = UltrasonicSensor::getBackDistance();
            State nextState = decideDirection(front, right, left, back);
            updateTrapDetection(nextState);
            currentState = nextState;
            stateStartTime = millis();
        }
    }
}

void VehicleStateMachine::handleAvoidRightState(long front, long right, long left) {
    int speed = calculateAdaptiveSpeed(front, right, left);
    MotorController::setTargetSpeed(speed);
    MotorController::setTargetSteerServoAngle(70);

    if (left > TURN_DISTANCE) {
        currentState = (front > PREPARE_DISTANCE) ? NORMAL : SLOW;
        Serial.printf("[FSM] AVOID_RIGHT->%d\n", currentState);
        return;
    }

    if (front <= STOP_DISTANCE) {
        long back = UltrasonicSensor::getBackDistance();
        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = millis();
        return;
    }

    if (right < SIDE_DANGER_DIST) {
        if (front > SLOW_DISTANCE) {
            MotorController::setTargetSteerServoAngle(90);
            currentState = SLOW;
        } else {
            long back = UltrasonicSensor::getBackDistance();
            State nextState = decideDirection(front, right, left, back);
            updateTrapDetection(nextState);
            currentState = nextState;
            stateStartTime = millis();
        }
    }
}

void VehicleStateMachine::handleTurnLeftState(long front, long right, long left, long back, unsigned long now) {
    MotorController::setTargetSpeed(TURN_BOOST);
    MotorController::setTargetSteerServoAngle(135);
    turnRight = false;

    // Thoát thành công
    if (front > TURN_DISTANCE && left > SIDE_DANGER_DIST) {
        currentState = NORMAL;
        Serial.println("[FSM] TURN_LEFT->NORMAL");
        return;
    }

    // Hết thời gian rẽ
    if (now - stateStartTime >= TURN_TIME) {
        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = now;
        Serial.printf("[FSM] TURN_LEFT timeout->%d\n", nextState);
    }
}

void VehicleStateMachine::handleTurnRightState(long front, long right, long left, long back, unsigned long now) {
    MotorController::setTargetSpeed(TURN_BOOST);
    MotorController::setTargetSteerServoAngle(45);
    turnRight = true;

    if (front > TURN_DISTANCE && right > SIDE_DANGER_DIST) {
        currentState = NORMAL;
        Serial.println("[FSM] TURN_RIGHT->NORMAL");
        return;
    }

    if (now - stateStartTime >= TURN_TIME) {
        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = now;
        Serial.printf("[FSM] TURN_RIGHT timeout->%d\n", nextState);
    }
}

void VehicleStateMachine::handleBackingState(long back, unsigned long now) {
    if (back <= BACK_DANGER_DISTANCE) {
        MotorController::stopMotor();
        currentState = STOP;
        Serial.println("[FSM] BACKING->STOP (sau bi chan)");
        return;
    }

    MotorController::setTargetSpeed(-BACK_SPEED);
    // [MỚI] Khi lùi, bẻ nhẹ servo để thay đổi góc tiếp cận
    // Ngược hướng rẽ cuối → tránh lùi vào cùng chỗ cũ
    if (lastTurnDirection == TURN_RIGHT) {
        MotorController::setTargetSteerServoAngle(110); // Lùi + bẻ trái nhẹ
    } else if (lastTurnDirection == TURN_LEFT) {
        MotorController::setTargetSteerServoAngle(70);  // Lùi + bẻ phải nhẹ
    } else {
        MotorController::setTargetSteerServoAngle(90);  // Lùi thẳng
    }

    if (now - stateStartTime >= BACK_TIME) {
        long front = UltrasonicSensor::getFrontDistance();
        long right = UltrasonicSensor::getRightDistance();
        long left  = UltrasonicSensor::getLeftDistance();

        State nextState = decideDirection(front, right, left, back);
        updateTrapDetection(nextState);
        currentState = nextState;
        stateStartTime = now;
        Serial.printf("[FSM] BACKING->%d\n", nextState);
    }
}

// ══════════════════════════════════════════════════════════════
// 2.1 ESCAPE STATE — Thoát góc chết
//
// Chiến thuật: LÙI + XOAY GÓC LỚN liên tục cho đến khi tìm
// được đường thoát. Giống U-turn nhưng aggressive hơn.
//
// Phase 1: Lùi 1.5s (nếu phía sau rộng)
// Phase 2: Xoay mạnh về hướng rộng nhất (servo 40° hoặc 140°)
//          với tốc độ cao (ESCAPE_SPEED)
// Phase 3: Kiểm tra liên tục — nếu front > TURN_DISTANCE → thoát
// ══════════════════════════════════════════════════════════════
void VehicleStateMachine::handleEscapeState(long front, long right, long left, long back, unsigned long now) {
    unsigned long elapsed = now - stateStartTime;

    // ── Thoát thành công? ──
    if (front > TURN_DISTANCE &&
        (left > SIDE_DANGER_DIST || right > SIDE_DANGER_DIST)) {
        Serial.println("[FSM] ESCAPE SUCCESS -> NORMAL");
        currentState = NORMAL;
        resetTrapCounters();
        return;
    }

    // ── Phase 1: LÙI (0 → 1500ms) ──
    if (elapsed < PHASE_ESCAPE_MS) {
        if (back > BACK_DANGER_DISTANCE) {
            MotorController::setTargetSpeed(-BACK_SPEED);
            // Lùi + bẻ lái ngược hướng cũ
            State escapeDir = decideEscapeDirection(right, left, back);
            if (escapeDir == TURN_LEFT) {
                MotorController::setTargetSteerServoAngle(120);
            } else {
                MotorController::setTargetSteerServoAngle(60);
            }
        } else {
            // Không lùi được → nhảy sang phase 2 ngay
            stateStartTime = now - PHASE_ESCAPE_MS;
        }
        return;
    }

    // ── Phase 2: XOAY MẠNH (1500ms → ESCAPE_TURN_TIME_MS + 1500ms) ──
    if (elapsed < PHASE_ESCAPE_MS + ESCAPE_TURN_TIME_MS) {
        State escapeDir = decideEscapeDirection(right, left, back);
        MotorController::setTargetSpeed(ESCAPE_SPEED);

        if (escapeDir == TURN_LEFT) {
            MotorController::setTargetSteerServoAngle(120); // Xoay trái tối đa
        } else {
            MotorController::setTargetSteerServoAngle(60);  // Xoay phải tối đa
        }
        return;
    }

    // ── Phase 3: Hết thời gian escape ──
    // Đánh giá lại tình hình
    MotorController::stopMotor();

    if (front > SLOW_DISTANCE) {
        currentState = SLOW;
        resetTrapCounters();
        Serial.println("[FSM] ESCAPE->SLOW");
    } else if (back > BACK_DANGER_DISTANCE) {
        // Thử lùi thêm 1 lần
        currentState = BACKING;
        stateStartTime = now;
        EncoderSensor::resetDistance();
        // KHÔNG reset trap counters — nếu vẫn kẹt sẽ vào ESCAPE lần nữa
        Serial.println("[FSM] ESCAPE->BACKING (retry)");
    } else {
        currentState = STOP;
        resetTrapCounters(); // Reset để khi sensor clear sẽ chạy lại
        Serial.println("[FSM] ESCAPE->STOP (bi chan hoan toan)");
    }
}

void VehicleStateMachine::handleStopState(long front, long right, long left, long back) {
    MotorController::setTargetSpeed(STOP_SPEED);
    MotorController::setTargetSteerServoAngle(90);

    // Dùng decideDirection để tự tìm lối thoát
    State nextState = decideDirection(front, right, left, back);
    if (nextState != STOP) {
        currentState = nextState;
        stateStartTime = millis();
        Serial.printf("[FSM] STOP->%d\n", nextState);
    }
}

void VehicleStateMachine::handleEmergencyState(long front) {
    MotorController::stopMotor();

    if (millis() - stateStartTime >= 3000) {
        if (!MPUSensor::checkTilt() && !MPUSensor::checkCollision()) {
            currentState = STOP;
            stateStartTime = millis();
            resetTrapCounters();
            Serial.println("[FSM] EMERGENCY->STOP (phuc hoi)");
        }
    }
}

// ══════════════════════════════════════════════════════════════
// DEBUG OUTPUT
// ══════════════════════════════════════════════════════════════
void VehicleStateMachine::debugOutput() {
    if (millis() - lastDebugTime < 500) return;
    lastDebugTime = millis();

    long front = UltrasonicSensor::getFrontDistance();
    long right = UltrasonicSensor::getRightDistance();
    long left  = UltrasonicSensor::getLeftDistance();
    long back  = UltrasonicSensor::getBackDistance();

    const char* stateNames[] = {
        "INIT", "NORMAL", "SLOW", "AVOID_L", "AVOID_R",
        "TURN_L", "TURN_R", "BACKING", "STOP", "EMERG", "MANUAL", "ESCAPE"
    };

    Serial.printf("[%s] F:%ld R:%ld L:%ld B:%ld | tgt:%d cur:%d reg:%d | ML:%d MR:%d | Steer:%d\n",
        stateNames[currentState],
        front, right, left, back,
        MotorController::getTargetSpeed(),
        MotorController::getCurrentSpeed(),
        MotorController::getRegulatedSpeed(),
        MotorController::getLeftMotorSpeed(),
        MotorController::getRightMotorSpeed(),
        MotorController::getSteerServoAngle()
    );

    Serial.printf("  ENC RPM:%.0f v:%.1fcm/s d:%.0fcm dir:%d PID:%d %s\n",
        EncoderSensor::getRPM(),
        EncoderSensor::getSpeedCmPerSec(),
        EncoderSensor::getDistanceCm(),
        EncoderSensor::getDirection(),
        MotorController::getPIDOutput(),
        EncoderSensor::isStalled() ? "STALL!" : "OK"
    );

    // [MỚI] Anti-trap info
    if (oscillationCount > 0 || backingCount > 0) {
        Serial.printf("  TRAP osc:%d back:%d escape:%s\n",
            oscillationCount, backingCount,
            escapeMode ? "YES" : "no"
        );
    }
}