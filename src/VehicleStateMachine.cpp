#include "VehicleStateMachine.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include <Arduino.h>

// Static member definitions
State VehicleStateMachine::currentState = INIT;
unsigned long VehicleStateMachine::stateStartTime = 0;
unsigned long VehicleStateMachine::lastDebugTime = 0;
bool VehicleStateMachine::turnRight = true;

void VehicleStateMachine::begin() {
    // Khởi tạo random seed
    randomSeed(analogRead(0) + micros()); 
    currentState = NORMAL;
    Serial.println("[FSM] State Machine initialized: NORMAL");
}

void VehicleStateMachine::update() {
    // 1. ƯU TIÊN CAO NHẤT: Kiểm tra an toàn khẩn cấp (Thread-safe)
    if (MPUSensor::checkCollision() || MPUSensor::checkTilt()) {
        if (currentState != EMERGENCY) {
            MotorController::stopMotor();
            currentState = EMERGENCY;
            stateStartTime = millis();
            Serial.println("[SAFETY] EMERGENCY STOP");
        }
        return;
    }
    
    // 2. ĐỌC 4 CẢM BIẾN (thread-safe, non-blocking)
    long front = UltrasonicSensor::getFrontDistance();
    long right = UltrasonicSensor::getRightDistance();
    long left  = UltrasonicSensor::getLeftDistance();
    long back  = UltrasonicSensor::getBackDistance();
    unsigned long now = millis();

    // 3. DỪNG KHẨN CẤP NẾU PHÍA TRƯỚC QUÁ GẦN
    if (front <= EMERGENCY_DIST && currentState != BACKING &&
        currentState != STOP && currentState != EMERGENCY) {
        MotorController::stopMotor();
        // Quyết định nhanh: rẽ hay lùi?
        if (right > TURN_DISTANCE) {
            currentState = TURN_RIGHT;
        } else if (left > TURN_DISTANCE) {
            currentState = TURN_LEFT;
        } else if (back > BACK_DANGER_DISTANCE) {
            currentState = BACKING;
        } else {
            currentState = STOP;
        }
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
        default:          currentState = NORMAL; break;
    }

    // 5. CẬP NHẬT OUTPUT
    MotorController::smoothSteerServoTransition();
    MotorController::limitSpeedBySteering();
    MotorController::smoothSpeedTransition();
}

// ════════════════════════════════════════════════════════════════════════════
// STATE HANDLERS
// ════════════════════════════════════════════════════════════════════════════
// ── NORMAL: Đường rộng, chạy thẳng tốc độ cruise ──
void VehicleStateMachine::handleNormalState(long front, long right, long left) {
    MotorController::setTargetSpeed(CRUISE_SPEED);
    MotorController::setTargetSteerServoAngle(90); // Thẳng

    // Kiểm tra cảm biến bên — né nhẹ nếu 1 bên hẹp
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

    // Kiểm tra phía trước
    if (front <= SLOW_DISTANCE) {
        // Gần — cần quyết định rẽ
        if (right > TURN_DISTANCE && right >= left) {
            currentState = TURN_RIGHT;
            stateStartTime = millis();
            Serial.println("[FSM] NORMAL->TURN_RIGHT");
        } else if (left > TURN_DISTANCE) {
            currentState = TURN_LEFT;
            stateStartTime = millis();
            Serial.println("[FSM] NORMAL->TURN_LEFT");
        } else {
            currentState = SLOW;
            Serial.println("[FSM] NORMAL->SLOW");
        }
    }
    else if (front <= PREPARE_DISTANCE) {
        currentState = SLOW;
        Serial.println("[FSM] NORMAL->SLOW");
    }
}

// ── SLOW: Giảm tốc, theo dõi cả 3 phía trước ──
void VehicleStateMachine::handleSlowState(long front, long right, long left) {
    MotorController::setTargetSpeed(MIN_RUN_SPEED);
    MotorController::setTargetSteerServoAngle(90);

    // Đường rộng lại → NORMAL
    if (front > PREPARE_DISTANCE && right > SIDE_DANGER_DIST && left > SIDE_DANGER_DIST) {
        currentState = NORMAL;
        Serial.println("[FSM] SLOW->NORMAL");
        return;
    }

    // Phía trước bị chặn → quyết định rẽ
    if (front <= STOP_DISTANCE) {
        if (right > TURN_DISTANCE && right >= left) {
            currentState = TURN_RIGHT;
            stateStartTime = millis();
            Serial.printf("[FSM] SLOW->TURN_RIGHT (R=%ld L=%ld)\n", right, left);
        } else if (left > TURN_DISTANCE) {
            currentState = TURN_LEFT;
            stateStartTime = millis();
            Serial.printf("[FSM] SLOW->TURN_LEFT (R=%ld L=%ld)\n", right, left);
        } else {
            // Cả 3 phía trước đều hẹp
            currentState = BACKING;
            stateStartTime = millis();
            Serial.println("[FSM] SLOW->BACKING (3 phia truoc bi chan)");
        }
        return;
    }

    // Né bên nếu cần
    if (right < SIDE_DANGER_DIST) {
        currentState = AVOID_LEFT;
        Serial.println("[FSM] SLOW->AVOID_LEFT");
    } else if (left < SIDE_DANGER_DIST) {
        currentState = AVOID_RIGHT;
        Serial.println("[FSM] SLOW->AVOID_RIGHT");
    }
}

// ── AVOID_LEFT: Né nhẹ sang trái (vật cản bên phải) ──
void VehicleStateMachine::handleAvoidLeftState(long front, long right, long left) {
    // Giữ tốc độ cruise, lái nhẹ sang trái
    MotorController::setTargetSpeed(CRUISE_SPEED);
    MotorController::setTargetSteerServoAngle(110); // Lái trái nhẹ

    // Đã né xong (bên phải rộng lại)
    if (right > TURN_DISTANCE) {
        if (front > PREPARE_DISTANCE) {
            currentState = NORMAL;
            Serial.println("[FSM] AVOID_LEFT->NORMAL");
        } else {
            currentState = SLOW;
            Serial.println("[FSM] AVOID_LEFT->SLOW");
        }
        return;
    }

    // Phía trước cũng hẹp → rẽ mạnh
    if (front <= STOP_DISTANCE) {
        if (left > TURN_DISTANCE) {
            currentState = TURN_LEFT;
            stateStartTime = millis();
            Serial.println("[FSM] AVOID_LEFT->TURN_LEFT");
        } else {
            currentState = BACKING;
            stateStartTime = millis();
            Serial.println("[FSM] AVOID_LEFT->BACKING");
        }
    }

    // Bên trái cũng hẹp → cả 2 bên đều hẹp
    if (left < SIDE_DANGER_DIST) {
        if (front > SLOW_DISTANCE) {
            // Chỉ có phía trước → chạy thẳng
            MotorController::setTargetSteerServoAngle(90);
            currentState = SLOW;
        } else {
            currentState = BACKING;
            stateStartTime = millis();
        }
    }
}

// ── AVOID_RIGHT: Né nhẹ sang phải (vật cản bên trái) ──
void VehicleStateMachine::handleAvoidRightState(long front, long right, long left) {
    MotorController::setTargetSpeed(CRUISE_SPEED);
    MotorController::setTargetSteerServoAngle(70); // Lái phải nhẹ

    if (left > TURN_DISTANCE) {
        if (front > PREPARE_DISTANCE) {
            currentState = NORMAL;
            Serial.println("[FSM] AVOID_RIGHT->NORMAL");
        } else {
            currentState = SLOW;
            Serial.println("[FSM] AVOID_RIGHT->SLOW");
        }
        return;
    }

    if (front <= STOP_DISTANCE) {
        if (right > TURN_DISTANCE) {
            currentState = TURN_RIGHT;
            stateStartTime = millis();
            Serial.println("[FSM] AVOID_RIGHT->TURN_RIGHT");
        } else {
            currentState = BACKING;
            stateStartTime = millis();
            Serial.println("[FSM] AVOID_RIGHT->BACKING");
        }
    }

    if (right < SIDE_DANGER_DIST) {
        if (front > SLOW_DISTANCE) {
            MotorController::setTargetSteerServoAngle(90);
            currentState = SLOW;
        } else {
            currentState = BACKING;
            stateStartTime = millis();
        }
    }
}

// ── TURN_LEFT: Rẽ trái mạnh ──
void VehicleStateMachine::handleTurnLeftState(long front, long right, long left, long back, unsigned long now) {
    MotorController::setTargetSpeed(TURN_BOOST);
    MotorController::setTargetSteerServoAngle(135); // Rẽ trái mạnh
    turnRight = false;

    // Điều kiện thoát: phía trước rộng lại HOẶC hết thời gian rẽ
    if (front > TURN_DISTANCE && left > SIDE_DANGER_DIST) {
        currentState = NORMAL;
        Serial.println("[FSM] TURN_LEFT->NORMAL (duong rong)");
        return;
    }

    if (now - stateStartTime >= TURN_TIME) {
        if (front > SLOW_DISTANCE) {
            currentState = SLOW;
            Serial.println("[FSM] TURN_LEFT->SLOW (het thoi gian)");
        } else if (right > TURN_DISTANCE) {
            // Đổi hướng
            currentState = TURN_RIGHT;
            stateStartTime = now;
            Serial.println("[FSM] TURN_LEFT->TURN_RIGHT (doi huong)");
        } else if (back > BACK_DANGER_DISTANCE) {
            currentState = BACKING;
            stateStartTime = now;
            Serial.println("[FSM] TURN_LEFT->BACKING");
        } else {
            currentState = STOP;
            Serial.println("[FSM] TURN_LEFT->STOP");
        }
    }
}

// ── TURN_RIGHT: Rẽ phải mạnh ──
void VehicleStateMachine::handleTurnRightState(long front, long right, long left, long back, unsigned long now) {
    MotorController::setTargetSpeed(TURN_BOOST);
    MotorController::setTargetSteerServoAngle(45); // Rẽ phải mạnh
    turnRight = true;

    if (front > TURN_DISTANCE && right > SIDE_DANGER_DIST) {
        currentState = NORMAL;
        Serial.println("[FSM] TURN_RIGHT->NORMAL (duong rong)");
        return;
    }

    if (now - stateStartTime >= TURN_TIME) {
        if (front > SLOW_DISTANCE) {
            currentState = SLOW;
            Serial.println("[FSM] TURN_RIGHT->SLOW (het thoi gian)");
        } else if (left > TURN_DISTANCE) {
            currentState = TURN_LEFT;
            stateStartTime = now;
            Serial.println("[FSM] TURN_RIGHT->TURN_LEFT (doi huong)");
        } else if (back > BACK_DANGER_DISTANCE) {
            currentState = BACKING;
            stateStartTime = now;
            Serial.println("[FSM] TURN_RIGHT->BACKING");
        } else {
            currentState = STOP;
            Serial.println("[FSM] TURN_RIGHT->STOP");
        }
    }
}

// ── BACKING: Lùi ──
void VehicleStateMachine::handleBackingState(long back, unsigned long now) {
    // Kiểm tra an toàn phía sau
    if (back <= BACK_DANGER_DISTANCE) {
        MotorController::stopMotor();
        currentState = STOP;
        Serial.println("[FSM] BACKING->STOP (sau bi chan)");
        return;
    }

    MotorController::setTargetSpeed(-BACK_SPEED);
    MotorController::setTargetSteerServoAngle(90); // Lùi thẳng

    if (now - stateStartTime >= BACK_TIME) {
        // Lùi xong — đọc lại cảm biến và quyết định
        long front = UltrasonicSensor::getFrontDistance();
        long right = UltrasonicSensor::getRightDistance();
        long left  = UltrasonicSensor::getLeftDistance();

        if (right > TURN_DISTANCE && right >= left) {
            currentState = TURN_RIGHT;
            Serial.println("[FSM] BACKING->TURN_RIGHT");
        } else if (left > TURN_DISTANCE) {
            currentState = TURN_LEFT;
            Serial.println("[FSM] BACKING->TURN_LEFT");
        } else if (front > SLOW_DISTANCE) {
            currentState = SLOW;
            Serial.println("[FSM] BACKING->SLOW");
        } else {
            currentState = STOP;
            Serial.println("[FSM] BACKING->STOP (van bi ket)");
        }
        stateStartTime = now;
    }
}

// ── STOP: Bị kẹt hoàn toàn ──
void VehicleStateMachine::handleStopState(long front, long right, long left, long back) {
    MotorController::setTargetSpeed(STOP_SPEED);
    MotorController::setTargetSteerServoAngle(90);

    // Liên tục kiểm tra xem có lối thoát không
    if (front > TURN_DISTANCE) {
        currentState = NORMAL;
        Serial.println("[FSM] STOP->NORMAL");
    } else if (right > TURN_DISTANCE) {
        currentState = TURN_RIGHT;
        stateStartTime = millis();
        Serial.println("[FSM] STOP->TURN_RIGHT");
    } else if (left > TURN_DISTANCE) {
        currentState = TURN_LEFT;
        stateStartTime = millis();
        Serial.println("[FSM] STOP->TURN_LEFT");
    } else if (back > BACK_DANGER_DISTANCE) {
        currentState = BACKING;
        stateStartTime = millis();
        Serial.println("[FSM] STOP->BACKING");
    }
    // Nếu tất cả vẫn bị chặn → giữ STOP, chờ
}

// ── EMERGENCY: Va chạm / nghiêng ──
void VehicleStateMachine::handleEmergencyState(long front) {
    MotorController::stopMotor();

    // Chờ 3 giây rồi thử phục hồi
    if (millis() - stateStartTime >= 3000) {
        if (!MPUSensor::checkTilt() && !MPUSensor::checkCollision()) {
            currentState = STOP;  // Chuyển về STOP để re-evaluate
            stateStartTime = millis();
            Serial.println("[FSM] EMERGENCY->STOP (phuc hoi)");
        }
    }
}

// ════════════════════════════════════════════════════════════════════════════
// DEBUG OUTPUT
// ═══════════════════════════════════════════════════════════════════════════
void VehicleStateMachine::debugOutput() {
    if (millis() - lastDebugTime < 500) return;
    lastDebugTime = millis();

    long front = UltrasonicSensor::getFrontDistance();
    long right = UltrasonicSensor::getRightDistance();
    long left  = UltrasonicSensor::getLeftDistance();
    long back  = UltrasonicSensor::getBackDistance();

    const char* stateNames[] = {
        "INIT", "NORMAL", "SLOW", "AVOID_L", "AVOID_R",
        "TURN_L", "TURN_R", "BACKING", "STOP", "EMERG", "MANUAL"
    };

    Serial.printf("[%s] F:%ld R:%ld L:%ld B:%ld | Spd:%d ML:%d MR:%d | Steer:%d [%s]\n",
        stateNames[currentState],
        front, right, left, back,
        MotorController::getCurrentSpeed(),
        MotorController::getLeftMotorSpeed(),
        MotorController::getRightMotorSpeed(),
        MotorController::getSteerServoAngle(),
        turnRight ? "R" : "L"
    );
}