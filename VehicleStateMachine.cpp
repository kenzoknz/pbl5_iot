#include "VehicleStateMachine.h"
#include "UltrasonicSensor.h"
#include "MPUSensor.h"
#include "MotorController.h"
#include <Arduino.h>

// Static member definitions
State VehicleStateMachine::currentState = INIT;
unsigned long VehicleStateMachine::stateStartTime = 0;
bool VehicleStateMachine::turnRight = true;
unsigned long VehicleStateMachine::lastDebugTime = 0;

void VehicleStateMachine::begin() {
    // Khởi tạo random seed
    randomSeed(analogRead(0) + analogRead(4) + micros());
    currentState = NORMAL;
}

void VehicleStateMachine::update() {
    // Cập nhật dữ liệu cảm biến
    MPUSensor::update();
    
    // Kiểm tra va chạm và nghiêng
    if (MPUSensor::checkCollision() || MPUSensor::checkTilt()) {
        MotorController::setTargetSpeed(STOP_SPEED);
        delay(2000);
        return;
    }
    
    long frontDist = UltrasonicSensor::readFrontDistance();
    long backDist = UltrasonicSensor::readBackDistance();
    unsigned long now = millis();

    switch (currentState) {
        case NORMAL:
            handleNormalState(frontDist);
            break;
            
        case SLOW:
            handleSlowState(frontDist);
            break;
            
        case TURN:
            handleTurnState(frontDist);
            break;
            
        case STOP:
            handleStopState(backDist, now);
            break;
            
        case BACKING:
            handleBackingState(now);
            break;
            
        case TURNING:
            handleTurningState(now);
            break;
            
        case RESUMING:
            handleResumingState(frontDist);
            break;
            
        default:
            break;
    }

    // Cập nhật motor controller
    MotorController::smoothServoTransition();
    MotorController::limitSpeedBySteering();
    MotorController::smoothSpeedTransition();
}

void VehicleStateMachine::handleNormalState(long frontDist) {
    if (frontDist > PREPARE_DISTANCE) {
        MotorController::setTargetSpeed(FAST_SPEED);
        MotorController::setTargetServoAngle(90);
    } else {
        currentState = SLOW;
    }
}

void VehicleStateMachine::handleSlowState(long frontDist) {
    if (frontDist > TURN_DISTANCE) {
        MotorController::setTargetSpeed(CRUISE_SPEED);
        MotorController::setTargetServoAngle(75);
    } else {
        currentState = TURN;
    }
}

void VehicleStateMachine::handleTurnState(long frontDist) {
    if (frontDist > STOP_DISTANCE) {
        MotorController::setTargetSpeed(TURN_BOOST);  // 145 - Cần lực cao khi cua gấp
        MotorController::setTargetServoAngle(55);     // Cua gắt hơn
    } else {
        currentState = STOP;
    }
}

void VehicleStateMachine::handleStopState(long backDist, unsigned long now) {
    MotorController::setTargetSpeed(STOP_SPEED);
    MotorController::setTargetServoAngle(90);

    if (backDist > BACK_DANGER_DISTANCE) {
        // Chọn ngẫu nhiên hướng quay: 50% phải, 50% trái
        turnRight = random(0, 2) == 0;  // random(0,2) trả về 0 hoặc 1
        
        Serial.print(">>> Chon huong: ");
        Serial.println(turnRight ? "QUAY PHAI" : "QUAY TRAI");
        
        currentState = BACKING;
        stateStartTime = now;
    }
}

void VehicleStateMachine::handleBackingState(unsigned long now) {
    MotorController::setTargetSpeed(-BACK_SPEED);
    MotorController::setTargetServoAngle(90);

    if (now - stateStartTime > BACK_TIME) {
        currentState = TURNING;
        stateStartTime = now;
    }
}

void VehicleStateMachine::handleTurningState(unsigned long now) {
    // Cua gắt cần tốc độ CAO để thắng ma sát
    MotorController::setTargetSpeed(SHARP_TURN_BOOST);  // 160 - Đủ lực kéo
    
    // Góc servo 45/135 thay vì 30/150 (ổn định hơn)
    if (turnRight) {
        MotorController::setTargetServoAngle(45);   // Quay phải gắt
    } else {
        MotorController::setTargetServoAngle(135);  // Quay trái gắt
    }

    if (now - stateStartTime > TURN_TIME) {
        currentState = RESUMING;
    }
}

void VehicleStateMachine::handleResumingState(long frontDist) {
    MotorController::setTargetSpeed(TURN_BOOST);  // 145 - Vẫn cần lực cao khi còn cua
    
    // Giữ hướng quay để thoát vật cản (góc vừa phải)
    if (turnRight) {
        MotorController::setTargetServoAngle(65);   // Tiếp tục quay phải nhẹ
    } else {
        MotorController::setTargetServoAngle(115);  // Tiếp tục quay trái nhẹ
    }

    // Thoát RESUMING khi đường trước an toàn
    if (frontDist > PREPARE_DISTANCE) {
        currentState = NORMAL;
    }
    // Nếu vẫn gặp vật cản gần, quay lại STOP
    else if (frontDist < STOP_DISTANCE) {
        currentState = STOP;
    }
}

void VehicleStateMachine::debugOutput() {
    if (millis() - lastDebugTime > 500) {
        long frontDist = UltrasonicSensor::readFrontDistance();
        long backDist = UltrasonicSensor::readBackDistance();
        
        Serial.print("F:");
        Serial.print(frontDist);
        Serial.print(" B:");
        Serial.print(backDist);
        Serial.print(" | L:");
        Serial.print(MotorController::getLeftMotorSpeed());
        Serial.print(" R:");
        Serial.print(MotorController::getRightMotorSpeed());
        Serial.print(" | Servo:");
        Serial.print(MotorController::getServoAngle());
        Serial.print(" [");
        Serial.print(turnRight ? "R" : "L");
        Serial.print("] | Tilt:");
        Serial.print(MPUSensor::getCurrentAngleX(), 1);
        Serial.print("/");
        Serial.print(MPUSensor::getCurrentAngleY(), 1);
        Serial.print(" | Accel:");
        Serial.println(MPUSensor::getCurrentAccelY(), 2);
        
        lastDebugTime = millis();
    }
}