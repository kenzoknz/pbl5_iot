#include "MotorController.h"
#include "MPUSensor.h"
#include <Arduino.h>

// Static member definitions
Servo MotorController::myServo;
int MotorController::currentSpeed = 0;
int MotorController::targetSpeed = 0;
int MotorController::servoAngle = 90;
int MotorController::targetServoAngle = 90;
int MotorController::leftMotorSpeed = 0;
int MotorController::rightMotorSpeed = 0;

void MotorController::begin() {
    // Setup PWM for motor driver
    ledcSetup(PWM_CHANNEL_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RPWM, PWM_CHANNEL_RPWM);
    ledcAttachPin(LPWM, PWM_CHANNEL_LPWM);

    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);
    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);

    // Setup servo
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    myServo.setPeriodHertz(50);
    myServo.attach(SERVO_PIN, 500, 2400);
    delay(500);
    myServo.write(90);
}

void MotorController::setServoAngle(int angle) {
    servoAngle = constrain(angle, 40, 140);
    myServo.write(servoAngle);
}

void MotorController::smoothServoTransition() {
    // Tăng step để servo phản ứng nhanh hơn khi cua gắt
    int step = 8;  // Tăng từ 5 lên 8
    if (servoAngle < targetServoAngle) servoAngle += step;
    else if (servoAngle > targetServoAngle) servoAngle -= step;

    servoAngle = constrain(servoAngle, 40, 140);  // Giới hạn an toàn hơn
    myServo.write(servoAngle);
}

void MotorController::limitSpeedBySteering() {
    int deviation = abs(servoAngle - 90);
    
    // TĂNG tốc độ khi cua gắt để thắng ma sát
    // Góc 100-130° (deviation 10-40°) cần tốc độ cao
    if (deviation > 35) {  // Cua RẤT GẮT (góc > 125° hoặc < 55°)
        if (targetSpeed > 0 && targetSpeed < SHARP_TURN_BOOST) {
            targetSpeed = SHARP_TURN_BOOST;  // 160 - Tốc độ max để kéo nổi
        }
    } 
    else if (deviation > 25) {  // Cua GẮT (góc 115-125° hoặc 55-65°)
        if (targetSpeed > 0 && targetSpeed < 150) {
            targetSpeed = 150;  // Tốc độ cao
        }
    }
    else if (deviation > 15) {  // Cua VỪA (góc 105-115° hoặc 65-75°)
        if (targetSpeed > 0 && targetSpeed < TURN_BOOST) {
            targetSpeed = TURN_BOOST;  // 145
        }
    }
    else if (deviation > 10) {  // Cua NHẸ (góc 100-105° hoặc 75-80°)
        if (targetSpeed > 0 && targetSpeed < 135) {
            targetSpeed = 135;
        }
    }
    // Nếu cua < 10° thì giữ nguyên targetSpeed (chạy thẳng)
}

void MotorController::smoothSpeedTransition() {
    int step = 18;  // Tăng từ 12 lên 18 để tăng tốc nhanh hơn khi cua

    if (targetSpeed > 0 && targetSpeed < MIN_RUN_SPEED)
        targetSpeed = MIN_RUN_SPEED;
    if (targetSpeed < 0 && targetSpeed > -MIN_RUN_SPEED)
        targetSpeed = -MIN_RUN_SPEED;

    if (currentSpeed < targetSpeed) currentSpeed += step;
    else if (currentSpeed > targetSpeed) currentSpeed -= step;

    // Tính tốc độ differential steering
    calculateDifferentialSteering(currentSpeed);
    
    // Điều khiển motor theo differential steering
    if (currentSpeed != 0) {
        moveDifferential(leftMotorSpeed, rightMotorSpeed);
    } else {
        stopMotor();
    }
}

void MotorController::calculateDifferentialSteering(int baseSpeed) {
    if (baseSpeed == 0) {
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        return;
    }
    
    // Tính độ lệch từ góc servo (90° = thẳng)
    int servoDeviation = servoAngle - 90;
    
    // === BÙ TRỪ GÓC NGHIÊNG NGANG ===
    float currentAngleX = MPUSensor::getCurrentAngleX();
    float currentAngleY = MPUSensor::getCurrentAngleY();
    float currentAccelY = MPUSensor::getCurrentAccelY();
    
    float tiltCompensation = currentAngleX * 0.3;  // Mỗi 1° nghiêng → 0.3% chênh lệch
    
    // === ĐIỀU CHỈNH THEO GIA TỐC LY TÂM ===
    float centrifugalFactor = 1.0;
    if (abs(currentAccelY) > 0.5) {
        centrifugalFactor = 0.7;  // Giảm 30% độ chênh lệch khi gia tốc lớn
    }
    
    // === ĐIỀU CHỈNH THEO GÓC DỐC ===
    float slopeCompensation = 0;
    if (currentAngleY < -10) {  // Xuống dốc
        slopeCompensation = abs(currentAngleY + 10) * 0.5;
    } else if (currentAngleY > 10) {  // Lên dốc
        slopeCompensation = -abs(currentAngleY - 10) * 0.3;
    }
    
    // Tính tỷ lệ chênh lệch tốc độ (0-100%)
    float steeringRatio = (servoDeviation / 60.0) * centrifugalFactor;
    steeringRatio += (tiltCompensation / 100.0);  // Thêm bù trừ nghiêng
    steeringRatio += (slopeCompensation / 100.0); // Thêm bù trừ dốc
    steeringRatio = constrain(steeringRatio, -1.0, 1.0);
    
    // Tính tốc độ từng motor
    if (baseSpeed > 0) {  // Tiến
        if (steeringRatio > 0) {  // Rẽ trái (servo > 90°)
            leftMotorSpeed = baseSpeed * (1.0 - abs(steeringRatio));
            rightMotorSpeed = baseSpeed;
        } else {  // Rẽ phải (servo < 90°)
            leftMotorSpeed = baseSpeed;
            rightMotorSpeed = baseSpeed * (1.0 - abs(steeringRatio));
        }
    } else {  // Lùi (đảo chiều steering)
        int absSpeed = abs(baseSpeed);
        if (steeringRatio > 0) {  // Lùi + rẽ trái
            leftMotorSpeed = -absSpeed;
            rightMotorSpeed = -absSpeed * (1.0 - abs(steeringRatio));
        } else {  // Lùi + rẽ phải
            leftMotorSpeed = -absSpeed * (1.0 - abs(steeringRatio));
            rightMotorSpeed = -absSpeed;
        }
    }
    
    // Giới hạn tốc độ trong phạm vi PWM
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
}

void MotorController::moveDifferential(int leftSpeed, int rightSpeed) {
    // Dùng MAX speed thay vì AVG để không mất lực khi cua gắt
    int maxSpeed = max(leftSpeed, rightSpeed);
    int minSpeed = min(leftSpeed, rightSpeed);
    
    // Xác định hướng dựa trên tốc độ lớn nhất (có dấu)
    if (maxSpeed > 0 && minSpeed >= 0) {
        // Tiến: Dùng tốc độ MAX để đảm bảo đủ lực
        int pwmValue = constrain(maxSpeed, 0, 255);
        ledcWrite(PWM_CHANNEL_RPWM, pwmValue);
        ledcWrite(PWM_CHANNEL_LPWM, 0);
    } 
    else if (minSpeed < 0 && maxSpeed <= 0) {
        // Lùi: Dùng tốc độ MIN (âm) để đảm bảo đủ lực
        int pwmValue = constrain(abs(minSpeed), 0, 255);
        ledcWrite(PWM_CHANNEL_RPWM, 0);
        ledcWrite(PWM_CHANNEL_LPWM, pwmValue);
    } 
    else {
        // Trường hợp đặc biệt: 1 bánh tiến 1 bánh lùi (xoay tại chỗ)
        int pwmValue = constrain(max(abs(maxSpeed), abs(minSpeed)), 0, 255);
        if (abs(maxSpeed) > abs(minSpeed)) {
            ledcWrite(PWM_CHANNEL_RPWM, pwmValue);
            ledcWrite(PWM_CHANNEL_LPWM, 0);
        } else {
            ledcWrite(PWM_CHANNEL_RPWM, 0);
            ledcWrite(PWM_CHANNEL_LPWM, pwmValue);
        }
    }
}

void MotorController::moveForward(int pwm) {
    ledcWrite(PWM_CHANNEL_RPWM, pwm);
    ledcWrite(PWM_CHANNEL_LPWM, 0);
}

void MotorController::moveBackward(int pwm) {
    ledcWrite(PWM_CHANNEL_RPWM, 0);
    ledcWrite(PWM_CHANNEL_LPWM, pwm);
}

void MotorController::stopMotor() {
    ledcWrite(PWM_CHANNEL_RPWM, 0);
    ledcWrite(PWM_CHANNEL_LPWM, 0);
}