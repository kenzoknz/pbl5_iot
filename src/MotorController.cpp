#include "MotorController.h"
#include "MPUSensor.h"
#include <Arduino.h>

// Định nghĩa Mutex để bảo vệ dữ liệu Motor (Thread-safety giữa Core 0 và Core 1)
SemaphoreHandle_t motorMutex;

// Static member definitions
Servo MotorController::steerServo;           // Servo lái bánh xe
int MotorController::currentSpeed = 0;
int MotorController::targetSpeed = 0;
int MotorController::steerServoAngle = 90;
int MotorController::targetSteerServoAngle = 90;
int MotorController::leftMotorSpeed = 0;
int MotorController::rightMotorSpeed = 0;

// Biến lưu trạng thái phần cứng cuối cùng (Tối ưu giảm tải Hardware Bus)
// Cache phần cứng -> chỉ ghi khi thay đổi
static int lastLeftPWM    = -1;
static int lastRightPWM   = -1;
static int lastSteerAngle = -1;

void MotorController::begin() {
    motorMutex = xSemaphoreCreateMutex();

    // PWM cho BTS7960 motor driver
    ledcSetup(PWM_CHANNEL_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RPWM, PWM_CHANNEL_RPWM);
    ledcAttachPin(LPWM, PWM_CHANNEL_LPWM);

    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);
    digitalWrite(REN, HIGH);
    digitalWrite(LEN, HIGH);

    // servo lái
    ESP32PWM::allocateTimer(2);
    steerServo.setPeriodHertz(50);
    steerServo.attach(SERVO_STEER_PIN, 500, 2400);
    steerServo.write(90);
    
    Serial.println("[MOTOR] Initialized: BTS7960 + Steering Servo");
}

// ========== SERVO LÁI ==========
void MotorController::smoothSteerServoTransition() {
    xSemaphoreTake(motorMutex, portMAX_DELAY);

    // Tăng step để servo phản ứng nhanh hơn khi cua gắt
    int step = 8;  // Tăng từ 5 lên 8
    if (steerServoAngle < targetSteerServoAngle) steerServoAngle += step;
    else if (steerServoAngle > targetSteerServoAngle) steerServoAngle -= step;

    steerServoAngle = constrain(steerServoAngle, 40, 140);  // Giới hạn an toàn hơn
    
    if (steerServoAngle != lastSteerAngle) {
        steerServo.write(steerServoAngle);
        lastSteerAngle = steerServoAngle;
    }
    xSemaphoreGive(motorMutex);
}

// ========== SPEED LIMITING BY STEERING ==========
void MotorController::limitSpeedBySteering() {
    int deviation = abs(steerServoAngle - 90);
    
    // TĂNG tốc độ khi cua gắt để thắng ma sát
    // Góc 100-130° (deviation 10-40°) cần tốc độ cao
    if (deviation > 35) {  // Cua RẤT GẮT (góc > 125° hoặc < 55°)
        if (targetSpeed > 0 && targetSpeed < SHARP_TURN_BOOST) {
            targetSpeed = SHARP_TURN_BOOST;  // 160 - Tốc độ max để kéo nổi
        }
    } 
    else if (deviation > 25) {  // Cua GẮT (góc 115-125° hoặc 55-65°)
        if (targetSpeed > 0 && targetSpeed < 150) {
            targetSpeed = MEDIUM_TURN_BOOST;  // Tốc độ cao
        }
    }
    else if (deviation > 15) {  // Cua VỪA (góc 105-115° hoặc 65-75°)
        if (targetSpeed > 0 && targetSpeed < TURN_BOOST) {
            targetSpeed = TURN_BOOST;  // 145
        }
    }
    else if (deviation > 10) {  // Cua NHẸ (góc 100-105° hoặc 75-80°)
        if (targetSpeed > 0 && targetSpeed < 135) {
            targetSpeed = LIGHT_TURN_BOOST;
        }
    }
    // Nếu cua < 10° thì giữ nguyên targetSpeed (chạy thẳng)

}

// ========== SMOOTH SPEED ==========
void MotorController::smoothSpeedTransition() {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    int step = 18;  // 12 -> 18: tăng tốc nhanh hơn khi cua

    if (targetSpeed > 0 && targetSpeed < MIN_RUN_SPEED)
        targetSpeed = MIN_RUN_SPEED;
    if (targetSpeed < 0 && targetSpeed > -MIN_RUN_SPEED)
        targetSpeed = -MIN_RUN_SPEED;

    if (targetSpeed == 0) {
        currentSpeed = 0;
    } else { 
        if (currentSpeed < targetSpeed) currentSpeed += step;
        else if (currentSpeed > targetSpeed) currentSpeed -= step;
    }

    // Tính tốc độ differential steering
    calculateDifferentialSteering(currentSpeed);
    
    // Điều khiển motor theo differential steering
    if (currentSpeed != 0) {
        moveDifferential(leftMotorSpeed, rightMotorSpeed);
    } else {
        stopMotor();
    }
    xSemaphoreGive(motorMutex);
}

// ========== DIFFERENTIAL STEERING ==========
void MotorController::calculateDifferentialSteering(int baseSpeed) {
    if (baseSpeed == 0) {
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        return;
    }
    
    // Tính độ lệch từ góc servo lái (90° = thẳng)
    int servoDeviation = steerServoAngle - 90;
    
    // Bù trừ góc nghiêng từ MPU6050
    float currentAngleX = MPUSensor::getCurrentAngleX();
    float currentAngleY = MPUSensor::getCurrentAngleY();
    float currentAccelY = MPUSensor::getCurrentAccelY();
    
    float tiltCompensation = currentAngleX * 0.3;  // Mỗi 1 độ nghiêng -> 0.3% chênh lệch
    
    
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
    // float steeringRatio = (servoDeviation / 60.0) * centrifugalFactor;
    float steeringRatio = constrain(servoDeviation / 60.0, -1.0, 1.0);
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

// ========== MOTOR OUTPUT ==========
void MotorController::moveDifferential(int leftSpeed, int rightSpeed) {
    // Dùng logic Max Speed để thắng ma sát khi cua
    int pwmValue;
    int currentLeftPWM, currentRightPWM;

    if (leftSpeed >= 0 && rightSpeed >= 0) { // Tiến hoặc rẽ tiến
        pwmValue = constrain(max(leftSpeed, rightSpeed), 0, 255);
        currentLeftPWM = pwmValue;
        currentRightPWM = 0; // Hướng tiến dùng RPWM
    } 
    else if (leftSpeed <= 0 && rightSpeed <= 0) { // Lùi cũ <=
        pwmValue = constrain(max(abs(leftSpeed), abs(rightSpeed)), 0, 255);
        currentLeftPWM = 0;
        currentRightPWM = pwmValue; // Hướng lùi dùng LPWM
    }
    else { // Xoay tại chỗ (Bánh tiến bánh lùi)
        pwmValue = constrain(max(abs(leftSpeed), abs(rightSpeed)), 0, 255);
        // Ưu tiên hướng bánh có tốc độ tuyệt đối lớn hơn
        if (abs(leftSpeed) > abs(rightSpeed)) {
            currentLeftPWM = pwmValue; currentRightPWM = 0;
        } else {
            currentLeftPWM = 0; currentRightPWM = pwmValue;
        }
    }

    // TỐI ƯU: Chỉ ghi vào thanh ghi PWM nếu giá trị thực sự thay đổi
    if (currentLeftPWM != lastLeftPWM) {
        ledcWrite(PWM_CHANNEL_RPWM, currentLeftPWM);
        lastLeftPWM = currentLeftPWM;
    }
    if (currentRightPWM != lastRightPWM) {
        ledcWrite(PWM_CHANNEL_LPWM, currentRightPWM);
        lastRightPWM = currentRightPWM;
    }
}

void MotorController::stopMotor() {
    if (lastLeftPWM != 0 || lastRightPWM != 0) {
        ledcWrite(PWM_CHANNEL_RPWM, 0);
        ledcWrite(PWM_CHANNEL_LPWM, 0);
        lastLeftPWM = 0;
        lastRightPWM = 0;
        currentSpeed = 0;
        targetSpeed = 0;
    }
}