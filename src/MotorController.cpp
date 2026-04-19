#include "MotorController.h"
#include "MPUSensor.h"
#include "EncoderSensor.h"
#include <Arduino.h>

// Định nghĩa Mutex để bảo vệ dữ liệu Motor (Thread-safety giữa Core 0 và Core 1)
SemaphoreHandle_t motorMutex;

// Static member definitions
Servo MotorController::steerServo;           // Servo lái bánh xe
int MotorController::currentSpeed = 0;
int MotorController::targetSpeed = 0;
int MotorController::regulatedSpeed = 0;
int MotorController::steerServoAngle = 90;
int MotorController::targetSteerServoAngle = 90;
int MotorController::leftMotorSpeed = 0;
int MotorController::rightMotorSpeed = 0;
RobotConfig MotorController::runtimeConfig = ConfigStorage::getDefaults();

bool  MotorController::pidEnabled   = false;
float MotorController::pidIntegral  = 0;
float MotorController::pidLastError = 0;
int   MotorController::pidOutput    = 0;

// Biến lưu trạng thái phần cứng cuối cùng (Tối ưu giảm tải Hardware Bus)
// Cache phần cứng -> chỉ ghi khi thay đổi
static int lastLeftPWM    = -1;
static int lastRightPWM   = -1;
static int lastSteerAngle = -1;

//  PID Gains
// Tinh chỉnh trên xe thực tế:
//   Bước 1: Kp=0.5, Ki=0, Kd=0 → tăng Kp đến khi motor phản ứng đủ nhanh
//   Bước 2: Thêm Ki=0.05 → tăng dần nếu còn steady-state error
//   Bước 3: Thêm Kd=0.02 → tăng nếu overshoot
static const float PID_KP = 0.8f;    // Proportional
static const float PID_KI = 0.15f;   // Integral
static const float PID_KD = 0.05f;   // Derivative
static const float PID_MAX_INTEGRAL = 100.0f;  // Anti-windup
static const float PID_DT = 0.05f;   // 20Hz = 50ms = 0.05s

// ── Ánh xạ PWM → RPM ước lượng ──
// Dùng để PID biết target RPM tương ứng với PWM mà State Machine đặt
// ⚠️ ĐO THỰC TẾ: Cho motor chạy ở các mức PWM, đọc RPM từ encoder, điền vào
// Hiện tại dùng công thức tuyến tính ước lượng:
//   PWM 0    → RPM 0
//   PWM 150  → RPM ~60   (MIN_RUN_SPEED)
//   PWM 200  → RPM ~100  (TURN_BOOST)
//   PWM 255  → RPM ~130  (Max)
static float pwmToEstimatedRPM(int pwm) {
    if (abs(pwm) < 30) return 0;
    // Tuyến tính đơn giản: RPM ≈ PWM × 0.5 + offset
    // Tinh chỉnh bằng cách đo thực tế!
    return (float)abs(pwm) * 0.5f;
}

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
        if (targetSpeed > 0 && targetSpeed < runtimeConfig.sharpTurnBoost) {
            targetSpeed = runtimeConfig.sharpTurnBoost;
        }
    } 
    else if (deviation > 25) {  // Cua GẮT (góc 115-125° hoặc 55-65°)
        if (targetSpeed > 0 && targetSpeed < runtimeConfig.mediumTurnBoost) {
            targetSpeed = runtimeConfig.mediumTurnBoost;
        }
    }
    else if (deviation > 15) {  // Cua VỪA (góc 105-115° hoặc 65-75°)
        if (targetSpeed > 0 && targetSpeed < runtimeConfig.turnBoost) {
            targetSpeed = runtimeConfig.turnBoost;
        }
    }
    else if (deviation > 10) {  // Cua NHẸ (góc 100-105° hoặc 75-80°)
        if (targetSpeed > 0 && targetSpeed < runtimeConfig.lightTurnBoost) {
            targetSpeed = runtimeConfig.lightTurnBoost;
        }
    }
    // Nếu cua < 10° thì giữ nguyên targetSpeed (chạy thẳng)

}


// ═══════════════════════════════════════════════════════════════
// PID CONTROLLER — SỬA LỖI FEEDBACK LOOP
//
// Luồng đúng:
//   State Machine → targetSpeed (KHÔNG ĐỔI)
//                         ↓
//   PID so sánh: target RPM vs actual RPM → pidOutput (±50)
//                         ↓
//   smoothSpeedTransition: currentSpeed ramp → regulatedSpeed = currentSpeed + pidOutput
//                         ↓
//   calculateDifferentialSteering(regulatedSpeed)
//                         ↓
//   Motor output
//
// ĐẢM BẢO: targetSpeed KHÔNG BAO GIỜ bị PID ghi đè
// ═══════════════════════════════════════════════════════════════
void MotorController::enablePID(bool enable) {
    pidEnabled = enable;
    if (!enable) {
        pidIntegral  = 0;
        pidLastError = 0;
        pidOutput    = 0;
        regulatedSpeed = 0;
    }
    Serial.printf("[MOTOR] PID %s\n", enable ? "ENABLED" : "DISABLED");
}

void MotorController::updatePID() {
    if (!pidEnabled) {
        pidOutput = 0;
        return;
    }

    // 1. Tính target RPM từ currentSpeed (đã qua smooth transition)
    //    Dùng currentSpeed thay vì targetSpeed vì đây là tốc độ đang ramp
    float targetRPM = pwmToEstimatedRPM(currentSpeed);

    // 2. Đọc RPM thực tế từ encoder
    float actualRPM = fabsf(EncoderSensor::getRPM());

    // 3. Nếu dừng → reset PID, không tính
    if (targetRPM < 1.0f) {
        pidIntegral  = 0;
        pidLastError = 0;
        pidOutput    = 0;
        return;
    }

    // 4. Tính sai số
    float error = targetRPM - actualRPM;

    // 5. Integral với anti-windup
    pidIntegral += error * PID_DT;
    pidIntegral = constrain(pidIntegral, -PID_MAX_INTEGRAL, PID_MAX_INTEGRAL);

    // 6. Derivative
    float derivative = (error - pidLastError) / PID_DT;
    pidLastError = error;

    // 7. PID output — CHỈ LƯU, KHÔNG SỬA targetSpeed
    float output = (PID_KP * error) + (PID_KI * pidIntegral) + (PID_KD * derivative);
    pidOutput = (int)constrain(output, -50, 50);

    // pidOutput sẽ được dùng trong smoothSpeedTransition()
    // để tạo ra regulatedSpeed = currentSpeed + pidOutput

    #ifdef DEBUG_SENSOR
    static unsigned long lastPID = 0;
    if (millis() - lastPID > 500) {
        lastPID = millis();
        Serial.printf("[PID] tgtRPM:%.0f actRPM:%.0f err:%.1f out:%d | tgtPWM:%d curPWM:%d regPWM:%d\n",
                      targetRPM, actualRPM, error, pidOutput,
                      targetSpeed, currentSpeed, regulatedSpeed);
    }
    #endif
}


// ========== SMOOTH SPEED ==========
// Cấu trúc mới:
//   1. Ramp currentSpeed → targetSpeed (giữ nguyên logic cũ)
//   2. Tính regulatedSpeed = currentSpeed + pidOutput (BÙ TRỪ, không ghi đè)
//   3. Feed regulatedSpeed vào differential steering
void MotorController::smoothSpeedTransition() {
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    int step = 18;  // 12 -> 18: tăng tốc nhanh hơn khi cua

    if (targetSpeed > 0 && targetSpeed < runtimeConfig.minRunSpeed)
        targetSpeed = runtimeConfig.minRunSpeed;
    if (targetSpeed < 0 && targetSpeed > -(int)runtimeConfig.minRunSpeed)
        targetSpeed = -(int)runtimeConfig.minRunSpeed;

    if (targetSpeed == 0) {
        currentSpeed = 0;
    } else { 
        if (currentSpeed < targetSpeed) currentSpeed += step;
        else if (currentSpeed > targetSpeed) currentSpeed -= step;
        // Tránh overshoot qua target
        if (abs(currentSpeed - targetSpeed) < step) {
            currentSpeed = targetSpeed;
        }
    }

    // ── TẠO regulatedSpeed = currentSpeed + PID bù trừ ──
    // currentSpeed: tốc độ base (từ smooth ramp)
    // pidOutput: bù trừ từ encoder feedback (±50)
    // regulatedSpeed: tốc độ thực sự feed vào motor
    if (pidEnabled && currentSpeed != 0) {
        if (currentSpeed > 0) {
            regulatedSpeed = constrain(currentSpeed + pidOutput, 0, 255);
        } else {
            regulatedSpeed = constrain(currentSpeed - pidOutput, -255, 0);
        }
    } else {
        regulatedSpeed = currentSpeed;
    }

    // Differential steering dùng regulatedSpeed (đã qua PID)
    calculateDifferentialSteering(regulatedSpeed);

    if (regulatedSpeed != 0) {
        moveDifferential(leftMotorSpeed, rightMotorSpeed);
    } else {
        stopMotor();
    }

    xSemaphoreGive(motorMutex);
}

void MotorController::applyConfig(const RobotConfig& cfg) {
    if (!ConfigStorage::isValidConfig(cfg)) {
        Serial.println("[MOTOR] Reject applyConfig: invalid values");
        return;
    }

    if (motorMutex != nullptr) {
        xSemaphoreTake(motorMutex, portMAX_DELAY);
    }
    runtimeConfig = cfg;
    if (motorMutex != nullptr) {
        xSemaphoreGive(motorMutex);
    }

    Serial.println("[MOTOR] Runtime config applied");
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
    // float centrifugalFactor = 1.0;
    // if (abs(currentAccelY) > 0.5) {
    //     centrifugalFactor = 0.7;  // Giảm 30% độ chênh lệch khi gia tốc lớn
    // }
    
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
        regulatedSpeed = 0;
    }
}

// // ================== PID FUNCTIONS ==================
// void MotorController::enablePID(bool enable) {
//     pidEnabled = enable;
//     if (!enable) {
//         // Reset PID state khi tắt
//         pidIntegral  = 0;
//         pidLastError = 0;
//         pidOutput    = 0;
//     }
//     Serial.printf("[MOTOR] PID %s\n", enable ? "ENABLED" : "DISABLED");
// }

// void MotorController::setTargetRPM(float rpm) {
//     targetRPM = rpm;
// }

// void MotorController::updatePID() {
//     if (!pidEnabled) return;

//     // 1. Đọc RPM thực tế từ encoder (thread-safe)
//     float actualRPM = fabsf(EncoderSensor::getRPM());
//     float target    = fabsf(targetRPM);

//     // 2. Nếu target = 0 → dừng, không cần PID
//     if (target < 1.0f) {
//         pidIntegral  = 0;
//         pidLastError = 0;
//         pidOutput    = 0;
//         return;
//     }

//     // 3. Tính sai số
//     float error = target - actualRPM;

//     // 4. Tính tích phân (với anti-windup)
//     pidIntegral += error * PID_DT;
//     pidIntegral = constrain(pidIntegral, -PID_MAX_INTEGRAL, PID_MAX_INTEGRAL);

//     // 5. Tính đạo hàm
//     float derivative = (error - pidLastError) / PID_DT;
//     pidLastError = error;

//     // 6. Tính output PID
//     float output = (PID_KP * error) + (PID_KI * pidIntegral) + (PID_KD * derivative);

//     // 7. Cộng PID output vào targetSpeed hiện tại
//     //    PID chỉ ĐIỀU CHỈNH, không thay thế hoàn toàn
//     pidOutput = (int)constrain(output, -50, 50);  // Giới hạn điều chỉnh ±50 PWM

//     // Áp dụng: targetSpeed đã được đặt bởi State Machine
//     // PID bù thêm/bớt để tốc độ thực tế khớp mong muốn
//     if (targetSpeed > 0) {
//         targetSpeed = constrain(targetSpeed + pidOutput, 0, 255);
//     } else if (targetSpeed < 0) {
//         targetSpeed = constrain(targetSpeed - pidOutput, -255, 0);
//     }

//     #ifdef DEBUG_SENSOR
//     static unsigned long lastPID = 0;
//     if (millis() - lastPID > 500) {
//         lastPID = millis();
//         Serial.printf("[PID] target:%.0f actual:%.0f err:%.1f I:%.1f D:%.1f out:%d → PWM:%d\n",
//                       target, actualRPM, error, pidIntegral, derivative, pidOutput, targetSpeed);
//     }
//     #endif
// }