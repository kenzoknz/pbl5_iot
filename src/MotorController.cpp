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

static float pwmToEstimatedRPM(int pwm) {
    if (abs(pwm) < 30) return 0;
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

void MotorController::setTargetSpeed(int speed) {
    speed = constrain(speed, -255, 255);

    if (motorMutex != nullptr) {
        xSemaphoreTake(motorMutex, portMAX_DELAY);
    }

    targetSpeed = speed;

    if (motorMutex != nullptr) {
        xSemaphoreGive(motorMutex);
    }
}

void MotorController::setTargetSteerServoAngle(int angle) {
    angle = constrain(angle, SERVO_RIGHT_MAX, SERVO_LEFT_MAX);

    if (motorMutex != nullptr) {
        xSemaphoreTake(motorMutex, portMAX_DELAY);
    }

    targetSteerServoAngle = angle;

    if (motorMutex != nullptr) {
        xSemaphoreGive(motorMutex);
    }
}

// ========== SERVO LÁI ==========
void MotorController::smoothSteerServoTransition() {
    xSemaphoreTake(motorMutex, portMAX_DELAY);

    const int step = 4; // 4-6 thường mượt hơn 8
    targetSteerServoAngle = constrain(targetSteerServoAngle, 45, 135);

    int diff = targetSteerServoAngle - steerServoAngle;

    if (abs(diff) <= step) {
        steerServoAngle = targetSteerServoAngle;
    } else {
        steerServoAngle += (diff > 0) ? step : -step;
    }

    if (steerServoAngle != lastSteerAngle) {
        steerServo.write(steerServoAngle);
        lastSteerAngle = steerServoAngle;
    }

    xSemaphoreGive(motorMutex);
}

// ========== SPEED LIMITING BY STEERING ==========
void MotorController::limitSpeedBySteering() {
    xSemaphoreTake(motorMutex, portMAX_DELAY);

    int deviation = abs(targetSteerServoAngle - SERVO_CENTER);

    // Chỉ boost khi đang chạy tiến.
    // Không boost khi STOP hoặc lùi.
    if (targetSpeed > 0) {
        if (deviation > 35 && targetSpeed < runtimeConfig.sharpTurnBoost) {
            targetSpeed = runtimeConfig.sharpTurnBoost;
        } else if (deviation > 25 && targetSpeed < runtimeConfig.mediumTurnBoost) {
            targetSpeed = runtimeConfig.mediumTurnBoost;
        } else if (deviation > 15 && targetSpeed < runtimeConfig.turnBoost) {
            targetSpeed = runtimeConfig.turnBoost;
        } else if (deviation > 10 && targetSpeed < runtimeConfig.lightTurnBoost) {
            targetSpeed = runtimeConfig.lightTurnBoost;
        }
    }

    xSemaphoreGive(motorMutex);
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

    const int accelStep = 18;
    const int decelStep = 24;

    int localTarget = constrain(targetSpeed, -255, 255);

    // Không ép tốc độ thấp lên MIN_RUN_SPEED ngay từ đầu.
    // Chỉ ép khi đã có lệnh chạy thật để tránh xe ì vì PWM quá thấp.
    if (localTarget > 0 && localTarget < runtimeConfig.minRunSpeed) {
        localTarget = runtimeConfig.minRunSpeed;
    } else if (localTarget < 0 && localTarget > -(int)runtimeConfig.minRunSpeed) {
        localTarget = -(int)runtimeConfig.minRunSpeed;
    }

    if (localTarget == 0) {
        if (currentSpeed > 0) {
            currentSpeed -= decelStep;
            if (currentSpeed < 0) currentSpeed = 0;
        } else if (currentSpeed < 0) {
            currentSpeed += decelStep;
            if (currentSpeed > 0) currentSpeed = 0;
        }
    } else {
        int step = (abs(localTarget) > abs(currentSpeed)) ? accelStep : decelStep;

        if (currentSpeed < localTarget) {
            currentSpeed += step;
            if (currentSpeed > localTarget) currentSpeed = localTarget;
        } else if (currentSpeed > localTarget) {
            currentSpeed -= step;
            if (currentSpeed < localTarget) currentSpeed = localTarget;
        }
    }

    if (pidEnabled && currentSpeed != 0) {
        if (currentSpeed > 0) {
            regulatedSpeed = constrain(currentSpeed + pidOutput, 0, 255);
        } else {
            regulatedSpeed = constrain(currentSpeed - pidOutput, -255, 0);
        }
    } else {
        regulatedSpeed = currentSpeed;
    }

    int pwmOut = regulatedSpeed;

    xSemaphoreGive(motorMutex);

    if (pwmOut != 0) {
        driveMotor(pwmOut);
    } else {
        stopMotor();
    }
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

// ========== MOTOR OUTPUT ==========
void MotorController::driveMotor(int pwm) {
    pwm = constrain(pwm, -255, 255);

    int rpwmValue = 0;
    int lpwmValue = 0;

    if (pwm > 0) {
        rpwmValue = pwm;
        lpwmValue = 0;
    } else if (pwm < 0) {
        rpwmValue = 0;
        lpwmValue = abs(pwm);
    }

    if (rpwmValue != lastLeftPWM) {
        ledcWrite(PWM_CHANNEL_RPWM, rpwmValue);
        lastLeftPWM = rpwmValue;
    }

    if (lpwmValue != lastRightPWM) {
        ledcWrite(PWM_CHANNEL_LPWM, lpwmValue);
        lastRightPWM = lpwmValue;
    }
}
void MotorController::moveDifferential(int leftSpeed, int rightSpeed) {
    // Robot thật chỉ có 1 motor kéo, nên lấy tốc độ lớn hơn về độ lớn.
    int pwm = 0;

    if (abs(leftSpeed) >= abs(rightSpeed)) {
        pwm = leftSpeed;
    } else {
        pwm = rightSpeed;
    }

    driveMotor(pwm);
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