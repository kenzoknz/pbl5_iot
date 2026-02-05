#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

/* ================== MPU6050 ================== */
#define SDA_PIN 32
#define SCL_PIN 33
#define COLLISION_THRESHOLD 2.0
#define TILT_THRESHOLD 20.0

/* ================== BTS7960 ================== */
#define RPWM 18
#define LPWM 19
#define REN 21
#define LEN 22

/* ================== SERVO ================== */
#define SERVO_PIN 23

/* ================== ULTRASONIC ================== */
#define TRIG_FRONT 16
#define ECHO_FRONT 17
#define TRIG_BACK  25
#define ECHO_BACK  26

/* ================== PWM ================== */
#define PWM_CHANNEL_RPWM 0
#define PWM_CHANNEL_LPWM 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

/* ================== SPEED ================== */
#define STOP_SPEED 0
#define MIN_RUN_SPEED 110
#define CRUISE_SPEED 115
#define FAST_SPEED 127
#define TURN_BOOST 120
#define BACK_SPEED 115

/* ================== DISTANCE ================== */
#define STOP_DISTANCE     20
#define SLOW_DISTANCE     30
#define TURN_DISTANCE     45
#define PREPARE_DISTANCE  60
#define BACK_DANGER_DISTANCE 20

/* ================== TIME ================== */
#define BACK_TIME 3000
#define TURN_TIME 2000

/* ================== STATE ================== */
enum State {
  INIT,
  NORMAL,
  SLOW,
  TURN,
  STOP,
  BACKING,
  TURNING,
  RESUMING
};

State currentState = INIT;
unsigned long stateStartTime = 0;

/* ================== GLOBAL ================== */
Servo myServo;
MPU6050 mpu(Wire);

int currentSpeed = 0;
int targetSpeed = 0;
int servoAngle = 90;
int targetServoAngle = 90;

// MPU6050 data
float currentAngleX = 0;    // Góc nghiêng ngang (trái/phải)
float currentAngleY = 0;    // Góc nghiêng dọc (trước/sau)
float currentAccelY = 0;    // Gia tốc ngang (ly tâm)
float lastAccelMagnitude = 0;

// Differential steering
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// Random turn direction
bool turnRight = true;  // true = quay phải, false = quay trái

/* ================================================= */
/* ================= ULTRASONIC ==================== */
/* ================================================= */

long readDistanceOnce(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0) return 999;

  long dist = duration * 0.034 / 2;
  if (dist < 2 || dist > 300) return 999;
  return dist;
}

/* Median filter – chống nhiễu */
long readDistanceFiltered(int trig, int echo) {
  long d[3];
  for (int i = 0; i < 3; i++) {
    d[i] = readDistanceOnce(trig, echo);
    delay(5);
  }

  // sort
  for (int i = 0; i < 2; i++)
    for (int j = i + 1; j < 3; j++)
      if (d[i] > d[j]) {
        long t = d[i];
        d[i] = d[j];
        d[j] = t;
      }

  return d[1]; // median
}

long readFrontDistance() {
  return readDistanceFiltered(TRIG_FRONT, ECHO_FRONT);
}

long readBackDistance() {
  return readDistanceFiltered(TRIG_BACK, ECHO_BACK);
}

/* ================================================= */
/* ================= MPU6050 ======================= */
/* ================================================= */

void updateMPU6050() {
  mpu.update();
  
  // Đọc gia tốc (đơn vị: g)
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  
  // Tính độ lớn gia tốc tổng
  float accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
  
  // Phát hiện va chạm đột ngột
  float accelChange = abs(accelMagnitude - lastAccelMagnitude);
  if (accelChange > COLLISION_THRESHOLD) {
    Serial.println("!!! VA CHAM PHAT HIEN !!!");
    targetSpeed = STOP_SPEED;
    delay(2000);
  }
  
  // Đọc góc nghiêng
  currentAngleX = mpu.getAngleX();  // Nghiêng trái/phải
  currentAngleY = mpu.getAngleY();  // Nghiêng trước/sau
  currentAccelY = ay;               // Gia tốc ngang (ly tâm)
  
  // Phát hiện nghiêng nguy hiểm
  if (abs(currentAngleX) > TILT_THRESHOLD || abs(currentAngleY) > TILT_THRESHOLD) {
    Serial.print("!!! NGHIENG NGUY HIEM: X=");
    Serial.print(currentAngleX);
    Serial.print("° Y=");
    Serial.print(currentAngleY);
    Serial.println("° !!!");
    targetSpeed = STOP_SPEED;
    delay(2000);
  }
  
  lastAccelMagnitude = accelMagnitude;
}

/* ================================================= */
/* ========= DIFFERENTIAL STEERING ================ */
/* ================================================= */

// Tính tốc độ motor trái/phải dựa trên góc servo và dữ liệu MPU6050
void calculateDifferentialSteering(int baseSpeed) {
  if (baseSpeed == 0) {
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    return;
  }
  
  // Tính độ lệch từ góc servo (90° = thẳng)
  int servoDeviation = servoAngle - 90;
  
  // === BÙ TRỪ GÓC NGHIÊNG NGANG ===
  // Nếu xe nghiêng trái → Tăng tốc motor trái để cân bằng
  // Nếu xe nghiêng phải → Tăng tốc motor phải để cân bằng
  float tiltCompensation = currentAngleX * 0.3;  // Mỗi 1° nghiêng → 0.3% chênh lệch
  
  // === ĐIỀU CHỈNH THEO GIA TỐC LY TÂM ===
  // Khi gia tốc ly tâm lớn → Giảm chênh lệch tốc độ để tránh trượt
  float centrifugalFactor = 1.0;
  if (abs(currentAccelY) > 0.5) {
    centrifugalFactor = 0.7;  // Giảm 30% độ chênh lệch khi gia tốc lớn
  }
  
  // === ĐIỀU CHỈNH THEO GÓC DỐC ===
  // Xuống dốc → Tăng chênh lệch để rẽ tốt hơn
  // Lên dốc → Giảm chênh lệch để tránh trượt
  float slopeCompensation = 0;
  if (currentAngleY < -10) {  // Xuống dốc
    slopeCompensation = abs(currentAngleY + 10) * 0.5;
  } else if (currentAngleY > 10) {  // Lên dốc
    slopeCompensation = -abs(currentAngleY - 10) * 0.3;
  }
  
  // Tính tỷ lệ chênh lệch tốc độ (0-100%)
  // servoDeviation: -60° đến +60° (rẽ phải đến rẽ trái)
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

/* ================================================= */
/* ================= SETUP ========================= */
/* ================================================= */

void setup() {
  Serial.begin(115200);
  Serial.println("=== KHOI TAO HE THONG ESP32 + MPU6050 ===");
  
  // Khởi tạo I2C cho MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Khởi tạo MPU6050
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  
  if (status != 0) {
    Serial.println("!!! LOI: Khong ket noi duoc MPU6050 !!!");
    Serial.println("Kiem tra day noi SDA/SCL va nguon cap 3.3V");
    while (1) { delay(1000); }
  }
  
  Serial.println("Dang hieu chuan MPU6050... Giu xe yen!");
  delay(1000);
  mpu.calcOffsets();  // Hiệu chuẩn (xe phải đứng yên)
  Serial.println("Hieu chuan hoan tat!");
  
  ledcSetup(PWM_CHANNEL_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RPWM, PWM_CHANNEL_RPWM);
  ledcAttachPin(LPWM, PWM_CHANNEL_LPWM);

  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2400);
  delay(500);
  myServo.write(90);

  // Khởi tạo random seed
  randomSeed(analogRead(0) + analogRead(4) + micros());
  
  currentState = NORMAL;
  
  Serial.println("=== HE THONG SAN SANG ===");
}

/* ================================================= */
/* ================= LOOP ========================== */
/* ================================================= */

void loop() {
  // Cập nhật dữ liệu MPU6050
  updateMPU6050();
  
  long frontDist = readFrontDistance();
  long backDist  = readBackDistance();
  unsigned long now = millis();

  switch (currentState) {

  case NORMAL:
    if (frontDist > PREPARE_DISTANCE) {
      targetSpeed = FAST_SPEED;
      targetServoAngle = 90;
    } else {
      currentState = SLOW;
    }
    break;

  case SLOW:
    if (frontDist > TURN_DISTANCE) {
      targetSpeed = CRUISE_SPEED;
      targetServoAngle = 75;
    } else {
      currentState = TURN;
    }
    break;

  case TURN:
    if (frontDist > STOP_DISTANCE) {
      targetSpeed = CRUISE_SPEED;
      targetServoAngle = 55;
    } else {
      currentState = STOP;
    }
    break;

  case STOP:
    targetSpeed = STOP_SPEED;
    targetServoAngle = 90;

    if (backDist > BACK_DANGER_DISTANCE) {
      // Chọn ngẫu nhiên hướng quay: 50% phải, 50% trái
      turnRight = random(0, 2) == 0;  // random(0,2) trả về 0 hoặc 1
      
      Serial.print(">>> Chon huong: ");
      Serial.println(turnRight ? "QUAY PHAI" : "QUAY TRAI");
      
      currentState = BACKING;
      stateStartTime = now;
    }
    break;

  case BACKING:
    targetSpeed = -BACK_SPEED;
    targetServoAngle = 90;

    if (now - stateStartTime > BACK_TIME) {
      currentState = TURNING;
      stateStartTime = now;
    }
    break;

  case TURNING:
    targetSpeed = TURN_BOOST;
    
    // Áp dụng góc servo theo hướng đã chọn
    if (turnRight) {
      targetServoAngle = 30;   // Quay phải (góc nhỏ)
    } else {
      targetServoAngle = 150;  // Quay trái (góc lớn)
    }

    if (now - stateStartTime > TURN_TIME) {
      currentState = RESUMING;
    }
    break;

  case RESUMING:
    targetSpeed = CRUISE_SPEED;
    
    // Giữ hướng quay để thoát vật cản
    if (turnRight) {
      targetServoAngle = 60;   // Tiếp tục quay phải nhẹ
    } else {
      targetServoAngle = 120;  // Tiếp tục quay trái nhẹ
    }

    if (frontDist > PREPARE_DISTANCE) {
      currentState = NORMAL;
    }
    break;
  }

  smoothServoTransition();
  limitSpeedBySteering();
  smoothSpeedTransition();
  
  // Debug output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    Serial.print("F:");
    Serial.print(frontDist);
    Serial.print(" B:");
    Serial.print(backDist);
    Serial.print(" | L:");
    Serial.print(leftMotorSpeed);
    Serial.print(" R:");
    Serial.print(rightMotorSpeed);
    Serial.print(" | Servo:");
    Serial.print(servoAngle);
    Serial.print(" [");
    Serial.print(turnRight ? "R" : "L");
    Serial.print("] | Tilt:");
    Serial.print(currentAngleX, 1);
    Serial.print("/");
    Serial.print(currentAngleY, 1);
    Serial.print(" | Accel:");
    Serial.println(currentAccelY, 2);
    lastDebug = millis();
  }

  delay(60);
}

/* ================================================= */
/* ================= SAFETY ======================== */
/* ================================================= */

void limitSpeedBySteering() {
  int deviation = abs(servoAngle - 90);
  if (deviation > 20 && targetSpeed > CRUISE_SPEED) {
    targetSpeed = CRUISE_SPEED;
  }
}

/* ================================================= */
/* ================= SMOOTH ======================== */
/* ================================================= */

void smoothServoTransition() {
  int step = 5;
  if (servoAngle < targetServoAngle) servoAngle += step;
  else if (servoAngle > targetServoAngle) servoAngle -= step;

  servoAngle = constrain(servoAngle, 30, 150);
  myServo.write(servoAngle);
}

void smoothSpeedTransition() {
  int step = 12;

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

/* ================================================= */
/* ================= MOTOR ========================= */
/* ================================================= */

// Điều khiển differential steering với 2 tốc độ riêng
// leftSpeed, rightSpeed: -255 đến 255
void moveDifferential(int leftSpeed, int rightSpeed) {
  // Giả sử: RPWM/LPWM điều khiển 1 BTS7960 cho cả 2 bánh
  // Với 2 motor riêng, bạn cần thêm 2 channel PWM cho motor thứ 2
  
  // Hiện tại: Dùng RPWM cho bánh phải, LPWM cho bánh trái
  // (Điều chỉnh theo cách đấu nối thực tế của bạn)
  
  int avgSpeed = (leftSpeed + rightSpeed) / 2;
  
  if (avgSpeed > 0) {
    // Tiến: RPWM điều khiển tốc độ
    int rightPWM = constrain(abs(rightSpeed), 0, 255);
    ledcWrite(PWM_CHANNEL_RPWM, rightPWM);
    ledcWrite(PWM_CHANNEL_LPWM, 0);
  } else if (avgSpeed < 0) {
    // Lùi: LPWM điều khiển tốc độ
    int leftPWM = constrain(abs(leftSpeed), 0, 255);
    ledcWrite(PWM_CHANNEL_RPWM, 0);
    ledcWrite(PWM_CHANNEL_LPWM, leftPWM);
  } else {
    stopMotor();
  }
  
}

void moveForward(int pwm) {
  ledcWrite(PWM_CHANNEL_RPWM, pwm);
  ledcWrite(PWM_CHANNEL_LPWM, 0);
}

void moveBackward(int pwm) {
  ledcWrite(PWM_CHANNEL_RPWM, 0);
  ledcWrite(PWM_CHANNEL_LPWM, pwm);
}

void stopMotor() {
  ledcWrite(PWM_CHANNEL_RPWM, 0);
  ledcWrite(PWM_CHANNEL_LPWM, 0);
}
