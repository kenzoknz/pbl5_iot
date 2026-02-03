#include <ESP32Servo.h>

// ===== BTS7960 Motor Driver =====
// ESP32 PWM channels: 0-15 available
#define RPWM 18  // GPIO18 - PWM cho chieu tien
#define LPWM 19  // GPIO19 - PWM cho chieu lui
#define REN  21  // GPIO21 - Enable phai
#define LEN  22  // GPIO22 - Enable trai

// ===== SERVO =====
#define SERVO_PIN 23  // GPIO23 - Servo dieu khien huong

// ===== ULTRASONIC Sensor (Front) =====
#define TRIG_FRONT 16  // GPIO16 - Trigger cảm biến siêu âm phía trước
#define ECHO_FRONT 17  // GPIO17 - Echo cảm biến siêu âm phía trước

// ===== ULTRASONIC Sensor (Back) =====
#define TRIG_BACK 25  // GPIO25 - Trigger cảm biến siêu âm phía sau
#define ECHO_BACK 26  // GPIO26 - Echo cảm biến siêu âm phía sau

#define AUTO_SPEED 80

#define STOP_DISTANCE 15   // <= 5cm: Dừng dần
#define SLOW_DISTANCE 30  // <= 15cm: Giảm tốc dần
#define TURN_DISTANCE 69  // <= 40cm: Quay trái 60 độ
#define SAFE_DISTANCE 70  // > 40cm: chạy bình thường

// PWM channels cho ESP32
#define PWM_CHANNEL_RPWM 0
#define PWM_CHANNEL_LPWM 1
#define PWM_FREQ 1000      // 1 kHz
#define PWM_RESOLUTION 8   // 8-bit (0-255)
#define PWM_TIMER_RPWM 0   // Timer cho RPWM
#define PWM_TIMER_LPWM 1   // Timer cho LPWM

Servo myServo;

int currentSpeed = 0;
int targetSpeed = 0;  // Tốc độ mục tiêu cho smooth transition
int servoAngle = 90;  // Góc servo hiện tại (90 = giữa)
int targetServoAngle = 90;  // Góc servo mục tiêu
bool turnDirection = true;  // true = phải, false = trái

// Filter cho ultrasonic để tránh nhiễu
long lastFrontDistance = 999;
const int FILTER_SAMPLES = 3;  // Số mẫu filter

long readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Timeout 25ms thay vì 30ms để nhanh hơn
  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0) return 999;
  long distance = duration * 0.034 / 2;
  
  // Giới hạn khoảng cách hợp lý (2-400cm)
  if (distance < 2 || distance > 400) return 999;
  return distance;
}

long readFrontDistance() {
  long distance = readDistance(TRIG_FRONT, ECHO_FRONT);
  
  // Simple filter: nếu thay đổi quá lớn thì dùng giá trị cũ
  if (abs(distance - lastFrontDistance) > 50 && lastFrontDistance < 999) {
    distance = lastFrontDistance;
  }
  
  lastFrontDistance = distance;
  return distance;
}

long readBackDistance() {
  return readDistance(TRIG_BACK, ECHO_BACK);
}

void setup() {
  // Khoi tao Serial
  Serial.begin(115200);
  Serial.println("=== KHOI TAO HE THONG ESP32 ===");

  // Cau hinh PWM channels cho BTS7960 (dung timer 0 va 1)
  ledcSetup(PWM_CHANNEL_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RPWM, PWM_CHANNEL_RPWM);
  ledcAttachPin(LPWM, PWM_CHANNEL_LPWM);

  // Cau hinh enable pins
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  // Cau hinh Ultrasonic (Front)
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  // Cau hinh Ultrasonic (Back)
  pinMode(TRIG_BACK, OUTPUT);
  pinMode(ECHO_BACK, INPUT);

  // Khoi tao Servo (dung timer 2 va 3 de tranh conflict voi PWM motor)
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);  // Standard 50Hz servo
  myServo.attach(SERVO_PIN, 500, 2400);  // Min 500us, Max 2400us
  myServo.write(90);
  delay(500);  // Cho servo on dinh

  Serial.println("=== HE THONG SAN SANG ===");
}

void loop() {
  long frontDistance = readFrontDistance();
  long backDistance = readBackDistance();

  // Logic điều khiển dựa trên khoảng cách
  if (frontDistance <= STOP_DISTANCE) {
    // <= 5cm: Dừng dần (giảm tốc độ từ từ về 0)
    targetSpeed = 0;
    targetServoAngle = 30;  // Giữ nguyên quay trái 60°
    
  } else if (frontDistance <= SLOW_DISTANCE) {
    // 6-15cm: Giảm tốc dần (map tốc độ theo khoảng cách)
    targetSpeed = map(frontDistance, STOP_DISTANCE, SLOW_DISTANCE, 30, 100);
    targetSpeed = constrain(targetSpeed, 30, 100);
    targetServoAngle = 30;  // Giữ nguyên quay trái 60°
    
  } else if (frontDistance <= TURN_DISTANCE) {
    // 16-40cm: Quay trái 60 độ và GIỮ NGUYÊN
    targetSpeed = 80;  // Giảm tốc độ xuống 80 khi rẽ để giảm dòng điện
    targetServoAngle = 30;  // Mục tiêu: Quay trái 60 độ (90 - 60 = 30)
    
  } else {
    // > 40cm: Chạy bình thường, quay về giữa
    targetSpeed = AUTO_SPEED;
    targetServoAngle = 90;  // Mục tiêu: Quay về giữa
  }

  // Smooth servo transition - Quay servo từ từ để tránh dòng điện đột ngột
  smoothServoTransition();
  
  // Smooth speed transition - Thay đổi tốc độ dần dần
  smoothSpeedTransition();

  showDebug(frontDistance, backDistance);
  
  // Tăng delay để giảm tải cho CPU và tránh watchdog timeout
  delay(100);
  yield();  // Feed watchdog
}

void showDebug(long frontDistance, long backDistance) {
  Serial.print("Front: ");
  Serial.print(frontDistance);
  Serial.print(" cm | Back: ");
  Serial.print(backDistance);
  Serial.print(" cm | Speed: ");
  Serial.print(currentSpeed);
  Serial.print("/");
  Serial.print(targetSpeed);
  Serial.print(" | Servo: ");
  Serial.print(servoAngle);
  Serial.print("° | Status: ");

  if (frontDistance <= STOP_DISTANCE) {
    Serial.println("STOPPING");
  } else if (frontDistance <= SLOW_DISTANCE) {
    Serial.println("SLOWING");
  } else if (frontDistance <= TURN_DISTANCE) {
    Serial.println("TURN LEFT 60°");
  } else {
    Serial.println("FORWARD");
  }
}

void smoothServoTransition() {
  // Quay servo từ từ để tránh dòng điện đột ngột gây reset
  int servoStep = 3;  // Quay 3 độ mỗi lần (100ms)
  
  if (servoAngle < targetServoAngle) {
    servoAngle += servoStep;
    if (servoAngle > targetServoAngle) servoAngle = targetServoAngle;
  } else if (servoAngle > targetServoAngle) {
    servoAngle -= servoStep;
    if (servoAngle < targetServoAngle) servoAngle = targetServoAngle;
  }
  
  myServo.write(servoAngle);
}

void smoothSpeedTransition() {
  // Thay đổi tốc độ dần dần thay vì đột ngột
  int speedStep = 10;  // Thay đổi 10 đơn vị mỗi lần (100ms)
  
  if (currentSpeed < targetSpeed) {
    // Tăng tốc
    currentSpeed += speedStep;
    if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
  } else if (currentSpeed > targetSpeed) {
    // Giảm tốc
    currentSpeed -= speedStep;
    if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
  }
  
  // Áp dụng tốc độ mới
  if (currentSpeed > 0) {
    moveForward(currentSpeed);
  } else {
    stopMotor();
  }
}

void moveForward(int pwm) {
  ledcWrite(PWM_CHANNEL_RPWM, pwm);
  ledcWrite(PWM_CHANNEL_LPWM, 0);
}

void stopMotor() {
  ledcWrite(PWM_CHANNEL_RPWM, 0);
  ledcWrite(PWM_CHANNEL_LPWM, 0);
}
