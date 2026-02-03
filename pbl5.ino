#include <ESP32Servo.h>

#define RPWM 18
#define LPWM 19
#define REN 21
#define LEN 22

#define SERVO_PIN 23

#define TRIG_FRONT 16
#define ECHO_FRONT 17

#define TRIG_BACK 25
#define ECHO_BACK 26

#define AUTO_SPEED 100

#define STOP_SPEED 0
#define MIN_RUN_SPEED 85
#define CRUISE_SPEED 95
#define FAST_SPEED 110

#define STOP_DISTANCE 10
#define SLOW_DISTANCE 20
#define TURN_DISTANCE 30
#define PREPARE_DISTANCE 50

#define BACK_DANGER_DISTANCE 30
#define BACK_SPEED 100

#define PWM_CHANNEL_RPWM 0
#define PWM_CHANNEL_LPWM 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_TIMER_RPWM 0
#define PWM_TIMER_LPWM 1

Servo myServo;

enum State
{
  NORMAL,
  BACKING,
  TURNING,
  RESUMING
};

State currentState = NORMAL;
unsigned long stateStartTime = 0;
const unsigned long BACK_TIME = 1500;
const unsigned long TURN_TIME = 2000;

int currentSpeed = 0;
int targetSpeed = 0;
int servoAngle = 90;
int targetServoAngle = 90;
bool turnDirection = true;

long lastFrontDistance = 999;
const int FILTER_SAMPLES = 3;

long readDistance(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);
  if (duration == 0)
    return 999;
  long distance = duration * 0.034 / 2;

  if (distance < 2 || distance > 400)
    return 999;
  return distance;
}

long readFrontDistance()
{
  long distance = readDistance(TRIG_FRONT, ECHO_FRONT);

  if (abs(distance - lastFrontDistance) > 50 && lastFrontDistance < 999)
  {
    distance = lastFrontDistance;
  }

  lastFrontDistance = distance;
  return distance;
}

long readBackDistance()
{
  return readDistance(TRIG_BACK, ECHO_BACK);
}

void setup()
{

  Serial.begin(115200);
  Serial.println("=== KHOI TAO HE THONG ESP32 ===");

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
  myServo.write(90);
  delay(500);

  Serial.println("=== HE THONG SAN SANG ===");
}

void loop()
{
  long frontDistance = readFrontDistance();
  long backDistance = readBackDistance();
  unsigned long currentTime = millis();

  switch (currentState)
  {
  case NORMAL:

    if (frontDistance < STOP_DISTANCE)
    {

      if (backDistance > BACK_DANGER_DISTANCE)
      {

        currentState = BACKING;
        stateStartTime = currentTime;
        targetSpeed = -BACK_SPEED;
        servoAngle = 90;
        targetServoAngle = 90;
        myServo.write(90);
      }
      else
      {

        targetSpeed = STOP_SPEED;
        targetServoAngle = 90;
      }
    }
    else if (frontDistance < SLOW_DISTANCE)
    {

      targetSpeed = map(frontDistance, STOP_DISTANCE, SLOW_DISTANCE, MIN_RUN_SPEED, FAST_SPEED);
      targetSpeed = constrain(targetSpeed, MIN_RUN_SPEED, FAST_SPEED);
      targetServoAngle = 30;
    }
    else if (frontDistance < TURN_DISTANCE)
    {

      targetSpeed = MIN_RUN_SPEED;
      targetServoAngle = 30;
    }
    else if (frontDistance < PREPARE_DISTANCE)
    {

      targetSpeed = CRUISE_SPEED;
      targetServoAngle = 50;
    }
    else
    {

      targetSpeed = AUTO_SPEED;
      targetServoAngle = 90;
    }
    break;

  case BACKING:

    targetSpeed = -BACK_SPEED;
    targetServoAngle = 90;

    if (currentTime - stateStartTime >= BACK_TIME || backDistance < BACK_DANGER_DISTANCE)
    {

      currentState = TURNING;
      stateStartTime = currentTime;
    }
    break;

  case TURNING:

    targetSpeed = MIN_RUN_SPEED;
    targetServoAngle = 30;

    if (currentTime - stateStartTime >= TURN_TIME)
    {

      currentState = RESUMING;
    }
    break;

  case RESUMING:

    targetSpeed = CRUISE_SPEED;
    targetServoAngle = 30;

    if (frontDistance > PREPARE_DISTANCE)
    {
      currentState = NORMAL;
    }
    break;
  }

  smoothServoTransition();

  if (abs(servoAngle - targetServoAngle) > 15)
  {
    if (targetSpeed > MIN_RUN_SPEED)
    {
      targetSpeed = MIN_RUN_SPEED;
    }
  }

  smoothSpeedTransition();

  showDebug(frontDistance, backDistance);

  delay(100);
  yield();
}

void showDebug(long frontDistance, long backDistance)
{
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
  Serial.print("° | State: ");

  switch (currentState)
  {
  case NORMAL:
    if (frontDistance < STOP_DISTANCE)
    {
      Serial.println("STOP!");
    }
    else if (frontDistance < SLOW_DISTANCE)
    {
      Serial.println("SLOWING");
    }
    else if (frontDistance < TURN_DISTANCE)
    {
      Serial.println("TURNING");
    }
    else if (frontDistance < PREPARE_DISTANCE)
    {
      Serial.println("PREPARE");
    }
    else
    {
      Serial.println("FORWARD");
    }
    break;
  case BACKING:
    Serial.println("BACKING UP!");
    break;
  case TURNING:
    Serial.println("TURNING SHARP!");
    break;
  case RESUMING:
    Serial.println("RESUMING...");
    break;
  }
}

void smoothServoTransition()
{

  int servoStep = 6;

  if (servoAngle < targetServoAngle)
  {
    servoAngle += servoStep;
    if (servoAngle > targetServoAngle)
      servoAngle = targetServoAngle;
  }
  else if (servoAngle > targetServoAngle)
  {
    servoAngle -= servoStep;
    if (servoAngle < targetServoAngle)
      servoAngle = targetServoAngle;
  }

  myServo.write(servoAngle);
}

void smoothSpeedTransition()
{

  int speedStep = 15;

  if (targetSpeed > 0 && targetSpeed < MIN_RUN_SPEED)
  {
    targetSpeed = MIN_RUN_SPEED;
  }
  if (targetSpeed < 0 && targetSpeed > -MIN_RUN_SPEED)
  {
    targetSpeed = -MIN_RUN_SPEED;
  }

  if (currentSpeed < targetSpeed)
  {

    currentSpeed += speedStep;
    if (currentSpeed > targetSpeed)
      currentSpeed = targetSpeed;
  }
  else if (currentSpeed > targetSpeed)
  {

    currentSpeed -= speedStep;
    if (currentSpeed < targetSpeed)
      currentSpeed = targetSpeed;
  }

  if (currentSpeed > 0)
  {
    moveForward(currentSpeed);
  }
  else if (currentSpeed < 0)
  {
    moveBackward(-currentSpeed);
  }
  else
  {
    stopMotor();
  }
}

void moveForward(int pwm)
{
  ledcWrite(PWM_CHANNEL_RPWM, pwm);
  ledcWrite(PWM_CHANNEL_LPWM, 0);
}

void moveBackward(int pwm)
{
  ledcWrite(PWM_CHANNEL_RPWM, 0);
  ledcWrite(PWM_CHANNEL_LPWM, pwm);
}

void stopMotor()
{
  ledcWrite(PWM_CHANNEL_RPWM, 0);
  ledcWrite(PWM_CHANNEL_LPWM, 0);
}
