#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"
#include <ESP32Servo.h>

class MotorController {
public:
    static void begin();
    static void setServoAngle(int angle);
    static void smoothServoTransition();
    static void limitSpeedBySteering();
    static void smoothSpeedTransition();
    static void calculateDifferentialSteering(int baseSpeed);
    static void moveDifferential(int leftSpeed, int rightSpeed);
    static void moveForward(int pwm);
    static void moveBackward(int pwm);
    static void stopMotor();
    
    // Getters/Setters
    static void setTargetSpeed(int speed) { targetSpeed = speed; }
    static void setTargetServoAngle(int angle) { targetServoAngle = angle; }
    static int getCurrentSpeed() { return currentSpeed; }
    static int getServoAngle() { return servoAngle; }
    static int getLeftMotorSpeed() { return leftMotorSpeed; }
    static int getRightMotorSpeed() { return rightMotorSpeed; }

private:
    static Servo myServo;
    static int currentSpeed;
    static int targetSpeed;
    static int servoAngle;
    static int targetServoAngle;
    static int leftMotorSpeed;
    static int rightMotorSpeed;
};

#endif