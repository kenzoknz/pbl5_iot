#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"
#include <ESP32Servo.h>

class MotorController {
public:
    static void begin();
    
    // Servo lái bánh xe
    static void smoothSteerServoTransition();
    static int getSteerServoAngle() { return steerServoAngle; }
    static void setTargetSteerServoAngle(int angle) { targetSteerServoAngle = angle; }
    
    // Motor control
    static void limitSpeedBySteering();
    static void smoothSpeedTransition();
    static void calculateDifferentialSteering(int baseSpeed);
    static void moveDifferential(int leftSpeed, int rightSpeed);
    static void stopMotor();
    
    // Getters/Setters
    static void setTargetSpeed(int speed) { targetSpeed = speed; }
    static int getCurrentSpeed() { return currentSpeed; }
    static int getLeftMotorSpeed() { return leftMotorSpeed; }
    static int getRightMotorSpeed() { return rightMotorSpeed; }

private:
    static Servo steerServo; 
    static int currentSpeed;
    static int targetSpeed;
    static int steerServoAngle;
    static int targetSteerServoAngle;
    // static int usSensorServoAngle;
    static int leftMotorSpeed;
    static int rightMotorSpeed;
};

#endif