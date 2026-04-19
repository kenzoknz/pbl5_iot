#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Config.h"
#include "ConfigStorage.h"
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

    // Speed control dựa trên Encoder 
    static void updatePID();                // Gọi mỗi chu kỳ logic (20Hz)
    static bool isPIDEnabled() { return pidEnabled; }
    static void enablePID(bool enable);
    static void applyConfig(const RobotConfig& cfg);
    static RobotConfig getConfig() { return runtimeConfig; }
    
    // Getters/Setters
    static void setTargetSpeed(int speed) { targetSpeed = speed; }
    static int  getTargetSpeed()    { return targetSpeed; }
    static int  getCurrentSpeed()   { return currentSpeed; }
    static int  getRegulatedSpeed() { return regulatedSpeed; }  // Sau PID
    static int  getLeftMotorSpeed()  { return leftMotorSpeed; }
    static int  getRightMotorSpeed() { return rightMotorSpeed; }
    static int  getPIDOutput() { return pidOutput; }

private:
    static Servo steerServo; 
    static int currentSpeed; // v hiện tai (sau smooth transition)
    static int targetSpeed; // v mong muốn (được State Machine đặt)
    static int regulatedSpeed;      // Tốc độ sau PID bù trừ (feed vào differential steering)
    static int steerServoAngle;
    static int targetSteerServoAngle;
    static int leftMotorSpeed;
    static int rightMotorSpeed;

    //PID Controller 
    static bool  pidEnabled;
    static float pidIntegral;
    static float pidLastError;
    static int   pidOutput;  // Bù trừ PID (±50 PWM)
    static RobotConfig runtimeConfig;
};

#endif