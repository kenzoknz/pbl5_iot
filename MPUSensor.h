#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include "Config.h"
#include <MPU6050_light.h>
#include <Wire.h>

class MPUSensor {
public:
    static bool begin();
    static void update();
    static float getCurrentAngleX() { return currentAngleX; }
    static float getCurrentAngleY() { return currentAngleY; }
    static float getCurrentAccelY() { return currentAccelY; }
    static bool checkCollision();
    static bool checkTilt();

private:
    static MPU6050 mpu;
    static float currentAngleX;
    static float currentAngleY;
    static float currentAccelY;
    static float lastAccelMagnitude;
};

#endif