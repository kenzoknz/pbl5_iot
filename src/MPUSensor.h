#ifndef MPU_SENSOR_H
#define MPU_SENSOR_H

#include "Config.h"
#include <MPU6050_light.h>
#include <Wire.h>

struct MpuSnapshot {
    float angleX;
    float angleY;
    float accelY;
    bool collision;
    bool tilt;
};

class MPUSensor {
public:
    static bool begin();
    static void mpuTask(void *pvParameters);
    static void update();
    static float getCurrentAngleX();
    static float getCurrentAngleY();
    static float getCurrentAccelY();
    static bool checkCollision();
    static bool checkTilt();
    static MpuSnapshot getSnapshot();

private:
    static MPU6050 mpu;
    static float currentAngleX;
    static float currentAngleY;
    static float currentAccelY;
    static float lastAccelMagnitude;
    static bool collisionFlag;
    static bool tiltFlag;
    static TaskHandle_t mpuTaskHandle;
};

#endif