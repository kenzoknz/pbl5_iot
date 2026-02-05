#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "Config.h"

class UltrasonicSensor {
public:
    static long readDistanceOnce(int trig, int echo);
    static long readDistanceFiltered(int trig, int echo);
    static long readFrontDistance();
    static long readBackDistance();
    static void begin();

private:
    static void sort(long* arr, int size);
};

#endif