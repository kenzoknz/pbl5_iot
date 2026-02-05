#include "UltrasonicSensor.h"
#include <Arduino.h>

void UltrasonicSensor::begin() {
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_BACK, OUTPUT);
    pinMode(ECHO_BACK, INPUT);
}

long UltrasonicSensor::readDistanceOnce(int trig, int echo) {
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

long UltrasonicSensor::readDistanceFiltered(int trig, int echo) {
    long d[3];
    for (int i = 0; i < 3; i++) {
        d[i] = readDistanceOnce(trig, echo);
        delay(5);
    }

    // Median filter - sort array
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (d[i] > d[j]) {
                long temp = d[i];
                d[i] = d[j];
                d[j] = temp;
            }
        }
    }

    return d[1]; // median
}

long UltrasonicSensor::readFrontDistance() {
    return readDistanceFiltered(TRIG_FRONT, ECHO_FRONT);
}

long UltrasonicSensor::readBackDistance() {
    return readDistanceFiltered(TRIG_BACK, ECHO_BACK);
}