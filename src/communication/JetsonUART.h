#ifndef JETSON_UART_H
#define JETSON_UART_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include "core/Config.h"
#include "sensors/UltrasonicSensor.h"

class JetsonUART {
public:
    static void begin();
    static bool checkForCommands();
    static void handleJetsonCommand();
    static void sendStatus();
    static uint32_t getLastCommandTime();
    static bool isJetsonConnected();

private:
    static uint8_t rxBuffer[JETSON_JSON_BUFFER_SIZE];
    static DynamicJsonDocument commandDoc;
    static OperationMode lastJetsonMode;
    static uint32_t lastCommandMillis;
    static bool commandPending;

    static size_t rxWriteIndex;
    static uint32_t lastStatusMillis;

    static void clearRxBuffer();
    static float readBatteryVoltage();
};

#endif
