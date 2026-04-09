#ifndef JETSON_UART_H
#define JETSON_UART_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include "Config.h"
#include "UltrasonicSensor.h"

class JetsonUART {
public:
    static void begin();
    static bool checkForCommands();
    static void handleJetsonCommand();
    static void sendStatus();
    static uint32_t getLastCommandTime();
    static bool isJetsonConnected();

private:
    static constexpr size_t RX_BUFFER_SIZE = JETSON_UART_JSON_BUFFER_SIZE;
    static constexpr uint32_t STATUS_INTERVAL_MS = JETSON_UART_STATUS_INTERVAL_MS;
    static constexpr uint32_t CONNECTION_TIMEOUT_MS = JETSON_UART_TIMEOUT_MS;

    static uint8_t rxBuffer[RX_BUFFER_SIZE];
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
