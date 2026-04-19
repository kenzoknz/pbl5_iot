#ifndef JETSON_UART_H
#define JETSON_UART_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
 
#include "Config.h"
#include "UltrasonicSensor.h"

enum JetsonCmdType : uint8_t {
    JETSON_CMD_NONE    = 0,   // Không có lệnh (queue trống)
    JETSON_CMD_STOP    = 1,   // Dừng motor, giữ hold cho đến khi có FORWARD
    JETSON_CMD_FORWARD = 2,   // Tiếp tục chạy (giải phóng stop hold)
};

class JetsonUART {
public:
    static void begin();
    
    static bool checkForCommands();
    static void handleJetsonCommand();
    // static void sendStatus();

    static bool processQueuedCommand();  // Dequeue → áp lên MotorController
    static void checkWatchdog();         // Giải phóng stopHold nếu Jetson timeout

    static uint32_t getLastCommandTime();
    static bool isJetsonConnected();
    static bool isStopHoldActive();
    static void clearStopHold();

private:
    static uint8_t rxBuffer[JETSON_JSON_BUFFER_SIZE];
    static DynamicJsonDocument commandDoc;
    static uint32_t lastCommandMillis;
    static bool commandPending;
    static volatile bool stopHoldActive;

    static size_t rxWriteIndex;
    static uint32_t lastStatusMillis;

    static QueueHandle_t xCmdQueue;

    static void engageStopHold();
    static void clearRxBuffer();
    static float readBatteryVoltage();
};

#endif