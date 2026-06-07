#include "JetsonUART.h"

#include <cstring>

#include "GPSSensor.h"
#include "MotorController.h"

uint8_t JetsonUART::rxBuffer[JETSON_JSON_BUFFER_SIZE] = {0};
DynamicJsonDocument JetsonUART::commandDoc(JETSON_JSON_BUFFER_SIZE);
uint32_t JetsonUART::lastCommandMillis = 0;
bool JetsonUART::commandPending = false;
volatile bool JetsonUART::stopHoldActive = false;
size_t JetsonUART::rxWriteIndex = 0;
uint32_t JetsonUART::lastStatusMillis = 0;
QueueHandle_t JetsonUART::xCmdQueue       = nullptr;

void JetsonUART::begin() {
    /* Tạo queue 1 phần tử kiểu JetsonCmdType (1 byte) */
    xCmdQueue = xQueueCreate(1, sizeof(JetsonCmdType));
    configASSERT(xCmdQueue != nullptr);   // Halt nếu heap không đủ

    Serial1.begin(JETSON_UART_BAUD, SERIAL_8N1, ROS_RX_PIN, ROS_TX_PIN);
    clearRxBuffer();
    commandDoc.clear();

    lastCommandMillis = millis();
    lastStatusMillis = 0;
    commandPending = false;
    stopHoldActive = false;
    Serial.printf("[UART] Jetson Serial1 init — RX:GPIO%d TX:GPIO%d @%d baud\n",
                  ROS_RX_PIN, ROS_TX_PIN, JETSON_UART_BAUD);
}

bool JetsonUART::checkForCommands() {
    while (Serial1.available() > 0) {
        const int raw = Serial1.read();
        if (raw < 0) break;

        const char ch = static_cast<char>(raw);
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (rxWriteIndex == 0) continue;
            rxBuffer[rxWriteIndex] = '\0';
            commandPending = true;
            return true;
        }

        if (rxWriteIndex < (JETSON_JSON_BUFFER_SIZE - 1)) {
            rxBuffer[rxWriteIndex++] = static_cast<uint8_t>(ch);
        } else {
            // Keep the newest bytes when the line is too long.
            memmove(rxBuffer, rxBuffer + 1, JETSON_JSON_BUFFER_SIZE - 2);
            rxBuffer[JETSON_JSON_BUFFER_SIZE - 2] = static_cast<uint8_t>(ch);
            rxBuffer[JETSON_JSON_BUFFER_SIZE - 1] = '\0';
            rxWriteIndex = JETSON_JSON_BUFFER_SIZE - 1;
            Serial.println("[UART] RX buffer full, dropped oldest byte");
        }
    }

    return commandPending;
}

void JetsonUART::handleJetsonCommand() {
    if (!commandPending) return;

    commandDoc.clear();

    String cmd;

    // Cho phép dạng text nhanh: STOP\n, FORWARD\n
    String rawLine(reinterpret_cast<const char*>(rxBuffer));
    rawLine.trim();

    const DeserializationError err = deserializeJson(commandDoc, reinterpret_cast<const char*>(rxBuffer));

    if (!err) {
        const char* cmdRaw = commandDoc["cmd"];
        if (cmdRaw != nullptr && cmdRaw[0] != '\0') {
            cmd = String(cmdRaw);
        }
    } else {
        // Nếu không phải JSON thì xử lý như lệnh text đơn giản
        cmd = rawLine;
    }

    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() == 0) {
        Serial.println("[JETSON] Empty command");
        commandPending = false;
        clearRxBuffer();
        return;
    }

    // A syntactically valid command line from Jetson keeps the link alive.
    
    JetsonCmdType cmdType = JETSON_CMD_NONE;

    if (cmd == "STOP" || cmd == "S") {
        cmdType = JETSON_CMD_STOP;
        Serial.println("[JETSON] Enqueue: STOP");

    } else if (cmd == "FORWARD" || cmd == "F" || cmd == "AUTONOMOUS" || cmd == "RESUME") {
        cmdType = JETSON_CMD_FORWARD;
        Serial.println("[JETSON] Enqueue: FORWARD");
        
    } else if (cmd == "MANUAL") {
        /* MANUAL từ Jetson bị bỏ qua — Jetson chỉ dùng STOP/FORWARD */
        Serial.println("[JETSON] MANUAL ignored (Jetson only: STOP/FORWARD)");
 
    } else {
        Serial.printf("[JETSON] Unknown cmd: '%s'\n", cmd.c_str());
    }

    if (cmdType != JETSON_CMD_NONE) {
        lastCommandMillis = millis();
        xQueueOverwrite(xCmdQueue, &cmdType);
    }

    commandPending = false;
    clearRxBuffer();
}


bool JetsonUART::processQueuedCommand() {
    if (xCmdQueue == nullptr) return false;
 
    JetsonCmdType cmdType = JETSON_CMD_NONE;
 
    /* xQueueReceive với timeout=0 → không block vLogicTask */
    if (xQueueReceive(xCmdQueue, &cmdType, 0) != pdTRUE) {
        return false;   // Queue trống — không có lệnh mới
    }
 
    switch (cmdType) {
        case JETSON_CMD_STOP:
            engageStopHold();
            Serial.println("[JETSON → Logic] STOP");
            break;
 
        case JETSON_CMD_FORWARD:
            clearStopHold();
            Serial.println("[JETSON → Logic] FORWARD — stopHold cleared");
            break;
 
        default:
            break;
    }
 
    return true;
}

void JetsonUART::checkWatchdog() {
    static bool timeoutLogged = false;
 
    if (!isJetsonConnected()) {
        if (!timeoutLogged) {
            clearStopHold();
            Serial.println("[JETSON Watchdog] Timeout — ESP32 takes primary control");
            timeoutLogged = true;
        }
    } else {
        timeoutLogged = false;   // Reset khi Jetson kết nối lại
    }
}

uint32_t JetsonUART::getLastCommandTime() {
    return lastCommandMillis;
}

bool JetsonUART::isJetsonConnected() {
    return (millis() - lastCommandMillis) <= JETSON_WATCHDOG_TIMEOUT_MS;
}

bool JetsonUART::isStopHoldActive() {
    return stopHoldActive;
}

void JetsonUART::clearStopHold() {
    stopHoldActive = false;
}

void JetsonUART::engageStopHold() {
    stopHoldActive = true;
    MotorController::setTargetSpeed(0);
    MotorController::stopMotor();
}

void JetsonUART::clearRxBuffer() {
    memset(rxBuffer, 0, sizeof(rxBuffer));
    rxWriteIndex = 0;
}

float JetsonUART::readBatteryVoltage() {
    // GPIO35 is used by encoder in this project, so keep a safe placeholder value.
    // Replace with real ADC conversion when battery divider is wired to a free ADC pin.
    return 12.0f;
}