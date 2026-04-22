#include "communication/JetsonUART.h"

#include <cstring>

#include "sensors/GPSSensor.h"
#include "control/MotorController.h"

uint8_t JetsonUART::rxBuffer[JETSON_JSON_BUFFER_SIZE] = {0};
DynamicJsonDocument JetsonUART::commandDoc(JETSON_JSON_BUFFER_SIZE);
uint32_t JetsonUART::lastCommandMillis = 0;
bool JetsonUART::commandPending = false;
volatile bool JetsonUART::stopHoldActive = false;
size_t JetsonUART::rxWriteIndex = 0;
uint32_t JetsonUART::lastStatusMillis = 0;

void JetsonUART::begin() {
    Serial1.begin(JETSON_UART_BAUD, SERIAL_8N1, ROS_RX_PIN, ROS_TX_PIN);
    clearRxBuffer();
    commandDoc.clear();
    lastCommandMillis = millis();
    lastStatusMillis = 0;
    commandPending = false;
    stopHoldActive = false;
    Serial.println("[UART] Jetson Serial1 initialized on GPIO39/15");
}

bool JetsonUART::checkForCommands() {
    while (Serial1.available() > 0) {
        const int raw = Serial1.read();
        if (raw < 0) {
            break;
        }

        const char ch = static_cast<char>(raw);
        if (ch == '\r') {
            continue;
        }

        if (ch == '\n') {
            if (rxWriteIndex == 0) {
                continue;
            }

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
    if (!commandPending) {
        return;
    }

    commandDoc.clear();

    const DeserializationError err = deserializeJson(commandDoc, reinterpret_cast<const char*>(rxBuffer));
    if (err) {
        Serial.printf("[JETSON] Invalid JSON: %s\n", err.c_str());
        commandPending = false;
        clearRxBuffer();
        return;
    }

    const char* cmdRaw = commandDoc["cmd"];
    if (cmdRaw == nullptr || cmdRaw[0] == '\0') {
        Serial.println("[JETSON] Missing cmd field");
        commandPending = false;
        clearRxBuffer();
        return;
    }

    String cmd(cmdRaw);
    cmd.trim();
    cmd.toUpperCase();

    // A syntactically valid command line from Jetson keeps the link alive.
    lastCommandMillis = millis();

    if (cmd == "STOP") {
        engageStopHold();
        Serial.println("[JETSON] STOP received -> hold enabled");

    } else if (cmd == "FORWARD" || cmd == "AUTONOMOUS" || cmd == "RESUME") {
        clearStopHold();
        Serial.println("[JETSON] FORWARD received -> hold cleared");

    } else if (cmd == "MANUAL") {
        Serial.println("[JETSON] MANUAL ignored (Jetson is limited to STOP/FORWARD)");

    } else {
        Serial.printf("[JETSON] Unsupported cmd: %s\n", cmd.c_str());
    }

    commandPending = false;
    clearRxBuffer();
}

void JetsonUART::sendStatus() {
    const uint32_t now = millis();
    if ((now - lastStatusMillis) < JETSON_STATUS_INTERVAL_MS) {
        return;
    }
    lastStatusMillis = now;

    StaticJsonDocument<JETSON_JSON_BUFFER_SIZE> statusDoc;
    statusDoc["type"] = "STATUS";

    const OperationMode mode = UltrasonicSensor::getMode();
    statusDoc["mode"] = (mode == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL";
    statusDoc["jetson_stop_hold"] = stopHoldActive;
    statusDoc["speed"] = MotorController::getCurrentSpeed();
    statusDoc["throttle"] = MotorController::getTargetSpeed();
    statusDoc["steering_angle"] = MotorController::getSteerServoAngle();
    statusDoc["battery_voltage"] = readBatteryVoltage();

    const GpsData gpsData = GPSSensor::getData();
    JsonObject gps = statusDoc.createNestedObject("gps");
    gps["lat"] = gpsData.fix ? gpsData.lat : 0.0;
    gps["lon"] = gpsData.fix ? gpsData.lng : 0.0;
    gps["heading"] = gpsData.course_deg;

    JsonObject sensors = statusDoc.createNestedObject("sensors");
    sensors["front"] = UltrasonicSensor::getFrontDistance();
    sensors["left"] = UltrasonicSensor::getLeftDistance();
    sensors["right"] = UltrasonicSensor::getRightDistance();
    sensors["back"] = UltrasonicSensor::getBackDistance();

    statusDoc["timestamp"] = now;

    String payload;
    const size_t bytes = serializeJson(statusDoc, payload);
    if (bytes == 0) {
        Serial.println("[JETSON] Failed to serialize STATUS payload");
        return;
    }

    Serial1.println(payload);
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
