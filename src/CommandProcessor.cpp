/**
 * CommandProcessor.cpp
 * Triển khai xử lý lệnh Web Server → ESP32.
 */

#include "CommandProcessor.h"
#include "GPSSensor.h"

namespace {
void appendConfigToJson(JsonObject obj, const RobotConfig& cfg) {
    obj["minRunSpeed"] = cfg.minRunSpeed;
    obj["cruiseSpeed"] = cfg.cruiseSpeed;
    obj["fastSpeed"] = cfg.fastSpeed;
    obj["backSpeed"] = cfg.backSpeed;
    obj["escapeSpeed"] = cfg.escapeSpeed;
    obj["sharpTurnBoost"] = cfg.sharpTurnBoost;
    obj["mediumTurnBoost"] = cfg.mediumTurnBoost;
    obj["turnBoost"] = cfg.turnBoost;
    obj["lightTurnBoost"] = cfg.lightTurnBoost;

    obj["emergencyDist"] = cfg.emergencyDist;
    obj["stopDistance"] = cfg.stopDistance;
    obj["slowDistance"] = cfg.slowDistance;
    obj["turnDistance"] = cfg.turnDistance;
    obj["prepareDistance"] = cfg.prepareDistance;
    obj["sideDangerDist"] = cfg.sideDangerDist;
    obj["backDangerDistance"] = cfg.backDangerDistance;
    obj["directionHysteresis"] = cfg.directionHysteresis;
}

const char* stateToName(State state) {
    switch (state) {
        case INIT: return "INIT";
        case NORMAL: return "NORMAL";
        case SLOW: return "SLOW";
        case AVOID_LEFT: return "AVOID_LEFT";
        case AVOID_RIGHT: return "AVOID_RIGHT";
        case TURN_LEFT: return "TURN_LEFT";
        case TURN_RIGHT: return "TURN_RIGHT";
        case BACKING: return "BACKING";
        case STOP: return "STOP";
        case EMERGENCY: return "EMERGENCY";
        case MANUAL_CONTROL: return "MANUAL";
        case ESCAPE: return "ESCAPE";
        default: return "UNKNOWN";
    }
}
}

// ── Static member definitions ──────────────────────────────────
OperationMode CommandProcessor::_mode          = AUTONOMOUS;
uint32_t      CommandProcessor::_lastPollTime  = 0;
uint32_t      CommandProcessor::_lastStatusTime = 0;
uint32_t      CommandProcessor::_lastModePollTime = 0;
uint32_t      CommandProcessor::_lastJoystickInputTime = 0;
bool          CommandProcessor::_isHandlingWsMessage = false;
bool          CommandProcessor::_joystickDriveActive = false;
uint32_t      CommandProcessor::_lastQueueFlushTime = 0;
bool          CommandProcessor::_queueFlushInProgress = false;
bool          CommandProcessor::_timedMoveActive = false;
uint32_t      CommandProcessor::_timedMoveEndTime = 0;

// ══════════════════════════════════════════
//  Init
// ══════════════════════════════════════════

void CommandProcessor::begin() {
    _mode = AUTONOMOUS;
    _joystickDriveActive = false;
    _lastJoystickInputTime = millis();
    UltrasonicSensor::setMode(AUTONOMOUS);
    Serial.println("[CMD] Khởi động — Chế độ: AUTONOMOUS");
    sendLog("system_start", "ESP32 online. Mode: AUTONOMOUS");
}

void CommandProcessor::tickSafety() {
    if (_timedMoveActive && millis() >= _timedMoveEndTime) {
        MotorController::setTargetSpeed(0);
        MotorController::setTargetSteerServoAngle(SERVO_CENTER);
        _timedMoveActive = false;
        Serial.println("[CMD] Timed MOVE ended -> STOP");
    }
    if (_mode != MANUAL) return;
    if (!_joystickDriveActive) return;

    if (millis() - _lastJoystickInputTime > JOYSTICK_WATCHDOG_TIMEOUT_MS) {
        MotorController::stopMotor();
        MotorController::setTargetSpeed(0);
        MotorController::setTargetSteerServoAngle(SERVO_CENTER);
        _joystickDriveActive = false;
        Serial.println("[CMD] Joystick timeout -> STOP for safety");
    }
}

// ══════════════════════════════════════════
//  HTTP Polling (fallback khi WS ngắt)
// ══════════════════════════════════════════

void CommandProcessor::pollCommands() {
    uint32_t interval = (_mode == MANUAL) ? POLL_INTERVAL_MANUAL : POLL_INTERVAL_AUTO;
    if (millis() - _lastPollTime < interval) return;
    _lastPollTime = millis();

    Serial.println("[CMD] Polling HTTP /api/commands/pending/all ...");
    String response = NetworkManager::httpGet("/api/commands/pending/all");
    if (response.isEmpty()) return;

    // Parse JSON — capacity 4096 để chứa nhiều lệnh cùng lúc
    DynamicJsonDocument doc(4096);
    DeserializationError err = deserializeJson(doc, response);
    if (err) {
        Serial.printf("[CMD] Lỗi JSON: %s\n", err.c_str());
        return;
    }

    if (!doc["success"].as<bool>()) {
        Serial.println("[CMD] Server trả về success=false");
        return;
    }

    JsonArray data = doc["data"].as<JsonArray>();
    Serial.printf("[CMD] Nhận được %u lệnh pending\n", data.size());

    for (JsonObject cmd : data) {
        processCommand(cmd);
    }
}

// ══════════════════════════════════════════
//  HTTP Mode Polling (fallback khi WS ngắt)
// ══════════════════════════════════════════

void CommandProcessor::pollMode() {
    if (millis() - _lastModePollTime < MODE_POLL_INTERVAL_MS) return;
    _lastModePollTime = millis();

    Serial.println("[CMD] Polling HTTP /api/robot/mode ...");
    String response = NetworkManager::httpGet("/api/robot/mode");
    if (response.isEmpty()) return;

    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, response);
    if (err) {
        Serial.printf("[CMD] Lỗi JSON mode poll: %s\n", err.c_str());
        return;
    }

    if (!doc["success"].as<bool>()) {
        Serial.println("[CMD] Mode poll: server trả về success=false");
        return;
    }

    String modeStr = doc["data"]["mode"] | "";
    Serial.printf("[CMD] Mode poll: server mode = %s, current mode = %s\n", 
                  modeStr.c_str(), 
                  (_mode == AUTONOMOUS ? "AUTONOMOUS" : "MANUAL"));

    // Sync mode nếu khác
    if (modeStr == "AUTONOMOUS" && _mode != AUTONOMOUS) {
        Serial.println("[CMD] 🔄 Mode sync: AUTONOMOUS (từ DB)");
        setMode(AUTONOMOUS);
    } else if (modeStr == "MANUAL" && _mode != MANUAL) {
        Serial.println("[CMD] 🔄 Mode sync: MANUAL (từ DB)");
        setMode(MANUAL);
    }
}

// ══════════════════════════════════════════
//  WebSocket message handler
// ══════════════════════════════════════════

void CommandProcessor::handleWsMessage(const String& message) {
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, message);
    if (err) {
        Serial.printf("[WS] Lỗi parse message: %s\n", err.c_str());
        return;
    }

    struct WsHandlerGuard {
        explicit WsHandlerGuard(bool& f) : flag(f) { flag = true; }
        ~WsHandlerGuard() { flag = false; }
        bool& flag;
    } guard(_isHandlingWsMessage);

    String type = doc["type"] | "";
    Serial.printf("[WS] Nhận type=%s\n", type.c_str());

    if (type == "COMMAND") {
        // Server push lệnh trực tiếp. Hỗ trợ cả 2 dạng:
        // 1) { "type": "COMMAND", "data": { "id": -1, "command": "...", "parameters": {...} } }
        // 2) { "type": "COMMAND", "data": { "id": -1, "data": { "command": "...", "parameters": {...} } } }
        JsonVariant dataVar = doc["data"];
        JsonObject cmd = dataVar.as<JsonObject>();

        // Fallback cho payload bị bọc thêm một lớp data.
        if ((!cmd.containsKey("command") || cmd["command"].isNull()) && cmd.containsKey("data")) {
            JsonObject nested = cmd["data"].as<JsonObject>();
            if (!nested.isNull()) {
                cmd = nested;
            }
        }

        processCommand(cmd);

    } else if (type == "MODE_CHANGE") {
        // Server yêu cầu đổi mode: { "type": "MODE_CHANGE", "data": {"mode": "MANUAL"} }
        String modeStr = doc["data"]["mode"] | "";
        Serial.printf("[WS] ▶ MODE_CHANGE request: %s\n", modeStr.c_str());
        
        if      (modeStr == "AUTONOMOUS") setMode(AUTONOMOUS);
        else if (modeStr == "MANUAL")     setMode(MANUAL);
        else Serial.printf("[WS] ⚠ Mode không hợp lệ: %s\n", modeStr.c_str());

    } else if (type == "PING") {
        NetworkManager::wsSend("{\"type\":\"PONG\"}");

    } else if (type == "CONFIG_GET") {
        sendConfigCurrent();

    } else if (type == "CONFIG_UPDATE") {
        RobotConfig newConfig = ConfigStorage::getDefaults();
        String errorField;
        String errorMessage;

        if (!parseRobotConfig(doc["data"], newConfig, errorField, errorMessage)) {
            StaticJsonDocument<320> ack;
            ack["type"] = "CONFIG_UPDATE_ACK";
            JsonObject data = ack.createNestedObject("data");
            data["success"] = false;
            data["message"] = errorMessage;
            data["field"] = errorField;

            String out;
            serializeJson(ack, out);
            NetworkManager::wsSend(out);
            return;
        }

        if (!ConfigStorage::save(newConfig)) {
            StaticJsonDocument<192> ack;
            ack["type"] = "CONFIG_UPDATE_ACK";
            ack["data"]["success"] = false;
            ack["data"]["message"] = "Failed to save config to NVS";

            String out;
            serializeJson(ack, out);
            NetworkManager::wsSend(out);
            return;
        }

        VehicleStateMachine::applyConfig(newConfig);
        MotorController::applyConfig(newConfig);

        sendLog("config_updated", "Robot config updated and saved to flash");

        StaticJsonDocument<512> ack;
        ack["type"] = "CONFIG_UPDATE_ACK";
        JsonObject data = ack.createNestedObject("data");
        data["success"] = true;
        data["message"] = "Config saved to NVS";
        data["source"] = "nvs";
        appendConfigToJson(data.createNestedObject("appliedConfig"), newConfig);

        String out;
        serializeJson(ack, out);
        NetworkManager::wsSend(out);

        sendConfigCurrent("runtime");

    } else if (type == "CONFIG_RESET") {
        bool resetOk = ConfigStorage::reset();
        RobotConfig defaults = ConfigStorage::getCurrent();
        VehicleStateMachine::applyConfig(defaults);
        MotorController::applyConfig(defaults);

        StaticJsonDocument<512> ack;
        ack["type"] = "CONFIG_UPDATE_ACK";
        JsonObject data = ack.createNestedObject("data");
        data["success"] = resetOk;
        data["action"] = "reset";
        data["source"] = "default";
        data["message"] = resetOk ? "Config reset to defaults" : "Failed to reset config";
        appendConfigToJson(data.createNestedObject("appliedConfig"), defaults);

        String out;
        serializeJson(ack, out);
        NetworkManager::wsSend(out);

        if (resetOk) {
            sendLog("config_reset", "Robot config reset to defaults");
            sendConfigCurrent("default");
        }

    } else if (type == "WELCOME") {
        Serial.printf("[WS] Server: %s\n",
                      doc["data"]["message"].as<const char*>());

    } else {
        Serial.printf("[WS] Bỏ qua type không xử lý: %s\n", type.c_str());
    }
}

bool CommandProcessor::readBoundedUInt16(const JsonVariantConst& data, const char* key,
                                         uint16_t minValue, uint16_t maxValue,
                                         uint16_t& outValue, String& errorField, String& errorMessage) {
    if (data.isNull() || !data[key].is<long>()) {
        errorField = key;
        errorMessage = String("Missing or invalid integer field: ") + key;
        return false;
    }

    long raw = data[key].as<long>();
    if (raw < minValue || raw > maxValue) {
        errorField = key;
        errorMessage = String(key) + " out of range [" + String(minValue) + "-" + String(maxValue) + "]";
        return false;
    }

    outValue = static_cast<uint16_t>(raw);
    return true;
}

bool CommandProcessor::parseRobotConfig(const JsonVariantConst& data, RobotConfig& out,
                                        String& errorField, String& errorMessage) {
    if (!readBoundedUInt16(data, "minRunSpeed", 0, 255, out.minRunSpeed, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "cruiseSpeed", 0, 255, out.cruiseSpeed, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "fastSpeed", 0, 255, out.fastSpeed, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "backSpeed", 0, 255, out.backSpeed, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "escapeSpeed", 0, 255, out.escapeSpeed, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "sharpTurnBoost", 0, 255, out.sharpTurnBoost, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "mediumTurnBoost", 0, 255, out.mediumTurnBoost, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "turnBoost", 0, 255, out.turnBoost, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "lightTurnBoost", 0, 255, out.lightTurnBoost, errorField, errorMessage)) return false;

    if (!readBoundedUInt16(data, "emergencyDist", 10, 200, out.emergencyDist, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "stopDistance", 10, 200, out.stopDistance, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "slowDistance", 10, 200, out.slowDistance, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "turnDistance", 10, 200, out.turnDistance, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "prepareDistance", 10, 200, out.prepareDistance, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "sideDangerDist", 10, 200, out.sideDangerDist, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "backDangerDistance", 10, 200, out.backDangerDistance, errorField, errorMessage)) return false;
    if (!readBoundedUInt16(data, "directionHysteresis", 1, 50, out.directionHysteresis, errorField, errorMessage)) return false;

    if (!ConfigStorage::isValidConfig(out)) {
        errorField = "config";
        errorMessage = "Invalid config consistency";
        return false;
    }

    return true;
}

void CommandProcessor::sendConfigCurrent(const char* source) {
    const RobotConfig cfg = ConfigStorage::getCurrent();
    StaticJsonDocument<512> doc;
    doc["type"] = "CONFIG_CURRENT";
    JsonObject data = doc.createNestedObject("data");
    appendConfigToJson(data, cfg);
    data["source"] = source ? source : (ConfigStorage::isCurrentFromNvs() ? "nvs" : "default");
    data["uptime_ms"] = millis();

    String out;
    serializeJson(doc, out);
    NetworkManager::wsSend(out);
}

// ══════════════════════════════════════════
//  Status heartbeat
// ══════════════════════════════════════════

void CommandProcessor::sendStatusUpdate() {
    if (millis() - _lastStatusTime < STATUS_INTERVAL_MS) return;
    _lastStatusTime = millis();

    StaticJsonDocument<640> doc;
    doc["type"] = "STATUS";
    JsonObject data = doc.createNestedObject("data");
    State currentState = VehicleStateMachine::getCurrentState();
    data["mode"]    = (_mode == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL";
    data["state"]   = stateToName(currentState);
    data["stateCode"] = (int)currentState;
    data["rssi"]    = WiFi.RSSI();
    data["uptime"]  = millis() / 1000;

    GpsData gpsData = GPSSensor::getData();
    JsonObject gps = data.createNestedObject("gps");
    gps["fix"] = gpsData.fix;
    if (gpsData.fix) {
        gps["lat"] = gpsData.lat;
        gps["lng"] = gpsData.lng;
    } else {
        gps["lat"] = nullptr;
        gps["lng"] = nullptr;
    }
    if (gpsData.preview_available) {
        gps["preview_lat"] = gpsData.preview_lat;
        gps["preview_lng"] = gpsData.preview_lng;
    } else {
        gps["preview_lat"] = nullptr;
        gps["preview_lng"] = nullptr;
    }
    gps["altitude_m"] = gpsData.altitude_m;
    gps["speed_kmh"] = gpsData.speed_kmh;
    gps["course_deg"] = gpsData.course_deg;
    gps["satellites"] = gpsData.satellites;
    gps["hdop"] = gpsData.hdop;
    if (gpsData.gps_time_utc[0] != '\0') {
        gps["gps_time_utc"] = gpsData.gps_time_utc;
    } else {
        gps["gps_time_utc"] = nullptr;
    }

    String msg;
    serializeJson(doc, msg);
    NetworkManager::wsSend(msg);
}

// ══════════════════════════════════════════
//  Core: xử lý 1 lệnh
// ══════════════════════════════════════════

void CommandProcessor::processCommand(const JsonObject& cmd) {
    if (cmd.isNull()) {
        Serial.println("[CMD] processCommand: null object, bỏ qua");
        return;
    }

    int    id      = cmd["id"]      | -1;
    String command = cmd["command"] | "";

    if (command.isEmpty()) {
        Serial.println("[CMD] Lệnh thiếu command, bỏ qua");
        return;
    }

    if (id < 0) {
        Serial.printf("[CMD] ▶ realtime command=%s (no id)\n", command.c_str());
    } else {
        Serial.printf("[CMD] ▶ id=%d  command=%s\n", id, command.c_str());
    }

    // ── Safety: Từ chối lệnh chuyển động khi đang EMERGENCY ──────
    State currentState = VehicleStateMachine::getCurrentState();
    if (currentState == EMERGENCY) {
        bool isSafe = (command == "SET_MODE" || command == "STOP" || command == "SET_WIFI");
        if (!isSafe) {
            String msg = "id=" + String(id) + " REJECTED (EMERGENCY state)";
            Serial.println("[CMD] ⚠ " + msg);
            sendLog("command_rejected", msg);
            markExecuted(id);
            return;
        }
    }

    // ── Lấy parameters (optional — có thể null) ──────────────────
    // Kiểm tra tồn tại trước khi dùng
    JsonObject params;
    if (cmd.containsKey("parameters") && !cmd["parameters"].isNull()) {
        params = cmd["parameters"].as<JsonObject>();
    }

    // ── Dispatch ─────────────────────────────────────────────────
    if (command == "SET_MODE") {
        String modeStr = params["mode"] | "";
        if      (modeStr == "AUTONOMOUS") setMode(AUTONOMOUS);
        else if (modeStr == "MANUAL")     setMode(MANUAL);
        else {
            Serial.printf("[CMD] SET_MODE: mode không hợp lệ '%s'\n", modeStr.c_str());
        }

    } else if (command == "SET_WIFI") {
        String ssid = params["ssid"] | "";
        String pass = params["password"] | "";

        if (ssid.isEmpty()) {
            Serial.println("[CMD] SET_WIFI: thiếu ssid");
            sendLog("wifi_config_failed", "missing ssid");
        } else {
            bool ok = NetworkManager::setWiFiCredentials(ssid, pass, true);
            if (ok) {
                sendLog("wifi_config", "wifi updated to ssid=" + ssid);
            } else {
                sendLog("wifi_config_failed", "failed to apply ssid=" + ssid);
            }
        }

    } else {
        // Tất cả lệnh chuyển động yêu cầu mode MANUAL
        if (_mode != MANUAL) {
            String msg = "id=" + String(id) + " IGNORED (không ở MANUAL mode)";
            Serial.println("[CMD] " + msg);
            sendLog("command_ignored", msg);
            markExecuted(id);
            return;
        }
        applyManualCommand(command, params);

        if (command == "STOP") {
            _joystickDriveActive = false;
        }
    }

    // ── Đánh dấu đã xử lý và ghi log ────────────────────────────
    // id < 0 = realtime DIRECT_COMMAND từ browser — không có bản ghi DB
    if (id >= 0) {
        markExecuted(id);
        sendLog("command_executed", "id=" + String(id) + " " + command + " OK");
    }
}

// ══════════════════════════════════════════
//  Ánh xạ lệnh → MotorController + Servo
// ══════════════════════════════════════════

void CommandProcessor::applyManualCommand(const String& command, const JsonObject& params) {

    if (command == "MOVE") {
        String dir      = params["direction"]   | "FORWARD";
        int    speed    = params["speed"]        | (int)MotorController::getConfig().cruiseSpeed;
        int    duration = params["duration_ms"]  | 0;

        dir.toUpperCase();
        speed = constrain(speed, 0, 255);

        if (dir == "FORWARD") {
            MotorController::setTargetSpeed(speed);
            MotorController::setTargetSteerServoAngle(SERVO_CENTER);
            Serial.printf("[CMD] MOVE FORWARD speed=%d\n", speed);

        } else if (dir == "BACKWARD") {
            MotorController::setTargetSpeed(-speed);
            MotorController::setTargetSteerServoAngle(SERVO_CENTER);
            Serial.printf("[CMD] MOVE BACKWARD speed=%d\n", speed);

        } else if (dir == "LEFT") {
            MotorController::setTargetSpeed(speed);
            MotorController::setTargetSteerServoAngle(SERVO_LEFT_MAX);
            Serial.printf("[CMD] MOVE LEFT speed=%d\n", speed);

        } else if (dir == "RIGHT") {
            MotorController::setTargetSpeed(speed);
            MotorController::setTargetSteerServoAngle(SERVO_RIGHT_MAX);
            Serial.printf("[CMD] MOVE RIGHT speed=%d\n", speed);

        } else {
            Serial.printf("[CMD] MOVE: direction không hợp lệ '%s'\n", dir.c_str());
            return;
        }

        // Non-blocking duration
        if (duration > 0) {
            _timedMoveActive = true;
            _timedMoveEndTime = millis() + (uint32_t)duration;
        } else {
            _timedMoveActive = false;
        }
    } else if (command == "STOP") {
        /*
         * Tham số: không cần
         * Dừng ngay lập tức, trả servo về thẳng
         */
        MotorController::setTargetSpeed(0);
        MotorController::setTargetSteerServoAngle(SERVO_CENTER);
        MotorController::stopMotor();
        _timedMoveActive = false;
        Serial.println("[CMD] STOP");

    } else if (command == "TURN") {
        /*
         * Tham số: direction (LEFT|RIGHT), angle (độ, 1-45)
         * Ví dụ: { "direction": "LEFT", "angle": 30 }
         * Chỉ điều chỉnh góc servo — motor vẫn đang chạy (nếu có)
         */
        String dir   = params["direction"] | "LEFT";
        int    angle = params["angle"]     | 30;

        angle = constrain(angle, 0, 45);

        dir.toUpperCase();

        if (dir == "LEFT") {
            int servoAngle = constrain(SERVO_CENTER + angle, SERVO_CENTER, SERVO_LEFT_MAX);
            MotorController::setTargetSteerServoAngle(servoAngle);
            Serial.printf("[CMD] TURN LEFT angle=%d -> servo=%d\n", angle, servoAngle);

        } else if (dir == "RIGHT") {
            int servoAngle = constrain(SERVO_CENTER - angle, SERVO_RIGHT_MAX, SERVO_CENTER);
            MotorController::setTargetSteerServoAngle(servoAngle);
            Serial.printf("[CMD] TURN RIGHT angle=%d -> servo=%d\n", angle, servoAngle);

        } else {
            Serial.printf("[CMD] TURN: direction không hợp lệ '%s'\n", dir.c_str());
        }

    } else if (command == "SET_SPEED") {
        /*
         * Tham số: speed (0-255)
         * Ví dụ: { "speed": 150 }
         */
        int speed = params["speed"] | (int)MotorController::getConfig().cruiseSpeed;
        speed = constrain(speed, 0, 255);
        MotorController::setTargetSpeed(speed);
        Serial.printf("[CMD] SET_SPEED %d\n", speed);

    } else if (command == "JOYSTICK") {
        /*
         * Realtime joystick — gửi từ browser mỗi ~150ms qua WebSocket
         * Tham số: { "speed": -255..255, "steer": -45..45 }
         *   speed > 0 = tiến, speed < 0 = lùi
         *   steer > 0 = phải, steer < 0 = trái (degrees)
         */
        int spd   = params["speed"] | 0;
        int angle = params["steer"] | 0;

        _lastJoystickInputTime = millis();

        spd   = constrain(spd,   -255, 255);
        angle = constrain(angle,  -45,  45);

        if (abs(spd) < 12 && abs(angle) < 4) {
            // Deadzone — dừng hẳn
            MotorController::stopMotor();
            MotorController::setTargetSpeed(0);
            MotorController::setTargetSteerServoAngle(SERVO_CENTER);
            _joystickDriveActive = false;
        } else {
            // Quy ước joystick:
            // angle > 0: rẽ phải
            // angle < 0: rẽ trái
            int servoAngle = SERVO_CENTER;

            if (angle < 0) {
                servoAngle = constrain(SERVO_CENTER + abs(angle), SERVO_CENTER, SERVO_LEFT_MAX);
            } else if (angle > 0) {
                servoAngle = constrain(SERVO_CENTER - angle, SERVO_RIGHT_MAX, SERVO_CENTER);
            }

            MotorController::setTargetSteerServoAngle(servoAngle);
            MotorController::setTargetSpeed(spd);
            _joystickDriveActive = true;
        }

    } else {
        Serial.printf("[CMD] Lệnh không xử lý: %s\n", command.c_str());
    }
}

// ══════════════════════════════════════════
//  Mode switching
// ══════════════════════════════════════════

void CommandProcessor::setMode(OperationMode mode) {
    if (_mode == mode) {
        Serial.printf("[CMD] setMode: đã ở mode %s, bỏ qua\n",
                      mode == AUTONOMOUS ? "AUTONOMOUS" : "MANUAL");
        return;
    }

    _mode = mode;
    UltrasonicSensor::setMode(mode);  // Suspend/resume sensor tasks

    if (mode == AUTONOMOUS) {
        Serial.println("[CMD] ══ Chuyển sang AUTONOMOUS ══");
        // [FIX] Reset trap counters khi chuyển sang AUTO để tránh trigger ESCAPE ngay lập tức
        VehicleStateMachine::resetTrapCounters();
        if (!_isHandlingWsMessage) {
            sendLog("mode_change", "switched to AUTONOMOUS");
        }
        _joystickDriveActive = false;
    } else {
        // Dừng motor an toàn trước khi nhận lệnh thủ công
        MotorController::setTargetSpeed(0);
        MotorController::setTargetSteerServoAngle(SERVO_CENTER);
        MotorController::stopMotor();
        _lastJoystickInputTime = millis();
        _joystickDriveActive = false;
        Serial.println("[CMD] ══ Chuyển sang MANUAL ══");
        if (!_isHandlingWsMessage) {
            sendLog("mode_change", "switched to MANUAL");
        }
    }

    // Thông báo lên server qua WebSocket
    StaticJsonDocument<128> doc;
    State currentState = VehicleStateMachine::getCurrentState();
    doc["type"]           = "STATUS";
    doc["data"]["mode"]   = (mode == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL";
    doc["data"]["state"]  = stateToName(currentState);
    doc["data"]["stateCode"] = (int)currentState;
    String msg;
    serializeJson(doc, msg);
    NetworkManager::wsSend(msg);
}

// ══════════════════════════════════════════
//  HTTP helpers
// ══════════════════════════════════════════

void CommandProcessor::markExecuted(int id) {
    String path = "/api/commands/" + String(id) + "/execute";
    int code = NetworkManager::httpPut(path);
    if (code == 200) {
        Serial.printf("[CMD] ✓ Đã đánh dấu id=%d executed\n", id);
    } else {
        Serial.printf("[CMD] ✗ markExecuted thất bại id=%d code=%d\n", id, code);
    }
}

void CommandProcessor::sendLog(const String& event, const String& message) {
    NetworkManager::wsSendSerialLog("INFO", event + ": " + message);

    StaticJsonDocument<256> doc;
    doc["event"]   = event;
    doc["message"] = message;
    String body;
    serializeJson(doc, body);

    int code = NetworkManager::httpPost("/api/logs", body);
    if (code != 201) {
        Serial.printf("[CMD] sendLog thất bại event=%s code=%d\n", event.c_str(), code);
    }
}

void CommandProcessor::tickQueueFlush() {
    // Chỉ flush khi WS connected (có mạng)
    if (!NetworkManager::wsConnected()) return;
    
    // Chỉ xử lý 1 batch / 500ms để không quá tải
    if (millis() - _lastQueueFlushTime < GPS_QUEUE_FLUSH_INTERVAL) return;
    _lastQueueFlushTime = millis();
    
    if (_queueFlushInProgress) return; // Chờ batch trước xong
    
    uint16_t queueSize = GpsQueue::size();
    if (queueSize == 0) return;
    
    // Lấy batch
    const uint8_t BATCH_SIZE = GPS_QUEUE_BATCH_SIZE;
    GpsQueueEntry batch[BATCH_SIZE];
    uint8_t batchCount = 0;
    
    for (uint8_t i = 0; i < BATCH_SIZE && GpsQueue::size() > 0; i++) {
        if (GpsQueue::dequeue(batch[i])) {
            batchCount++;
        }
    }
    
    if (batchCount == 0) return;
    
    _queueFlushInProgress = true;
    Serial.printf("[CMD] Flushing %d GPS points from queue...\n", batchCount);
    
    // Tạo JSON array
    StaticJsonDocument<2048> doc;
    JsonArray gpsArray = doc.createNestedArray("gps_log");
    
    for (uint8_t i = 0; i < batchCount; i++) {
        JsonObject obj = gpsArray.createNestedObject();
        obj["lat"] = batch[i].lat;
        obj["lng"] = batch[i].lng;
        obj["altitude_m"] = batch[i].altitude_m;
        obj["speed_kmh"] = batch[i].speed_kmh;
        obj["course_deg"] = batch[i].course_deg;
        obj["satellites"] = batch[i].satellites;
        obj["hdop"] = batch[i].hdop;
        obj["fix"] = batch[i].fix;
        
        // ISO time from GPS
        char timeStr[32];
        snprintf(timeStr, sizeof(timeStr), 
                 "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 batch[i].year, batch[i].month, batch[i].day,
                 batch[i].hour, batch[i].minute, batch[i].second);
        obj["gps_time_utc"] = timeStr;
    }
    
    String body;
    serializeJson(doc, body);
    
    // POST
    int code = NetworkManager::httpPost("/api/gps/batch", body);
    _queueFlushInProgress = false;
    
    if (code == 201 || code == 200) {
        Serial.printf("[CMD] ✓ Queue flush OK (sent %d points)\n", batchCount);
    } else {
        Serial.printf("[CMD] ✗ Queue flush failed code=%d, points discarded\n", code);
    }
}
