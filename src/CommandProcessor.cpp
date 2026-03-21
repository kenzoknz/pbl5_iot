/**
 * CommandProcessor.cpp
 * Triển khai xử lý lệnh Web Server → ESP32.
 */

#include "CommandProcessor.h"

// ── Static member definitions ──────────────────────────────────
OperationMode CommandProcessor::_mode          = AUTONOMOUS;
uint32_t      CommandProcessor::_lastPollTime  = 0;
uint32_t      CommandProcessor::_lastStatusTime = 0;
uint32_t      CommandProcessor::_lastModePollTime = 0;
uint32_t      CommandProcessor::_lastJoystickInputTime = 0;
bool          CommandProcessor::_isHandlingWsMessage = false;
bool          CommandProcessor::_joystickDriveActive = false;

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
    if (_mode != MANUAL) return;
    if (!_joystickDriveActive) return;

    if (millis() - _lastJoystickInputTime > JOYSTICK_WATCHDOG_TIMEOUT_MS) {
        MotorController::stopMotor();
        MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
        MotorController::setTargetSpeed(0);
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

    } else if (type == "WELCOME") {
        Serial.printf("[WS] Server: %s\n",
                      doc["data"]["message"].as<const char*>());

    } else {
        Serial.printf("[WS] Bỏ qua type không xử lý: %s\n", type.c_str());
    }
}

// ══════════════════════════════════════════
//  Status heartbeat
// ══════════════════════════════════════════

void CommandProcessor::sendStatusUpdate() {
    if (millis() - _lastStatusTime < STATUS_INTERVAL_MS) return;
    _lastStatusTime = millis();

    StaticJsonDocument<256> doc;
    doc["type"] = "STATUS";
    JsonObject data = doc.createNestedObject("data");
    data["mode"]    = (_mode == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL";
    data["state"]   = (int)VehicleStateMachine::getCurrentState();
    data["rssi"]    = WiFi.RSSI();
    data["uptime"]  = millis() / 1000;

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
        /*
         * Tham số: direction (FORWARD|BACKWARD|LEFT|RIGHT), speed (0-255), duration_ms
         * Ví dụ: { "direction": "FORWARD", "speed": 120, "duration_ms": 1000 }
         */
        String dir      = params["direction"]   | "FORWARD";
        int    speed    = params["speed"]        | CRUISE_SPEED;
        int    duration = params["duration_ms"]  | 0;

        speed = constrain(speed, 0, 255);

        if (dir == "FORWARD") {
            MotorController::setTargetSpeed(speed);
            MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
            MotorController::moveDifferential(speed, speed);
            Serial.printf("[CMD] MOVE FORWARD speed=%d\n", speed);

        } else if (dir == "BACKWARD") {
            MotorController::setTargetSpeed(-speed);
            MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
            MotorController::moveDifferential(-speed, -speed);
            Serial.printf("[CMD] MOVE BACKWARD speed=%d\n", speed);

        } else if (dir == "LEFT") {
            MotorController::setTargetSpeed(speed);
            MotorController::setTargetSteerServoAngle(SERVO_LEFT_MAX);
            MotorController::moveDifferential(speed, speed);
            Serial.printf("[CMD] MOVE LEFT speed=%d\n", speed);

        } else if (dir == "RIGHT") {
            MotorController::setTargetSpeed(speed);
            MotorController::setTargetSteerServoAngle(SERVO_RIGHT_MAX);
            MotorController::moveDifferential(speed, speed);
            Serial.printf("[CMD] MOVE RIGHT speed=%d\n", speed);

        } else {
            Serial.printf("[CMD] MOVE: direction không hợp lệ '%s'\n", dir.c_str());
            return;
        }

        // Nếu có duration — chạy rồi tự dừng
        if (duration > 0) {
            vTaskDelay(pdMS_TO_TICKS(duration));
            MotorController::stopMotor();
            MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
            Serial.printf("[CMD] MOVE kết thúc sau %d ms\n", duration);
        }

    } else if (command == "STOP") {
        /*
         * Tham số: không cần
         * Dừng ngay lập tức, trả servo về thẳng
         */
        MotorController::stopMotor();
        MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
        MotorController::setTargetSpeed(0);
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

        if (dir == "LEFT") {
            int servoAngle = constrain(SERVO_STRAIGHT - angle, SERVO_LEFT_MAX, SERVO_STRAIGHT);
            MotorController::setTargetSteerServoAngle(servoAngle);
            Serial.printf("[CMD] TURN LEFT angle=%d → servo=%d\n", angle, servoAngle);

        } else if (dir == "RIGHT") {
            int servoAngle = constrain(SERVO_STRAIGHT + angle, SERVO_STRAIGHT, SERVO_RIGHT_MAX);
            MotorController::setTargetSteerServoAngle(servoAngle);
            Serial.printf("[CMD] TURN RIGHT angle=%d → servo=%d\n", angle, servoAngle);

        } else {
            Serial.printf("[CMD] TURN: direction không hợp lệ '%s'\n", dir.c_str());
        }

    } else if (command == "SET_SPEED") {
        /*
         * Tham số: speed (0-255)
         * Ví dụ: { "speed": 150 }
         */
        int speed = params["speed"] | CRUISE_SPEED;
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
            MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
            MotorController::setTargetSpeed(0);
            _joystickDriveActive = false;
        } else {
            // Đặt servo lái
            int servoAngle = constrain(SERVO_STRAIGHT + angle, SERVO_LEFT_MAX, SERVO_RIGHT_MAX);
            MotorController::setTargetSteerServoAngle(servoAngle);
            // Điều khiển motor
            MotorController::setTargetSpeed(spd);
            MotorController::moveDifferential(spd, spd);
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
        MotorController::stopMotor();
        MotorController::setTargetSteerServoAngle(SERVO_STRAIGHT);
        MotorController::setTargetSpeed(0);
        _lastJoystickInputTime = millis();
        _joystickDriveActive = false;
        Serial.println("[CMD] ══ Chuyển sang MANUAL ══");
        if (!_isHandlingWsMessage) {
            sendLog("mode_change", "switched to MANUAL");
        }
    }

    // Thông báo lên server qua WebSocket
    StaticJsonDocument<128> doc;
    doc["type"]           = "STATUS";
    doc["data"]["mode"]   = (mode == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL";
    doc["data"]["state"]  = (int)VehicleStateMachine::getCurrentState();
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
