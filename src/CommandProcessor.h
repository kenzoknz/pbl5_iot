#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

/**
 * CommandProcessor.h
 * Xử lý lệnh điều khiển nhận từ Web Server (qua WebSocket hoặc HTTP polling).
 *
 * Luồng:
 *   WebSocket (realtime) → handleWsMessage()  ─┐
 *   HTTP polling (fallback) → pollCommands()  ─┴→ processCommand() → applyManualCommand()
 *                                                                   → markExecuted() + sendLog()
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "NetworkManager.h"
#include "UltrasonicSensor.h"
#include "MotorController.h"
#include "VehicleStateMachine.h"
#include "GpsQueue.h"

// ══════════════════════════════════════════
//  Góc servo lái — chỉnh theo xe thực tế
// ══════════════════════════════════════════
#define SERVO_STRAIGHT    90    // Độ — đi thẳng
#define SERVO_LEFT_MAX    55    // Độ — rẽ trái mạnh nhất
#define SERVO_RIGHT_MAX  125    // Độ — rẽ phải mạnh nhất

// Tần suất gửi status update qua WebSocket (ms)
#define STATUS_INTERVAL_MS  1000

// Tần suất poll mode từ HTTP (ms) - fallback khi WS ngắt
#define MODE_POLL_INTERVAL_MS  3000

// Timeout joystick realtime (ms): quá thời gian này mà không có JOYSTICK mới thì dừng xe
#define JOYSTICK_WATCHDOG_TIMEOUT_MS  400

// ══════════════════════════════════════════
class CommandProcessor {
public:
    /**
     * Gọi một lần trong AppTask trước vòng lặp chính.
     * Khởi tạo chế độ mặc định (AUTONOMOUS).
     */
    static void begin();

    /**
     * Polling lệnh từ HTTP GET /api/commands/pending/all
     * Chỉ thực sự gọi server khi đến interval (non-blocking).
     * Dùng làm fallback khi WebSocket ngắt kết nối.
     */
    static void pollCommands();

    /**
     * Polling mode từ HTTP GET /api/robot/mode
     * Sync mode với database khi WebSocket không hoạt động.
     * Tự throttle — chỉ poll mỗi MODE_POLL_INTERVAL_MS.
     */
    static void pollMode();

    /**
     * Xử lý message JSON nhận trực tiếp từ WebSocket.
     * Giao thức: { "type": "COMMAND" | "MODE_CHANGE" | "PING", "data": {...} }
     */
    static void handleWsMessage(const String& message);

    /**
     * Gửi trạng thái robot (mode, state, RSSI, uptime) lên server qua WebSocket.
     * Tự throttle — chỉ gửi mỗi STATUS_INTERVAL_MS.
     */
    static void sendStatusUpdate();

    /**
     * Watchdog an toàn cho MANUAL joystick:
     * nếu đang chạy bằng joystick mà mất cập nhật quá JOYSTICK_WATCHDOG_TIMEOUT_MS
     * thì tự dừng xe.
     */
    static void tickSafety();

    /**
     * Flush GPS queue khi có mạng lại.
     * Gửi batch điểm GPS qua HTTP POST /api/gps/batch.
     * Tự throttle — chỉ flush mỗi GPS_QUEUE_FLUSH_INTERVAL.
     */
    static void tickQueueFlush();

    static OperationMode getCurrentMode() { return _mode; }

private:
    static OperationMode _mode;
    static uint32_t      _lastPollTime;
    static uint32_t      _lastStatusTime;
    static uint32_t      _lastModePollTime;
    static uint32_t      _lastJoystickInputTime;
    static bool          _isHandlingWsMessage;
    static bool          _joystickDriveActive;
    static uint32_t      _lastQueueFlushTime;
    static bool          _queueFlushInProgress;

    // ── Core ──
    static void processCommand(const JsonObject& cmd);
    static void setMode(OperationMode mode);

    /**
     * Ánh xạ lệnh di chuyển sang MotorController + Servo.
     * @param command  Tên lệnh: "MOVE" | "STOP" | "TURN" | "SET_SPEED"
     * @param params   JSON object chứa tham số (có thể null/empty)
     */
    static void applyManualCommand(const String& command, const JsonObject& params);

    // ── HTTP helpers ──
    static void markExecuted(int id);
    static void sendLog(const String& event, const String& message);
};

#endif // COMMAND_PROCESSOR_H
