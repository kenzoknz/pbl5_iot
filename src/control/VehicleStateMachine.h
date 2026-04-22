#ifndef VEHICLE_STATE_MACHINE_H
#define VEHICLE_STATE_MACHINE_H

#include "core/Config.h"
#include "core/ConfigStorage.h"

class VehicleStateMachine {
public:
    static void begin();
    static void update();
    static void debugOutput();
    static void applyConfig(const RobotConfig& cfg);
    static RobotConfig getConfig() { return runtimeConfig; }
    static State getCurrentState() { 
        return currentState; }
    
    // [FIX] Public để CommandProcessor có thể reset khi đổi mode
    static void resetTrapCounters();

private:
    static State currentState;
    static unsigned long stateStartTime;
    static unsigned long lastDebugTime; 
    static bool turnRight;  // Hướng rẽ cuối cùng được chọn

    // ══════ [MỚI] Anti-Trap Counters ══════
    static int   oscillationCount;       // Đếm số lần đổi hướng liên tiếp (LEFT↔RIGHT)
    static int   backingCount;           // Đếm số lần BACKING liên tục
    static State lastTurnDirection;      // Hướng rẽ trước đó (TURN_LEFT / TURN_RIGHT)
    static unsigned long trapStartTime;  // Thời điểm bắt đầu chuỗi tránh vật cản
    static unsigned long lastNormalTime; // Lần cuối ở trạng thái NORMAL
    static bool  escapeMode;             // Đang trong chế độ thoát bẫy
    static RobotConfig runtimeConfig;

    // ══════ Thresholds ══════
    static const int       MAX_OSCILLATIONS    = 3;    // Sau 3 lần đổi hướng → escape
    //  Giảm xuống 2 nếu muốn escape nhanh hơn
    static const int       MAX_BACKING_RETRIES = 5;    // Sau 5 lần lùi → forced turn
    static const unsigned long TRAP_TIMEOUT_MS = 4000; // 4s không NORMAL → escape
    // Nếu robot escape quá sớm khi đang cua bình thường → tăng lên 6s
    static const unsigned long ESCAPE_TURN_TIME_MS = 2500; // Xoay mạnh 2.5s khi escape
    static const unsigned long PHASE_ESCAPE_MS = 1500; // Thời gian 1 pha lùi để espace

    // ── QUYẾT ĐỊNH HƯỚNG ĐI ──
    // Phân tích 4 cảm biến và trả về State phù hợp
    static State decideDirection(long front, long right, long left, long back);
    static State decideEscapeDirection(long right, long left, long back);

    // ══════ Anti-Trap Logic ══════
    static void updateTrapDetection(State newState);
    static bool shouldEscape();

    // ── HANDLERS ──
    static void handleNormalState(long front, long right, long left);
    static void handleSlowState(long front, long right, long left);
    static void handleAvoidLeftState(long front, long right, long left);
    static void handleAvoidRightState(long front, long right, long left);
    static void handleTurnLeftState(long front, long right, long left, long back, unsigned long now);
    static void handleTurnRightState(long front, long right, long left, long back, unsigned long now);
    static void handleBackingState(long back, unsigned long now);
    static void handleStopState(long front, long right, long left, long back);
    static void handleEmergencyState(long front);
    static void handleEscapeState(long front, long right, long left, long back, unsigned long now);
};

#endif
