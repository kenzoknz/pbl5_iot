#ifndef VEHICLE_STATE_MACHINE_H
#define VEHICLE_STATE_MACHINE_H

#include "Config.h"

class VehicleStateMachine {
public:
    static void begin();
    static void update();
    static void debugOutput();
    static State getCurrentState() { 
        return currentState; }

private:
    static State currentState;
    static unsigned long stateStartTime;
    static unsigned long lastDebugTime; 
    static bool turnRight;  // Hướng rẽ cuối cùng được chọn

    // ── QUYẾT ĐỊNH HƯỚNG ĐI ──
    // Phân tích 4 cảm biến và trả về State phù hợp
    static State decideDirection(long front, long right, long left, long back);

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
};

#endif