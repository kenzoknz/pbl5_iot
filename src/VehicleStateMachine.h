#ifndef VEHICLE_STATE_MACHINE_H
#define VEHICLE_STATE_MACHINE_H

#include "Config.h"

class VehicleStateMachine {
public:
    static void begin();
    static void update();
    static void debugOutput();
    static State getCurrentState() { return currentState; }

private:
    static State currentState;
    static unsigned long stateStartTime;
    static unsigned long lastDebugTime;
    
    // Dữ liệu quét
    static long scannedRightDist;
    static long scannedLeftDist;
    static bool turnRight;
    static bool scanCompleted;  // Đánh dấu đã quét xong
    
    // Xử lý từng state
    static void handleNormalState(long frontDist);
    static void handleSlowState(long frontDist);
    static void handleTurnState(long frontDist, long backDist);
    static void handleBackingState(long backDist, unsigned long now);
    static void handleTurningState(unsigned long now);
    static void handleResumingState(long frontDist, unsigned long now);
    static void handleStopState(long frontDist);
};

#endif