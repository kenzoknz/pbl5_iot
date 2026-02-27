#ifndef VEHICLE_STATE_MACHINE_H
#define VEHICLE_STATE_MACHINE_H

#include "Config.h"

enum ScanPhase {
    SCAN_IDLE,              // Chưa bắt đầu quét
    SCAN_MOVING_RIGHT,      // Đã ra lệnh quay phải, đang chờ servo ổn định
    SCAN_READING_RIGHT,     // Servo ổn định → đọc buffer ngay
    SCAN_MOVING_LEFT,       // Đã ra lệnh quay trái, đang chờ servo ổn định
    SCAN_READING_LEFT,      // Servo ổn định → đọc buffer ngay
    SCAN_RETURNING_CENTER,  // Về giữa, chờ ổn định
    SCAN_COMPLETED          // Ra quyết định hướng đi
};

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

    //ScanPhase
    static ScanPhase currentScanPhase;
    static unsigned long lastScanStepTime;
    
    // Dữ liệu quét
    static long scannedRightDist;
    static long scannedLeftDist;
    static bool turnRight;
    static bool scanCompleted;  // Đánh dấu đã quét xong
    
    // Handlers
    static void handleNormalState(long frontDist);
    static void handleSlowState(long frontDist);
    static void handleTurnState(long frontDist, long backDist);
    static void handleBackingState(long backDist, unsigned long now);
    static void handleTurningState(unsigned long now);
    static void handleResumingState(long frontDist, unsigned long now);
    static void handleStopState(long frontDist);

};

#endif