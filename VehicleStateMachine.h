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
    static bool turnRight;
    static unsigned long lastDebugTime;
    
    static void handleNormalState(long frontDist);
    static void handleSlowState(long frontDist);
    static void handleTurnState(long frontDist);
    static void handleStopState(long backDist, unsigned long now);
    static void handleBackingState(unsigned long now);
    static void handleTurningState(unsigned long now);
    static void handleResumingState(long frontDist);
};

#endif