#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include <Arduino.h>
#include "Config.h"

struct RobotConfig {
    // Speed constants (PWM)
    uint16_t minRunSpeed;
    uint16_t cruiseSpeed;
    uint16_t fastSpeed;
    uint16_t backSpeed;
    uint16_t escapeSpeed;
    uint16_t sharpTurnBoost;
    uint16_t mediumTurnBoost;
    uint16_t turnBoost;
    uint16_t lightTurnBoost;

    // Distance thresholds (cm)
    uint16_t emergencyDist;
    uint16_t stopDistance;
    uint16_t slowDistance;
    uint16_t turnDistance;
    uint16_t prepareDistance;
    uint16_t sideDangerDist;
    uint16_t backDangerDistance;
    uint16_t directionHysteresis;
};

class ConfigStorage {
public:
    static bool begin();

    // Returns true when loaded from NVS, false when fallback to defaults.
    static bool load(RobotConfig& out);
    static bool save(const RobotConfig& cfg);
    static bool reset();

    static RobotConfig getDefaults();
    static RobotConfig getCurrent();
    static bool isCurrentFromNvs();

    static bool isValidConfig(const RobotConfig& cfg);
    static void printConfig(const RobotConfig& cfg, const char* prefix = "[CFG]");

private:
    static bool _initialized;
    static bool _currentFromNvs;
    static RobotConfig _current;
};

#endif // CONFIG_STORAGE_H
