#include "core/ConfigStorage.h"

#include <Preferences.h>
#include <nvs_flash.h>

namespace {
constexpr const char* NVS_NAMESPACE = "robot_cfg";
constexpr const char* NVS_KEY = "robot_config";
constexpr size_t CONFIG_SIZE = sizeof(RobotConfig);

bool _inRange(uint16_t value, uint16_t minVal, uint16_t maxVal) {
    return value >= minVal && value <= maxVal;
}

bool _validateSpeedGroup(const RobotConfig& cfg) {
    return _inRange(cfg.minRunSpeed, 0, 255)
        && _inRange(cfg.cruiseSpeed, 0, 255)
        && _inRange(cfg.fastSpeed, 0, 255)
        && _inRange(cfg.backSpeed, 0, 255)
        && _inRange(cfg.escapeSpeed, 0, 255)
        && _inRange(cfg.sharpTurnBoost, 0, 255)
        && _inRange(cfg.mediumTurnBoost, 0, 255)
        && _inRange(cfg.turnBoost, 0, 255)
        && _inRange(cfg.lightTurnBoost, 0, 255);
}

bool _validateDistanceGroup(const RobotConfig& cfg) {
    return _inRange(cfg.emergencyDist, 10, 200)
        && _inRange(cfg.stopDistance, 10, 200)
        && _inRange(cfg.slowDistance, 10, 200)
        && _inRange(cfg.turnDistance, 10, 200)
        && _inRange(cfg.prepareDistance, 10, 200)
        && _inRange(cfg.sideDangerDist, 10, 200)
        && _inRange(cfg.backDangerDistance, 10, 200)
        && _inRange(cfg.directionHysteresis, 1, 50);
}
} // namespace

bool ConfigStorage::_initialized = false;
bool ConfigStorage::_currentFromNvs = false;
RobotConfig ConfigStorage::_current = ConfigStorage::getDefaults();

bool ConfigStorage::begin() {
    if (_initialized) return true;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("[CFG] NVS partition issue, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    if (err != ESP_OK) {
        Serial.printf("[CFG] nvs_flash_init failed: %d\n", (int)err);
        _initialized = false;
        return false;
    }

    _initialized = true;
    return true;
}

RobotConfig ConfigStorage::getDefaults() {
    RobotConfig cfg = {};

    cfg.minRunSpeed = MIN_RUN_SPEED;
    cfg.cruiseSpeed = CRUISE_SPEED;
    cfg.fastSpeed = FAST_SPEED;
    cfg.backSpeed = BACK_SPEED;
    cfg.escapeSpeed = ESCAPE_SPEED;
    cfg.sharpTurnBoost = SHARP_TURN_BOOST;
    cfg.mediumTurnBoost = MEDIUM_TURN_BOOST;
    cfg.turnBoost = TURN_BOOST;
    cfg.lightTurnBoost = LIGHT_TURN_BOOST;

    cfg.emergencyDist = EMERGENCY_DIST;
    cfg.stopDistance = STOP_DISTANCE;
    cfg.slowDistance = SLOW_DISTANCE;
    cfg.turnDistance = TURN_DISTANCE;
    cfg.prepareDistance = PREPARE_DISTANCE;
    cfg.sideDangerDist = SIDE_DANGER_DIST;
    cfg.backDangerDistance = BACK_DANGER_DISTANCE;
    cfg.directionHysteresis = DIRECTION_HYSTERESIS;

    return cfg;
}

bool ConfigStorage::isValidConfig(const RobotConfig& cfg) {
    if (!_validateSpeedGroup(cfg) || !_validateDistanceGroup(cfg)) {
        return false;
    }

    // Keep threshold order consistent to avoid broken state transitions.
    if (!(cfg.emergencyDist <= cfg.stopDistance
        && cfg.stopDistance <= cfg.slowDistance
        && cfg.slowDistance <= cfg.turnDistance
        && cfg.turnDistance <= cfg.prepareDistance)) {
        return false;
    }

    return true;
}

bool ConfigStorage::load(RobotConfig& out) {
    if (!begin()) {
        out = getDefaults();
        _current = out;
        _currentFromNvs = false;
        return false;
    }

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        Serial.println("[CFG] Failed to open NVS namespace for read");
        out = getDefaults();
        _current = out;
        _currentFromNvs = false;
        return false;
    }

    size_t storedSize = prefs.getBytesLength(NVS_KEY);
    if (storedSize != CONFIG_SIZE) {
        prefs.end();
        out = getDefaults();
        _current = out;
        _currentFromNvs = false;
        return false;
    }

    RobotConfig loaded = {};
    size_t readSize = prefs.getBytes(NVS_KEY, &loaded, CONFIG_SIZE);
    prefs.end();

    if (readSize != CONFIG_SIZE || !isValidConfig(loaded)) {
        Serial.println("[CFG] Stored config invalid. Falling back to defaults");
        out = getDefaults();
        _current = out;
        _currentFromNvs = false;
        return false;
    }

    out = loaded;
    _current = loaded;
    _currentFromNvs = true;
    return true;
}

bool ConfigStorage::save(const RobotConfig& cfg) {
    if (!begin()) return false;

    if (!isValidConfig(cfg)) {
        Serial.println("[CFG] Reject save: config out of range");
        return false;
    }

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[CFG] Failed to open NVS namespace for write");
        return false;
    }

    size_t written = prefs.putBytes(NVS_KEY, &cfg, CONFIG_SIZE);
    prefs.end();

    if (written != CONFIG_SIZE) {
        Serial.println("[CFG] putBytes failed");
        return false;
    }

    _current = cfg;
    _currentFromNvs = true;
    return true;
}

bool ConfigStorage::reset() {
    if (!begin()) return false;

    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[CFG] Failed to open NVS namespace for reset");
        return false;
    }

    bool removed = prefs.remove(NVS_KEY);
    prefs.end();

    _current = getDefaults();
    _currentFromNvs = false;
    (void)removed;
    return true;
}

RobotConfig ConfigStorage::getCurrent() {
    return _current;
}

bool ConfigStorage::isCurrentFromNvs() {
    return _currentFromNvs;
}

void ConfigStorage::printConfig(const RobotConfig& cfg, const char* prefix) {
    Serial.printf("%s speed min=%u cruise=%u fast=%u back=%u escape=%u\n",
                  prefix,
                  cfg.minRunSpeed,
                  cfg.cruiseSpeed,
                  cfg.fastSpeed,
                  cfg.backSpeed,
                  cfg.escapeSpeed);
    Serial.printf("%s turn sharp=%u medium=%u turn=%u light=%u\n",
                  prefix,
                  cfg.sharpTurnBoost,
                  cfg.mediumTurnBoost,
                  cfg.turnBoost,
                  cfg.lightTurnBoost);
    Serial.printf("%s dist emg=%u stop=%u slow=%u turn=%u prep=%u side=%u back=%u hyst=%u\n",
                  prefix,
                  cfg.emergencyDist,
                  cfg.stopDistance,
                  cfg.slowDistance,
                  cfg.turnDistance,
                  cfg.prepareDistance,
                  cfg.sideDangerDist,
                  cfg.backDangerDistance,
                  cfg.directionHysteresis);
}

