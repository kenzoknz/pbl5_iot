#include "GPSSensor.h"
#include "GpsQueue.h"
#include "Config.h"
#include "NetworkManager.h"

TinyGPSPlus GPSSensor::_gps;
GpsData GPSSensor::_data = {
    false,
    0.0,
    0.0,
    false,
    0.0,
    0.0,
    0.0f,
    0.0f,
    0.0f,
    0,
    0.0f,
    "",
    0,
};
portMUX_TYPE GPSSensor::_dataMux = portMUX_INITIALIZER_UNLOCKED;

bool GPSSensor::begin() {
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[GPS] Neo-7N init UART2 RX=%d TX=%d BAUD=%d\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);
    return true;
}

void GPSSensor::update() {
    static uint32_t lastGpsInfoLogTime = 0;
    static uint32_t lastGpsNoFixLogTime = 0;

    while (Serial2.available() > 0) {
        _gps.encode(Serial2.read());
    }

    GpsData next = getData();
    next.updated_at_ms = millis();
    next.fix = _gps.location.isValid() && _gps.location.age() < 5000;

    // Preview tọa độ để debug: có thể dùng khi đã thấy vệ tinh nhưng chưa đạt fix ổn định.
    next.preview_available = false;
    next.preview_lat = 0.0;
    next.preview_lng = 0.0;

    if (_gps.location.isValid()) {
        const double rawLat = _gps.location.lat();
        const double rawLng = _gps.location.lng();
        if (validCoordinates(rawLat, rawLng)) {
            next.preview_available = true;
            next.preview_lat = rawLat;
            next.preview_lng = rawLng;
        }
    }

    if (next.fix) {
        const double lat = _gps.location.lat();
        const double lng = _gps.location.lng();
        if (validCoordinates(lat, lng)) {
            next.lat = lat;
            next.lng = lng;
        } else {
            next.fix = false;
        }
    }

    next.altitude_m = _gps.altitude.isValid() ? _gps.altitude.meters() : 0.0f;
    next.speed_kmh = _gps.speed.isValid() ? _gps.speed.kmph() : 0.0f;
    next.course_deg = _gps.course.isValid() ? _gps.course.deg() : 0.0f;
    next.satellites = _gps.satellites.isValid() ? static_cast<int>(_gps.satellites.value()) : 0;
    next.hdop = _gps.hdop.isValid() ? _gps.hdop.hdop() : 0.0f;
    next.gps_time_utc = buildIsoTime();

    if (!next.fix) {
        next.lat = 0.0;
        next.lng = 0.0;
    }

    // Gửi serial log GPS định kỳ lên server để hiển thị trên web serial monitor.
    if (next.fix && (millis() - lastGpsInfoLogTime >= 2000)) {
        char gpsMsg[192];
        snprintf(
            gpsMsg,
            sizeof(gpsMsg),
            "GPS FIX lat=%.6f lng=%.6f sat=%d speed=%.2fkm/h hdop=%.2f",
            next.lat,
            next.lng,
            next.satellites,
            next.speed_kmh,
            next.hdop
        );
        NetworkManager::wsSendSerialLog("INFO", String(gpsMsg));
        lastGpsInfoLogTime = millis();
    }

    if (!next.fix && (millis() - lastGpsNoFixLogTime >= 5000)) {
        char gpsWarn[128];
        snprintf(
            gpsWarn,
            sizeof(gpsWarn),
            "GPS NO_FIX sat=%d hdop=%.2f age=%lu",
            next.satellites,
            next.hdop,
            (unsigned long)_gps.location.age()
        );
        NetworkManager::wsSendSerialLog("WARN", String(gpsWarn));
        lastGpsNoFixLogTime = millis();
    }

    portENTER_CRITICAL(&_dataMux);
    _data = next;
    portEXIT_CRITICAL(&_dataMux);
    
    // ──────────────────────────────────────────────────
    // THÊM LOGIC QUEUE
    // ──────────────────────────────────────────────────
    static uint32_t lastQueueTime = 0;
    
    if (next.fix && (millis() - lastQueueTime >= GPS_QUEUE_MIN_INTERVAL)) {
        GpsQueueEntry entry;
        entry.lat = next.lat;
        entry.lng = next.lng;
        entry.altitude_m = next.altitude_m;
        entry.speed_kmh = next.speed_kmh;
        entry.course_deg = next.course_deg;
        entry.hdop = next.hdop;
        entry.satellites = next.satellites;
        entry.fix = next.fix;
        entry.timestamp_ms = millis();
        
        // GPS time
        if (_gps.date.isValid() && _gps.time.isValid()) {
            entry.year = _gps.date.year();
            entry.month = _gps.date.month();
            entry.day = _gps.date.day();
            entry.hour = _gps.time.hour();
            entry.minute = _gps.time.minute();
            entry.second = _gps.time.second();
        } else {
            entry.year = 0;
        }
        
        if (GpsQueue::enqueue(entry)) {
            lastQueueTime = millis();
        }
    }
}

GpsData GPSSensor::getData() {
    GpsData copy;
    portENTER_CRITICAL(&_dataMux);
    copy = _data;
    portEXIT_CRITICAL(&_dataMux);
    return copy;
}

String GPSSensor::buildIsoTime() {
    if (!_gps.date.isValid() || !_gps.time.isValid()) {
        return "";
    }

    char buf[32];
    snprintf(
        buf,
        sizeof(buf),
        "%04d-%02d-%02dT%02d:%02d:%02dZ",
        _gps.date.year(),
        _gps.date.month(),
        _gps.date.day(),
        _gps.time.hour(),
        _gps.time.minute(),
        _gps.time.second()
    );
    return String(buf);
}

bool GPSSensor::validCoordinates(double lat, double lng) {
    return lat >= -90.0 && lat <= 90.0 && lng >= -180.0 && lng <= 180.0;
}
