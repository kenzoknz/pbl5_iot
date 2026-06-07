#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <Arduino.h>
#include <TinyGPS++.h>

typedef struct {
    bool fix;
    double lat;
    double lng;
    bool preview_available;
    double preview_lat;
    double preview_lng;
    float altitude_m;
    float speed_kmh;
    float course_deg;
    int satellites;
    float hdop;
    char gps_time_utc[32];
    uint32_t updated_at_ms;
} GpsData;

class GPSSensor {
public:
    static bool begin();
    static void update();
    static GpsData getData();

private:
    static TinyGPSPlus _gps;
    static GpsData _data;
    static portMUX_TYPE _dataMux;

    static bool buildIsoTime(char* out, size_t outSize);
    static bool validCoordinates(double lat, double lng);
};

#endif
