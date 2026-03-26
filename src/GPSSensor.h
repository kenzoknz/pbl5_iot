#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <Arduino.h>
#include <TinyGPS++.h>

typedef struct {
    bool fix;
    double lat;
    double lng;
    float altitude_m;
    float speed_kmh;
    float course_deg;
    int satellites;
    float hdop;
    String gps_time_utc;
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

    static String buildIsoTime();
    static bool validCoordinates(double lat, double lng);
};

#endif
