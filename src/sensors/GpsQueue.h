#ifndef GPS_QUEUE_H
#define GPS_QUEUE_H

#include <Arduino.h>

typedef struct {
    double lat;           // 8 bytes
    double lng;           // 8 bytes
    float altitude_m;     // 4 bytes
    float speed_kmh;      // 4 bytes
    float course_deg;     // 4 bytes
    float hdop;           // 4 bytes
    int satellites;       // 4 bytes
    bool fix;             // 1 byte
    uint16_t year;        // 2 bytes
    uint8_t month;        // 1 byte
    uint8_t day;          // 1 byte
    uint8_t hour;         // 1 byte
    uint8_t minute;       // 1 byte
    uint8_t second;       // 1 byte
    uint32_t timestamp_ms; // 4 bytes
    uint8_t _reserved;    // 1 byte
    // Total: 48 bytes
} GpsQueueEntry;

class GpsQueue {
public:
    static void begin();
    static bool enqueue(const GpsQueueEntry& entry);
    static bool dequeue(GpsQueueEntry& entry);
    static uint16_t size();
    static void clear();
    static uint16_t available() { return size(); }

private:
    static constexpr uint16_t QUEUE_SIZE = 256;
    static GpsQueueEntry _buffer[QUEUE_SIZE];
    static uint16_t _head;
    static uint16_t _tail;
    static uint16_t _count;
    static portMUX_TYPE _mux;
};

#endif
