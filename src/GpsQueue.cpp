#include "GpsQueue.h"

GpsQueueEntry GpsQueue::_buffer[GpsQueue::QUEUE_SIZE];
uint16_t GpsQueue::_head = 0;
uint16_t GpsQueue::_tail = 0;
uint16_t GpsQueue::_count = 0;
portMUX_TYPE GpsQueue::_mux = portMUX_INITIALIZER_UNLOCKED;

void GpsQueue::begin() {
    _head = _tail = _count = 0;
    // Serial.println("[GpsQueue] Initialized, capacity=" + String(QUEUE_SIZE));
}

bool GpsQueue::enqueue(const GpsQueueEntry& entry) {
    portENTER_CRITICAL(&_mux);
    
    if (_count >= QUEUE_SIZE) {
        portEXIT_CRITICAL(&_mux);
        return false; // Queue full
    }
    
    _buffer[_head] = entry;
    _head = (_head + 1) % QUEUE_SIZE;
    _count++;
    
    portEXIT_CRITICAL(&_mux);
    return true;
}

bool GpsQueue::dequeue(GpsQueueEntry& entry) {
    portENTER_CRITICAL(&_mux);
    
    if (_count == 0) {
        portEXIT_CRITICAL(&_mux);
        return false;
    }
    
    entry = _buffer[_tail];
    _tail = (_tail + 1) % QUEUE_SIZE;
    _count--;
    
    portEXIT_CRITICAL(&_mux);
    return true;
}

uint16_t GpsQueue::size() {
    portENTER_CRITICAL(&_mux);
    uint16_t result = _count;
    portEXIT_CRITICAL(&_mux);
    return result;
}

void GpsQueue::clear() {
    portENTER_CRITICAL(&_mux);
    _head = _tail = _count = 0;
    portEXIT_CRITICAL(&_mux);
}
