#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <algorithm>



// ── Buffer & timing ──────────────────────────────────────────────────────────
#define BUFFER_SIZE     3        // Cũ 5 (phản ứng hơi chậm, test 3) Số mẫu trong circular buffer
#define SENSOR_PERIOD   60      // ms — HC-SR04 yêu cầu >= 60ms giữa 2 trigger

// ── Bật log debug: thêm -DDEBUG_SENSOR vào build flags hoặc uncomment dưới ──
#define DEBUG_SENSOR

// ─────────────────────────────────────────────────────────────────────────────
// [+] MỚI: Enum chế độ hoạt động
//     Ý nghĩa: Giúp UltrasonicSensor biết đang ở chế độ nào
//              để quyết định suspend/resume background task.
//     Dùng chung cho toàn project (MotorController, DecisionTask... đều import)
// ─────────────────────────────────────────────────────────────────────────────
enum class OperationMode {
    AUTONOMOUS,   // Robot tự hành — background sensor task chạy liên tục
    MANUAL        // Người điều khiển — suspend sensor task, tiết kiệm tài nguyên
};

class UltrasonicSensor {
public:
    // ── Khởi tạo ─────────────────────────────────────────────────────────────
    static void begin();

    // ── FreeRTOS Tasks (chạy trên Core 0) ────────────────────────────────────
    static void frontSensorTask(void* pvParameters);
    static void backSensorTask(void* pvParameters);

    // ── Public API đọc khoảng cách (thread-safe) ─────────────────────────────
    static long getFrontDistance();
    static long getBackDistance();

    // ── Quét servo (gọi từ Decision Task trên Core 1) ────────────────────────
    static void scanAllDirections(long& rightDist, long& leftDist);
        // ── Suspend / Resume riêng frontTask khi quét servo ──────────────────────
    //    Ý nghĩa: Khi servo quay sang góc khác (30° hoặc 150°), frontSensorTask
    //    vẫn tiếp tục đọc và nạp vào buffer — nhưng đó là khoảng cách của góc
    //    lệch, không phải góc quét. Buffer bị "ô nhiễm" dữ liệu sai → quyết
    //    định hướng đi sai.
    static void suspendFrontTask();
    static void resumeFrontTask();

    // ── [+] MỚI: Chuyển chế độ hoạt động ────────────────────────────────────
    //    Ý nghĩa: Khi MANUAL → suspend cả 2 sensor task để nhường tài nguyên
    //             cho task xử lý lệnh từ app. Khi AUTONOMOUS → resume lại.
    static void setMode(OperationMode mode);

    // ── [+] MỚI: Lấy chế độ hiện tại (các module khác có thể query) ──────────
    static OperationMode getMode();

    // ── Đọc thô ──────────────────────────────────────────────────────────────
    static long readDistanceRaw(int trig, int echo);

private:

    // ── Cập nhật buffer ──────────────────────────────────────────────────────
    static void updateFrontBuffer();
    static void updateBackBuffer();

    // ── Tính median ──────────────────────────────────────────────────────────
    static long getMedian(long* buffer);

    // ── [+] MỚI: Task handles — lưu lại để suspend/resume ───────────────────
    //    V3 gốc tạo task nhưng không lưu handle → không thể kiểm soát sau này
    static TaskHandle_t frontTaskHandle;
    static TaskHandle_t backTaskHandle;

    // ── [+] MỚI: Trạng thái chế độ hiện tại ─────────────────────────────────
    static OperationMode currentMode;
};

// ── [+] MỚI: Mutex khai báo extern — định nghĩa 1 lần trong .cpp ─────────────
//    V3 gốc khai báo global không có extern → multiple definition khi include
//    nhiều file. Sửa bằng cách khai báo extern ở .h, định nghĩa ở .cpp.
extern SemaphoreHandle_t sensorMutex;