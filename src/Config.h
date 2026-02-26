#ifndef CONFIG_H
#define CONFIG_H

/* ================== SYSTEM & FREERTOS ================== */
// Ưu tiên Task (Priority 0 là thấp nhất)
#define PRIORITY_MOTOR    5
#define PRIORITY_SENSORS  3
#define PRIORITY_LOGIC    5
#define PRIORITY_APP      1  // cao để realtime

// Thêm ngưỡng dừng khẩn cấp (Priority Break)
#define EMERGENCY_STOP_DIST 10 
#define BUFFER_SIZE 3 // Giảm từ 5 xuống 3 để nhạy hơn

// Kích thước bộ nhớ Task (Bytes)
#define STACK_SIZE_SENSORS 4096
#define STACK_SIZE_LOGIC   4096
#define STACK_SIZE_MOTOR   2048

/* ================== MPU6050 ================== */
#define SDA_PIN 32
#define SCL_PIN 33
#define COLLISION_THRESHOLD 2.0
#define TILT_THRESHOLD 20.0
#define MPU_UPDATE_RATE_MS 20  // Tương đương 50Hz

/* ================== BTS7960 ================== */
#define RPWM 18
#define LPWM 19
#define REN 21
#define LEN 22

/* ================== SERVO ================== */
#define SERVO_STEER_PIN 23        // Servo lái bánh xe
#define SERVO_US_FRONT_PIN 27    // Servo quay cảm biến siêu âm trước

/* ================== SERVO ANGLES ================== */
#define US_SCAN_CENTER 90         // Thẳng (giữa của SG90: 0-180°)
#define US_SCAN_RIGHT 30          // Quay phải 60° (30° từ 0°)
#define US_SCAN_LEFT 130          // Quay trái 60° (150° từ 180°)
#define SERVO_SCAN_DELAY_MS 500   // Sử dụng vTaskDelay thay vì delay()

/* ================== ULTRASONIC ================== */
#define TRIG_FRONT 16
#define ECHO_FRONT 17
#define TRIG_BACK  25
#define ECHO_BACK  26
#define US_UPDATE_RATE_MS 60      // Quan trọng: Giảm nhiệt cho cảm biến (15-20Hz là đủ)
#define MAX_DIST_TIMEOUT 25000    // microSeconds

/* ================== PWM ================== */
#define PWM_CHANNEL_RPWM 0
#define PWM_CHANNEL_LPWM 1
#define PWM_FREQ 1000 // đề xuất 5000 để chạy êm
#define PWM_RESOLUTION 8

/* ================== SPEED ================== */
#define STOP_SPEED 0
#define MIN_RUN_SPEED 150 // Tốc độ tối thiểu để thắng lực ma sát 190
#define CRUISE_SPEED 90 // 3 tầng: 111, test 2 tầng 90
#define FAST_SPEED 90 //3 tâng 250
#define BACK_SPEED 150 // 3 tầng 220

#define TURN_BOOST 200       // Cua vừa (góc 60-80°)
#define SHARP_TURN_BOOST 220   // Cua gắt (góc 100-130°) - Tốc độ cao thắng ma sát

/* ================== DISTANCE ================== */
#define STOP_DISTANCE     20
#define SLOW_DISTANCE     30
#define TURN_DISTANCE     45
#define PREPARE_DISTANCE  60
#define BACK_DANGER_DISTANCE 25

/* ================== TIME ================== */
#define BACK_TIME 3000
#define TURN_TIME 2000
#define RESUME_TIME 1000

/* ================== STATE ================== */
enum State {
  INIT,
  NORMAL,
  SLOW,
  TURN,           // Dừng xe và quét servo siêu âm
  STOP,
  BACKING,
  TURNING,        // Thực hiện rẽ sau khi quét
  RESUMING,
  MANUAL_CONTROL  // chế độ người dùng điều khiển
};

/* ================== SCAN SUBSTATES ================== */
enum ScanPhase {
  SCAN_IDLE,
  SCAN_RIGHT,
  SCAN_LEFT,
  SCAN_CENTER_RETURN,
  SCAN_COMPLETE
};

#endif