#ifndef CONFIG_H
#define CONFIG_H

/* ================== SYSTEM & FREERTOS ================== */
// Ưu tiên Task (Default: 0 (min) -> 24)
#define PRIORITY_MOTOR    5
#define PRIORITY_SENSORS  3
#define PRIORITY_LOGIC    5
#define PRIORITY_APP      7  // cao để realtime

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
#define MPU_UPDATE_RATE_MS 20  // <=> 50Hz

/* ================== BTS7960 ================== */
#define RPWM 18
#define LPWM 19
#define REN 21
#define LEN 22

/* ================== SERVO ================== */
#define SERVO_STEER_PIN 23        // Servo lái bánh xe

/* ================== ULTRASONIC - 4 ================== */
// Front - center
#define TRIG_FRONT 16
#define ECHO_FRONT 17
// Front - right
#define TRIG_RIGHT  4
#define ECHO_RIGHT  2
// Front - left
#define TRIG_LEFT   13
#define ECHO_LEFT   14
// Back - center
#define TRIG_BACK  25
#define ECHO_BACK  26

#define US_UPDATE_RATE_MS 60      // Giảm nhiệt cho cảm biến (15-20Hz là đủ) ->  >= 60ms cho HC-SR04
#define MAX_DIST_TIMEOUT 15000    // pulseIn timeout (us) ~ 255cm max (cũ 25000)

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

#define SHARP_TURN_BOOST 200   // Cua gắt (góc 100-130°) - Tốc độ cao thắng ma sát
#define MEDIUM_TURN_BOOST 170  // Cua vừa (góc 60-80°) - Tăng tốc để thắng ma sát
#define TURN_BOOST 150       // Cua vừa (góc 60-80°) 200
#define LIGHT_TURN_BOOST 135   // Cua nhẹ (góc 15-60°) - Tăng tốc nhẹ để thắng ma sát

//  3 tầng (thêm 1 tầng giảm ~10-15%)

/* ================== DISTANCE THRESHOLDS================== */
#define EMERGENCY_DIST    15    // Dừng khẩn cấp
#define STOP_DISTANCE     20
#define SLOW_DISTANCE     30    // Giảm tốc dần
#define TURN_DISTANCE     45    // Qđ rẽ
#define PREPARE_DISTANCE  60
#define SIDE_DANGER_DIST  25    // Ngưỡng nguy hiểm cho cảm biến bên
#define BACK_DANGER_DISTANCE 25

/* ================== TIME ================== */
// #define BACK_TIME 3000
// #define TURN_TIME 2000
// #define RESUME_TIME 1000
#define BACK_TIME    2000
#define TURN_TIME    1800
#define RESUME_TIME  800

/* ================== STATE ================== */
enum State {
  INIT,
  NORMAL,         // Chạy thẳng, đường rộng
  SLOW,           // Giảm tốc, có vật cản phía trước xa
  AVOID_LEFT,     // Né sang trái (vật cản bên phải)
  AVOID_RIGHT,    // Né sang phải (vật cản bên trái)
  TURN_LEFT,      // Rẽ trái mạnh (phía trước bị chặn, trái rộng hơn)
  TURN_RIGHT,     // Rẽ phải mạnh (phía trước bị chặn, phải rộng hơn)
  BACKING,        // Lùi (cả 3 phía trước đều bị chặn)
  STOP,           // Bị kẹt hoàn toàn
  EMERGENCY,      // Va chạm / nghiêng
  MANUAL_CONTROL  // Chế độ người dùng điều khiển
};

#endif