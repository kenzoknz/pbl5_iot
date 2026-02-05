#ifndef CONFIG_H
#define CONFIG_H

/* ================== MPU6050 ================== */
#define SDA_PIN 32
#define SCL_PIN 33
#define COLLISION_THRESHOLD 2.0
#define TILT_THRESHOLD 20.0

/* ================== BTS7960 ================== */
#define RPWM 18
#define LPWM 19
#define REN 21
#define LEN 22

/* ================== SERVO ================== */
#define SERVO_PIN 23

/* ================== ULTRASONIC ================== */
#define TRIG_FRONT 16
#define ECHO_FRONT 17
#define TRIG_BACK  25
#define ECHO_BACK  26

/* ================== PWM ================== */
#define PWM_CHANNEL_RPWM 0
#define PWM_CHANNEL_LPWM 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

/* ================== SPEED ================== */
#define STOP_SPEED 0
#define MIN_RUN_SPEED 115
#define CRUISE_SPEED 120
#define FAST_SPEED 130
#define TURN_BOOST 142         // Cua vừa (góc 60-80°)
#define SHARP_TURN_BOOST 165     // Cua gắt (góc 100-130°) - Tốc độ cao thắng ma sát
#define BACK_SPEED 120

/* ================== DISTANCE ================== */
#define STOP_DISTANCE     20
#define SLOW_DISTANCE     30
#define TURN_DISTANCE     45
#define PREPARE_DISTANCE  60
#define BACK_DANGER_DISTANCE 25

/* ================== TIME ================== */
#define BACK_TIME 3000
#define TURN_TIME 2000

/* ================== STATE ================== */
enum State {
  INIT,
  NORMAL,
  SLOW,
  TURN,
  STOP,
  BACKING,
  TURNING,
  RESUMING
};

#endif