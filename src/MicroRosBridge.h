#ifndef MICRO_ROS_BRIDGE_H
#define MICRO_ROS_BRIDGE_H

#include <Arduino.h>
#include <ArduinoJson.h>

// micro-ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS messages
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/float32.h>

class MicroRosBridge {
public:
    static bool begin(HardwareSerial& serialPort = Serial);
    static void spinSome();
    static bool isReady();

    // gọi định kỳ trong AppTask (ví dụ mỗi vòng 50ms là được)
    static void tick();

    // reset odom nếu cần
    static void resetOdometry();

private:
    // ROS core
    static bool _ready;
    static rcl_allocator_t _allocator;
    static rclc_support_t _support;
    static rcl_node_t _node;
    static rclc_executor_t _executor;

    // pub/sub
    static rcl_subscription_t _subCmdVel;
    static rcl_publisher_t _pubOdom;
    static rcl_publisher_t _pubTf;
    static rcl_publisher_t _pubImu;
    static rcl_publisher_t _pubRobotMode;
    static rcl_publisher_t _pubEncoderSpeed;

    // timer
    static rcl_timer_t _timerOdom;

    // messages
    static geometry_msgs__msg__Twist _cmdMsg;
    static nav_msgs__msg__Odometry _odomMsg;
    static tf2_msgs__msg__TFMessage _tfMsg;
    static geometry_msgs__msg__TransformStamped _tfStamped;
    static sensor_msgs__msg__Imu _imuMsg;
    static std_msgs__msg__String _modeMsg;
    static std_msgs__msg__Float32 _encSpeedMsg;

    // odom state
    static float _x;
    static float _y;
    static float _th;
    static uint32_t _lastMs;

    // command watchdog
    static uint32_t _lastCmdVelMs;
    static const uint32_t CMD_VEL_TIMEOUT_MS = 400;

    // adaptive traction / speed mapping
    static float _pwmPerMps;       // hệ số map m/s -> PWM (online adaptive)
    static float _surfaceDragGain; // ước lượng lực cản bề mặt
    static float _emaSpeedErr;     // theo dõi sai số dài hạn

    // params robot
    static constexpr float WHEELBASE_M = 0.14f;
    static constexpr int SERVO_CENTER = 90;
    // static constexpr int SERVO_LEFT_MAX = 125;
    // static constexpr int SERVO_RIGHT_MAX = 55;
    static constexpr float STEER_MAX_RAD = 30.0f * 3.1415926f / 180.0f; // ±30 deg
    static constexpr int PWM_MIN_RUN = 140;
    static constexpr int PWM_MAX = 255;

    // helpers
    static float clampf(float v, float lo, float hi);
    static float servoDegToSteerRad(int servoDeg);
    static int steerRadToServoDeg(float deltaRad);
    static float normalizeAngle(float a);

    // mapping + adaptive
    static int speedMpsToPwmAdaptive(float v_mps, float curvature_abs);
    static void updateAdaptiveTraction(float target_mps, float measured_mps, float curvature_abs);

    // ROS callbacks
    static void cmdVelCallback(const void* msgin);
    static void odomTimerCallback(rcl_timer_t* timer, int64_t last_call_time);

    // publish
    static void publishOdometryAndTf();
    static void publishImu();
    static void publishDiagnostics();
};

#endif // MICRO_ROS_BRIDGE_H