#include "MicroRosBridge.h"
#include "Config.h"
#include "MotorController.h"
#include "EncoderSensor.h"
#include "MPUSensor.h"
#include "CommandProcessor.h"

// ===== static definitions =====
bool MicroRosBridge::_ready = false;
rcl_allocator_t MicroRosBridge::_allocator;
rclc_support_t MicroRosBridge::_support;
rcl_node_t MicroRosBridge::_node;
rclc_executor_t MicroRosBridge::_executor;

rcl_subscription_t MicroRosBridge::_subCmdVel;
rcl_publisher_t MicroRosBridge::_pubOdom;
rcl_publisher_t MicroRosBridge::_pubTf;
rcl_publisher_t MicroRosBridge::_pubImu;
rcl_publisher_t MicroRosBridge::_pubRobotMode;
rcl_publisher_t MicroRosBridge::_pubEncoderSpeed;

rcl_timer_t MicroRosBridge::_timerOdom;

geometry_msgs__msg__Twist MicroRosBridge::_cmdMsg;
nav_msgs__msg__Odometry MicroRosBridge::_odomMsg;
tf2_msgs__msg__TFMessage MicroRosBridge::_tfMsg;
geometry_msgs__msg__TransformStamped MicroRosBridge::_tfStamped;
sensor_msgs__msg__Imu MicroRosBridge::_imuMsg;
std_msgs__msg__String MicroRosBridge::_modeMsg;
std_msgs__msg__Float32 MicroRosBridge::_encSpeedMsg;

float MicroRosBridge::_x = 0.0f;
float MicroRosBridge::_y = 0.0f;
float MicroRosBridge::_th = 0.0f;
uint32_t MicroRosBridge::_lastMs = 0;

uint32_t MicroRosBridge::_lastCmdVelMs = 0;

// adaptive traction state
float MicroRosBridge::_pwmPerMps = 420.0f;     // seed ban đầu
float MicroRosBridge::_surfaceDragGain = 1.0f; // >1: mặt sần, <1: mặt trơn
float MicroRosBridge::_emaSpeedErr = 0.0f;

// Khởi tạo Serial1 cho micro-ROS
bool serial1_transport_open(struct uxrCustomTransport * transport) {
    // Sử dụng chân đã định nghĩa trong Config.h
    Serial1.begin(115200, SERIAL_8N1, ROS_RX_PIN, ROS_TX_PIN); 
    return true;
}
// Đóng Serial1
bool serial1_transport_close(struct uxrCustomTransport * transport) {
    Serial1.end();
    return true;
}

// Hàm ghi dữ liệu
size_t serial1_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err) {
    return Serial1.write(buf, len);
}

// Hàm đọc dữ liệu
size_t serial1_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
    Serial1.setTimeout(timeout);
    return Serial1.readBytes((char*)buf, len);
}
float MicroRosBridge::clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

float MicroRosBridge::normalizeAngle(float a) {
    while (a > 3.1415926f) a -= 2.0f * 3.1415926f;
    while (a < -3.1415926f) a += 2.0f * 3.1415926f;
    return a;
}

float MicroRosBridge::servoDegToSteerRad(int servoDeg) {
    // 90 -> 0 rad, 120 -> +max, 60 -> -max
    float ratio = (float)(servoDeg - SERVO_CENTER) / (float)(SERVO_LEFT_MAX - SERVO_CENTER); // ±1
    ratio = clampf(ratio, -1.0f, 1.0f);
    return ratio * STEER_MAX_RAD;
}

int MicroRosBridge::steerRadToServoDeg(float deltaRad) {
    float d = clampf(deltaRad, -STEER_MAX_RAD, STEER_MAX_RAD);
    float ratio = d / STEER_MAX_RAD; // [-1..1]
    int servo = (int)(SERVO_CENTER + ratio * (SERVO_LEFT_MAX - SERVO_CENTER)); // 30 deg span
    if (servo > SERVO_LEFT_MAX) servo = SERVO_LEFT_MAX;
    if (servo < SERVO_RIGHT_MAX) servo = SERVO_RIGHT_MAX;
    return servo;
}

int MicroRosBridge::speedMpsToPwmAdaptive(float v_mps, float curvature_abs) {
    float sign = (v_mps >= 0.0f) ? 1.0f : -1.0f;
    float av = fabsf(v_mps);

    if (av < 1e-3f) return 0;

    // tối ưu quẹo: độ cong lớn => giảm vận tốc lệnh để giữ bám đường
    // curvature_abs xấp xỉ |tan(delta)|/L
    float curveFactor = 1.0f / (1.0f + 0.35f * curvature_abs);
    curveFactor = clampf(curveFactor, 0.45f, 1.0f);

    float v_eff = av * curveFactor;

    // pwm base theo mô hình online
    float pwm = v_eff * _pwmPerMps * _surfaceDragGain;

    // đảm bảo thắng ma sát tĩnh
    if (pwm > 1.0f && pwm < PWM_MIN_RUN) pwm = PWM_MIN_RUN;

    pwm = clampf(pwm, 0.0f, (float)PWM_MAX);
    return (int)(sign * pwm);
}

void MicroRosBridge::updateAdaptiveTraction(float target_mps, float measured_mps, float curvature_abs) {
    float at = fabsf(target_mps);
    if (at < 0.05f) return; // bỏ qua vùng rất thấp

    float am = fabsf(measured_mps);
    float err = at - am; // dương = thiếu lực kéo
    _emaSpeedErr = 0.9f * _emaSpeedErr + 0.1f * err;

    // học chậm, ổn định
    float k_drag = 0.02f;
    float k_pwm  = 1.5f;

    // khi quẹo gắt, cho phép tăng bù mạnh hơn chút
    float curveBoost = 1.0f + clampf(curvature_abs * 0.15f, 0.0f, 0.35f);

    _surfaceDragGain += k_drag * _emaSpeedErr * curveBoost;
    _surfaceDragGain = clampf(_surfaceDragGain, 0.80f, 1.45f);

    _pwmPerMps += k_pwm * _emaSpeedErr * curveBoost;
    _pwmPerMps = clampf(_pwmPerMps, 280.0f, 620.0f);
}

void MicroRosBridge::cmdVelCallback(const void* msgin) {
    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
    _lastCmdVelMs = millis();

    float v = (float)msg->linear.x;   // m/s
    float w = (float)msg->angular.z;  // rad/s

    // Ackermann inverse kinematics
    float delta = 0.0f;
    if (fabsf(v) > 1e-3f) {
        delta = atanf((WHEELBASE_M * w) / v);
    } else {
        // v ~ 0: không thể quay tại chỗ đúng nghĩa Ackermann
        // chọn steering theo hướng w để chuẩn bị quẹo khi có tiến/lùi
        delta = (w >= 0.0f) ? (0.35f * STEER_MAX_RAD) : (-0.35f * STEER_MAX_RAD);
        v = 0.0f;
    }
    delta = clampf(delta, -STEER_MAX_RAD, STEER_MAX_RAD);

    int servo = steerRadToServoDeg(delta);

    // curvature for speed optimization
    float curvature_abs = fabsf(tanf(delta) / WHEELBASE_M);

    int pwm = speedMpsToPwmAdaptive(v, curvature_abs);

    MotorController::setTargetSteerServoAngle(servo);
    MotorController::setTargetSpeed(pwm);
}

void MicroRosBridge::odomTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;
    publishOdometryAndTf();
    publishImu();
    publishDiagnostics();
}

void MicroRosBridge::publishOdometryAndTf() {
    uint32_t nowMs = millis();
    float dt = (_lastMs == 0) ? 0.02f : (nowMs - _lastMs) / 1000.0f;
    _lastMs = nowMs;
    if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

    // measured speed from encoder
    float v = EncoderSensor::getSpeedCmPerSec() / 100.0f; // m/s
    int servoDeg = MotorController::getSteerServoAngle();
    float delta = servoDegToSteerRad(servoDeg);

    float w = v * tanf(delta) / WHEELBASE_M;

    // integrate
    _x += v * cosf(_th) * dt;
    _y += v * sinf(_th) * dt;
    _th = normalizeAngle(_th + w * dt);

    // adapt traction estimator from current command vs measurement
    float target_v_est = (float)MotorController::getTargetSpeed() / max(_pwmPerMps, 1.0f);
    float curvature_abs = fabsf(tanf(delta) / WHEELBASE_M);
    updateAdaptiveTraction(target_v_est, v, curvature_abs);

    // timestamp
    rcl_time_point_value_t nowNs = 0;
    rcl_clock_t clock;
    rcl_clock_init(RCL_STEADY_TIME, &clock, &_allocator);
    rcl_clock_get_now(&clock, &nowNs);

    // fill odom
    _odomMsg.header.stamp.sec = (int32_t)(nowNs / 1000000000ULL);
    _odomMsg.header.stamp.nanosec = (uint32_t)(nowNs % 1000000000ULL);

    _odomMsg.header.frame_id.data = (char*)"odom";
    _odomMsg.header.frame_id.size = 4;
    _odomMsg.header.frame_id.capacity = 5;

    _odomMsg.child_frame_id.data = (char*)"base_link";
    _odomMsg.child_frame_id.size = 9;
    _odomMsg.child_frame_id.capacity = 10;

    _odomMsg.pose.pose.position.x = _x;
    _odomMsg.pose.pose.position.y = _y;
    _odomMsg.pose.pose.position.z = 0.0;

    float cy = cosf(_th * 0.5f);
    float sy = sinf(_th * 0.5f);
    _odomMsg.pose.pose.orientation.x = 0.0;
    _odomMsg.pose.pose.orientation.y = 0.0;
    _odomMsg.pose.pose.orientation.z = sy;
    _odomMsg.pose.pose.orientation.w = cy;

    _odomMsg.twist.twist.linear.x = v;
    _odomMsg.twist.twist.linear.y = 0.0;
    _odomMsg.twist.twist.angular.z = w;

    // covariance cơ bản (bạn tune dần)
    for (int i = 0; i < 36; i++) {
        _odomMsg.pose.covariance[i] = 0.0;
        _odomMsg.twist.covariance[i] = 0.0;
    }
    _odomMsg.pose.covariance[0] = 0.03;   // x
    _odomMsg.pose.covariance[7] = 0.03;   // y
    _odomMsg.pose.covariance[35] = 0.08;  // yaw
    _odomMsg.twist.covariance[0] = 0.04;  // vx
    _odomMsg.twist.covariance[35] = 0.1;  // wz

    rcl_publish(&_pubOdom, &_odomMsg, NULL);

    // TF odom -> base_link
    _tfStamped.header.stamp = _odomMsg.header.stamp;
    _tfStamped.header.frame_id = _odomMsg.header.frame_id;
    _tfStamped.child_frame_id = _odomMsg.child_frame_id;

    _tfStamped.transform.translation.x = _x;
    _tfStamped.transform.translation.y = _y;
    _tfStamped.transform.translation.z = 0.0;
    _tfStamped.transform.rotation.x = 0.0;
    _tfStamped.transform.rotation.y = 0.0;
    _tfStamped.transform.rotation.z = sy;
    _tfStamped.transform.rotation.w = cy;

    _tfMsg.transforms.data = &_tfStamped;
    _tfMsg.transforms.size = 1;
    _tfMsg.transforms.capacity = 1;

    rcl_publish(&_pubTf, &_tfMsg, NULL);

    rcl_clock_fini(&clock);
}

void MicroRosBridge::publishImu() {
    // MPU6050_light không trả gyro bias-comp chuẩn ROS IMU đầy đủ ở code hiện tại,
    // nên publish mức "usable raw-ish" + orientation unknown.
    rcl_time_point_value_t nowNs = 0;
    rcl_clock_t clock;
    rcl_clock_init(RCL_STEADY_TIME, &clock, &_allocator);
    rcl_clock_get_now(&clock, &nowNs);

    _imuMsg.header.stamp.sec = (int32_t)(nowNs / 1000000000ULL);
    _imuMsg.header.stamp.nanosec = (uint32_t)(nowNs % 1000000000ULL);
    _imuMsg.header.frame_id.data = (char*)"imu_link";
    _imuMsg.header.frame_id.size = 8;
    _imuMsg.header.frame_id.capacity = 9;

    // orientation unknown
    _imuMsg.orientation.x = 0.0;
    _imuMsg.orientation.y = 0.0;
    _imuMsg.orientation.z = 0.0;
    _imuMsg.orientation.w = 1.0;
    _imuMsg.orientation_covariance[0] = -1.0;

    // chỉ có accelY trong API hiện tại; giữ tối thiểu để downstream biết có data
    _imuMsg.linear_acceleration.x = 0.0;
    _imuMsg.linear_acceleration.y = MPUSensor::getCurrentAccelY() * 9.80665f; // g -> m/s^2 xấp xỉ
    _imuMsg.linear_acceleration.z = 0.0;

    _imuMsg.angular_velocity.x = 0.0;
    _imuMsg.angular_velocity.y = 0.0;
    _imuMsg.angular_velocity.z = 0.0;

    for (int i = 0; i < 9; i++) {
        _imuMsg.angular_velocity_covariance[i] = 0.0;
        _imuMsg.linear_acceleration_covariance[i] = 0.0;
    }
    _imuMsg.linear_acceleration_covariance[0] = 0.3;
    _imuMsg.linear_acceleration_covariance[4] = 0.3;
    _imuMsg.linear_acceleration_covariance[8] = 0.5;

    rcl_publish(&_pubImu, &_imuMsg, NULL);
    rcl_clock_fini(&clock);
}

void MicroRosBridge::publishDiagnostics() {
    // mode
    const char* modeStr = (CommandProcessor::getCurrentMode() == AUTONOMOUS) ? "AUTONOMOUS" : "MANUAL";
    _modeMsg.data.data = (char*)modeStr;
    _modeMsg.data.size = strlen(modeStr);
    _modeMsg.data.capacity = _modeMsg.data.size + 1;
    rcl_publish(&_pubRobotMode, &_modeMsg, NULL);

    // encoder speed cm/s
    _encSpeedMsg.data = EncoderSensor::getSpeedCmPerSec();
    rcl_publish(&_pubEncoderSpeed, &_encSpeedMsg, NULL);
}

bool MicroRosBridge::begin(HardwareSerial& serialPort) {
    if (_ready) return true;

    // set_microros_serial_transports(serialPort);
    set_microros_transports(); 
    delay(300);
    // Serial1.begin(115200, SERIAL_8N1, 16, 17);
    // set_microros_serial_transports(Serial1);

    // --- Xóa dòng code bị gạch đỏ và thay bằng đoạn này ---
    // rmw_uros_set_custom_transport(
    //     true,
    //     NULL,
    //     serial1_transport_open,
    //     serial1_transport_close,
    //     serial1_transport_write,
    //     serial1_transport_read
    // );
    // delay(300);
    // --------------------------------------------------------

    _allocator = rcl_get_default_allocator();

    rcl_ret_t rc = rclc_support_init(&_support, 0, NULL, &_allocator);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_node_init_default(&_node, "esp32_base_node", "", &_support);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_subscription_init_default(
        &_subCmdVel,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel");
    if (rc != RCL_RET_OK) return false;

    rc = rclc_publisher_init_default(
        &_pubOdom,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom");
    if (rc != RCL_RET_OK) return false;

    rc = rclc_publisher_init_default(
        &_pubTf,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "/tf");
    if (rc != RCL_RET_OK) return false;

    rc = rclc_publisher_init_default(
        &_pubImu,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data_raw");
    if (rc != RCL_RET_OK) return false;

    rc = rclc_publisher_init_default(
        &_pubRobotMode,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/robot_mode");
    if (rc != RCL_RET_OK) return false;

    rc = rclc_publisher_init_default(
        &_pubEncoderSpeed,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/encoder/speed_cms");
    if (rc != RCL_RET_OK) return false;

    rc = rclc_timer_init_default(
        &_timerOdom,
        &_support,
        RCL_MS_TO_NS(20), // 50Hz
        odomTimerCallback);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_executor_init(&_executor, &_support.context, 2, &_allocator);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_executor_add_subscription(&_executor, &_subCmdVel, &_cmdMsg, &cmdVelCallback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) return false;

    rc = rclc_executor_add_timer(&_executor, &_timerOdom);
    if (rc != RCL_RET_OK) return false;

    _lastMs = millis();
    _lastCmdVelMs = millis();
    _ready = true;
    return true;
}

void MicroRosBridge::spinSome() {
    if (!_ready) return;
    rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(3));
}

void MicroRosBridge::tick() {
    if (!_ready) return;

    spinSome();

    // cmd_vel timeout safety
    if (millis() - _lastCmdVelMs > CMD_VEL_TIMEOUT_MS) {
        MotorController::setTargetSpeed(0);
        MotorController::setTargetSteerServoAngle(SERVO_CENTER);
    }
}

bool MicroRosBridge::isReady() {
    return _ready;
}

void MicroRosBridge::resetOdometry() {
    _x = 0.0f;
    _y = 0.0f;
    _th = 0.0f;
    _lastMs = millis();
}