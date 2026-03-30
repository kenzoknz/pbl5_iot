#include <micro_ros_arduino.h>
#include <Arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>

// ====== Gắn với code hiện tại của bạn ======
#include "MotorController.h"
#include "EncoderSensor.h"

// ---------- Robot params ----------
static const float WHEELBASE_L = 0.22f;          // m (đo thực tế)
static const float STEER_MAX_RAD = 0.61f;        // ~35 deg
static const int   SERVO_STRAIGHT = 90;
static const float SERVO_RAD_PER_DEG = (STEER_MAX_RAD / 35.0f);

// Odometry state
float x_ = 0.0f, y_ = 0.0f, th_ = 0.0f;
uint32_t last_ms_ = 0;

// micro-ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t sub_cmd_vel;
rcl_publisher_t pub_odom;
rcl_publisher_t pub_tf;
rcl_timer_t timer_odom;
rclc_executor_t executor;

geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped tf_stamped;

// Helpers
float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi ? hi : v); }

int speedMpsToPwm(float v_mps) {
  // TODO: hiệu chuẩn từ dữ liệu encoder thực tế
  // map đơn giản ví dụ:
  float sign = (v_mps >= 0) ? 1.0f : -1.0f;
  float a = fabs(v_mps);
  int pwm = (int)(a * 450.0f);    // hệ số thử nghiệm
  if (pwm > 255) pwm = 255;
  if (pwm < 0) pwm = 0;
  return (int)(sign * pwm);
}

float getSteerRadFromServoDeg(int servo_deg) {
  int d = servo_deg - SERVO_STRAIGHT; // trái/phải quanh 90
  return d * SERVO_RAD_PER_DEG;
}

void cmdVelCallback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  float v = msg->linear.x;      // m/s
  float w = msg->angular.z;     // rad/s

  // Ackermann: delta = atan(L * w / v)
  float delta = 0.0f;
  if (fabs(v) > 1e-3f) {
    delta = atanf((WHEELBASE_L * w) / v);
  } else {
    delta = 0.0f; // tránh chia 0
  }
  delta = clampf(delta, -STEER_MAX_RAD, STEER_MAX_RAD);

  // đổi ra servo angle
  int servo = SERVO_STRAIGHT + (int)(delta / SERVO_RAD_PER_DEG);
  servo = constrain(servo, 55, 125);

  // tốc độ motor
  int pwm = speedMpsToPwm(v);

  MotorController::setTargetSteerServoAngle(servo);
  MotorController::setTargetSpeed(pwm);
}

void fillAndPublishTf(rcl_time_point_value_t now_ns) {
  // header
  tf_stamped.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
  tf_stamped.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);

  // frame ids (cấp phát 1 lần đơn giản)
  static bool inited = false;
  if (!inited) {
    tf_stamped.header.frame_id.data = (char*)"odom";
    tf_stamped.header.frame_id.size = 4;
    tf_stamped.header.frame_id.capacity = 5;
    tf_stamped.child_frame_id.data = (char*)"base_link";
    tf_stamped.child_frame_id.size = 9;
    tf_stamped.child_frame_id.capacity = 10;
    tf_msg.transforms.data = &tf_stamped;
    tf_msg.transforms.size = 1;
    tf_msg.transforms.capacity = 1;
    inited = true;
  }

  tf_stamped.transform.translation.x = x_;
  tf_stamped.transform.translation.y = y_;
  tf_stamped.transform.translation.z = 0.0;

  // yaw -> quaternion
  float cy = cosf(th_ * 0.5f), sy = sinf(th_ * 0.5f);
  tf_stamped.transform.rotation.x = 0.0;
  tf_stamped.transform.rotation.y = 0.0;
  tf_stamped.transform.rotation.z = sy;
  tf_stamped.transform.rotation.w = cy;

  rcl_publish(&pub_tf, &tf_msg, NULL);
}

void odomTimerCb(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;

  uint32_t now_ms = millis();
  float dt = (last_ms_ == 0) ? 0.02f : (now_ms - last_ms_) / 1000.0f;
  last_ms_ = now_ms;
  if (dt <= 0.0f || dt > 0.2f) dt = 0.02f;

  // v thực từ encoder (cm/s -> m/s)
  float v = EncoderSensor::getSpeedCmPerSec() / 100.0f;

  // delta từ servo đang lái
  float delta = getSteerRadFromServoDeg(MotorController::getSteerServoAngle());

  // Ackermann yaw rate
  float w = 0.0f;
  if (fabs(cosf(delta)) > 1e-3f) {
    w = v * tanf(delta) / WHEELBASE_L;
  }

  // integrate
  x_ += v * cosf(th_) * dt;
  y_ += v * sinf(th_) * dt;
  th_ += w * dt;

  // publish odom
  rcl_time_point_value_t now_ns;
  rcl_clock_t clock;
  rcl_clock_init(RCL_STEADY_TIME, &clock, &allocator);
  rcl_clock_get_now(&clock, &now_ns);

  odom_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
  odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);
  odom_msg.header.frame_id.data = (char*)"odom";
  odom_msg.header.frame_id.size = 4;
  odom_msg.header.frame_id.capacity = 5;
  odom_msg.child_frame_id.data = (char*)"base_link";
  odom_msg.child_frame_id.size = 9;
  odom_msg.child_frame_id.capacity = 10;

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;
  float cy = cosf(th_ * 0.5f), sy = sinf(th_ * 0.5f);
  odom_msg.pose.pose.orientation.z = sy;
  odom_msg.pose.pose.orientation.w = cy;

  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = w;

  rcl_publish(&pub_odom, &odom_msg, NULL);
  fillAndPublishTf(now_ns);
  rcl_clock_fini(&clock);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  MotorController::begin();
  EncoderSensor::begin();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_base_node", "", &support);

  rclc_subscription_init_default(
    &sub_cmd_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");

  rclc_publisher_init_default(
    &pub_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom");

  rclc_publisher_init_default(
    &pub_tf, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf");

  rclc_timer_init_default(&timer_odom, &support, RCL_MS_TO_NS(20), odomTimerCb); // 50Hz

  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &cmd_msg, &cmdVelCallback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer_odom);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
  delay(5);
}