#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

// Định nghĩa chân điều khiển L298N
#define ENA 13
#define IN1 5
#define IN2 4
#define ENB 14
#define IN3 18
#define IN4 19
// Định nghĩa chân encoder
#define ENCODER_L_A 26 // Đổi từ 21 để tránh xung đột I2C
#define ENCODER_L_B 27 // Đổi từ 22 để tránh xung đột I2C
#define ENCODER_R_A 23
#define ENCODER_R_B 25
// Định nghĩa chân LED
#define LED_PIN 2 // ESP32 built-in LED pin

// Thông số robot
const float WHEEL_RADIUS = 0.02;          // Bán kính bánh xe (m)
const float WHEEL_BASE = 0.1;             // Khoảng cách giữa hai bánh (m)
const float TICKS_PER_REV = 1050;         // Số xung encoder mỗi vòng quay
const float WHEEL_CIRCUMFERENCE = 2 * PI * WHEEL_RADIUS;
const int PWM_THRESHOLD = 150;            // Ngưỡng PWM để động cơ bắt đầu quay

// Hằng số PID
double kp_l = 1.8, ki_l = 5.0, kd_l = 0.01; // PID động cơ trái
double kp_r = 2.25, ki_r = 5.0, kd_r = 0.01; // PID động cơ phải

// Biến encoder
volatile long encoder_count_l = 0, encoder_count_r = 0;
double vel_l = 0.0, vel_r = 0.0; // Tốc độ bánh xe (m/s)

// Biến PID
double target_vel_l = 0, target_vel_r = 0;
double pwm_l = 0, pwm_r = 0;
PID pid_l(&vel_l, &pwm_l, &target_vel_l, kp_l, ki_l, kd_l, DIRECT);
PID pid_r(&vel_r, &pwm_r, &target_vel_r, kp_r, ki_r, kd_r, DIRECT);

// Biến micro-ROS
rcl_subscription_t subscriber;
rcl_publisher_t odom_publisher, imu_publisher;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// IMU
Adafruit_MPU6050 mpu;
bool is_imu_connected = false;
unsigned long last_time = 0;

// Kiểm tra lỗi micro-ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("Publish failed");}}

// Hàm báo lỗi
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Hàm ngắt encoder
void IRAM_ATTR encoder_isr_l() {
  if (digitalRead(ENCODER_L_B) == digitalRead(ENCODER_L_A)) encoder_count_l++;
  else encoder_count_l--;
}
void IRAM_ATTR encoder_isr_r() {
  if (digitalRead(ENCODER_R_B) == digitalRead(ENCODER_R_A)) encoder_count_r++;
  else encoder_count_r--;
}

// Điều khiển động cơ
void setMotor(int motor, int speed, bool forward) {
  int in1_pin = (motor == 0) ? IN1 : IN3;
  int in2_pin = (motor == 0) ? IN2 : IN4;
  int en_pin = (motor == 0) ? ENA : ENB;

  digitalWrite(in1_pin, forward ? HIGH : LOW);
  digitalWrite(in2_pin, forward ? LOW : HIGH);
  ledcWrite(motor, constrain(abs(speed) + PWM_THRESHOLD, 0, 255));
}

// Callback nhận lệnh /cmd_vel
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  // Chuyển đổi sang tốc độ bánh xe (m/s)
  target_vel_l = linear - (angular * WHEEL_BASE / 2.0);
  target_vel_r = linear + (angular * WHEEL_BASE / 2.0);
}

// Lấy thời gian hiện tại
void get_current_time(builtin_interfaces__msg__Time *time_msg) {
  int64_t nanos = rmw_uros_epoch_nanos();
  time_msg->sec = nanos / 1000000000;
  time_msg->nanosec = nanos % 1000000000;
}

// Callback xuất bản IMU
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && is_imu_connected) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Chuyển đổi dữ liệu giống mã Python
    get_current_time(&imu_msg.header.stamp);
    imu_msg.header.frame_id.data = (char *)"base_footprint";
    imu_msg.header.frame_id.size = strlen("base_footprint");
    imu_msg.header.frame_id.capacity = imu_msg.header.frame_id.size + 1;
    imu_msg.linear_acceleration.x = a.acceleration.x / 1670.13 * 9.81; // Chuyển LSB sang m/s²
    imu_msg.linear_acceleration.y = a.acceleration.y / 1670.13 * 9.81;
    imu_msg.linear_acceleration.z = a.acceleration.z / 1670.13 * 9.81;
    imu_msg.angular_velocity.x = g.gyro.x / 7509.55; // Chuyển LSB sang rad/s
    imu_msg.angular_velocity.y = g.gyro.y / 7509.55;
    imu_msg.angular_velocity.z = g.gyro.z / 7509.55;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}

// Xuất bản odometry
void publish_odometry() {
  static float x = 0.0, y = 0.0, theta = 0.0;
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  // Tính toán tốc độ từ encoder
  float delta_ticks_l = encoder_count_l;
  float delta_ticks_r = encoder_count_r;
  encoder_count_l = 0;
  encoder_count_r = 0;

  float dist_l = (delta_ticks_l / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
  float dist_r = (delta_ticks_r / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
  vel_l = dist_l / dt;
  vel_r = dist_r / dt;

  // Tính toán odometry
  float v = (vel_r + vel_l) / 2.0;
  float omega = (vel_r - vel_l) / WHEEL_BASE;
  x += v * cos(theta) * dt;
  y += v * sin(theta) * dt;
  theta += omega * dt;

  // Điền thông điệp odometry
  get_current_time(&odom_msg.header.stamp);
  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.header.frame_id.size = strlen("odom");
  odom_msg.header.frame_id.capacity = odom_msg.header.frame_id.size + 1;
  odom_msg.child_frame_id.data = (char *)"base_link";
  odom_msg.child_frame_id.size = strlen("base_link");
  odom_msg.child_frame_id.capacity = odom_msg.child_frame_id.size + 1;
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
  odom_msg.twist.twist.linear.x = v;
  odom_msg.twist.twist.angular.z = omega;

  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void setup() {
  // Khởi tạo Serial để debug
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // Thiết lập chân
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENCODER_L_A, INPUT);
  pinMode(ENCODER_L_B, INPUT);
  pinMode(ENCODER_R_A, INPUT);
  pinMode(ENCODER_R_B, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // Thiết lập PWM
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(ENA, 0);
  ledcAttachPin(ENB, 1);

  // Thiết lập ngắt encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoder_isr_l, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoder_isr_r, RISING);

  // Khởi tạo PID
  pid_l.SetMode(AUTOMATIC);
  pid_l.SetSampleTime(10); // 10ms
  pid_l.SetOutputLimits(-255 + PWM_THRESHOLD, 255 - PWM_THRESHOLD);
  pid_r.SetMode(AUTOMATIC);
  pid_r.SetSampleTime(10); // 10ms
  pid_r.SetOutputLimits(-255 + PWM_THRESHOLD, 255 - PWM_THRESHOLD);

  // Khởi tạo IMU
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Failed to find MPU6050 chip");
    is_imu_connected = false;
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setSampleRateDivisor(7);
  is_imu_connected = true;

  // Khởi tạo micro-ROS
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));

  // Tạo subscriber /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Tạo publisher /odom
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // Tạo publisher /imu/data
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

  // Tạo timer cho IMU (100Hz)
  const unsigned int timer_timeout = 10; // 10ms = 100Hz
  RCCHECK(rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // Tạo executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  // Tính toán tốc độ từ encoder để cập nhật input cho PID
  float delta_ticks_l = encoder_count_l;
  float delta_ticks_r = encoder_count_r;
  encoder_count_l = 0;
  encoder_count_r = 0;

  float dist_l = (delta_ticks_l / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
  float dist_r = (delta_ticks_r / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
  vel_l = dist_l / dt;
  vel_r = dist_r / dt;

  // Tính toán PID
  pid_l.Compute();
  pid_r.Compute();

  // Điều khiển động cơ
  setMotor(0, (int)pwm_l, pwm_l >= 0);
  setMotor(1, (int)pwm_r, pwm_r >= 0);

  // Xuất bản odometry
  publish_odometry();

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
