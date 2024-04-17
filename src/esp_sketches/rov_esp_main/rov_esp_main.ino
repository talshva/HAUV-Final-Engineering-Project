#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define RCCHECK(fn) ((fn) == RCL_RET_OK)
#define LED_PIN_CONNECTED 2 // LED to indicate connection status

int esc1_pin = 5;
int esc2_pin = 19;
int light1_pin = 15;
int light_couple_pin = 17;
int cam_servo_pin = 18;

Servo esc1, esc2, esc3, esc4, esc5; // ESC controllers
Servo cam_servo; // Pan-tilt servo
Servo light1; // Lights
Servo light_couple;

Adafruit_MPU6050 mpu;

//Adafruit_BME280 bme; // I2C

// Publisher and messages for imu
rcl_publisher_t gyro_accel_pub;
geometry_msgs__msg__Twist gyro_accel_msg;

// Subscribers and messages for motors
rcl_subscription_t motors_sub;
geometry_msgs__msg__Twist motors_msg;

// Subscribers and messages for lights
rcl_subscription_t lights_sub;
geometry_msgs__msg__Vector3 lights_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool connected = false;


void subscription_callback_motors(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  int motor1_val = static_cast<int>(msg->linear.x);
  int motor2_val = static_cast<int>(msg->linear.y);
  int motor3_val = static_cast<int>(msg->linear.z);
  int motor4_val = static_cast<int>(msg->angular.x);
  int motor5_val = static_cast<int>(msg->angular.y);
  int cam_servo_val = static_cast<int>(msg->angular.z);

  if (motor1_val >= 1100 && motor1_val <= 1900) {
    esc1.writeMicroseconds(motor1_val); // Send PWM signal to ESC/motor 1.
  }
  if (motor2_val >= 1100 && motor2_val <= 1900) {
    esc2.writeMicroseconds(motor2_val); // Send PWM signal to ESC/motor 2.
  }
  if (motor3_val >= 1100 && motor3_val <= 1900) {
    esc3.writeMicroseconds(motor3_val); // Send PWM signal to ESC/motor 3.
  }
  if (motor4_val >= 1100 && motor4_val <= 1900) {
    esc4.writeMicroseconds(motor4_val); // Send PWM signal to ESC/motor 4.
  }
  if (motor5_val >= 1100 && motor5_val <= 1900) {
    esc5.writeMicroseconds(motor5_val); // Send PWM signal to ESC/motor 5.
  }
  cam_servo.write(cam_servo_val); // Sendval PWM signal to camera servo.
}


void subscription_callback_lights(const void * msgin) {
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
  int light1_val = static_cast<int>(msg->x);
  int light2_val = static_cast<int>(msg->y);
  int light_couple_val = static_cast<int>(msg->z);

  if (light1_val >= 1100 && light1_val <= 1900) {
    light1.writeMicroseconds(light1_val); // Send PWM signal to lights.
  }
  if (light_couple_val >= 1100 && light_couple_val <= 1900) {
    light_couple.writeMicroseconds(light_couple_val); // Send PWM signal to lights.
  }
}

void gyro_accel_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  /*RCLC_UNUSED(last_call_time);
  if (timer != NULL && connected) {
    float temperature = bme.readTemperature();
    int_msg.data = static_cast<int>(temperature * 100); // Convert to integer to avoid float issues in ROS messages
    if (!RCCHECK(rcl_publish(&publisher, &int_msg, NULL))) {
      connected = false; // Set connected to false if publish fails
    }
  }
  */

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && connected) {
    gyro_accel_msg.linear.x = static_cast<double>(a.acceleration.x);
    gyro_accel_msg.linear.y = static_cast<double>(a.acceleration.y);
    gyro_accel_msg.linear.z = static_cast<double>(a.acceleration.z);
    gyro_accel_msg.angular.x = static_cast<double>(g.gyro.x);
    gyro_accel_msg.angular.y = static_cast<double>(g.gyro.y);
    gyro_accel_msg.angular.z = static_cast<double>(g.gyro.z);

    if (!RCCHECK(rcl_publish(&gyro_accel_pub, &gyro_accel_msg, NULL))) {
      connected = false; // Set connected to false if publish fails
    }
  }
    // Serial.print("Temperature: ");
    // Serial.print(temp.temperature);
    // Serial.println(" degC");
  }


void attempt_reconnection() {
    while (!connected) {
        digitalWrite(LED_PIN_CONNECTED, HIGH); delay(500);
        digitalWrite(LED_PIN_CONNECTED, LOW); delay(500);
        // Try to setup node and entities for reconnection
        connected = setup_node_and_entities();
        // Optionally, limit the number of retries or implement a backoff strategy
    }
    // Once connected, keep the LED on
    digitalWrite(LED_PIN_CONNECTED, HIGH);
}

bool setup_node_and_entities() {
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  if (!RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))) return false;
  if (!RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support))) return false;
  if (!RCCHECK(rclc_publisher_init_default(&gyro_accel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/gyro_accel_data"))) return false;
  if (!RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(20), gyro_accel_timer_callback))) return false;
  if (!RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator))) return false;
  if (!RCCHECK(rclc_executor_add_timer(&executor, &timer))) return false;

  // Motor Data Subscriber
  RCCHECK(rclc_subscription_init_default(
      &motors_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/motor_data"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &motors_sub, &motors_msg,
      subscription_callback_motors, ON_NEW_DATA));

  // Light Subscriber
  RCCHECK(rclc_subscription_init_default(
      &lights_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/light_data"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &lights_sub, &lights_msg,
      subscription_callback_lights, ON_NEW_DATA));

  return true;
}

void setup() {
  // ESCs setup
  esc1.attach(esc1_pin);  // Replace with actual GPIO numbers
  esc2.attach(esc2_pin);  // Replace with actual GPIO numbers

  // Lights setup
  light1.attach(light1_pin);

  light_couple.attach(light_couple_pin);

  // Pan-tilt servo setup
  cam_servo.attach(cam_servo_pin);

  // Initial states
  // ESCs in "stop" position
  esc1.writeMicroseconds(1500);
  esc2.writeMicroseconds(1500);

  // Lights on
  light1.writeMicroseconds(1100);
  light_couple.writeMicroseconds(1100);

  cam_servo.write(90);
  delay(1000);
  cam_servo.write(120);


  Serial.begin(115200);
  delay(3000); // Delay to allow the ESC to recognize the stopped signal
  
  connected = setup_node_and_entities();
  if (!connected) {
    // Attempt reconnection here, similar to your previous logic
  }

  //if (!bme.begin(0x76)) { // Initialize BME280 sensor
  //  Serial.println("Could not find a valid BME280 sensor, check wiring!");
  //} 

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  else {
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  }

}

void loop() {
  if (connected) {
    if (!RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)))) {
      connected = false; // If spin_some fails, connection is considered lost
    }
  } else {
    attempt_reconnection();
  }
  delay(10); // Small delay to avoid running the loop too fast
}
