#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <ESP32Servo.h>



#define RCCHECK(fn) ((fn) == RCL_RET_OK)

#define LED_PIN_CONNECTED 2 // LED to indicate connection status

int esc1_pin = 19;
int light1_pin = 15;
int light_couple_pin = 17;
int cam_servo_pin = 18;

Servo esc1; // ESC controllers
Servo cam_servo; // Pan-tilt servo
Servo light1; // Lights
Servo light_couple;

rcl_publisher_t publisher;
std_msgs__msg__Int32 int_msg;

// Subscribers and messages for motors
rcl_subscription_t motor_sub;
std_msgs__msg__Int32 motor_msg;

// Subscribers and messages for lights
rcl_subscription_t light_sub, light_couple_sub;
std_msgs__msg__Int32 light_msg, light_couple_msg;

// Subscriber and message for camera servo
rcl_subscription_t cam_servo_sub;
std_msgs__msg__Int32 cam_servo_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool connected = false;

void subscription_callback_motor(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int val = msg->data;
  if (val >= 1100 && val <= 1900) {
    esc1.writeMicroseconds(val); // Send PWM signal to ESC/motor.
  }
}

void subscription_callback_light(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int val = msg->data;
  if (val >= 1100 && val <= 1900) {
    light1.writeMicroseconds(val); // Send PWM signal to lights.
  }
}

void subscription_callback_light_couple(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int val = msg->data;
  if (val >= 1100 && val <= 1900) {
    light_couple.writeMicroseconds(val); // Send PWM signal to lights.
  }
}

void subscription_callback_cam_servo(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int val = msg->data;
  if (val >= 0 && val <= 360) {
    cam_servo.write(val); // Send PWM signal to cam_servo [deg]
  }
}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL && connected) {
    int_msg.data++;
    if (!RCCHECK(rcl_publish(&publisher, &int_msg, NULL))) {
      connected = false; // Set connected to false if publish fails
    }
  }
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
  if (!RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "micro_ros_pub"))) return false;
  if (!RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(500), timer_callback))) return false;
  if (!RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator))) return false;
  if (!RCCHECK(rclc_executor_add_timer(&executor, &timer))) return false;

  // Motor Data Subscriber
  RCCHECK(rclc_subscription_init_default(
      &motor_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/motor_data"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &motor_sub, &motor_msg,
      subscription_callback_motor, ON_NEW_DATA));

  // Light Subscriber
  RCCHECK(rclc_subscription_init_default(
      &light_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/light_data"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &light_sub, &light_msg,
      subscription_callback_light, ON_NEW_DATA));

  // Light (Couple) Subscriber
  RCCHECK(rclc_subscription_init_default(
      &light_couple_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/light_couple_data"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &light_couple_sub, &light_couple_msg,
      subscription_callback_light_couple, ON_NEW_DATA));

  // Camera Servo Subscriber
  RCCHECK(rclc_subscription_init_default(
      &cam_servo_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/cam_servo"));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cam_servo_sub, &cam_servo_msg,
      subscription_callback_cam_servo, ON_NEW_DATA));


  int_msg.data = 0;
  return true;
}

void setup() {
  // ESCs setup
  esc1.attach(esc1_pin);  // Replace with actual GPIO numbers

  // Lights setup
  light1.attach(light1_pin);

  light_couple.attach(light_couple_pin);

  // Pan-tilt servo setup
  cam_servo.attach(cam_servo_pin);


  // Initial states
  // ESCs in "stop" position
  esc1.writeMicroseconds(1500);


  // Lights off
  light1.writeMicroseconds(1500);
  light_couple.writeMicroseconds(1100);

  cam_servo.write(0);


  Serial.begin(115200);
  delay(3000); // Delay to allow the ESC to recognize the stopped signal
  
  connected = setup_node_and_entities();
  if (!connected) {
    // Attempt reconnection here, similar to your previous logic
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
