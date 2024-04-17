#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) ((fn) == RCL_RET_OK)
#define LED_PIN_CONNECTED 2

void publish_sensor_data_task(void *pvParameters);
void subscribe_and_command_task(void *pvParameters);
void connectivity_management_task(void *pvParameters);

//========================== Pinout =========================================================
Servo esc1, esc2, esc3, esc4, esc5;
Servo light1, light2, light_couple;
Servo cam_servo;
int esc_pins[] = {19, 23, 25, 27, 29};
int light_pins[] = {15, 30, 17}; // light1, light2, couple
int cam_servo_pin = 18;
//========================== Publishers definitions =========================================
rcl_publisher_t publisher;
std_msgs__msg__Int32 int_msg;
//========================== Subscribers definitions ========================================
// Subscribers for motors, lights, and cam_servo
rcl_subscription_t motor_sub, light_sub, cam_servo_sub;
std_msgs__msg__Int32 motor_msg, light_msg, cam_servo_msg;
//========================== ROS variables ==================================================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool connected = false;
//========================== Subscribers Callbacks ==========================================
void subscription_callback_motor(const void * msgin) {
  Serial.println("In motor1 callback");
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int val = msg->data;
  if (val >= 1100 && val <= 1900) {
    esc1.writeMicroseconds(val); // Send PWM signal to ESC/motor.
  }
}

void subscription_callback_light(const void * msgin) {
  Serial.println("In light1 callback");
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int val = msg->data;
  if (val >= 1100 && val <= 1900) {
    light1.writeMicroseconds(val); // Send PWM signal to lights.
  }
}

void subscription_callback_cam_servo(const void * msgin) {
    Serial.println("In cam_servo callback");
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    cam_servo.write(msg->data);
}
//========================== Thread Functions ===============================================
void connectivity_management_task(void *pvParameters) {
    while(1) {
        if (!connected) {
            Serial.println("Trying to reconnect...");
            attempt_reconnection();
        }
        Serial.println("Connected!");
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 seconds
    }
}
void subscribe_and_command_task(void *pvParameters) {
    while(1) {
        if (connected) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void publish_sensor_data_task(void *pvParameters) {
    while(1) {
        if (connected) {
            int_msg.data++;
            std_msgs__msg__Int32 msg;
            msg.data = int_msg.data; // Example sensor data
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
            Serial.println("Published sensor data");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 sec timer
    }
}
//========================== ROS initialization =============================================
void setup_node_and_entities() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    if (!RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))) Serial.print("Failed: ");
    if (!RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support))) Serial.print("Failed: ");

    // Initialize publisher for sensor data (as an example)
    if (!RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/sensor_data"))) Serial.print("Failed: ");
    // Initialize subscriber for motors
    if (!RCCHECK(rclc_subscription_init_default(&motor_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/motor_data"))) Serial.print("Failed: ");
    if (!RCCHECK(rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, subscription_callback_motor, ON_NEW_DATA))) Serial.print("Failed: ");
    // Initialize subscriber for lights
    if (!RCCHECK(rclc_subscription_init_default(&light_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/light_data"))) Serial.print("Failed: ");
    if (!RCCHECK(rclc_executor_add_subscription(&executor, &light_sub, &light_msg, subscription_callback_light, ON_NEW_DATA))) Serial.print("Failed: ");
    // Initialize subscriber for camera servo
    if (!RCCHECK(rclc_subscription_init_default(&cam_servo_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/cam_servo_data"))) Serial.print("Failed: ");
    if (!RCCHECK(rclc_executor_add_subscription(&executor, &cam_servo_sub, &cam_servo_msg, subscription_callback_cam_servo, ON_NEW_DATA))) Serial.print("Failed: ");

    // Timer initialization if needed... (publisher does not need timer anymore in rtos)

    // Executor initialization with adjusted load
    if (!RCCHECK(rclc_executor_init(&executor, &support.context, 8, &allocator))) Serial.print("Failed: "); // Adjust the executor size accordingly

    int_msg.data = 0;
    connected = true;
}
//========================== Reconnect Function =============================================
void attempt_reconnection() {
    setup_node_and_entities();
    Serial.println("Reconnected successfully");
}
//========================== System Setup ===================================================
void setup() {
    Serial.println("In Setup");
    pinMode(LED_PIN_CONNECTED, OUTPUT);
    digitalWrite(LED_PIN_CONNECTED, LOW);
    // Attach motor controllers:
    esc1.attach(esc_pins[0]);
    //esc2.attach(esc_pins[1]);
    //esc3.attach(esc_pins[2]);
    //esc4.attach(esc_pins[3]);
    //esc5.attach(esc_pins[4]);
    // Attach lights:
    light1.attach(light_pins[0]);
    //light2.attach(light_pins[1]);
    //light_couple.attach(light_pins[2]);
    // Attach Pan-tilt servo:
    cam_servo.attach(cam_servo_pin);

    Serial.begin(115200);
    delay(1000); // Delay to allow the ESC to recognize the stopped signal
    // Initial states
    // ESCs in "stop" position
    esc1.writeMicroseconds(1500);

    // Lights off
    light1.writeMicroseconds(1100);
    //light_couple.writeMicroseconds(1100);timer_callback
    // Create FreeRTOS tasks
    //                        name      ,    thread_func     , stack,param,prior,handle
    xTaskCreate(subscribe_and_command_task, "SubscribeAndCommand", 6000, NULL, 3, NULL);
    xTaskCreate(publish_sensor_data_task, "PublishSensorData", 6000, NULL, 2, NULL);
    xTaskCreate(connectivity_management_task, "ConnectivityManagement", 6000, NULL, 1, NULL);
}

void loop() {
    // Empty - Tasks are managed by FreeRTOS
  }
