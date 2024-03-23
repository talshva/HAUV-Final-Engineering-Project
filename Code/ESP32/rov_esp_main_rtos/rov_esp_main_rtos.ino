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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {Serial.println("Failed: " #fn); vTaskDelete(NULL);} }

#define LED_PIN_CONNECTED 2

//------------------------------ Pinout ---------------------------------
Servo esc1, light1, light_couple, cam_servo;
int esc1_pin = 19, light1_pin = 15, light_couple_pin = 17, cam_servo_pin = 18;
//---------------------- Publishers definitions -------------------------
rcl_publisher_t publisher;
std_msgs__msg__Int32 int_msg;
//---------------------- Subscribers definitions ------------------------
rcl_subscription_t motor_sub, light_sub, light_couple_sub, cam_servo_sub;
std_msgs__msg__Int32 motor_msg, light_msg, light_couple_msg, cam_servo_msg;
//--------------------------- ROS variables -----------------------------
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool connected = false;
//---------------------- Subscribers Callbacks ---------------------------
void subscription_callback_motor(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    esc1.writeMicroseconds(msg->data);
}
void subscription_callback_light(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    light1.writeMicroseconds(msg->data);
}
void subscription_callback_light_couple(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    light_couple.writeMicroseconds(msg->data);
}
void subscription_callback_cam_servo(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    cam_servo.write(msg->data);
}
//----------------------- Thread Functions ------------------------------
void connectivity_management_task(void *pvParameters) {
    while(1) {
        if (!connected) {
            Serial.println("Trying to reconnect...");
            attempt_reconnection();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
void subscribe_and_command_task(void *pvParameters) {
    while(1) {
        if (connected) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void publish_sensor_data_task(void *pvParameters) {
    while(1) {
        // Example: Publish dummy sensor data
        int_msg.data++;
        RCCHECK(rcl_publish(&publisher, &int_msg, NULL));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
//----------------------- ROS initialization ----------------------------
void setup_node_and_entities() {
    // Initialize micro-ROS
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));
    // Initialize publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/esp32/sensor_data"));
    // Initialize subscribers
    RCCHECK(rclc_subscription_init_default(
        &motor_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/motor_data"));
    RCCHECK(rclc_subscription_init_default(
        &light_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/light_data"));
    RCCHECK(rclc_subscription_init_default(
        &light_couple_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/light_couple_data"));
    RCCHECK(rclc_subscription_init_default(
        &cam_servo_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/cam_servo"));
    // Initialize executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, subscription_callback_motor, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &light_sub, &light_msg, subscription_callback_light, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &light_couple_sub, &light_couple_msg, subscription_callback_light_couple, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cam_servo_sub, &cam_servo_msg, subscription_callback_cam_servo, ON_NEW_DATA));
    connected = true; // Assume connected after successful setup
}
//----------------------- Reconnect Function ----------------------------
void attempt_reconnection() {
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));
    // Re-setup the entity here as per your need, similar to setup_node_and_entities();
    connected = true;
    Serial.println("Reconnected successfully");
}

//========================== System Setup ===============================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN_CONNECTED, OUTPUT);
    digitalWrite(LED_PIN_CONNECTED, LOW);
    // Attach servos to their respective pins
    esc1.attach(esc1_pin);
    light1.attach(light1_pin);
    light_couple.attach(light_couple_pin);
    cam_servo.attach(cam_servo_pin);
    // Initialize micro-ROS
    setup_node_and_entities();
      // Create FreeRTOS tasks
      //                        name          ,         thread_func     , stack,param,prior,handle
      xTaskCreate(connectivity_management_task, "ConnectivityManagement", 10000, NULL, 1, NULL);
      xTaskCreate(subscribe_and_command_task, "SubscribeAndCommand", 10000, NULL, 1, NULL);
      xTaskCreate(publish_sensor_data_task, "PublishSensorData", 10000, NULL, 1, NULL);
}

void loop() {
    // Empty - Tasks are managed by FreeRTOS
}

