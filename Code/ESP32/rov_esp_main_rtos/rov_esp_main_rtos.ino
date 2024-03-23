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
#include <std_msgs/msg/int32_multi_array.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {Serial.println("Failed: " #fn); vTaskDelete(NULL);} }

#define LED_PIN_CONNECTED 2

//========================== Pinout =========================================================
Servo esc1, esc2, esc3, esc4, esc5;
Servo light1, light2, light_couple;
Servo cam_servo;
int esc1_pin = 19, esc2_pin = 23, esc3_pin = 25, esc4_pin = 27, esc5_pin = 29;
int light1_pin = 15, light2_pin = 30, light_couple_pin = 17;
int cam_servo_pin = 18;
//========================== Publishers definitions =========================================
rcl_publisher_t publisher;
std_msgs_msg_Int32 int_msg;
//========================== Subscribers definitions ========================================
rcl_subscription_t motor_data_sub, light_sub, light_couple_sub, cam_servo_sub;
std_msgs_msg_Int32MultiArray motor_data_msg, light_data_msg;
std_msgs_msg_Int32 cam_servo_msg;
//========================== ROS variables ==================================================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool connected = false;
//========================== Subscribers Callbacks ==========================================
void subscription_callback_motor(const void * msgin) {
    const std_msgs_msgInt32MultiArray * msg = (const std_msgsmsg_Int32MultiArray *)msgin;
    esc1.writeMicroseconds(msg->data.data[0]);
    esc2.writeMicroseconds(msg->data.data[1]);
    esc3.writeMicroseconds(msg->data.data[2]);
    esc4.writeMicroseconds(msg->data.data[3]);
    esc5.writeMicroseconds(msg->data.data[4]);
}
void subscription_callback_light(const void * msgin) {
    const std_msgs_msgInt32MultiArray * msg = (const std_msgsmsg_Int32MultiArray *)msgin;
    light1.writeMicroseconds(msg->data.data[0]);
    light2.writeMicroseconds(msg->data.data[1]);
    light_couple.writeMicroseconds(msg->data.data[2]);
}
void subscription_callback_cam_servo(const void * msgin) {
    const std_msgs_msgInt32 * msg = (const std_msgsmsg_Int32 *)msgin;
    cam_servo.write(msg->data);
}
//========================== Thread Functions ===============================================
void connectivity_management_task(void *pvParameters) {
    while(1) {
        if (!connected) {
            Serial.println("Trying to reconnect...");
            attempt_reconnection();
        }
        vTaskDelay(pdMS_TO_TICKS(3000)); // 3 seconds
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
            std_msgs_msg_Int32 msg;
            msg.data = int_msg.data; // Example sensor data
            RCCHECK(rcl_publish(&publisher, &msg, NULL));
            Serial.println("Published sensor data");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 sec timer
    }
}
//========================== ROS initialization =============================================
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
        &motor_data_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/motor_data"));
    RCCHECK(rclc_subscription_init_default(
        &light_data_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/light_data"));
    RCCHECK(rclc_subscription_init_default(
        &cam_servo_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/cam_servo"));
    // Initialize executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator)); // adjust this number based on the actual entities (pub/subs)
    RCCHECK(rclc_executor_add_subscription(&executor, &motor_data_sub, &motor_data_msg, subscription_callback_motor, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &light_data_sub, &light_data_msg, subscription_callback_light, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cam_servo_sub, &cam_servo_msg, subscription_callback_cam_servo, ON_NEW_DATA));
    connected = true; // Assume connected after successful setup
}
//========================== Reconnect Function =============================================
void attempt_reconnection() {
    setup_node_and_entities();
    Serial.println("Reconnected successfully");
}
//========================== System Setup ===================================================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN_CONNECTED, OUTPUT);
    digitalWrite(LED_PIN_CONNECTED, LOW);
    // Attach motor controllers:
    esc1.attach(esc1_pin);
    esc2.attach(esc2_pin);
    esc3.attach(esc3_pin);
    esc4.attach(esc4_pin);
    esc5.attach(esc5_pin);
    // Attach lights:
    light1.attach(light1_pin);
    light2.attach(light2_pin);
    light_couple.attach(light_couple_pin);
    // Attach Pan-tilt servo:
    cam_servo.attach(cam_servo_pin);
    // Initialize micro-ROS
    setup_node_and_entities();
    // Create FreeRTOS tasks
    //                        name      ,    thread_func     , stack,param,prior,handle
    xTaskCreate(publish_sensor_data_task, "PublishSensorData", 2048, NULL, 3, NULL);
    xTaskCreate(subscribe_and_command_task, "SubscribeAndCommand", 2048, NULL, 2, NULL);
    xTaskCreate(connectivity_management_task, "ConnectivityManagement", 2048, NULL, 1, NULL);
}

void loop() {
    // Empty - Tasks are managed by FreeRTOS
}
