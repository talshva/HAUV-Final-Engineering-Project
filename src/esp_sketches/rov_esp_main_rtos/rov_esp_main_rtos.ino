#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MPU6050.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define RCCHECK(fn) ((fn) == RCL_RET_OK)
#define LED_PIN_CONNECTED 2

void publish_sensor_data_task(void *pvParameters);
void subscribe_and_command_task(void *pvParameters);
void connectivity_management_task(void *pvParameters);

//========================== Pinout =========================================================
Servo esc1, esc2, esc3, esc4, esc5;
Servo light1, light2, light_couple;
Servo cam_servo;
Adafruit_MPU6050 mpu;

#define BUFFER_SIZE 10
#define ESC_MIN 1100
#define ESC_MAX 1900
#define LIGHT_MIN 1100
#define LIGHT_MAX 1900
#define CAM_SERVO_MIN 0
#define CAM_SERVO_MAX 180

int motor_buffers[6][BUFFER_SIZE] = {{0}};
int light_buffers[3][BUFFER_SIZE] = {{0}};
int motor_buffer_index = 0;
int light_buffer_index = 0;

//Adafruit_BME280 bme; // I2C

int esc_pins[] = {5, 19, 25, 27, 29};
int light_pins[] = {15, 30, 17}; // light1, light2, couple
int cam_servo_pin = 18;

//========================== Publishers definitions =========================================
rcl_publisher_t gyro_accel_pub;
sensors_event_t a, g, temp; // accel, gyro and temp
geometry_msgs__msg__Twist gyro_accel_msg;
//========================== Subscribers definitions ========================================
// Subscribers for motors and lights
rcl_subscription_t motors_sub;
rcl_subscription_t lights_sub;
geometry_msgs__msg__Twist motors_msg;
geometry_msgs__msg__Vector3 lights_msg;
//========================== ROS variables ==================================================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool connected = false;

//========================== Subscribers Callbacks ==========================================
void subscription_callback_motors(const void *msgin) {
    Serial.println("Motor callback called");
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    motor_buffers[0][motor_buffer_index] = static_cast<int>(msg->linear.x);
    motor_buffers[1][motor_buffer_index] = static_cast<int>(msg->linear.y);
    motor_buffers[2][motor_buffer_index] = static_cast<int>(msg->linear.z);
    motor_buffers[3][motor_buffer_index] = static_cast<int>(msg->angular.x);
    motor_buffers[4][motor_buffer_index] = static_cast<int>(msg->angular.y);
    motor_buffers[5][motor_buffer_index] = static_cast<int>(msg->angular.z); // Servo

    motor_buffer_index = (motor_buffer_index + 1) % BUFFER_SIZE;
}

void subscription_callback_lights(const void *msgin) {
    Serial.println("Light callback called");
    const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;

    light_buffers[0][light_buffer_index] = static_cast<int>(msg->x);
    light_buffers[1][light_buffer_index] = static_cast<int>(msg->y);
    light_buffers[2][light_buffer_index] = static_cast<int>(msg->z);

    light_buffer_index = (light_buffer_index + 1) % BUFFER_SIZE;
}

int calculate_average(int buffer[], int size) {
    int sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

//========================== Thread Functions ===============================================
void connectivity_management_task(void *pvParameters) {
    while(1) {
        if (!connected) {
            digitalWrite(LED_PIN_CONNECTED, LOW);
            Serial.println("Trying to reconnect...");
            connected = setup_node_and_entities();
        }
        Serial.println("Connected!");
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5 seconds
    }
}

void subscribe_and_command_task(void *pvParameters) {
    while(1) {
        if (connected) {
            // Spin the executor
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

            // Calculate and set motor values
            int motor1_val = calculate_average(motor_buffers[0], BUFFER_SIZE);
            int motor2_val = calculate_average(motor_buffers[1], BUFFER_SIZE);
            int motor3_val = calculate_average(motor_buffers[2], BUFFER_SIZE);
            int motor4_val = calculate_average(motor_buffers[3], BUFFER_SIZE);
            int motor5_val = calculate_average(motor_buffers[4], BUFFER_SIZE);
            int cam_servo_val = calculate_average(motor_buffers[5], BUFFER_SIZE);

            if (motor1_val >= ESC_MIN && motor1_val <= ESC_MAX) {
                esc1.writeMicroseconds(motor1_val);
            }
            if (motor2_val >= ESC_MIN && motor2_val <= ESC_MAX) {
                esc2.writeMicroseconds(motor2_val);
            }
            if (motor3_val >= ESC_MIN && motor3_val <= ESC_MAX) {
                esc3.writeMicroseconds(motor3_val);
            }
            if (motor4_val >= ESC_MIN && motor4_val <= ESC_MAX) {
                esc4.writeMicroseconds(motor4_val);
            }
            if (motor5_val >= ESC_MIN && motor5_val <= ESC_MAX) {
                esc5.writeMicroseconds(motor5_val);
            }
            if (cam_servo_val >= CAM_SERVO_MIN && cam_servo_val <= CAM_SERVO_MAX) {
                cam_servo.write(cam_servo_val);
            }

            // Calculate and set light values
            int light1_val = calculate_average(light_buffers[0], BUFFER_SIZE);
            int light2_val = calculate_average(light_buffers[1], BUFFER_SIZE);
            int light_couple_val = calculate_average(light_buffers[2], BUFFER_SIZE);

            if (light1_val >= LIGHT_MIN && light1_val <= LIGHT_MAX) {
                light1.writeMicroseconds(light1_val);
            }
            if (light2_val >= LIGHT_MIN && light2_val <= LIGHT_MAX) {
                light2.writeMicroseconds(light2_val);
            }
            if (light_couple_val >= LIGHT_MIN && light_couple_val <= LIGHT_MAX) {
                light_couple.writeMicroseconds(light_couple_val);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Adjust delay as needed
    }
}

void publish_sensor_data_task(void *pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();  // Initialize the xLastWakeTime variable with the current time
    const TickType_t frequency = pdMS_TO_TICKS(20);  // Set the frequency to 20ms (50Hz)

    while (1) {
        if (connected) {
            mpu.getEvent(&a, &g, &temp);
            gyro_accel_msg.linear.x = static_cast<double>(a.acceleration.x);
            gyro_accel_msg.linear.y = static_cast<double>(a.acceleration.y);
            gyro_accel_msg.linear.z = static_cast<double>(a.acceleration.z);
            gyro_accel_msg.angular.x = static_cast<double>(g.gyro.x);
            gyro_accel_msg.angular.y = static_cast<double>(g.gyro.y);
            gyro_accel_msg.angular.z = static_cast<double>(g.gyro.z);
            if (!RCCHECK(rcl_publish(&gyro_accel_pub, &gyro_accel_msg, NULL))) {
                connected = false; // Set connected to false if publish fails
            }
            //Serial.println("Published sensor data");
        }

        // Use vTaskDelayUntil for consistent timing
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}


//========================== ROS initialization =============================================
bool setup_node_and_entities() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support));
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    // Initialize publisher for sensor data
    RCCHECK(rclc_publisher_init_default(&gyro_accel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/gyro_accel_data"));
    // Initialize subscriber for motors
    RCCHECK(rclc_subscription_init_default(&motors_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/motor_data"));
    RCCHECK(rclc_executor_add_subscription(&executor, &motors_sub, &motors_msg, subscription_callback_motors, ON_NEW_DATA));
    // Initialize subscriber for lights
    RCCHECK(rclc_subscription_init_default(&lights_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/light_data"));
    RCCHECK(rclc_executor_add_subscription(&executor, &lights_sub, &lights_msg, subscription_callback_lights, ON_NEW_DATA));
    return true;
}

//========================== System Setup ===================================================
void setup() {
    Serial.begin(115200);
    delay(1000); // Delay to allow serial port to initialize

    pinMode(LED_PIN_CONNECTED, OUTPUT);
    digitalWrite(LED_PIN_CONNECTED, LOW);
    // Attach motor controllers:
    esc1.attach(esc_pins[0]);
    esc2.attach(esc_pins[1]);
    //esc3.attach(esc_pins[2]);
    //esc4.attach(esc_pins[3]);
    //esc5.attach(esc_pins[4]);
    // Attach lights:
    light1.attach(light_pins[0]);
    light_couple.attach(light_pins[2]);
    // Attach Pan-tilt servo:
    cam_servo.attach(cam_servo_pin);

    // Initial states
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(1500);
    light1.writeMicroseconds(1100);
    light_couple.writeMicroseconds(1100);
    cam_servo.write(90);
    delay(1000);
    cam_servo.write(120);

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    }   

    connected = setup_node_and_entities();

    // Create FreeRTOS tasks
    //                        name      ,    thread_func     , stack,param,prior,handle
    xTaskCreate(subscribe_and_command_task, "SubscribeAndCommand", 6000, NULL, 3, NULL);
    xTaskCreate(publish_sensor_data_task, "PublishSensorData", 6000, NULL, 2, NULL);
    xTaskCreate(connectivity_management_task, "ConnectivityManagement", 6000, NULL, 1, NULL);
}

void loop() {
    // Empty - Tasks are managed by FreeRTOS
}
