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
#include "KellerLD.h"


#define SEALEVELPRESSURE_HPA (1013.25)
#define RCCHECK(fn) ((fn) == RCL_RET_OK)
#define LED_PIN_CONNECTED 2 // LED to indicate connection status

//========================== Pinout =========================================================
Servo esc1, esc2, esc3, esc4, esc5;
Servo light1, light2, light_couple;
Servo cam_servo;
Adafruit_MPU6050 mpu;
KellerLD bar100;

#define ESC_MIN 1100
#define ESC_MAX 1900
#define LIGHT_MIN 1100
#define LIGHT_MAX 1900
#define CAM_SERVO_MIN 0
#define CAM_SERVO_MAX 180

//Adafruit_BME280 bme; // I2C

int esc_pins[] = {19, 5, 16, 4, 2};
int light_pins[] = {15, 30, 17}; // light1, light2, couple
int cam_servo_pin = 18;

//========================== Publishers definitions =========================================
rcl_publisher_t gyro_accel_pub;
geometry_msgs__msg__Twist gyro_accel_msg;
rcl_publisher_t bar100_pub;
geometry_msgs__msg__Twist bar100_msg;
//========================== Subscribers definitions ========================================
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
    //Serial.println("In motor callback");
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    int motor1_val = static_cast<int>(msg->linear.x);
    int motor2_val = static_cast<int>(msg->linear.y);
    int motor3_val = static_cast<int>(msg->linear.z);
    int motor4_val = static_cast<int>(msg->angular.x);
    int motor5_val = static_cast<int>(msg->angular.y);
    int cam_servo_val = static_cast<int>(msg->angular.z);

    if (motor1_val >= ESC_MIN && motor1_val <= ESC_MAX) {
        esc1.writeMicroseconds(motor1_val); // Send PWM signal to ESC/motor 1.
    }
    if (motor2_val >= ESC_MIN && motor2_val <= ESC_MAX) {
        esc2.writeMicroseconds(motor2_val); // Send PWM signal to ESC/motor 2.
    }
    if (motor3_val >= ESC_MIN && motor3_val <= ESC_MAX) {
        esc3.writeMicroseconds(motor3_val); // Send PWM signal to ESC/motor 3.
    }
    if (motor4_val >= ESC_MIN && motor4_val <= ESC_MAX) {
        esc4.writeMicroseconds(motor4_val); // Send PWM signal to ESC/motor 4.
    }
    if (motor5_val >= ESC_MIN && motor5_val <= ESC_MAX) {
        esc5.writeMicroseconds(motor5_val); // Send PWM signal to ESC/motor 5.
    }
    if (cam_servo_val >= CAM_SERVO_MIN && cam_servo_val <= CAM_SERVO_MAX) {
        cam_servo.write(cam_servo_val); // Send PWM signal to camera servo.
    }
}

void subscription_callback_lights(const void *msgin) {
    //Serial.println("In lights callback");
    const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
    int light1_val = static_cast<int>(msg->x);
    int light2_val = static_cast<int>(msg->y);
    int light_couple_val = static_cast<int>(msg->z);

    if (light1_val >= LIGHT_MIN && light1_val <= LIGHT_MAX) {
        light1.writeMicroseconds(light1_val); // Send PWM signal to lights.
    }
    if (light_couple_val >= LIGHT_MIN && light_couple_val <= LIGHT_MAX) {
        light_couple.writeMicroseconds(light_couple_val); // Send PWM signal to lights.
    }
}

void gyro_accel_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    //bar100.read();
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
        
        // bar100_msg.linear.x = static_cast<double>(bar100.pressure()); //[mbar]
        // bar100_msg.linear.y = static_cast<double>(bar100.temperature()); //[degC]
        // bar100_msg.linear.z = static_cast<double>(bar100.depth());  //[m]
        // bar100_msg.angular.x = static_cast<double>(bar100.altitude()); // [m] above mean sea level
        // bar100_msg.angular.y = 0.0;
        // bar100_msg.angular.z = 0.0;
        // if (!RCCHECK(rcl_publish(&bar100_pub, &bar100_msg, NULL))) {
        //     connected = false; // Set connected to false if publish fails
        // }
    }
}

void attempt_reconnection() {
    while (!connected) {
        digitalWrite(LED_PIN_CONNECTED, HIGH); delay(500);
        digitalWrite(LED_PIN_CONNECTED, LOW); delay(500);
        // Try to setup node and entities for reconnection
        connected = setup_node_and_entities();
    }
    // Once connected, keep the LED on
    digitalWrite(LED_PIN_CONNECTED, HIGH);
}

//========================== ROS Initialization =============================================
bool setup_node_and_entities() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    if (!RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))) return false;
    if (!RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support))) return false;
    if (!RCCHECK(rclc_publisher_init_default(&gyro_accel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/gyro_accel_data"))) return false;
    if (!RCCHECK(rclc_publisher_init_default(&bar100_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/bar100_data"))) return false;
    if (!RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), gyro_accel_timer_callback))) return false;
    if (!RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator))) return false;
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

//========================== System Setup ===================================================
void setup() {
    //Serial.println("In Setup");
    pinMode(LED_PIN_CONNECTED, OUTPUT);
    digitalWrite(LED_PIN_CONNECTED, LOW);
    // Attach motor controllers:
    esc1.attach(esc_pins[0]);
    esc2.attach(esc_pins[1]);
    esc3.attach(esc_pins[2]);
    esc4.attach(esc_pins[3]);
    esc5.attach(esc_pins[4]);
    // Attach lights:
    //light1.attach(light_pins[0]);
    //light_couple.attach(light_pins[2]);
    // Attach Pan-tilt servo:
    cam_servo.attach(cam_servo_pin);
    // Initial states
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(1500);
    esc3.writeMicroseconds(1500);
    esc4.writeMicroseconds(1500);
    esc5.writeMicroseconds(1500);
    //light1.writeMicroseconds(1100);
    //light_couple.writeMicroseconds(1100);
    cam_servo.write(90);
    delay(1000);
    cam_servo.write(120);
    Serial.begin(115200);
    delay(3000); // Delay to allow the ESC to recognize the stopped signal
    connected = setup_node_and_entities();
    if (!connected) {
        attempt_reconnection();  // Attempt reconnection here, similar to your previous logic
    }
    //if (!bme.begin(0x76)) { // Initialize BME280 sensor
    //  Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //}

    //bar100.init();
    //bar100.setFluidDensity(1029); // kg/m^3 (seawater, 997 for freshwater)

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    }
}

//========================== Main Loop ======================================================
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
