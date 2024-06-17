#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
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
#include <Adafruit_BNO055.h>
// #include <Adafruit_BME280.h>
// #include <Adafruit_MPU6050.h>
#include <SparkFun_Qwiic_OLED.h> //http://librarymanager/All#SparkFun_Qwiic_OLED
// #include "KellerLD.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define RCCHECK(fn) ((fn) == RCL_RET_OK)
#define INDICATOR_LED_PIN 0 // LED to indicate connection status
#define DEBUG_TX_PIN 17 
#define DEBUG_RX_PIN 16 
#define ESC_MIN 1100
#define ESC_MAX 1900
#define LIGHT_MIN 1100
#define LIGHT_MAX 1900
#define CAM_SERVO_MIN 0
#define CAM_SERVO_MAX 180

//========================== Pinout =========================================================
Servo esc1, esc2, esc3, esc4, esc5, esc6;
Servo light_single, light_couple;
Servo cam_servo;
// Adafruit_MPU6050 mpu;
// KellerLD bar100;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
HardwareSerial MainSerial(0);  // Declaring a Serial object on UART0
HardwareSerial DebugSerial(1); // Use UART1 for debugging

//Adafruit_BME280 bme; // I2C

int esc_pins[] = {27, 26, 25, 33, 32, 14};
int light_pins[] = {18, 19}; // light_single, light_couple
int cam_servo_pin = 12;

//========================== Publishers definitions =========================================
rcl_publisher_t bar100_pub;
geometry_msgs__msg__Twist bar100_msg;
rcl_publisher_t bno055_pub;
geometry_msgs__msg__Twist bno055_msg;
//========================== Subscribers definitions ========================================
rcl_subscription_t motors_sub;
rcl_subscription_t lights_servo_sub;
geometry_msgs__msg__Twist motors_msg;
geometry_msgs__msg__Vector3 lights_servo_msg;

//========================== ROS variables ==================================================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool connected = false;

//========================== OLED display ====================================================
QwiicNarrowOLED myOLED;

// 'ROV-LOGO', 128x32px
const unsigned char myBitmap[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xF0, 0xF0, 0xB8, 0x18, 0x18, 0x08,
0x08, 0x0C, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
0x04, 0x04, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44,
0x44, 0x40, 0x40, 0x40, 0x40, 0x46, 0x46, 0x7E, 0x7E, 0x7E, 0xE6, 0xC0, 0x80, 0x80, 0x80, 0x84,
0x84, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0C,
0x0C, 0x0C, 0x08, 0x18, 0x18, 0x38, 0xF0, 0xF0, 0xF0, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87, 0x83, 0x33, 0x7B, 0x78, 0x78, 0xCC, 0xCC,
0xB4, 0xBC, 0xFC, 0xFC, 0xFE, 0xFE, 0xEE, 0xCE, 0xCE, 0xFE, 0xEE, 0xCE, 0xEE, 0xFE, 0xFE, 0xFC,
0xF4, 0xB4, 0x84, 0xCC, 0xDC, 0x78, 0x78, 0x70, 0x20, 0xA0, 0xA0, 0xE0, 0xE0, 0xCC, 0xFE, 0xFF,
0x7F, 0x73, 0xE1, 0xE1, 0xE0, 0xCC, 0xCE, 0xDE, 0xDB, 0x99, 0x99, 0xB9, 0xB9, 0xB9, 0xB9, 0xB9,
0xB9, 0xB9, 0xBD, 0xBD, 0xBF, 0xBF, 0xBF, 0x9F, 0x9F, 0x9F, 0xDE, 0xDE, 0xCE, 0xCC, 0xC1, 0xE1,
0xE3, 0xFF, 0xFF, 0x7E, 0xFE, 0xEC, 0xE0, 0xE0, 0xA0, 0xA0, 0x70, 0x78, 0x78, 0xF8, 0xCC, 0x84,
0xB4, 0xFC, 0xFC, 0xFC, 0xFE, 0xEE, 0xCE, 0xEE, 0xFE, 0xCE, 0xCE, 0xEE, 0xFE, 0xFE, 0xFC, 0xF4,
0xB4, 0x84, 0xCC, 0xFC, 0x78, 0x78, 0x33, 0x03, 0x87, 0x87, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE,
0xFE, 0xFE, 0xFC, 0x1C, 0x1D, 0x1D, 0x1D, 0x1D, 0x1D, 0x1D, 0x1D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9C,
0x9E, 0x9E, 0x9E, 0x9E, 0x9E, 0x9E, 0x1E, 0x1E, 0x3C, 0x3C, 0x3D, 0x3D, 0x3D, 0x7B, 0xFB, 0xFB,
0xFB, 0xFB, 0xFF, 0x7F, 0x77, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x17, 0x97, 0x97, 0x97, 0x97,
0x97, 0x97, 0x97, 0x97, 0x97, 0x97, 0x97, 0x17, 0x37, 0x37, 0x37, 0x37, 0x37, 0x77, 0x77, 0x77,
0xFF, 0xFB, 0xFB, 0xDB, 0x9B, 0x19, 0x1D, 0x1D, 0x1D, 0x1C, 0x3E, 0x3E, 0x7E, 0x7E, 0xFE, 0xFE,
0xFE, 0xFE, 0xFD, 0xFD, 0xFD, 0xFD, 0x7D, 0x3D, 0x3D, 0x1D, 0x1D, 0x1D, 0x1D, 0x1D, 0x9C, 0x9C,
0xDE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x07, 0x0F, 0x0F, 0x0F, 0x1F, 0x1F,
0x1F, 0x1F, 0x1F, 0x10, 0x10, 0x30, 0x30, 0x30, 0x30, 0x30, 0x38, 0x3C, 0x3C, 0x3C, 0x3C, 0x3C,
0x3C, 0x38, 0x38, 0x38, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x36, 0x36, 0x36, 0x3E, 0x3F, 0x3F,
0x3F, 0x3F, 0x3C, 0x3C, 0x38, 0x38, 0x38, 0x38, 0x30, 0x30, 0x33, 0x33, 0x33, 0x33, 0x37, 0x37,
0x37, 0x37, 0x37, 0x37, 0x37, 0x33, 0x33, 0x33, 0x31, 0x30, 0x38, 0x38, 0x38, 0x38, 0x3C, 0x3C,
0x3E, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3F, 0x3E, 0x3E, 0x3C, 0x3C, 0x38, 0x38, 0x30, 0x30, 0x30,
0x31, 0x33, 0x33, 0x31, 0x30, 0x30, 0x30, 0x30, 0x38, 0x3C, 0x3C, 0x3E, 0x1E, 0x1F, 0x1F, 0x1F,
0x1F, 0x1F, 0x1F, 0x1F, 0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00
};

//========================== Subscribers Callbacks ==========================================
void subscription_callback_motors(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    int motor1_val = static_cast<int>(msg->linear.x);
    int motor2_val = static_cast<int>(msg->linear.y);
    int motor3_val = static_cast<int>(msg->linear.z);
    int motor4_val = static_cast<int>(msg->angular.x);
    int motor5_val = static_cast<int>(msg->angular.y);
    int motor6_val = static_cast<int>(msg->angular.z);

    if (motor1_val >= ESC_MIN && motor1_val <= ESC_MAX) {
        esc1.writeMicroseconds(motor1_val); // Send PWM signal to ESC/motor 1.s
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
    if (motor6_val >= ESC_MIN && motor6_val <= ESC_MAX) {
        esc6.writeMicroseconds(motor6_val); // Send PWM signal to ESC/motor 6.
    }
}

void subscription_callback_lights_servo(const void *msgin) {
    const geometry_msgs__msg__Vector3 *msg = (const geometry_msgs__msg__Vector3 *)msgin;
    int light_single_val = static_cast<int>(msg->x);
    int light_couple_val = static_cast<int>(msg->y);
    int cam_servo_val = static_cast<int>(msg->z);

    if (light_single_val >= LIGHT_MIN && light_single_val <= LIGHT_MAX) {
        light_single.writeMicroseconds(light_single_val); // Send PWM signal to lights.
    }
    if (light_couple_val >= LIGHT_MIN && light_couple_val <= LIGHT_MAX) {
        light_couple.writeMicroseconds(light_couple_val); // Send PWM signal to lights.
    }
    if (cam_servo_val >= CAM_SERVO_MIN && cam_servo_val <= CAM_SERVO_MAX) {
        cam_servo.write(cam_servo_val); // Send PWM signal to camera servo.
    }    

}

void sensors_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    /* Get new sensor events with the readings */
    sensors_event_t orientationData, temp;
    // bar100.read();
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    RCLC_UNUSED(last_call_time);
    if (timer != NULL && connected) {
        bno055_msg.linear.x = static_cast<double>(orientationData.orientation.x);
        bno055_msg.linear.y = static_cast<double>(orientationData.orientation.y);
        bno055_msg.linear.z = static_cast<double>(orientationData.orientation.z);
        bno055_msg.angular.x = 0.0;
        bno055_msg.angular.y = 0.0;
        bno055_msg.angular.z = 0.0;
        if (!RCCHECK(rcl_publish(&bno055_pub, &bno055_msg, NULL))) {
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

        // Update OLED display
        myOLED.setCursor(0, 0); // Set cursor to top-left corner
        myOLED.print("Yaw: ");
        myOLED.print(orientationData.orientation.x, 2); // Print yaw with 2 decimal places
        myOLED.setCursor(0, 10); // Move cursor to the next line
        myOLED.print("Pitch: ");
        myOLED.print(orientationData.orientation.y, 2); // Print pitch with 2 decimal places
        myOLED.setCursor(0, 20); // Move cursor to the next line
        myOLED.print("Roll: ");
        myOLED.print(orientationData.orientation.z, 2); // Print roll with 2 decimal places
        myOLED.display(); // Update the display
        myOLED.erase();
    }
}

bool reconnect_to_agent() {
    cleanup_ros_entities();
    digitalWrite(INDICATOR_LED_PIN, LOW); // Indicate successful connection
    delay(1000); // Wait a bit before trying to reconnect
    digitalWrite(INDICATOR_LED_PIN, HIGH); // Indicate successful connection
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    if (!RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))) {
        DebugSerial.println("Failed to reinitialize support.");
        return false;
    }
    if (!RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support))) {
        DebugSerial.println("Failed to reinitialize node.");
        return false;
    }
    if (!RCCHECK(rclc_publisher_init_default(&bno055_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/bno055_data"))) {
        DebugSerial.println("Failed to reinitialize bno055_pub.");
        return false;
    }
    if (!RCCHECK(rclc_publisher_init_default(&bar100_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/bar100_data"))) {
        DebugSerial.println("Failed to reinitialize bar100_pub.");
        return false;
    }
    if (!RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), sensors_timer_callback))) {
        DebugSerial.println("Failed to reinitialize timer.");
        return false;
    }
    if (!RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator))) {
        DebugSerial.println("Failed to reinitialize executor.");
        return false;
    }
    if (!RCCHECK(rclc_executor_add_timer(&executor, &timer))) {
        DebugSerial.println("Failed to add timer to executor.");
        return false;
    }
    if (!RCCHECK(rclc_subscription_init_default(&motors_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/motor_data"))) {
        DebugSerial.println("Failed to reinitialize motors_sub.");
        return false;
    }
    if (!RCCHECK(rclc_executor_add_subscription(&executor, &motors_sub, &motors_msg, subscription_callback_motors, ON_NEW_DATA))) {
        DebugSerial.println("Failed to add motors_sub to executor.");
        return false;
    }
    if (!RCCHECK(rclc_subscription_init_default(&lights_servo_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/lights_servo_data"))) {
        DebugSerial.println("Failed to reinitialize lights_servo_sub.");
        return false;
    }
    if (!RCCHECK(rclc_executor_add_subscription(&executor, &lights_servo_sub, &lights_servo_msg, subscription_callback_lights_servo, ON_NEW_DATA))) {
        DebugSerial.println("Failed to add lights_servo_sub to executor.");
        return false;
    }
    DebugSerial.println("Reconnection successful.");
    return true;
}

bool check_connection() {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    if (ret != RCL_RET_OK) {
        DebugSerial.println("Connection check failed.");
        return false;
    }
    return true;
}

void cleanup_ros_entities() {
    rcl_publisher_fini(&bno055_pub, &node);
    rcl_publisher_fini(&bar100_pub, &node);
    rcl_subscription_fini(&motors_sub, &node);
    rcl_subscription_fini(&lights_servo_sub, &node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    rcl_node_fini(&node);
}

bool setup_node_and_entities() {
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    if (!RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))) return false;
    if (!RCCHECK(rclc_node_init_default(&node, "esp32_node", "", &support))) return false;
    if (!RCCHECK(rclc_publisher_init_default(&bno055_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/bno055_data"))) return false;
    if (!RCCHECK(rclc_publisher_init_default(&bar100_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/esp32/bar100_data"))) return false;
    if (!RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), sensors_timer_callback))) return false;
    if (!RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator))) return false;
    if (!RCCHECK(rclc_executor_add_timer(&executor, &timer))) return false;
    // Motor Data Subscriber
    RCCHECK(rclc_subscription_init_default(
        &motors_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/motor_data"));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &motors_sub, &motors_msg,
        subscription_callback_motors, ON_NEW_DATA));
    // Light/servo Subscriber
    RCCHECK(rclc_subscription_init_default(
        &lights_servo_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "/lights_servo_data"));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &lights_servo_sub, &lights_servo_msg,
        subscription_callback_lights_servo, ON_NEW_DATA));
    return true;
}

void showBitmap() {
    myOLED.erase();
    myOLED.bitmap(0, 0, (uint8_t*)myBitmap, 128, 32); // Adjust the width and height according to your bitmap
    myOLED.display();
    delay(2000);
    myOLED.erase();
}


void setup() {
    pinMode(INDICATOR_LED_PIN, OUTPUT);
    digitalWrite(INDICATOR_LED_PIN, HIGH);
    // Attach motor controllers:
    esc1.attach(esc_pins[0]);
    esc2.attach(esc_pins[1]);
    esc3.attach(esc_pins[2]);
    esc4.attach(esc_pins[3]);
    esc5.attach(esc_pins[4]);
    esc6.attach(esc_pins[5]);

    // Attach lights:
    light_single.attach(light_pins[0]);
    light_couple.attach(light_pins[1]);
    // Attach Pan-tilt servo:
    cam_servo.attach(cam_servo_pin);
    // Initial states
    esc1.writeMicroseconds(1500);
    esc2.writeMicroseconds(1500);
    esc3.writeMicroseconds(1500);
    esc4.writeMicroseconds(1500);
    esc5.writeMicroseconds(1500);
    esc6.writeMicroseconds(1500);
    light_single.writeMicroseconds(1100);
    light_couple.writeMicroseconds(1100);
    cam_servo.write(90);

    // Setup debugging serial
    DebugSerial.begin(115200, SERIAL_8N1, DEBUG_RX_PIN, DEBUG_TX_PIN);
    // Setup main Agent Serial
    MainSerial.begin(115200);

    // bar100.init();
    // bar100.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)

    // if (bar100.isInitialized()) {
    //     DebugSerial.println("Bar100 is connected.");
    // } else {
    //     DebugSerial.println("Bar100 is not connected.");
    // }

    if (!bno.begin()) {
        DebugSerial.println("Failed to find BNO055 chip.");
    } else {
        bno.setExtCrystalUse(true);
    }

    // Initialize the OLED display
    if (myOLED.begin() == false) {
        DebugSerial.println("OLED begin failed.");
    }
    else{
        DebugSerial.println("OLED begin success");
    }

    // Display bitmap at startup
    showBitmap();

    // if (!mpu.begin()) {
    //     DebugSerial.println("Failed to find MPU6050 chip");
    // } else {
    //     mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    //     mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    //     mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    // }

    connected = setup_node_and_entities();
}

void loop() {
        // Check connection periodically
    if (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        connected = false;
        DebugSerial.println("Agent not reachable. Attempting to reconnect...");
    }

    if (!connected || !check_connection()) {
        DebugSerial.println("Connection lost, attempting to reconnect...");
        connected = reconnect_to_agent();  // Attempt to reconnect
        if (connected) {
            DebugSerial.println("Reconnected successfully.");
        } else {
            DebugSerial.println("Reconnection failed.");
        }
    }

    delay(10); // Delay to avoid running the loop too fast
}

