#include <ESP32Servo.h>

int servoPin = 18;
Servo servo;

void setup() {
 servo.attach(servoPin);
 Serial.begin(115200);
}

void loop() {
  if(Serial.available()){
    int angle = Serial.parseInt();
    servo.write(angle);
  }
  delay(20);
}