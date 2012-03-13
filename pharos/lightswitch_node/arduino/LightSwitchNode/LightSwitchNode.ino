/**
 * The Arduino sketch that receives commands from the ROS node
 * "lightswitch_node".  It accepts messages consisting of a single
 * byte.  A zero means turn the light off.  A non-zero means
 * turn the light on.
 *
 * Attach servo's signal pin to PWM port 9 of the Arduino Mega. 
 * Power the servo using an external power supply that can provide 
 * at least 1A of current.  This is to prevent the servo from drawing
 * too much power causing the Arduino to reset.
 *
 * After programing the Arduino with this sketch, launch
 * the ROS node "lightswitch_node".
 *
 * @author Chien-Liang Fok
 * @date 03/13/2012
 */
#include <Servo.h>

#define SERVO_PIN 9
#define LED_PIN 13  // This is connected to the on-board LED
#define SERVO_CENTER 1500
#define OFF_POSITION 1000
#define ON_POSITION 2000

Servo servo;
byte _ledState = 0;
 
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);  // Initialize LED
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(SERVO_CENTER);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 0)
      servo.writeMicroseconds(OFF_POSITION);
    else
      servo.writeMicroseconds(ON_POSITION);
    digitalWrite(LED_PIN, _ledState);
    _ledState = !_ledState; 
  }
}
