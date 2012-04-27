/**
 * \file  GyroRead.ino
 * \brief Reads the yaw value and reference level 
 * and transmits to a waiting ROS node using the serial connection
 *
 * \author piyushk (piyushk@cs.utexas.edu)
 *
 * Copyright (C) 2012, UT Austin
 */

int sensorPin0 = A0;    // select the input pin for the potentiometer
int sensorPin1 = A1;    // select the input pin for the potentiometer
int sensorPin2 = A2;    // select the input pin for the potentiometer

int ledPin = 13;      // select the pin for the LED
int sensorValue0 = 0;  // variable to store the value coming from the sensor
int sensorValue1 = 0;  // variable to store the value coming from the sensor
int sensorValue2 = 0;  // variable to store the value coming from the sensor

int ledState = 0;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  sensorValue0 = analogRead(sensorPin0); // Yaw
  sensorValue1 = analogRead(sensorPin1); // Pitch  
  sensorValue2 = analogRead(sensorPin2); // Reference level
  
  float val = (float) sensorValue0 / (float) sensorValue2;
  Serial.println(val);
  
  if (ledState) {
    // turn the ledPin on
    digitalWrite(ledPin, HIGH);  
    ledState = 0;
  } else {
    digitalWrite(ledPin, LOW);   
    ledState = 1;
  }
  
  delay(50);          
}
