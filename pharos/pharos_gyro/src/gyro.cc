/**
 * \file  GyroRosserial.ino
 * \brief rosserial node for publishing the yaw value at about 20hz
 *
 * \author piyushk (piyushk@cs.utexas.edu)
 *
 * Copyright (C) 2012, UT Austin
 */

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 gyro_msg;
ros::Publisher gyro_publisher("gyro", &gyro_msg);

float gyro_scale_correction = 1.0;
float stationary_gyro_value = 0.0;
int led_state = 0;
int read_count = 0;
float yaw_sum = 0;

int yaw_pin = A0;
int pitch_pin = A1;
int ref_pin = A2;
int led_pin = 13;

int yaw_value;
int ref_value;

char log_msg[50];

void logStationaryGyroValue() {
  // A bit of hacking as sprintf does not support floats
  float temp = stationary_gyro_value;
  int temp1 = (temp - (int)temp) * 100;
  sprintf(log_msg, "stationary gyro value = %0d.%d", (int)temp, temp1);
  nh.loginfo(log_msg);
}

void logGyroScaleCorrection() {
  // A bit of hacking as sprintf does not support floats
  float temp = gyro_scale_correction;
  int temp1 = (temp - (int)temp) * 100;
  sprintf(log_msg, "gyro scale correction = %0d.%d", (int)temp, temp1);
  nh.loginfo(log_msg);
}

void setup() {

  nh.initNode();
  nh.advertise(gyro_publisher);

  // Wait for connection to the rosserial node before getting parameters
  while (!nh.connected()) {
    nh.spinOnce();
  }
  nh.getParam("gyro_scale_correction", &gyro_scale_correction, 1.0);
  logGyroScaleCorrection();
 
  // Use the led pin to display state
  pinMode(led_pin, OUTPUT);  
}

void loop() {

  // Calculate the yaw value from the analog device
  yaw_value = analogRead(yaw_pin);
  ref_value = analogRead(ref_pin);

  float yaw = (float) yaw_value / (float) ref_value;
  
  if (read_count < 5 && read_count >= 0) {
    yaw_sum += yaw;
    read_count++;
  } else {
    if (read_count == 5) {
      stationary_gyro_value = yaw_sum / read_count;
      logStationaryGyroValue();
      read_count = -1;  
    }
    gyro_msg.data = gyro_scale_correction * (yaw - stationary_gyro_value); 
    gyro_publisher.publish(&gyro_msg);
  } 

  // Change LED state to indicate the next spin cycle
  if (led_state) {
    digitalWrite(led_pin, HIGH);  
    led_state = 0;
  } else {
    digitalWrite(led_pin, LOW);   
    led_state = 1;
  }

  // Try to approximately maintain 20 hz
  nh.spinOnce();
  delay(50);
}

