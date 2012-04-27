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
 * In addition, this sketch periodically takes light readings and
 * sends the light level to the ROS node.  It uses the light sensor
 * on the SRF08 sonar sensor, which connects via I2C.  For more details,
 * see: http://pharos.ece.utexas.edu/wiki/index.php/Attaching_the_SRF08_Ultra_Sound_Range_Finder_to_the_Arduino_Mega_-_09/02/2011#Software
 *
 * After programing the Arduino with this sketch, launch
 * the ROS node "lightswitch_node".
 *
 * @author Chien-Liang Fok
 * @date 03/13/2012
 */
#include <Servo.h>
#include <Wire.h>

// Servo constants
#define SERVO_PIN 9
#define SERVO_CENTER 1500
#define OFF_POSITION 1000
#define ON_POSITION 1600

// Light sensor constants
#define srfAddress 0x72                           // Address of the SRF08
#define cmdByte 0x00                              // Command byte
#define lightByte 0x01                            // Byte to read light sensor
#define rangeByte 0x02                            // Byte for start of ranging data
#define LIGHT_SENSOR_READING_PERIOD 100           // The period at which to sense light in milliseconds

#define LED_PIN 13  // This is connected to the on-board LED

Servo servo;
byte _ledState = 0;

/**
 * Records when we last took a light level measurement.
 * Unit is in milliseconds since power on.
 */
unsigned long _prevLightSenseTime = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);  // Initialize LED
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(SERVO_CENTER);
  delay(100); // Make sure everything is powered up before sending or receiving data
}

void loop() {
  unsigned long currTime = millis();
  
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 0)
      servo.writeMicroseconds(OFF_POSITION);
    else
      servo.writeMicroseconds(ON_POSITION);
    digitalWrite(LED_PIN, _ledState);
    _ledState = !_ledState; 
  }
  
  /**
   * Take a light sensor reading every LIGHT_SENSOR_READING_PERIOD ms.
   */
  if (calcTimeDiff(_prevLightSenseTime, currTime) >= LIGHT_SENSOR_READING_PERIOD) {
    _prevLightSenseTime = currTime;
    int rangeData = getRange(); // for some reason, need to get range data to update light
    byte lightLevel = getLight();
    Serial.write(lightLevel);
  }
}

int getRange(){                                   // This function gets a ranging from the SRF08
  int range = 0; 
  
  Wire.beginTransmission(srfAddress);             // Start communicating with SRF08
  Wire.write((byte)cmdByte);                             // Send Command Byte
  Wire.write(0x51);                                // Send 0x51 to start a ranging in cm
  Wire.endTransmission();
  
  delay(100);                                     // Wait for ranging to be complete
  
  Wire.beginTransmission(srfAddress);             // start communicating with SRFmodule
  Wire.write(rangeByte);                           // Call the register for start of ranging data
  Wire.endTransmission();
  
  Wire.requestFrom(srfAddress, 2);                // Request 2 bytes from SRF module
  while(Wire.available() < 2);                    // Wait for data to arrive
  byte highByte = Wire.read();                 // Get high byte
  byte lowByte = Wire.read();                  // Get low byte
  
  range = (highByte << 8) + lowByte;              // Put them together
  
  return(range);                                  // Returns Range
}

byte getLight() {                                  // Function to get light reading
  Wire.beginTransmission(srfAddress);
  Wire.write(lightByte);                           // Call register to get light reading
  Wire.endTransmission();
  
  Wire.requestFrom(srfAddress, 1);                 // Request 1 byte
  while(Wire.available() < 0);                     // While byte available
  byte lightLevel = Wire.read();                   // Get light reading
  
  return lightLevel;                               // Returns the light level
  
}
unsigned long calcTimeDiff(unsigned long time1, unsigned long time2) {
  if (time1 > time2) {
    unsigned long maxUL = 0xFFFFFFFF;
    return maxUL - time2 + time1;
  } else
    return time2 - time1;
}
