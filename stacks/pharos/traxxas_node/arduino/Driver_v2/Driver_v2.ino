/**
 * This is the firmware for the Arduino Pro Mini that controls the 
 * Traxxas Stampede mobility plane.
 *
 * Pin assignments:
 *  - Pin 2: Encoder channel A
 *  - Pin 3: Encoder channel B
 *  - Pin 9: Steering servo 
 *  - Pin 10: Motor controller
 *  - Pin 13: LED
 *
 * Drive command message:  The following message must be sent at 10Hz:
 * [PROTEUS_BEGIN] [STEERING ANGLE] [MOTOR SPEED] [CHECKSUM]
 *
 * A status message indicating the steering angle and motor speed
 * is sent back to the x86 at 10Hz.
 *
 * The motor controller is a SyRen 25A.  It is configured to be in packetized
 * serial mode.  DIP switch setting: OFF, OFF, OFF, ON, ON, ON.
 *
 * @author Chien-Liang Fok
 * @date 02/15/2012
 */

#include <SoftwareSerial.h> // for communicating with SyRen 25A motor controller
#include "ProteusServo.h"   // for generating R/C PWM signal for steering servo

#define PROTEUS_BEGIN 0x24

/*
 * Define constants, variables, and range of parameters used for controlling 
 * the SyRen 25A motor controller.
 */ 
SoftwareSerial _motorPort(11, 10);  // RX, TX
byte MOTOR_START_BYTE = 170;
byte MOTOR_ADDR = 128;
byte MOTOR_CMD_FORWARD = 0;
byte MOTOR_CMD_BACKWARD = 1;
byte MOTOR_PWR_MAX = 127;
byte MOTOR_PWR_STOP = 0;
int _currMotorPwr = MOTOR_PWR_STOP;   // used by the PID controller, range -MOTOR_PWR_MAX to MOTOR_PWR_MAX

/**
 * Whether an init byte was sent to the motor controller.
 */
boolean _motorInit = false;

#define MOTOR_POS_ACCEL_LIMIT 25  // The max positive acceleration in m/s/100ms
#define MOTOR_NEG_ACCEL_LIMIT 50  // The max negative acceleration in m/s/100ms
#define MOTOR_MAX_ERROR 100 // The maximum error in cm/s

/**
 * Define the coefficients of the motor PID controller.
 */
#define MOTOR_PID_P 10
#define MOTOR_PID_I 0
#define MOTOR_PID_D 0

/*
 * Define the range of parameters for controlling the steering angle.
 */
#define STEERING_MIN_PULSE 1000
#define STEERING_MAX_PULSE 2000
#define STEERING_MAX_LEFT 20
#define STEERING_CENTER 100
#define STEERING_MAX_RIGHT 180

/**
 * This is the number of milliseconds that can pass in which no
 * move command is received before the motor is stopped.
 */
#define SAFETY_STOP_INTERVAL 500

struct MoveCmd {
  uint8_t begin;
  int16_t steering; // the steering angle in tenths of degrees
  int16_t speed;
  uint8_t checksum;
} moveCmd;

struct StatusMsg {
  uint8_t begin;
  int16_t targetSpeed;
  int16_t currSpeed;
  uint16_t motorPwr;
  int16_t prevErr;
  int16_t totalErr;
  int16_t targetSteeringAngle;
  int16_t currSteeringAngle;
  uint16_t steeringAngleCmd;
  uint8_t checksum;
} statusMsg;

/*
 * Define the pins used by this program.
 */
enum PIN_ASSIGNMENTS {
  ENCODER_PIN_A = 2,
  ENCODER_PIN_B = 3,
  MOTOR_STATUS_PIN = 4,   // input indicating whether the motor controller is on
  LED_MOTOR_INIT_PIN = 5, // high when the motor controller is initialized
  LED_SAFETY_STOP_PIN = 6,
  STEERING_PWM_PIN = 9,
  MOTOR_PIN = 10,
  LED_CMD_RCVD_PIN = 13, // toggled when ackermann command received
};

byte _ledStateCmdRcvd = HIGH;
byte* _moveCmdBuff = (byte*)&moveCmd;
byte* _statusMsgBuff = (byte*)&statusMsg;

volatile int _encoderCnt = 0;

boolean _A_set = false;
boolean _B_set = false;

/**
 * Records when the speed command was last updated by the PID controller.
 * Unit is in milliseconds since power on.
 */
unsigned long _prevUpdateTime = 0;

/**
 * Records when the most recent move command was received from the x86.
 * Unit is in milliseconds since power on.
 */
unsigned long _prevCmdTime = 0;

int _currSpeed = 0; // units is cm/s (50cm/s = 0.5m/s)
int _targetSpeed = 0; // units is cm/s (50cm/s = 0.5m/s)
int _throttledTargetSpeed = 0; // units is cm/s

int _prevErr = 0;  // The previous error.  This is used when computing the "D" term of the PID controller.
int _totalErr = 0; // The cumulative error since the system started.  This is used by the "I" term in the PID controller

ProteusServo _steeringServo;
int _targetSteeringAngle; // 1/10 degree
int _currSteeringAngle; // 1/10 degree
int _currSteeringAngleCmd = STEERING_CENTER; // servo units
// TODO: Add a throttled steering angle that changes the steering angle based on a set rate

boolean _servoDone = false;
boolean _sendMotorPkt = false;

void servoDone() {
  _servoDone = true;
}

void setup() {
  pinMode(LED_CMD_RCVD_PIN, OUTPUT);  // Initialize LED
  
  pinMode(MOTOR_STATUS_PIN, INPUT);
  pinMode(LED_MOTOR_INIT_PIN, OUTPUT);
  pinMode(LED_SAFETY_STOP_PIN, OUTPUT);
  
  // Configure the encoder pins
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  digitalWrite(ENCODER_PIN_A, HIGH);  // turn on pullup resistor
  digitalWrite(ENCODER_PIN_B, HIGH);  // turn on pullup resistor

  // Configure the encoder interrupts
  attachInterrupt(0, doEncoderA, CHANGE); // encoder channel A is on pin 2, interrupt 0
  attachInterrupt(1, doEncoderB, CHANGE); // encoder channel B is on pin 3, interrupt 1
 
  // Initialize the motor controller.  It uses a packetized serial protocol
  _motorPort.begin(19200);
  
  _steeringServo.addDoneListener(servoDone);
  _steeringServo.attach(STEERING_PWM_PIN, STEERING_MIN_PULSE, STEERING_MAX_PULSE);
  _steeringServo.write(STEERING_CENTER);
  
 
  // Initialize the serial port.  This is for communicating the PID controller's state to an attached PC.
  Serial.begin(9600);
}

/**
 * Sends a packet to the motor controller telling it how fast to move.
 */
void sendMotorPacket() {
  byte currMotorAbsPwr = abs(_currMotorPwr);
  
  byte currMotorCmd = MOTOR_CMD_FORWARD;
  if (_currMotorPwr < 0) {
    currMotorCmd = MOTOR_CMD_BACKWARD;
  }
  
  byte motorChecksum = 0x7F & (MOTOR_ADDR + currMotorCmd + currMotorAbsPwr);
  
  if (_motorInit && !_steeringServo.isBusy()) {  
    _motorPort.write(MOTOR_ADDR); // send address byte
    _motorPort.write(currMotorCmd);
    _motorPort.write(currMotorAbsPwr);
    _motorPort.write(motorChecksum);
  }
}

void loop() {
  unsigned long currTime = millis();
  
  // check status of motor controller
  checkMotorControllerStatus();
  
  // Read at most one command
  rcvAckermannCmd();
  
  /**
   * Check for safety stop condition.  This is triggered when no
   * data is received within the SAFETY_STOP_INTERVAL.
   */
   if (calcTimeDiff(_prevCmdTime, currTime) > SAFETY_STOP_INTERVAL) {
     digitalWrite(LED_SAFETY_STOP_PIN, HIGH);
     _targetSpeed = 0;  // set speed to be 0 cm/s
     _steeringServo.write(STEERING_CENTER);
   } else {
     digitalWrite(LED_SAFETY_STOP_PIN, LOW);
   }
  
  /**
   * Update the throttled target speed and PID controller every 100ms.
   */
  if (calcTimeDiff(_prevUpdateTime, currTime) >= 100) {
    int encoderCnt = _encoderCnt;
    _encoderCnt = 0;
    _prevUpdateTime = currTime;
    
    // Compute the number of encoder counts per second.
    // Since encoderCnt is the number of counter per 100ms, multiply by 10 to get
    // number of counts per second.
    int cntPerSecond = encoderCnt * 10;
   
    // Calculate speed in cm/s.  To do this, divide cntPerSecond
    // by 1000 (since there are 1000 encoder counts per wheel rotation), then multiple
    // by 36 (since the wheel's circumference is 36cm).  Collectivity,
    // this is approximately equal to dividing cntPerSecond by 28.
    _currSpeed = cntPerSecond / 28; // / 1000 * 36; 
    
    // Update the throttled target speed.  This implements software-based acceleration/deceleration.
    if (_throttledTargetSpeed != _targetSpeed) {
      if (_throttledTargetSpeed < _targetSpeed) {
        // Must increase speed
        if (MOTOR_POS_ACCEL_LIMIT == 0 || _targetSpeed - _throttledTargetSpeed < MOTOR_POS_ACCEL_LIMIT)
          _throttledTargetSpeed = _targetSpeed;
        else
          _throttledTargetSpeed += MOTOR_POS_ACCEL_LIMIT;
      } else {
        // Must decrease speed
        if (MOTOR_NEG_ACCEL_LIMIT == 0 || _throttledTargetSpeed - _targetSpeed < MOTOR_NEG_ACCEL_LIMIT)
          _throttledTargetSpeed = _targetSpeed;
        else
          _throttledTargetSpeed -= MOTOR_NEG_ACCEL_LIMIT;
      }
    }
    
    // Check for special stop condition
    if (_throttledTargetSpeed == 0) {
      _currMotorPwr = MOTOR_PWR_STOP;
      _prevErr = _totalErr = 0;
    } else {
      // Calculate the speed error in cm/s.  A positive value means the robot
      // is currently traveling too slow and must speed up.
      int currSpeedErr = _throttledTargetSpeed - _currSpeed;
   
      // Update the PID controller terms.
      _totalErr += currSpeedErr;
      int deltaErr = currSpeedErr - _prevErr;
      _prevErr = currSpeedErr;
    
      if (abs(_prevErr) > MOTOR_MAX_ERROR) {
        _currMotorPwr = MOTOR_PWR_STOP;
        _prevErr = _totalErr = _throttledTargetSpeed = 0;
      } else {
        // Generate the new motor command
        _currMotorPwr += currSpeedErr/MOTOR_PID_P + MOTOR_PID_I * _totalErr + MOTOR_PID_D * deltaErr;
      }
      
      // Apply the cutoffs to ensure the robot does not move too fast
      if (_currMotorPwr > MOTOR_PWR_MAX) {
        _currMotorPwr = MOTOR_PWR_MAX;
      }
    }
    
    // Send the new motor command to the motor.
    _sendMotorPkt = true;
    
    sendStatusMsg();
  }
  
  
  if (_servoDone && _sendMotorPkt) {
    _servoDone = false;
    _sendMotorPkt = false;
    sendMotorPacket();
  }
}

/**
 * Sends a status message to the x86.
 */
void sendStatusMsg() {
  statusMsg.begin = PROTEUS_BEGIN;
  statusMsg.targetSpeed = _targetSpeed;
  statusMsg.currSpeed = _currSpeed;
  statusMsg.motorPwr = _currMotorPwr;
  statusMsg.prevErr = _prevErr;
  statusMsg.totalErr = _totalErr;
  statusMsg.targetSteeringAngle = _targetSteeringAngle;
  statusMsg.currSteeringAngle = _currSteeringAngle;
  statusMsg.steeringAngleCmd = _currSteeringAngleCmd;
    
  // compute checksum for message
  byte checksum = 0;
  for (int i=0; i < sizeof(StatusMsg) - 1; i++) {
    checksum ^= _statusMsgBuff[i];
  }
  statusMsg.checksum = checksum;
    
  Serial.write((byte*)&statusMsg, sizeof(statusMsg));
  Serial.flush();
}

unsigned long calcTimeDiff(unsigned long time1, unsigned long time2) {
  if (time1 > time2) {
    unsigned long maxUL = 0xFFFFFFFF;
    return maxUL - time2 + time1;
  } else
    return time2 - time1;
}

// The following encoder interrupts were taken from:
// http://www.arduino.cc/playground/Main/RotaryEncoders

// Interrupt on A changing state
void doEncoderA(){
  // Test transition
  _A_set = digitalRead(ENCODER_PIN_A) == HIGH;
  // and adjust counter + if A leads B
  _encoderCnt += (_A_set != _B_set) ? +1 : -1;
}

// Interrupt on B changing state
void doEncoderB(){
  // Test transition
  _B_set = digitalRead(ENCODER_PIN_B) == HIGH;
  // and adjust counter + if B follows A
  _encoderCnt += (_A_set == _B_set) ? +1 : -1;
}
