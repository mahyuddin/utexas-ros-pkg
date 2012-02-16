
#define MOTOR_BOOT_INTERVAL 3000

/**
 * Whether the motor is on.
 */
boolean _motorOn = false;

/**
 * Records when the speed command was last updated by the PID controller.
 * Unit is in milliseconds since power on.
 */
unsigned long _motorOnTime = 0;

/**
 * Checks whether an init byte should be sent to the motor controller.
 */
void checkMotorControllerStatus() {
  int motorIsOn = digitalRead(MOTOR_STATUS_PIN);
  if (!motorIsOn) {
    _motorOn = false;
    _motorInit = false;
    digitalWrite(LED_MOTOR_INIT_PIN, LOW);
  } else {
    if (!_motorOn) {
      _motorOn = true;
      _motorOnTime = millis();
    } else {
      if (!_motorInit && calcTimeDiff(_motorOnTime, millis()) > MOTOR_BOOT_INTERVAL) {
        _motorPort.write(MOTOR_START_BYTE); // set the baud rate
        _motorInit = true;
        digitalWrite(LED_MOTOR_INIT_PIN, HIGH);
      }
    }
  }
}
