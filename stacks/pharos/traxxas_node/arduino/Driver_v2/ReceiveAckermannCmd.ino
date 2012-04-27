/**
 * Receives incoming AckermannCmd messages.
 */
void rcvAckermannCmd() {
  if (Serial.available() >= sizeof(MoveCmd)) {
    
    byte startByte = Serial.read();
    if (startByte == PROTEUS_BEGIN) {
      _moveCmdBuff[0] = startByte;
      
      // grab the rest of the bytes
      for (int i=1; i < sizeof(MoveCmd); i++) {
        _moveCmdBuff[i] = Serial.read();
      }
      
      // compute checksum
      byte checksum = 0;
      for (int i=0; i < sizeof(MoveCmd) - 1; i++) {
        checksum ^= _moveCmdBuff[i];
      }
      if (checksum == _moveCmdBuff[sizeof(moveCmd)-1]) {
        // Checksum passed, accept command
        _currSteeringAngle = _targetSteeringAngle = moveCmd.steering; 
        
        _currSteeringAngleCmd = 100 - _targetSteeringAngle * 8 / 20; // convert from 1/10 degree into servo command unit.  Valid range -200 to 200.
        _targetSpeed = moveCmd.speed;
        
        _prevCmdTime = millis();
        
        // If we've received the first motion command, go ahead and
        // initialize the motor controller.
        // TODO: Can we use the 5V output of the motor controller to 
        // trigger the Arduino to send the initialize byte?  For example,
        // after the 5V goes high, wait a few seconds, then send the
        // initialize byte?
        /*if (!_motorInit) {
          _motorPort.write(MOTOR_START_BYTE); // set the baud rate
          _motorInit = true;
        }*/
        toggleCmdRcvdLED();
      } else {
        // Checksum failed!  TODO: Toggle a status LED
        toggleErrChecksumLED();
      }
      
    } else {
      // first byte not start byte, discard it!  
      toggleErrPktLED();
    }
    
    _steeringServo.write(_currSteeringAngleCmd);
  }
}

void toggleCmdRcvdLED() {
  digitalWrite(LED_CMD_RCVD_PIN, _ledStateCmdRcvd);
  if (_ledStateCmdRcvd == HIGH)
    _ledStateCmdRcvd = LOW;
  else
    _ledStateCmdRcvd = HIGH;
}

void toggleErrChecksumLED() {
}

void toggleErrPktLED() {
}

