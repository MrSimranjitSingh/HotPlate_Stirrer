void initPID() {

  for (int i = 0; i < 3; i++) {
    pidParams[i] = DEFAULT_PID[i];
  }

  initPID2();
}

void initPID2() {
  // Set the PID controller parameters
  pid.SetTunings(pidParams[0], pidParams[1], pidParams[2]);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(minHeaterPower, maxHeaterPower);
  pid.SetSampleTime(windowSize);  //Set to higher value for slow systems
}

uint8_t setPidOutput(float *temperature) {
  // Update the input value for the PID controller
  input = *temperature;
  // Compute the output value using the PID controller
  pid.Compute();
  // Write the output value to the heater
  digitalWrite(RELAY_PIN, output);
  return output;
}


/*void savePID(double kp, double ki, double kd) {

  //Clear the EEPROM
  EEPROM.begin();
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();

  //Display message
  DEBUG_PRINT("EEPROM cleared...");

  // Save the PID parameters
  pidParams[0] = kp;
  pidParams[1] = ki;
  pidParams[2] = kd;

  EEPROM.put(EEPROM_PID_ADDRESS, pidParams);
}*/


// Generate distributed pulse pattern using Bresenham's algorithm
void generatePulsePattern(double outputPercent) {
  int onCount = (outputPercent * segments) / 100;
  int phase = 0;

  for (int i = 0; i < segments; i++) {
    phase += onCount;
    if (phase >= segments) {
      pulsePattern[i] = true;
      //Serial.print("1");
      phase -= segments;
    } else {
      pulsePattern[i] = false;
      //Serial.print("0");
    }
  }
  //Serial.println("");
}