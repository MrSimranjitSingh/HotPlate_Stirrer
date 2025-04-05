void getTemperature(float *p_temperature) {
  uint8_t i;
  float average = analogRead(THERMISTOR_PIN);

  // convert the value to resistance
  average = 4095.0 / average - 1.0;
  average = SERIESRESISTOR * average;

  float steinhart;
  steinhart = log(average / THERMISTORNOMINAL) / BCOEFFICIENT;  // (1/B) * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);             // + (1/To)
  steinhart = (1.0 / steinhart) - 273.15;                       // Invert and convert absolute temp to C

  *p_temperature = steinhart;
}

void getPot() {

  static uint8_t previous_value = 255;

  int inputValue = constrain(analogRead(POT_PIN), 0, 4095);            // Ensure the input value is within the specified range
  uint8_t per_value = map(inputValue, 0, 4095, MOTOR_MIN_POWER, 100);  // Map the input value to the Percentage range

  if (per_value == previous_value) {
    return;
  }
  previous_value = per_value;

  if (per_value <= 20) {
    per_value = 0;
  }

  ledcWrite(MOTOR_PIN_A, MAX_DUTY * (per_value / 100.0));

  u8x8.drawString(0, 0, "Mot:");
  char buf[4] = {};
  sprintf(buf, "%3d", per_value);
  buf[3] = '%';
  u8x8.drawString(4, 0, buf);
}

void getRotary() {

  static int temperature = 0;

  if (RE_1->encoderChanged()) {
    temperature = numberRE1Selector.getValue();  //Time in milliseconds with duty cycle

    String s = "(" + String(temperature) + "C)";
    const char *foo = s.c_str();
    u8x8.setCursor(10, 7);
    u8x8.print("      ");
    u8x8.drawString(10, 7, foo);
  }

  if (RE_1->isEncoderButtonDown()) {
    //Do something if button is pressed
    setpoint = temperature;
  }
}