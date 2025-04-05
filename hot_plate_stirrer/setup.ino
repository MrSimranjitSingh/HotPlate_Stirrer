void setupPins() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
}

void initDisplay() {
  while(!u8x8.begin());
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.drawString(0, 0, "Hello :)");
  delay(500);
  u8x8.clear();
}

void setupEncoders() {
  RE_1->begin();
  RE_1->setup(RE_1_ISR);
  RE_1->setBoundaries(RE_1_MIN, RE_1_MAX, false);  //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  numberRE1Selector.attachEncoder(RE_1);
  numberRE1Selector.setRange(RE_1_MIN, RE_1_MAX, RE_1_STEP, false, 0);
  numberRE1Selector.setValue(default_RE_1);

  RE_1->disableAcceleration();  //acceleration is now enabled by default
  //RE_1->setAcceleration(100);
}

void setupMotor(){
  ledcAttach(MOTOR_PIN_A, MOTOR_FREQUENCY, RESOLUTION);
  ledcWrite(MOTOR_PIN_A, 0);
  digitalWrite(MOTOR_PIN_B, LOW);
}
