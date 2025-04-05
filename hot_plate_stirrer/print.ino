void printOutput() {
  u8x8.setCursor(0, 5);
  u8x8.print("Power ");
  u8x8.print(output);
  u8x8.print(" ");
}

void printTemp(float *filter_temperature) {
  u8x8.setCursor(0, 7);
  u8x8.print("          ");
  u8x8.setCursor(0, 7);
  u8x8.print(*filter_temperature, 2);
  u8x8.print("> ");
  u8x8.print((int)setpoint);
}


void printPID(){
  u8x8.setCursor(10, 0);
  u8x8.print(pidParams[0], 3);
  u8x8.setCursor(10, 1);
  u8x8.print(pidParams[1], 3);
  u8x8.setCursor(10, 2);
  u8x8.print(pidParams[2], 3);
}