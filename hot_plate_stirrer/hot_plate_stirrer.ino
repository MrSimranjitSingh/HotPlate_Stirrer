#include <PID_v1.h>
#include <U8x8lib.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <Smoothed.h>
#include "AiEsp32RotaryEncoder.h"
#include "AiEsp32RotaryEncoderNumberSelector.h"

//#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTLN(x)
#endif

//KP, KI, KD step values (for use with rotary encoder)
#define DECIMAL_PLACES 4
#define kp_STEP 0.050
#define ki_STEP 0.005
#define kd_STEP 0.050

#define THERMISTORNOMINAL 97500  // resistance at 25 degrees C
#define TEMPERATURENOMINAL 25    // temp. for nominal resistance (almost always 25 C)
#define BCOEFFICIENT 3950        // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 29940     // the value of the 'other' resistor

#define RELAY_FREQ 60      //frequency (in Hz)

#define THERMISTOR_PIN 34  // which analog pin to connect
#define RELAY_PIN 4
#define POT_PIN 32
#define MOTOR_PIN_A 13
#define MOTOR_PIN_B 12
#define MOTOR_MIN_POWER 20

//Rotary Encoder
#define RE_1_A_PIN 25
#define RE_1_B_PIN 26
#define RE_1_BUTTON_PIN 27
#define RE_VCC_PIN -1  //Put -1 if Rotary encoder Vcc is connected directly to 3.3V
#define RE_STEPS 2     //depending on your encoder - try 1,2 or 4 to get expected behaviour

#define RE_1_MIN 10
#define RE_1_MAX 300
#define RE_1_STEP 1
const int default_RE_1 = 15;


#define RESOLUTION 8
#define MOTOR_FREQUENCY 20000

const int MAX_DUTY = (int)(pow(2, RESOLUTION) - 1);

#define minHeaterPower 0
#define maxHeaterPower 100
const long windowSize = 2000;  // 2-second control window

double pidParams[3];
const double DEFAULT_PID[3] = { 3.000, 0.015, 0.000 };
double input, output;
double setpoint = 27.0;  // Default to Green Tea

// Define Power Control parameters
int totalCycles = 100;                   // Number of cycles in the control period
int onCycles = 0;                        // Number of ON cycles
bool cycleStates[100];                   // Array to store ON/OFF states for individual cycles
int currentCycle = 0;                    // Current cycle in the control period
const unsigned long cycleDuration = 16;  // ~16 ms for 60 Hz

Smoothed<float> mySensor;


AiEsp32RotaryEncoder* RE_1 = new AiEsp32RotaryEncoder(RE_1_A_PIN, RE_1_B_PIN, RE_1_BUTTON_PIN, RE_VCC_PIN, RE_STEPS);
AiEsp32RotaryEncoderNumberSelector numberRE1Selector = AiEsp32RotaryEncoderNumberSelector();

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
PID pid(&input, &output, &setpoint, DEFAULT_PID[0], DEFAULT_PID[1], DEFAULT_PID[2], DIRECT);

void IRAM_ATTR RE_1_ISR() {
  RE_1->readEncoder_ISR();
}

/*
Number of Segments = Segment Duration / Control Window Size
2000ms / 20ms = 100segments
2000ms / 16ms = 125 segments (60Hz signal)
*/

// Distributed Pulse Control
const int segments = 125;  // Number of segments per window
unsigned long segmentDuration = windowSize / segments;

bool pulsePattern[segments];  // Array to store the switching pattern
int currentSegment = 0;       // Track current segment

void setup() {

  Serial.begin(115200);

  setupPins();
  initDisplay();
  setupEncoders();
  setupMotor();

  mySensor.begin(SMOOTHED_EXPONENTIAL, 3);

  initPID();
  generatePulsePattern(0);  // Initialize with 0% output
}

void loop() {

  getPot();
  getRotary();

  static float smooth_temp = 0;

  static unsigned long previousMillis_temp = 0;  // will store last time LED was updated
  static unsigned long windowStartTime = millis();

  unsigned long currentMillis_temp = millis();
  if (currentMillis_temp - previousMillis_temp >= 500) {
    previousMillis_temp = currentMillis_temp;
    //setpoint = 50;
    printTemp(&smooth_temp);
    printOutput();

    getTemperature(&smooth_temp);  //Get the temperature
    mySensor.add(smooth_temp);     //Add the new value to value stores
    smooth_temp = mySensor.get();  // Get the smoothed values

    printPID();
  }
  
  
  unsigned long currentTime = millis();
  // Update PID and generate new pulse pattern at window end
  if (currentTime - windowStartTime >= windowSize) {
    windowStartTime = currentTime;
    input = smooth_temp;
    pid.Compute();
    generatePulsePattern(output);
    currentSegment = 0;
  }

  // Calculate current segment
  unsigned long elapsed = currentTime - windowStartTime;
  int newSegment = elapsed / segmentDuration;

  u8x8.setCursor(12, 3);
  u8x8.print(newSegment);
  u8x8.print(" ");

  // Update SSR state when segment changes
  if (newSegment != currentSegment) {
    currentSegment = newSegment;
    if (currentSegment < segments) {

      if (pulsePattern[currentSegment]) {
        digitalWrite(RELAY_PIN, HIGH);
      } else {
        digitalWrite(RELAY_PIN, LOW);
      }
    }
  }
}
