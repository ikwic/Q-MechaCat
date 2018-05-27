#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH       550 // equal for 0°
#define MAX_PULSE_WIDTH       2300 // equal for 180°
#define DEFAULT_PULSE_WIDTH   1500 // equal for 90°
#define FREQUENCY             50 // for digital 50 Hz, for analog 60 Hz

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
