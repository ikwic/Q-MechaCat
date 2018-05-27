
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH       550 // equal for 0°
#define MAX_PULSE_WIDTH       2300 // equal for 180°
#define DEFAULT_PULSE_WIDTH   1500 // equal for 90°
#define FREQUENCY             50 // for digital 50 Hz, for analog 60 Hz

uint8_t servoNum = 0; // number of the servo
uint8_t servoAngle = 0; // angle for the servo

// neutral positions for the servos adjusted from nominal 90°
const float servoDeg0[8] = {90, 90, 90, 90, 90, 90, 90, 90};
// number of channel for servos on PCB with PCA9685
// order: left-front(LF), left-rear(LR), right-front(RF), right-rear(RR)
// for each one in order - knee and ankle 
const int servoOrder[8] = {0, 1, 8, 9, 4, 5, 12, 13};
// turning direction for servos (positive is servo counter-clockwise)
// const int servoDir[8];

String incomingStr; // for incoming serial data

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  // setting all servo at 0°
  for(servoNum = 0; servoNum <16; servoNum++) {
    pwm.setPWM(servoNum, 0, pulseWidth(87));
  }

  // Input string MUST BE format "number number"
  Serial.println("Write Servo number and angle separated by '-'");
}

int pulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.print("Analog value: ");
  Serial.println(analog_value);
  return analog_value;
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingStr = Serial.readString(); 

    servoNum = (incomingStr.substring(0, incomingStr.indexOf('-'))).toInt();
    servoAngle = (incomingStr.substring(incomingStr.indexOf('-')+1,incomingStr.length())).toInt();

    Serial.print("Servo #: ");
    Serial.print(servoNum);
    Serial.print("\tAngle: ");
    Serial.println(servoAngle);

    pwm.setPWM(servoNum, 0, pulseWidth(servoAngle));
    }
}
