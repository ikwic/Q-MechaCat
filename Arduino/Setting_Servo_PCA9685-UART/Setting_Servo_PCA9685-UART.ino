#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN       500
#define SERVOMAX       2500
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             240

int pos;
int pulse;

void setup() {
  // veify connection
  Serial.begin(115200);
  Serial.setTimeout(2000);
  Serial.println("Connected");
  while (!Serial);
  
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(200);
  
  for(uint8_t i=0;i<16;i++) {
    //Serial.print("Servo #:"); Serial.print(i); Serial.print("\t");
    pwm.setPWM(i, 0, DEFAULT_PULSE_WIDTH);
  }
}

void loop() {
  Serial.print("Write Servo # and pulse width: ");
  Serial.flush();   while(!Serial.available());  // empty buffer and wait for input
  pos = Serial.parseInt();
  pulse = Serial.parseInt();
  Serial.print(pos); Serial.print("-->"); Serial.println(pulse);
  pwm.setPWM(pos, 0, pulse);
  delay(1000);
}
