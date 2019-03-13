#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN       680
#define SERVOMAX       2200
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             240

byte servoNum;
int width;

void setServoPulse (byte n, double pulse) {
  double pulseLength;
  
  pulseLength = 1000000;  // 1,000,000 us per second
  pulseLength /= FREQUENCY;
  Serial.print(pulseLength); Serial.println(" us per period");
  pulseLength /= 4096;  // 12 bits of resolution
  Serial.print(pulseLength); Serial.println(" us per bit");

  pulse /= pulseLength;
  Serial.print("Setting "); Serial.print(n); Serial.print(" servo to "); Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void setup() {
  // verify connection
  Serial.begin(115200);
  Serial.setTimeout(2000);
  Serial.println("Connected");
  while (!Serial);
  
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  delay(200);
  
  Serial.println("Setting all servos to default position");  
  for(uint8_t i=0;i<16;i++) {
    setServoPulse(i, DEFAULT_PULSE_WIDTH);
  }
}

void loop() {
  Serial.print("Write Servo # and pulse length in us: ");
  Serial.flush();   while(!Serial.available());  // empty buffer and wait for input
  servoNum = Serial.parseInt();
  width = Serial.parseInt();
  Serial.print(servoNum); Serial.print("-->"); Serial.println(width);
  setServoPulse(servoNum, width);
  delay(1000);
}
