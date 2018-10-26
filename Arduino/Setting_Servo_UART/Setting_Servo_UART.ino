#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(5);  // attaches the servo on pin 5 to the servo object
  Serial.begin(115200);
}

void loop() {
  if(Serial.available()>0) {
    pos = Serial.parseInt();
    Serial.print("Received ");
    Serial.println(pos);
    
    myservo.write(pos);
  }
}

