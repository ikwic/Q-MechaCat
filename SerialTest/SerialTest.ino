#include "Q-Mecha.h"
#include "Instinct.h"

#define CMD_LEN 10
char *lastCmd = new char[CMD_LEN];
char *newCmd = new char[CMD_LEN];
byte newCmdIdx = 0;
boolean newData = false;
char token;
String inBuffer;

// testing this
uint8_t timer = 0;
byte firstWalkingJoint;
byte jointIdx = 0;

void rcvdChars() {
  char endMarker = '\n';
  char rc;
  
  while(Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      newCmd[newCmdIdx] = rc;
      newCmdIdx++;
      if (newCmdIdx >= CMD_LEN) {
        newCmdIdx = CMD_LEN - 1;
      }
    }
    else {
      newCmd[newCmdIdx] = '\0'; // terminate the string
      newCmdIdx = 0;
      newData = true;
      //while (Serial.available() && Serial.read()); // empty buffer
    }
  }
}

void cmdMenu() {
  if (newCmd[0] == 'h') {
    PTL("\n\t****** INFORMATION ******");
    PTL("Write 'cX,Y' to calibrate servos,");
    PTL("\twhere X - # of joint, Y - angle in degrees"); 
    PTL("Write 'mX,Y' to move joint #X to Y degress");
    PTL("Write 's' to show all calibrations");
    PTL("Write 'd' to shut down all servos");
  }
  else if (newCmd[0] == 'c' || newCmd[0] == 'm') {
    int8_t target[2] = {};
    char inBuffer[CMD_LEN];
    strcpy(inBuffer, newCmd+1); // ignoring first character from command to start from numbers
    char * pch;
    pch = strtok (inBuffer, " ,");
    for (byte c = 0; pch != NULL; c++) {
      target[c] = atoi(pch);
      pch = strtok(NULL, " ,");
    }
    if (newCmd[0] == 'c') {
      PT("Calibration for joint #"); PT(target[0]); PT(" is "); PT(target[1]); PTL(" degrees");
      servoCalibs[target[0]] = target[1];
    }
    else if (newCmd[0] == 'm') {
      PT("Moving servo from joint #"); PT(target[0]); PT(" to "); PT(target[1]); PTL(" degrees");
      //motion.dutyAngles[target[0]] = target[1];
    }
    int duty = SERVOMIN + PWM_RANGE / 2 + float(middleShifts[target[0]] + calibs[target[0]] + target[1]) * pulsePerDegree[target[0]] * rotationDirections[target[0]];
    PT("Duty = "); PTL(duty);
    pwm.setPWM(pins[target[0]], 0, duty);
    yield();
  }
  else if (newCmd[0] == 's') {
    PTL("Servo calibration values:");
    for (byte i = 0; i < DOF; i++) {
      PT(i); PT(",\t");
    }
    PTL();
    printList(servoCalibs);
  }
  else if (newCmd[0] == 'd') {
    PTL("Shutting down servos...");
    shutServos();
  }

  // motion block
  else if (newCmd[0] == 'k') {
    inBuffer = Serial.readStringUntil('\n');
    strcpy(newCmd, inBuffer.c_str());
    motion.loadBySkillName(newCmd);
    timer = 0;
    firstWalkingJoint = 8;
    jointIdx = firstWalkingJoint;
    transform(motion.dutyAngles, 1, firstWalkingJoint);
  }
}

void setup() {
  Serial.begin(57600);
  while(!Serial); // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  delay(100);
  PTL("Connection successful");
  
  //servo
  pwm.begin();
  pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates
  delay(200);
  for (byte i = 0; i < DOF; i++) {
    //motion.dutyAngles[i] = 0;
    pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRanges[i];
    calibratedDuty0[i] = SERVOMIN + PWM_RANGE / 2 + float(middleShifts[i] + calibs[i]) * pulsePerDegree[i] * rotationDirections[i];
    calibratedPWM(i, 0 /*motion.dutyAngles[i]*/);
  }
  printList(calibratedDuty0);
  
  PTL("Write command or 'h' for help");
}

void loop() {
  //newCmd[0] = '\0';
  //newCmdIdx = 0;

  //while (Serial.available() && Serial.read()); //flush the remaining serial buffer in case the commands are parsed incorrectly

  
  rcvdChars();
  if (newData) {
    PT("Main loop - Received: ");
    PTL(newCmd);
    newData = false;
    cmdMenu();
    }
}
