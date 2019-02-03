#include "Adafruit_PWMServoDriver.h"

//abbreviations
#define PT(s) Serial.print(s)  //makes life easier
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))//trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))

#define QMECHA
#ifdef QMECHA
byte pins[] = {16, 16, 16, 16,
			         16, 16, 14, 16,
			          6, 8, 9, 7,
			          3, 12, 13, 4
			         };
#endif

//servo constants
#define DOF 16
#define PWM_FACTOR 4
#define MG92B_MIN 170*PWM_FACTOR
#define MG92B_MAX 550*PWM_FACTOR
#define MG92B_RANGE 150

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

#define SERVOMIN  MG92B_MIN // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  MG92B_MAX // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_ANG_RANGE MG92B_RANGE
#define PWM_RANGE (SERVOMAX - SERVOMIN)

float degPerRad = 180 / M_PI;
float radPerDeg = M_PI / 180;

//byte pins[] = {16, 16, 16, 16, 16, 16, 16, 16, 6, 8, 9, 7, 3, 12, 13, 4};
int8_t calibs[] = {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};

int8_t middleShifts[] = {0, 0, 0, 0,
                         0, 0, 0, 0,
                         0, 0, 0, 0,
                         0, 0, 0, 0
                        };

int8_t rotationDirections[] = {1, -1, 1, 1,
                               1, -1, 1, -1,
                               1, -1, -1, 1,
                               -1, 1, 1, -1
                              };

byte servoAngleRanges[] =  {MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE,
                            MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE,
                            MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE,
                            MG92B_RANGE, MG92B_RANGE, MG92B_RANGE, MG92B_RANGE
                           };

float pulsePerDegree[DOF] = {};
int8_t servoCalibs[DOF] = {};
char currentAng[DOF] = {};
int calibratedDuty0[DOF] = {};

class Motion {
  public:
  byte pins[DOF];
  uint8_t period;
  char* dutyAngles;

  Motion() {
    period = 0;
    dutyAngles = NULL; 
  }
};

Motion motion;

void calibratedPWM(byte i, float angle) {
  currentAng[i] = angle;
  int duty = calibratedDuty0[i] + angle * pulsePerDegree[i] * rotationDirections[i];
  duty = max(SERVOMIN, min(SERVOMAX, duty));
  PT("Moving servo from joint #"); PT(i); PT(" to "); PT(angle); PTL(" degrees");
  PT("Duty = "); PTL(duty);
  pwm.setPWM(pins[i], 0, duty);
}

void shutServos() {
  delay(100);
  for (int8_t i = DOF-1; i >=0; i--) {
    pwm.setPWM(i, 0, 4096);
  }
}

//short tools
template <typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T> void printList(T * arr, byte len = DOF) {
  for (byte i = 0; i < len; i++) {
    PT((T)(arr[i]));
    PT('\t');
  }
  PTL();
}

char getUserInput() {//limited to one character
  while (!Serial.available());
  return Serial.read();
}
