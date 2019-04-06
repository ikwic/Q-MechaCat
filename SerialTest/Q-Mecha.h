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
			          2, 12, 13, 3
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
// joints          0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
// byte pins[] = {16, 16, 16, 16, 16, 16, 16, 16,  6,  8,  9,  7,  2, 12, 13,  3};
int8_t calibs[] = {0,  0,  0,  0,  0,  0,  0,  0,  5,-12,  0,  3,-12, 12,  3, -4};

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

    int lookupAddressByName(char* skillName) {
      int skillAddressShift = 0;
      for (byte s = 0; s < NUM_SKILLS; s++) {//save skill info to on-board EEPROM, load skills to SkillList
        byte nameLen = EEPROM.read(SKILLS + skillAddressShift++);
        char* readName = new char[nameLen + 1];
        for (byte l = 0; l < nameLen; l++) {
          readName[l] = EEPROM.read(SKILLS + skillAddressShift++);
        }
        readName[nameLen] = '\0';
        if (!strcmp(readName, skillName)) {
          delete[]readName;
          return SKILLS + skillAddressShift;
        }
        delete[]readName;
        skillAddressShift += 3;//1 byte type, 1 int address
      }
      PTLF("wrong key!");
      return -1;
    }
    
    void loadBySkillName(char* skillName) {//get lookup information from on-board EEPROM and read the data array from storage
      int onBoardEepromAddress = lookupAddressByName(skillName);
      if (onBoardEepromAddress == -1)
        return;
      loadDataByOnboardEepromAddress(onBoardEepromAddress);
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

void transform(char* target, float speedRatio = 1, byte offset = 0) {
  char *diff = new char[DOF - offset], maxDiff = 0;

  for (byte i = offset; i < DOF; i++) {
    diff[i - offset] = currentAng[i] - target[i-offset];
    maxDiff = max(maxDiff, abs(diff[i - offset]));
  }

  byte steps = byte(round(maxDiff / 1.0/*degreeStep*/ / speedRatio)); // default speed is 1 degree per step

  for (byte s = 0; s <= steps; s++) {
    for (byte i = offset; i < DOF; i++) {
      float dutyAng = (target[i - offset] + (steps == 0 ? 0 : (1 + cos(M_PI * s / steps)) / 2 * diff[i - offset]));
      calibratedPWM(i, dutyAng);
    }
  }
  delete [] diff;
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
