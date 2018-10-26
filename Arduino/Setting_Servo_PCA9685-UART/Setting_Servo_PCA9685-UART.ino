
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// called this way it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// MPU6050 default address is 0x68
// MPU6050 mpu(0x69); // <-- use for AD0 high
MPU6050 mpu;

#define MIN_PULSE_WIDTH       550 // equal for 0 degrees
#define MAX_PULSE_WIDTH       2300 // equal for 180 degrees
#define DEFAULT_PULSE_WIDTH   1500 // equal for 90 degrees
#define FREQUENCY             50 // for digital 50 Hz, for analog 60 Hz

#define INTERRUPT_PIN 2 // use pin 2 for gyro INT
#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set True if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return staatus after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer 

// orientation/motion vars
Quaternion q;        // [w,x,y,z]           quaternion container
VectorFloat gravity; // [x, y, z]           gravity vector
float ypr[3];        // [yaw, pitch, roll]  yaw/pitch/roll container and gravity vector


// vars for Servo
uint8_t servoNum = 0; // number of the servo
uint8_t servoAngle = 0; // angle for the servo

// neutral positions for the servos adjusted from nominal 90 degrees
const float servoDeg0[8] = {90, 90, 90, 90, 90, 90, 90, 90};
// number of channel for servos on PCB with PCA9685
// order: left-front(LF), left-rear(LR), right-front(RF), right-rear(RR)
// for each one in order - knee and ankle 
const int servoOrder[8] = {0, 1, 8, 9, 4, 5, 12, 13};
// turning direction for servos (positive is servo counter-clockwise)
// const int servoDir[8];

String incomingStr; // for incoming serial data

// ================================================================
// ===               CONVERT DEG TO PULSE WIDTH                 ===
// ================================================================

int pulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.print("Analog value: ");
  Serial.println(analog_value);
  return analog_value;
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // opens serial port, sets data rate to 115200 bps
  Serial.begin(115200);

  // initialize MPU6050
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
    
  // initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  // veify connection
  Serial.println(F("\nTesting device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed")); 

  // setting all servos at 0°
  Serial.println("Setting all servos to 0 deg");
  for(int i = 0; i <8; i++) {
    pwm.setPWM(servoOrder[i], 0, pulseWidth(servoDeg0[i]));
  }
  
  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supple your own gyro offsets here, scaled for min sensitivity
  // to determine offsets use sketch "IMU_Zero"
  mpu.setXGyroOffset(90);
  mpu.setYGyroOffset(30);
  mpu.setZGyroOffset(54);
  mpu.setXAccelOffset(-922);
  mpu.setYAccelOffset(-5702);
  mpu.setZAccelOffset(699);
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detecion (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparsion
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track fifo count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }
}
