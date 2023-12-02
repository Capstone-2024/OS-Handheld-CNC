#include <TMCStepper.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "SerialTransfer.h"

SerialTransfer myTransfer;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

#define PIN A11      // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 16 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

// MUST TURN ON POWER SUPPLY POWER FIRST BEFORE UPLOADING CODE / CONNECTING SERIAL PIN //
// MAKE SURE TO SET SERIAL MONITOR BAUD RATE TO 9600 //
// SET POWER SUPPLY CURRENT TO 2.2A//

#define STEP_PIN 54
// #define X_DIR_PIN          55
#define EN_PIN 38
#define X_MIN_PIN 3
#define X_MAX_PIN 2 // PIN 2 is used

#define Y_STEP_PIN 60
// #define Y_DIR_PIN          61
#define Y_ENABLE_PIN 56
// #define Y_MIN_PIN          14
// #define Y_MAX_PIN          -1 //PIN 15 is used

#define Z_STEP_PIN 46
// #define Z_DIR_PIN          48
#define Z_ENABLE_PIN 62
// #define Z_MIN_PIN          18
// #define Z_MAX_PIN          -1 //PIN 19 is used

#define LeftbuttonPin 18  // left hand button
#define RightbuttonPin 19 // Right hand button

#define STALL_PIN_Z 23       // Arduino pin that diag pin is attached to, reads when stall occurs
#define SERIAL_PORT Serial2  // Hardware Serial for TMC2209 #1 and TMC2209#2 in Arduino
#define DRIVER_ADDRESS 0b00  // TMC2209 Driver address according to MS1 and MS2 - BigTreeTech default driver address. Dont change
#define DRIVER_ADDRESS2 0b01 // Second TMC2209 Driver - in Y position. USE A JUMPER ON MS1 TO GET 0B01 ADDRESS
#define DRIVER_ADDRESS3 0b10 // Third TMC2209 Driver - Z axis
#define R_SENSE 0.11f        // Specific sense resistance

#define DEG_TO_RAD 0.017453
#define RAD_TO_DEG 57.2957795
#define PI 3.14159265358979323846264338327950
#define MICROSTEP 1 / 16

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 40 // [0..255] // Need to calibrate
int stepTime = 100;    // Determines speed of stepper. 160Hz step frequency
bool startup = false;  // set false after homing

// Initializing stepper driver in UART
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);
TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS3);

// Variables
bool shaftVal = true;
bool stalled_X = false;
bool leftShaftVal;
bool rightShaftVal;
bool zShaftVal = false;

float Nema17Resolution = 1.8;
const float l1 = 7.5;
const float l2 = 110.0;
const float l3 = 100.0;
const float l4 = 7.5;
const float l5 = 170.0;

// const float penOriginX = 84.104;
// const float penOriginY = 104.820;

const float penOriginX = 86.538;
const float penOriginY = 59.947;

float currentPosX;
float currentPosY;

float currentTheta1;
float currentTheta4;

float penPoints[400][3];

const float homedTheta1 = 162.2107;
const float homedTheta4 = 77.8594;

int cycle = 1;

int completed = 0;

void leftInterrupt()
{
  int breakTime = millis();
  while (((digitalRead(LeftbuttonPin) || digitalRead(RightbuttonPin))) != 0)
  {
    // Serial.println("Press both buttons!!!!");
    // Serial.print("Left");
    // Serial.println(digitalRead(LeftbuttonPin));
    // Serial.print("Right");
    // Serial.println(digitalRead(RightbuttonPin));
    delay(1000);
    if ((millis() - breakTime) >= 5000)
    {
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(Y_ENABLE_PIN, HIGH);
    }
  }
}

void rightInterrupt()
{
  int breakTime = millis();
  while (((digitalRead(LeftbuttonPin) || digitalRead(RightbuttonPin))) != 0)
  {
    // Serial.println("Press both buttons!!!!");
    // Serial.print("Left");
    // Serial.println(digitalRead(LeftbuttonPin));
    // Serial.print("Right");
    // Serial.println(digitalRead(RightbuttonPin));
    delay(1000);
    if ((millis() - breakTime) >= 5000)
    {
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(Y_ENABLE_PIN, HIGH);
    }
  }
}

void setup()
{

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // shaft direction controlled through uart: driver.shaft(true or false)

  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);

  pinMode(Z_ENABLE_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(STALL_PIN_Z, INPUT);

  pinMode(X_MAX_PIN, INPUT);
  pinMode(X_MIN_PIN, INPUT);

  Serial.flush();
  Serial.begin(115200); // Arduino USB serial for serial monitor
  myTransfer.begin(Serial);

  SERIAL_PORT.begin(9600); // HW UART drivers
  Wire.begin();            // I2C comms for MPU6050

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(50, 50, 50));
  }
  pixels.show(); // Send the updated pixel colors to the hardware.

  if (!accel.begin())
  {
    //  Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1)
      ;
  }

  // Pen Origin w.r.t center of Actuator 1 - From CAD
  currentPosX = penOriginX;
  currentPosY = penOriginY;

  currentTheta1 = homedTheta1;
  currentTheta4 = homedTheta4;

  // Compute Data Points for Possible Pen Locations

  int count = 0;

  for (int i = 0; i < 20; i++)
  {
    for (int j = 0; j < 20; j++)
    {

      float theta1 = DEG_TO_RAD * (360 / 20) * i; // The Constant value is 360 / Loop Iteration Limit!!!!
      float theta4 = DEG_TO_RAD * (360 / 20) * j;

      float xb = l1 * cos(theta1);
      float yb = l1 * sin(theta1);

      float xd = l5 + l4 * cos(theta4);
      float yd = l4 * sin(theta4);

      float r1 = sqrt(pow(xd - xb, 2) + pow(yd - yb, 2));
      float r2 = sqrt(pow(xd, 2) + pow(yd, 2));

      float theta2_1 = acos((pow(l2, 2) + pow(r1, 2) - pow(l3, 2)) / (2 * l2 * r1));
      float theta2_2 = acos((pow(l1, 2) + pow(r1, 2) - pow(r2, 2)) / (2 * l1 * r1));

      float xc;
      float yc;

      if (theta1 >= 0 && (theta1 < (PI / 2)))
      {
        float theta2in = theta2_1 + theta2_2;
        float theta2 = theta2in - PI;
        xc = xb + l2 * cos(theta1 + theta2);
        yc = yb + l2 * sin(theta1 + theta2);
      }
      else if ((theta1 >= (PI / 2)) && (theta1 < PI))
      {
        float theta2in = theta2_1 + theta2_2;
        float theta2 = PI - theta2in;
        xc = xb + l2 * cos(theta1 - theta2);
        yc = yb + l2 * sin(theta1 - theta2);
      }
      else if ((theta1 >= PI) && (theta1 < (3 * PI / 2)))
      {
        float theta2in = theta2_1 - theta2_2;
        float theta2 = PI - theta2in;
        xc = xb + l2 * cos(theta1 - theta2);
        yc = yb + l2 * sin(theta1 - theta2);
      }
      else if ((theta1 >= (3 * PI / 2)) && (theta1 <= (2 * PI)))
      {
        float theta2in = theta2_2 - theta2_1;
        float theta2 = PI - theta2in;
        xc = xb + l2 * cos(theta1 + theta2);
        yc = yb + l2 * sin(theta1 + theta2);
      }

      float thetaPen = asin((xc - xb) / l2) + DEG_TO_RAD * (29.1 + 90);
      float thetaPenInv = PI - thetaPen;
      penPoints[count][0] = xc - 45.011 * sin(thetaPenInv);
      penPoints[count][1] = yc + 45.011 * cos(thetaPenInv);
      penPoints[count][2] = thetaPenInv;

      count++;
    }
  }

  driver.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver.toff(4); // Enables driver in software, changed from 5
  driver.blank_time(24);
  driver.rms_current(1600); // Set motor RMS current
  driver.microsteps(16);    // Set microsteps to 1/16th

  // driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  // driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true); // Needed for stealthChop
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.shaft(shaftVal); // direction of rotation
  // TCOOLTHRS needs to be set for stallgaurd to work //
  // driver.TCOOLTHRS(0xFFFFF); // 20bit max
  // driver.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
  // attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING); // Interrupt on diag pin @ pin 21. Pin 21 can no longer be used for I2C
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  driver2.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver2.toff(4); // Enables driver in software, changed from 5
  driver2.blank_time(24);
  driver2.rms_current(1600); // Set motor RMS current
  driver2.microsteps(16);    // Set microsteps to 1/16th

  // driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  // driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver2.pwm_autoscale(true); // Needed for stealthChop
  driver2.semin(5);
  driver2.semax(2);
  driver2.sedn(0b01);
  driver2.shaft(shaftVal); // direction of rotation
  // TCOOLTHRS needs to be set for stallgaurd to work //
  // driver2.TCOOLTHRS(0xFFFFF); // 20bit max
  // driver2.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
  // attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING); // Interrupt on diag pin @ pin 21. Pin 21 can no longer be used for I2C
  digitalWrite(Y_ENABLE_PIN, LOW); // Enable driver in hardware

  driver3.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver3.toff(4); // Enables driver in software, changed from 5
  driver3.blank_time(24);
  driver3.rms_current(1000); // Set motor RMS current
  driver3.microsteps(8);     // Set microsteps to 1/16th

  driver3.pwm_autoscale(true); // Needed for stealthChop
  driver3.semin(5);
  driver3.semax(2);
  driver3.sedn(0b01);
  driver3.shaft(zShaftVal); // direction of rotation
  // TCOOLTHRS needs to be set for stallgaurd to work //
  driver3.TCOOLTHRS(0xFFFFF);  // 20bit max
  driver3.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
  // attachInterrupt(digitalPinToInterrupt(STALL_PIN_Z), stallInterruptZ, RISING);
  digitalWrite(Z_ENABLE_PIN, LOW); // Enable driver in hardware

  pinMode(RightbuttonPin, INPUT_PULLUP);
  pinMode(LeftbuttonPin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(LeftbuttonPin), leftInterrupt, RISING);
  // attachInterrupt(digitalPinToInterrupt(RightbuttonPin), rightInterrupt, RISING);

  // startupSequence();

  // zHomingSequence();

  // homingSequence();
}

void loop()
{
  // if (Serial.available() > 0)
  // {
  //  char instruction = Serial.read();
  //  // 2 Packet sizes, either 1 byte for simple instruction, or 9 bytes for complex instruction
  //
  //  if((int)instruction == 73)
  //  {
  //      float xPacket = Serial.parseFloat();
  //      char comma = Serial.read(); // Read and discard comma delimiter
  //      float yPacket = Serial.parseFloat();
  //
  ////      Serial.println(xPacket);
  ////      Serial.println(yPacket);
  //      autoCorrection(xPacket, yPacket);
  //  }
  //
  //  else if((int)instruction == 72)
  //  {
  //    homingSequence();
  //  }
  //
  //  else if((int)instruction == 82)
  //  {
  //    zRetract(2000);
  //  }
  //  else if((int)instruction == 90)
  //  {
  //    zHomingSequence();
  //  }
  //  else if((int)instruction == 65)
  //  {
  //    sampleAccelerometer();
  //  }
  //  else if((int)instruction == 83)
  //  {
  //    int via = 4;
  //    float poses[via][2]={{2,2},{-2,2},{-2,-2},{2,-2}};
  //    for(int i = 0; i<via; i++)
  //    {
  //      autoCorrection(poses[i][0], poses[i][1]);
  //    }
  //  }
  //  else
  //  {
  //
  //  }
  // }
  if (myTransfer.available())
  {
    uint8_t instruction = myTransfer.packet.rxBuff[0];

    if (int(instruction) == 73)
    {
      uint16_t recSize = 1; // Start after the character byte
      float xPacket;
      float yPacket;

      recSize = myTransfer.rxObj(xPacket, recSize);
      recSize = myTransfer.rxObj(yPacket, recSize);

      autoCorrection(xPacket, yPacket);
    }

    else if (int(instruction) == 72)
    {
      homingSequence();
    }

    else if (int(instruction) == 83)
    {
      zRetract(2000);
    }

    else if (int(instruction) == 90)
    {
      zHomingSequence();
    }

    else if (int(instruction) == 65)
    {
      sampleAccelerometer();
    }
  }

  digitalWrite(EN_PIN, LOW);       // Enable driver in hardware
  digitalWrite(Y_ENABLE_PIN, LOW); // Enable driver in hardware
  digitalWrite(Z_ENABLE_PIN, LOW);

  Serial.flush();
}

void autoCorrection(float desiredDeltaX, float desiredDeltaY)
{
  float desiredJointAngles[2] = {0, 0};

  // Going from Global to Local Coordinate System
  float currentSpindleDeltaX = currentPosX - penOriginX;
  float currentSpindleDeltaY = currentPosY - penOriginY;

  // Serial.print("currentSpindleDeltaX: ");
  // Serial.println(currentSpindleDeltaX);
  // Serial.print("currentSpindleDeltaY: ");
  // Serial.println(currentSpindleDeltaY);

  desiredDeltaX = desiredDeltaX - currentSpindleDeltaX;
  desiredDeltaY = desiredDeltaY - currentSpindleDeltaY;

  InvKin(desiredDeltaX, desiredDeltaY, currentPosX, currentPosY, desiredJointAngles);

  int desiredSteps[2] = {0, 0};

  desiredSteps[0] = DegToSteps(desiredJointAngles[0] - currentTheta1);
  desiredSteps[1] = DegToSteps(desiredJointAngles[1] - currentTheta4);

  // Serial.print("Steps 1:");
  // Serial.println(desiredSteps[0]);
  // Serial.print("Steps 4:");
  // Serial.println(desiredSteps[1]);

  if (desiredSteps[0] > 0)
  {
    driver.shaft(true);
    leftShaftVal = true;
  }
  else
  {
    driver.shaft(false);
    leftShaftVal = false;
  }

  if (desiredSteps[1] > 0)
  {
    driver2.shaft(true);
    rightShaftVal = true;
  }
  else
  {
    driver2.shaft(false);
    rightShaftVal = false;
  }

  desiredSteps[0] = abs(desiredSteps[0]);
  desiredSteps[1] = abs(desiredSteps[1]);

  // Moving in Straight Line
  int stepRatio = max(desiredSteps[0], desiredSteps[1]) / min(desiredSteps[0], desiredSteps[1]);
  int stepRemainder = max(desiredSteps[0], desiredSteps[1]) % min(desiredSteps[0], desiredSteps[1]);
  int loopIterations = min(desiredSteps[0], desiredSteps[1]);
  int greater = (desiredSteps[0] > desiredSteps[1]) ? 1 : 0; // if condition is true, set to 1, else set to 0.

  int randomArray[loopIterations];

  remainderArray(loopIterations, stepRemainder, randomArray);

  fineTuning(stepRatio, loopIterations, randomArray, greater);
}

void InvKin(float desiredDeltaX, float desiredDeltaY, float currentPosX, float currentPosY, float desiredJointAngles[])
{

  float x = currentPosX + desiredDeltaX;
  float y = currentPosY + desiredDeltaY;

  //  Serial.print("Pen X: ");
  //  Serial.println(x);
  //  Serial.print("Pen Y: ");
  //  Serial.println(y);

  //  float minDiff = 1000;
  //  int minIndex = 0;
  //
  //  for(int i = 0; i<400; i++)
  //  {
  //    float xDiff = abs(x - penPoints[i][0]);
  //    float yDiff = abs(y - penPoints[i][1]);
  //    float totDiff = xDiff + yDiff;
  //
  //    if(totDiff < minDiff)
  //    {
  //      minDiff = totDiff;
  //      minIndex = i;
  //    }
  //  }
  //
  //  float thetaPenCurr = penPoints[minIndex][2];
  //
  //  Serial.print("Theta Pen: ");
  //  Serial.println(RAD_TO_DEG*thetaPenCurr);
  //
  //  x = x + 45.011*sin(thetaPenCurr);
  //  y = y - 45.011*cos(thetaPenCurr);

  // Serial.print("End Effector X: ");
  // Serial.println(x);
  // Serial.print("End Effector Y: ");
  // Serial.println(y);

  float w = l5;

  float c1 = sqrt(x * x + y * y);
  float c2 = sqrt(c1 * c1 - 2 * x * w + w * w);

  float alpha1 = acos(x / c1);
  float alpha2 = acos((-x + w) / c2);

  float beta1 = acos((l2 * l2 - l1 * l1 - c1 * c1) / (-2 * l1 * c1));
  float beta2 = acos((l3 * l3 - l4 * l4 - c2 * c2) / (-2 * l4 * c2));

  // For manipulator of working mode 1
  float theta1 = RAD_TO_DEG * (alpha1 + beta1);
  float theta4 = RAD_TO_DEG * (PI - (alpha2 + beta2));

  if (theta4 < 0)
  {
    theta4 = 360 + theta4;
  }

  if (isnan(theta1))
  {
    theta1 = 0;
    theta4 = 0;
  }
  if (isnan(theta4))
  {
    theta1 = 0;
    theta4 = 0;
  }

  // Serial.print("T1: ");
  // Serial.println(theta1);
  // Serial.print("T4: ");
  // Serial.println(theta4);

  desiredJointAngles[0] = theta1;
  desiredJointAngles[1] = theta4;
}

void forwardKin(float theta1, float theta4)
{
  theta1 = DEG_TO_RAD * theta1;
  theta4 = DEG_TO_RAD * theta4;

  float xb = l1 * cos(theta1);
  float yb = l1 * sin(theta1);

  float xd = l5 + l4 * cos(theta4);
  float yd = l4 * sin(theta4);

  float r1 = sqrt(pow(xd - xb, 2) + pow(yd - yb, 2));
  float r2 = sqrt(pow(xd, 2) + pow(yd, 2));

  float theta2_1 = acos((pow(l2, 2) + pow(r1, 2) - pow(l3, 2)) / (2 * l2 * r1));
  float theta2_2 = acos((pow(l1, 2) + pow(r1, 2) - pow(r2, 2)) / (2 * l1 * r1));

  float xc;
  float yc;

  if (theta1 >= 0 && (theta1 < (PI / 2)))
  {
    float theta2in = theta2_1 + theta2_2;
    float theta2 = theta2in - PI;
    xc = xb + l2 * cos(theta1 + theta2);
    yc = yb + l2 * sin(theta1 + theta2);
  }
  else if ((theta1 >= (PI / 2)) && (theta1 < PI))
  {
    float theta2in = theta2_1 + theta2_2;
    float theta2 = PI - theta2in;
    xc = xb + l2 * cos(theta1 - theta2);
    yc = yb + l2 * sin(theta1 - theta2);
  }
  else if ((theta1 >= PI) && (theta1 < (3 * PI / 2)))
  {
    float theta2in = theta2_1 - theta2_2;
    float theta2 = PI - theta2in;
    xc = xb + l2 * cos(theta1 - theta2);
    yc = yb + l2 * sin(theta1 - theta2);
  }
  else if ((theta1 >= (3 * PI / 2)) && (theta1 <= (2 * PI)))
  {
    float theta2in = theta2_2 - theta2_1;
    float theta2 = PI - theta2in;
    xc = xb + l2 * cos(theta1 + theta2);
    yc = yb + l2 * sin(theta1 + theta2);
  }

  currentPosX = xc;
  currentPosY = yc;

  // Serial.print("Curr Pos X: ");
  // Serial.println(currentPosX);
  // Serial.print("Curr Pos Y: ");
  // Serial.println(currentPosY);
}

int DegToSteps(float deltaTheta)
{
  return (int)(deltaTheta / (Nema17Resolution * MICROSTEP));
}

float StepsToDeg(int steps)
{
  return float(steps * Nema17Resolution * MICROSTEP);
}

/*
 * Random computes an array of size total steps. Fills with 0's
 * Fills 1's in the array up to the modulo.
 */
void remainderArray(int arraySize, int stepRemainder, int randomArray[])
{
  // Fill the array with zeros
  for (int i = 0; i < arraySize; i++)
  {
    randomArray[i] = 0;
  }

  // Add y occurrences of 1's randomly
  for (int i = 0; i < stepRemainder; i++)
  {
    int randomIndex = random(0, arraySize); // Generate a random index
    while (randomArray[randomIndex] != 0)
    {
      randomIndex = random(0, arraySize);
    }
    randomArray[randomIndex] = 1; // Set the value at the random index to 1
  }
}

/* Fine Tuning Method
 * Will step the motors in the ratio described by stepRatio.
 * Will also steps randomly based on the randomArray.
 */
void fineTuning(int stepRatio, int loopIterations, int randomArray[], int greater)
{
  int leftStepsTaken = 0;
  int rightStepsTaken = 0;

  // Serial.print("T1 Upon Entry: ");
  // Serial.println(currentTheta1);
  // Serial.print("T4 Upon Entry: ");
  // Serial.println(currentTheta4);

  for (int i = 0; i < loopIterations; i++)
  {
    if (greater == 1)
    {
      motorLeft(stepRatio + randomArray[i], stepTime);
      motorRight(1, stepTime);

      leftStepsTaken = leftStepsTaken + (stepRatio + randomArray[i]);
      rightStepsTaken = rightStepsTaken + 1;
    }
    else
    {
      motorLeft(1, stepTime);
      motorRight(stepRatio + randomArray[i], stepTime);

      leftStepsTaken = leftStepsTaken + 1;
      rightStepsTaken = rightStepsTaken + (stepRatio + randomArray[i]);
    }
    
    if (myTransfer.available() || Serial.available())
    {
      if (myTransfer.packet.rxBuff[0] == 73)
      {
        // Update position w.r.t how far we actually travelled.

        // Serial.print("RIGHT STEPS TAKEN: ");
        // Serial.println(rightStepsTaken);
        // Serial.print("LEFT STEPS TAKEN: ");
        // Serial.println(leftStepsTaken);

        if (leftShaftVal == true)
        {
          currentTheta1 = currentTheta1 + StepsToDeg(leftStepsTaken);
          // Serial.print("CurrTheta1: ");
          // Serial.println(currentTheta1);
        }
        else
        {
          currentTheta1 = currentTheta1 - StepsToDeg(leftStepsTaken);
          //   Serial.print("CurrTheta1: ");
          // Serial.println(currentTheta1);
        }

        if (rightShaftVal == true)
        {
          currentTheta4 = currentTheta4 + StepsToDeg(rightStepsTaken);
          //   Serial.print("CurrTheta4: ");
          // Serial.println(currentTheta4);
        }
        else
        {
          currentTheta4 = currentTheta4 - StepsToDeg(rightStepsTaken);
          //   Serial.print("CurrTheta4: ");
          // Serial.println(currentTheta4);
        }

        forwardKin(currentTheta1, currentTheta4);

        break;
      }
    }
  }
  // Update Pos and Angles

  // Serial.print("RIGHT STEPS TAKEN: ");
  // Serial.println(rightStepsTaken);
  // Serial.print("LEFT STEPS TAKEN: ");
  // Serial.println(leftStepsTaken);

  if (leftShaftVal == true)
  {
    currentTheta1 = currentTheta1 + StepsToDeg(leftStepsTaken);
    // Serial.print("Left Degrees Taken: ");
    // Serial.println(StepsToDeg(leftStepsTaken));
    // Serial.print("CurrTheta1: ");
    // Serial.println(currentTheta1);
  }
  else
  {
    currentTheta1 = currentTheta1 - StepsToDeg(leftStepsTaken);
    // Serial.print("Left Degrees Taken: ");
    // Serial.println(StepsToDeg(leftStepsTaken));
    // Serial.print("CurrTheta1: ");
    // Serial.println(currentTheta1);
  }

  if (rightShaftVal == true)
  {
    currentTheta4 = currentTheta4 + StepsToDeg(rightStepsTaken);
    //   Serial.print("Right Degrees Taken: ");
    //   Serial.println(StepsToDeg(rightStepsTaken));
    //   Serial.print("CurrTheta4: ");
    // Serial.println(currentTheta4);
  }
  else
  {
    currentTheta4 = currentTheta4 - StepsToDeg(rightStepsTaken);
    //   Serial.print("Right Degrees Taken: ");
    //   Serial.println(StepsToDeg(rightStepsTaken));
    //   Serial.print("CurrTheta4: ");
    // Serial.println(currentTheta4);
  }

  forwardKin(currentTheta1, currentTheta4);
}

void startupSequence()
{
  while (((digitalRead(LeftbuttonPin) || digitalRead(RightbuttonPin))) != 0)
  {
    // Serial.println("Press both buttons!!!!");
    // Serial.print("Left");
    // Serial.println(digitalRead(LeftbuttonPin));
    // Serial.print("Right");
    // Serial.println(digitalRead(RightbuttonPin));
    delay(1000);
  }
}

void homingSequence()
{
  int flagLeft = 0;
  int flagRight = 0;

  driver.shaft(true);
  driver2.shaft(true);

  while (flagLeft * flagRight != 1)
  {
    if (flagLeft != 1)
    {
      motorLeft(1, 450);
    }

    if (flagRight != 1)
    {
      motorRight(1, 450);
    }

    if (digitalRead(X_MAX_PIN) == 0)
    {
      flagRight = 1;
    }
    if (digitalRead(X_MIN_PIN) == 0)
    {
      flagLeft = 1;
    }
  }

  currentPosX = penOriginX;
  currentPosY = penOriginY;
  currentTheta1 = homedTheta1;
  currentTheta4 = homedTheta4;

  //    pixels.clear();
  //    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
  //
  //    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
  //    // Here we're using a moderately bright green color:
  //    pixels.setPixelColor(i, pixels.Color(0, 150, 0));
  //
  //    pixels.show();   // Send the updated pixel colors to the hardware.
  //
  //    delay(DELAYVAL); // Pause before next pass through loop
  //    }
  //    pixels.clear();

  // delay(2000);
}

void motorLeft(int steps, int stepDelay)
{
  // Producing steps to drive stepper motor
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay); // Uses the stepping frequency to determine speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
  // digitalWrite(EN_PIN,HIGH); // Disables in hardware at the end of each instruction. Limits power draw
}

void motorRight(int steps, int stepDelay)
{
  // Producing steps to drive stepper motor
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(stepDelay); // Uses the stepping frequency to determine speed
    digitalWrite(Y_STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
  // digitalWrite(Y_ENABLE_PIN,HIGH); // Disables in hardware at the end of each instruction. Limits power draw
}

void motorVert(int steps, int stepDelay)
{
  driver3.shaft(zShaftVal);
  // Producing steps to drive stepper motor
  for (int i = 0; i < steps; i++)
  {
    digitalWrite(Z_STEP_PIN, HIGH);
    delayMicroseconds(stepDelay); // Uses the stepping frequency to determine speed
    digitalWrite(Z_STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

void zHomingSequence()
{
  digitalWrite(Z_ENABLE_PIN, LOW);
  zShaftVal = false;
  motorVert(6000, 200);
  bool operation = true;
  while (operation)
  {
    motorVert(5, 750);

    if (digitalRead(STALL_PIN_Z) == HIGH)
    {
      digitalWrite(Z_ENABLE_PIN, HIGH);
      operation = false;
    }
  }
  delay(500);
  // zRetract(400);
}

void zRetract(int steps)
{
  digitalWrite(Z_ENABLE_PIN, LOW);
  zShaftVal = true;
  driver3.shaft(shaftVal);
  motorVert(steps, 200);
}

void sampleAccelerometer()
{
  sensors_event_t event;

  accel.getEvent(&event);

  float z_accel = event.acceleration.z;
  float y_accel = event.acceleration.y;

  uint16_t sendSize = 0;
  sendSize = myTransfer.txObj(z_accel, sendSize);
  sendSize = myTransfer.txObj(y_accel, sendSize);
  myTransfer.sendData(sendSize);

  return;
}
