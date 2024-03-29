#include <TMCStepper.h>

// MUST TURN ON POWER SUPPLY POWER FIRST BEFORE UPLOADING CODE / CONNECTING SERIAL PIN //
// MAKE SURE TO SET SERIAL MONITOR BAUD RATE TO 9600 //
// SET POWER SUPPLY CURRENT TO 2.2A//

#define STEP_PIN         54
//#define X_DIR_PIN          55
#define EN_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN          2 //PIN 2 is used

#define Y_STEP_PIN         60
//#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
//#define Y_MIN_PIN          14
//#define Y_MAX_PIN          -1 //PIN 15 is used

//#define Z_STEP_PIN         46
//#define Z_DIR_PIN          48
//#define Z_ENABLE_PIN       62
//#define Z_MIN_PIN          18
//#define Z_MAX_PIN          -1 //PIN 19 is used


#define STALL_PIN_X 21 // Arduino pin that diag pin is attached to, reads when stall occurs
#define SERIAL_PORT Serial // Hardware Serial for TMC2209 #1 in Arduino
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2 - BigTreeTech default driver address. Dont change
#define DRIVER_ADDRESS2 0b01 // Second TMC2209 Driver - in Y position. USE A JUMPER ON MS1 TO GET 0B01 ADDRESS
#define R_SENSE 0.11f // Specific sense resistance

#define DEG_TO_RAD 0.017453
#define RAD_TO_DEG 57.2957795
#define PI 3.14159265358979323846264338327950
#define MICROSTEP 1/16

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 50 // [0..255] // Need to calibrate
int stepTime = 160; // Determines speed of stepper. 160Hz step frequency
bool startup = false; // set false after homing

// Initializing stepper driver in UART
  TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
  TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);

// Variables
bool shaftVal = true;
bool stalled_X = false;

float Nema17Resolution = 1.8;
const float l1 = 7.5;
const float l2 = 110.0;
const float l3 = 100.0;
const float l4 = 7.5;
const float l5 = 170.0;

float currentTheta1;
float currentTheta4;

float homedPosX;
float homedPosY;

float currentPosX;
float currentPosY;

float A;
float B;
float C;

const float homedTheta1 = 162.88*DEG_TO_RAD;
const float homedTheta4 = 216.99*DEG_TO_RAD;

int cycle = 1;

void setup() {
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // shaft direction controlled through uart: driver.shaft(true or false)
  pinMode(STALL_PIN_X, INPUT);

  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);

  pinMode(X_MAX_PIN, INPUT);
  pinMode(X_MIN_PIN, INPUT);

  //Serial.begin(9600); // Arduino USB serial for serial monitor
  SERIAL_PORT.begin(9600); // HW UART drivers

  A = 2*l3*l4*sin(homedTheta4) - 2*l1*l3*cos(homedTheta1);
  B = 2*l3*l5 - 2*l1*l3*cos(homedTheta1) + 2*l3*l4*cos(homedTheta4);
  C = l1*l1-l2*l2+l3*l3+l4*l4+l5*l5-l1*l4*sin(homedTheta1)*sin(homedTheta4)-2*l1*l5*cos(homedTheta1)+2*l4*l5*cos(homedTheta4)-2*l1*l4*cos(homedTheta1)*cos(homedTheta4);

  float theta3_1 = 2*atan((A+sqrt(A*A+B*B-C*C))/(B-C));

  float theta2 = asin((l3*sin(theta3_1) + l4*sin(homedTheta4) - l1*sin(homedTheta1)) / l2);

  homedPosX = l1*cos(homedTheta1) + l2*cos(theta2);
  homedPosY = l1*sin(homedTheta1) + l2*sin(theta2);

  currentPosX = homedPosX;
  currentPosY = homedPosY;

  currentTheta1 = RAD_TO_DEG*homedTheta1;
  currentTheta4 = RAD_TO_DEG*homedTheta4;

  driver.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver.toff(4); // Enables driver in software, changed from 5
  driver.blank_time(24);
  driver.rms_current(1600); // Set motor RMS current
  driver.microsteps(16); // Set microsteps to 1/16th

  //driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  //driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver.pwm_autoscale(true); // Needed for stealthChop
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.shaft(shaftVal); // direction of rotation
  // TCOOLTHRS needs to be set for stallgaurd to work //
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
  //attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING); // Interrupt on diag pin @ pin 21. Pin 21 can no longer be used for I2C
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  driver2.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver2.toff(4); // Enables driver in software, changed from 5
  driver2.blank_time(24);
  driver2.rms_current(1600); // Set motor RMS current
  driver2.microsteps(16); // Set microsteps to 1/16th

  //driver.en_pwm_mode(true); // Toggle stealthChop on TMC2130/2160/5130/5160
  //driver.en_spreadCycle(false); // Toggle spreadCycle on TMC2208/2209/2224
  driver2.pwm_autoscale(true); // Needed for stealthChop
  driver2.semin(5);
  driver2.semax(2);
  driver2.sedn(0b01);
  driver2.shaft(shaftVal); // direction of rotation
  // TCOOLTHRS needs to be set for stallgaurd to work //
  driver2.TCOOLTHRS(0xFFFFF); // 20bit max
  driver2.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
  //attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING); // Interrupt on diag pin @ pin 21. Pin 21 can no longer be used for I2C
  digitalWrite(Y_ENABLE_PIN, LOW); // Enable driver in hardware

  homingSequence();

}

void loop() 
{  

  digitalWrite(EN_PIN, LOW); // Enable driver in hardware
  digitalWrite(Y_ENABLE_PIN, LOW); // Enable driver in hardware
  
  float desiredJointAngles[2] = {0, 0};

  float desiredDeltaX;
  float desiredDeltaY;

  if(cycle%2 == 1)
  {
    desiredDeltaX = 5;
    desiredDeltaY = 5;
  }

  else
  {
    desiredDeltaX = -5;
    desiredDeltaY = -5;
  }

  cycle++;

  InvKin(desiredDeltaX, desiredDeltaY, currentPosX, currentPosY, desiredJointAngles);

  currentPosX = currentPosX + desiredDeltaX;
  currentPosY = currentPosY + desiredDeltaY;

//  Serial.print("Current X: ");
//  Serial.println(currentPosX);
//  Serial.print("Current Y: ");
//  Serial.println(currentPosY);
    
  int desiredSteps[2] = {0, 0};

  desiredSteps[0] = DegToSteps(desiredJointAngles[0] - currentTheta1);
  desiredSteps[1] = DegToSteps(desiredJointAngles[1] - currentTheta4);  

  currentTheta1 = desiredJointAngles[0];
  currentTheta4 = desiredJointAngles[1];

  Serial.print("Desired Steps Left: ");
  Serial.println(desiredSteps[0]);
  Serial.print("Desired Steps Right: ");
  Serial.println(desiredSteps[1]);
  
  if(desiredSteps[0] > 0)
  {
    driver.shaft(false);
  }
  else
  {
    driver.shaft(true);
  }

  if(desiredSteps[1] > 0)
  {
    driver2.shaft(false);
  }
  else
  {
    driver2.shaft(true);
  }

  desiredSteps[0] = abs(desiredSteps[0]);
  desiredSteps[1] = abs(desiredSteps[1]);

  // Moving in Straight Line
  int stepRatio = max(desiredSteps[0], desiredSteps[1]) / min(desiredSteps[0], desiredSteps[1]);
  int stepRemainder = max(desiredSteps[0], desiredSteps[1]) % min(desiredSteps[0], desiredSteps[1]);
  int loopIterations = min(desiredSteps[0], desiredSteps[1]);
  int greater = (desiredSteps[0] > desiredSteps[1]) ? 1 : 0; //if condition is true, set to 1, else set to 0.
  
  Serial.print("Step Ratio: ");
  Serial.println(stepRatio);
  Serial.print("Step Remainder: ");
  Serial.println(stepRemainder);
  Serial.print("Loop Iterations: ");
  Serial.println(loopIterations);

  int randomArray[loopIterations];

  remainderArray(loopIterations, stepRemainder, randomArray);

  fineTuning(stepRatio, loopIterations, randomArray, greater);

  Serial.print("Theta 1: ");
  Serial.println(currentTheta1);
  Serial.print("Theta 4: ");
  Serial.println(currentTheta4);

  digitalWrite(EN_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);

  delay(5000);

  digitalWrite(EN_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
}

void InvKin(float desiredDeltaX, float desiredDeltaY, float currentPosX, float currentPosY, float desiredJointAngles[])
{
  float desiredPosX = currentPosX + desiredDeltaX;
  float desiredPosY = currentPosY + desiredDeltaY;

  A = desiredPosX;
  B = desiredPosY;
  C = (l1*l1-l2*l2+desiredPosX*desiredPosX+desiredPosY*desiredPosY)/(2*l1);

  float theta1 = 2*atan((-B-sqrt(A*A+B*B-C*C))/(-A-C));
  theta1 = RAD_TO_DEG*theta1;
  
  A = desiredPosX-l5;
  B = desiredPosY;
  C = (l4*l4+l5*l5-l3*l3-2*desiredPosX*l5+desiredPosX*desiredPosX+desiredPosY*desiredPosY)/(2*l4);
    
  float theta4 = 2*atan((-B+sqrt(A*A+B*B-C*C))/(-A-C));
  theta4 = RAD_TO_DEG*theta4;

  desiredJointAngles[0] = theta1;
  desiredJointAngles[1] = theta4;
 
}

int DegToSteps(float deltaTheta)
{
   return (int)(deltaTheta / (Nema17Resolution * MICROSTEP));
}

/*
 * Random computes an array of size total steps. Fills with 0's
 * Fills 1's in the array up to the modulo.
 */
void remainderArray(int arraySize, int stepRemainder, int randomArray[])
{
  // Fill the array with zeros
  for (int i = 0; i < arraySize; i++) {
    randomArray[i] = 0;
  }

  // Add y occurrences of 1's randomly
  for (int i = 0; i < stepRemainder; i++) {
    int randomIndex = random(0, arraySize); // Generate a random index
    randomArray[randomIndex] = 1;       // Set the value at the random index to 1
  }
}


/* Fine Tuning Method
 * Will step the motors in the ratio described by stepRatio. 
 * Will also steps randomly based on the randomArray.
 */
void fineTuning(int stepRatio, int loopIterations, int randomArray [], int greater)
{
  for (int i=1; i <= loopIterations; i++)
  {
    if(greater == 1)
    { 
    motorLeft(stepRatio + randomArray[i], 160);
    motorRight(1, 160);
    }
    else
    {
    motorLeft(1, 160);
    motorRight(stepRatio + randomArray[i], 160);
    }
  }
}

void homingSequence()
{
    shaftVal = true;

    while(digitalRead(X_MAX_PIN) != 0 || digitalRead(X_MIN_PIN) != 0)
    {
    motorLeft(1,240);
    motorRight(1,240);

    if(digitalRead(X_MAX_PIN)==0)
    {
      digitalWrite(EN_PIN, HIGH);
    }
    if(digitalRead(X_MIN_PIN)==0)
    {
      digitalWrite(Y_ENABLE_PIN, HIGH);
    }
    }
}

void motorLeft(int steps, int stepDelay)
{
  
  // Producing steps to drive stepper motor
  for(int i = 0; i<steps; i++){
    digitalWrite(STEP_PIN,HIGH);
    delayMicroseconds(stepDelay); // Uses the stepping frequency to determine speed
    digitalWrite(STEP_PIN,LOW);
    delayMicroseconds(stepDelay);
  }
  //digitalWrite(EN_PIN,HIGH); // Disables in hardware at the end of each instruction. Limits power draw
}

void motorRight(int steps, int stepDelay)
{
 
  // Producing steps to drive stepper motor
  for(int i = 0; i<steps; i++){
    digitalWrite(Y_STEP_PIN,HIGH);
    delayMicroseconds(stepDelay); // Uses the stepping frequency to determine speed
    digitalWrite(Y_STEP_PIN,LOW);
    delayMicroseconds(stepDelay);
  }
  //digitalWrite(Y_ENABLE_PIN,HIGH); // Disables in hardware at the end of each instruction. Limits power draw
}
