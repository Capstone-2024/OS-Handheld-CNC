#include <TMCStepper.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define PIN        1 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 16 // Popular NeoPixel ring size
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

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

#define Z_STEP_PIN         46
//#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
//#define Z_MIN_PIN          18
//#define Z_MAX_PIN          -1 //PIN 19 is used

#define LeftbuttonPin 18  // left hand button 
#define RightbuttonPin 19 // Right hand button

#define STALL_PIN_Z 23 // Arduino pin that diag pin is attached to, reads when stall occurs
#define SERIAL_PORT Serial2 // Hardware Serial for TMC2209 #1 and TMC2209#2 in Arduino
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2 - BigTreeTech default driver address. Dont change
#define DRIVER_ADDRESS2 0b01 // Second TMC2209 Driver - in Y position. USE A JUMPER ON MS1 TO GET 0B01 ADDRESS
#define DRIVER_ADDRESS3 0b10 // Third TMC2209 Driver - Z axis
#define R_SENSE 0.11f // Specific sense resistance

#define DEG_TO_RAD 0.017453
#define RAD_TO_DEG 57.2957795
#define PI 3.14159265358979323846264338327950
#define MICROSTEP 1/16

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 75 // [0..255] // Need to calibrate
int stepTime = 160; // Determines speed of stepper. 160Hz step frequency
bool startup = false; // set false after homing

// Initializing stepper driver in UART
  TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
  TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);
  TMC2209Stepper driver3(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS3);

// Variables
bool shaftVal = true;
bool stalled_X = false;
bool zShaftVal = false;

float Nema17Resolution = 0.9;
const float l1 = 7.5;
const float l2 = 110.0;
const float l3 = 100.0;
const float l4 = 7.5;
const float l5 = 170.0;

const float penOriginX = 84.077;
const float penOriginY = 104.673; 

float currentPosX;
float currentPosY;

float currentTheta1;
float currentTheta4;

float penPoints[400][3];

const float homedTheta1 = 130.44;
const float homedTheta4 = 128.69;

int cycle = 1;

void leftInterrupt() {
  int breakTime = millis();
  while(((digitalRead(LeftbuttonPin)||digitalRead(RightbuttonPin))) != 0)
  {
    Serial.println("Press both buttons!!!!");
    Serial.print("Left");
    Serial.println(digitalRead(LeftbuttonPin));
    Serial.print("Right");
    Serial.println(digitalRead(RightbuttonPin));
    delay(1000);
    if((millis() - breakTime) >= 5000)
    {
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(Y_ENABLE_PIN, HIGH);
    }
  }
}

void rightInterrupt() {
  int breakTime = millis();
  while(((digitalRead(LeftbuttonPin)||digitalRead(RightbuttonPin))) != 0)
  {
    Serial.println("Press both buttons!!!!");
    Serial.print("Left");
    Serial.println(digitalRead(LeftbuttonPin));
    Serial.print("Right");
    Serial.println(digitalRead(RightbuttonPin));
    delay(1000);
    if((millis() - breakTime) >= 5000)
    {
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(Y_ENABLE_PIN, HIGH);
    }
  }
}

void setup() {

  Serial.begin(11520);
  
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

  Serial.begin(9600); // Arduino USB serial for serial monitor
  SERIAL_PORT.begin(9600); // HW UART drivers

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

  // Pen Origin w.r.t center of Actuator 1 - From CAD
  currentPosX = 84.077;
  currentPosY = 104.673;

  currentTheta1 = homedTheta1;
  currentTheta4 = homedTheta4;

  // Compute Data Points for Possible Pen Locations

  int count = 0;

  for(int i = 0; i<20; i++)
  {
    for(int j = 0 ; j<20; j++)
    {

      float theta1 = DEG_TO_RAD*36*i;
      float theta4 = DEG_TO_RAD*36*j;

      float xb = l1*cos(theta1);
      float yb = l1*sin(theta1);

      float xd = l5 + l4*cos(theta4);
      float yd = l4*sin(theta4);

      float A = 2*l3*l4*sin(theta4) - 2*l1*l3*sin(theta1);
      float B = 2*l3*l5 - 2*l1*l3*cos(theta1) + 2*l3*l4*cos(theta4);
      float C = l1*l1 - l2*l2 + l3*l3 + l4*l4 + l5*l5 - 2*l1*l4*sin(theta1) - 2*l1*l5*cos(theta1) + 2*l4*l5*cos(theta4) - 2*l1*l4*cos(theta1)+cos(theta4);

      float theta3 = 2*atan((A+sqrt(A*A+B*B-C*C))/(B-C));

      float theta2 = asin((l3*sin(theta3) + l4*sin(theta4) - l1*sin(theta1)) / l2);

      float xc = l1*cos(theta1) + l2*cos(theta2);
      float yc = l1*sin(theta1) + l2*sin(theta2);

      float thetaPen = asin((xc-xb)/l2) + DEG_TO_RAD*(119.70);
      float thetaPenInv = PI - thetaPen;
      penPoints[count][0] = xc - 45.011*sin(thetaPenInv);
      penPoints[count][1] = yc + 45.011*cos(thetaPenInv);
      penPoints[count][2] = thetaPenInv;

      count++;
    }
  }  

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
  //driver.TCOOLTHRS(0xFFFFF); // 20bit max
  //driver.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
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
  //driver2.TCOOLTHRS(0xFFFFF); // 20bit max
  //driver2.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
  //attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING); // Interrupt on diag pin @ pin 21. Pin 21 can no longer be used for I2C
  digitalWrite(Y_ENABLE_PIN, LOW); // Enable driver in hardware

  driver3.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver3.toff(4); // Enables driver in software, changed from 5
  driver3.blank_time(24);
  driver3.rms_current(1000); // Set motor RMS current
  driver3.microsteps(8); // Set microsteps to 1/16th

  driver3.pwm_autoscale(true); // Needed for stealthChop
  driver3.semin(5);
  driver3.semax(2);
  driver3.sedn(0b01);
  driver3.shaft(zShaftVal); // direction of rotation
  // TCOOLTHRS needs to be set for stallgaurd to work //
  driver3.TCOOLTHRS(0xFFFFF); // 20bit max
  driver3.SGTHRS(STALL_VALUE); // Write the stall value to the driver board
  //attachInterrupt(digitalPinToInterrupt(STALL_PIN_Z), stallInterruptZ, RISING);
  digitalWrite(Z_ENABLE_PIN, LOW); // Enable driver in hardware

  pinMode(RightbuttonPin,INPUT_PULLUP);
  pinMode(LeftbuttonPin,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LeftbuttonPin), leftInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RightbuttonPin), rightInterrupt, RISING);

  int incomingByte = 0; 
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    if (incomingByte == 1) {
      Serial.write("HI");
      homingSequence();
    }
  }
  zHomingSequence();
  
  //homingSequence();

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
    desiredDeltaX = 10;
    desiredDeltaY = 0;
  }

  else if(cycle%2 == 0)
  {
    desiredDeltaX = -10;
    desiredDeltaY = 0;
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

//  Serial.print("Curr Theta 1: ");
//  Serial.println(currentTheta1);
//  Serial.print("Curr Theta 4: ");
//  Serial.println(currentTheta4);  

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
    driver2.shaft(true);
  }
  else
  {
    driver2.shaft(false);
  }

  desiredSteps[0] = abs(desiredSteps[0]);
  desiredSteps[1] = abs(desiredSteps[1]);

  // Moving in Straight Line
  int stepRatio = max(desiredSteps[0], desiredSteps[1]) / min(desiredSteps[0], desiredSteps[1]);
  int stepRemainder = max(desiredSteps[0], desiredSteps[1]) % min(desiredSteps[0], desiredSteps[1]);
  int loopIterations = min(desiredSteps[0], desiredSteps[1]);
  int greater = (desiredSteps[0] > desiredSteps[1]) ? 1 : 0; //if condition is true, set to 1, else set to 0.
  
//  Serial.print("Step Ratio: ");
//  Serial.println(stepRatio);
//  Serial.print("Step Remainder: ");
//  Serial.println(stepRemainder);
//  Serial.print("Loop Iterations: ");
//  Serial.println(loopIterations);

  int randomArray[loopIterations];

  remainderArray(loopIterations, stepRemainder, randomArray);

  fineTuning(stepRatio, loopIterations, randomArray, greater);

//  Serial.print("Theta 1: ");
//  Serial.println(currentTheta1);q
//  Serial.print("Theta 4: ");
//  Serial.println(currentTheta4);
}

void InvKin(float desiredDeltaX, float desiredDeltaY, float currentPosX, float currentPosY, float desiredJointAngles[])
{
  
  float x = currentPosX + desiredDeltaX;
  float y = currentPosY + desiredDeltaY;

  Serial.print("Pen X: ");
  Serial.println(x);
  Serial.print("Pen Y: ");
  Serial.println(y);

  float minDiff = 1000;
  int minIndex = 0;

  for(int i = 0; i<400; i++)
  {
    float xDiff = abs(x - penPoints[i][0]);
    float yDiff = abs(y - penPoints[i][1]);
    float totDiff = xDiff + yDiff;

    if(totDiff < minDiff)
    {
      minDiff = totDiff;
      minIndex = i;
    }
  }

  float thetaPenCurr = penPoints[minIndex][2];

  Serial.print("Theta Pen: ");
  Serial.println(RAD_TO_DEG*thetaPenCurr);

  x = x + 45.011*sin(thetaPenCurr);
  y = y - 45.011*cos(thetaPenCurr);

  Serial.print("End Effector X: ");
  Serial.println(x);
  Serial.print("End Effector Y: ");
  Serial.println(y);

  float w = l5;

  float c1 = sqrt(x*x+y*y);
  float c2 = sqrt(c1*c1-2*x*w+w*w);

  float alpha1 = acos(x/c1);
  float alpha2 = acos((-x+w)/c2);

  float beta1 = acos((l2*l2-l1*l1-c1*c1)/(-2*l1*c1));
  float beta2 = acos((l3*l3-l4*l4-c2*c2)/(-2*l4*c2));

  // For manipulator of working mode 1
  float theta1 = RAD_TO_DEG*(alpha1+beta1);
  float theta4 = RAD_TO_DEG*(alpha2+beta2);

  Serial.print("T1: ");
  Serial.println(theta1);
  Serial.print("T4: ");
  Serial.println(theta4);

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
    motorLeft(stepRatio + randomArray[i], 240);
    motorRight(1, 240);
    }
    else
    {
    motorLeft(1, 240);
    motorRight(stepRatio + randomArray[i], 240);
    }
  }
}

void startupSequence()
{
  while(((digitalRead(LeftbuttonPin)||digitalRead(RightbuttonPin))) != 0)
  {
    Serial.println("Press both buttons!!!!");
    Serial.print("Left");
    Serial.println(digitalRead(LeftbuttonPin));
    Serial.print("Right");
    Serial.println(digitalRead(RightbuttonPin));
    delay(1000);
  }
}

void homingSequence()
{
    shaftVal = true;
    int flagLeft = 0;
    int flagRight = 0;


    while(flagLeft*flagRight != 1)
    {
      if(flagLeft != 1)
      {
      motorLeft(1,240);
      }

      if(flagRight != 1)
      {
      motorRight(1,240);
      }
      
    if(digitalRead(X_MAX_PIN)==0)
    {
      flagRight = 1;
    }
    if(digitalRead(X_MIN_PIN)==0)
    {
      flagLeft = 1;
    }
    }


    pixels.clear();
    for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(0, 150, 0));

    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop
    }
    pixels.clear();
    
    delay(2000);
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

void motorVert(int steps, int stepDelay)
{
  driver3.shaft(zShaftVal);
 // Producing steps to drive stepper motor
  for(int i = 0; i<steps; i++){
    digitalWrite(Z_STEP_PIN,HIGH);
    delayMicroseconds(stepDelay); // Uses the stepping frequency to determine speed
    digitalWrite(Z_STEP_PIN,LOW);
    delayMicroseconds(stepDelay);
  } 
}

void zHomingSequence()
{
  digitalWrite(Z_ENABLE_PIN, LOW);
  while (digitalRead(STALL_PIN_Z) == 0)
  {
  motorVert(100, 50);
  }
  digitalWrite(Z_ENABLE_PIN, HIGH);
  delay(1000);
  digitalWrite(Z_ENABLE_PIN, LOW);

  shaftVal =! shaftVal;
  driver3.shaft(shaftVal);

  for(int i = 0; i<5000; i++){
    digitalWrite(Z_STEP_PIN,HIGH);
    delayMicroseconds(50); // Uses the stepping frequency to determine speed
    digitalWrite(Z_STEP_PIN,LOW);
    delayMicroseconds(50);
  }
}
