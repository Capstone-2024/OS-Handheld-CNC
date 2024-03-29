#include <TMCStepper.h>

// MUST TURN ON POWER SUPPLY POWER FIRST BEFORE UPLOADING CODE / CONNECTING SERIAL PIN //
// MAKE SURE TO SET SERIAL MONITOR BAUD RATE TO 9600 //

#define STEP_PIN         54
//#define X_DIR_PIN          55
#define EN_PIN       38
//#define X_MIN_PIN           3
//#define X_MAX_PIN          -1 //PIN 2 is used

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
#define SERIAL_PORT Serial2 // Hardware Serial for TMC2209 #1 in Arduino
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2 - BigTreeTech default driver address. Dont change
#define DRIVER_ADDRESS2 0b01 // Second TMC2209 Driver - in Y position. USE A JUMPER ON MS1 TO GET 0B01 ADDRESS
#define R_SENSE 0.11f // Specific sense resistance

// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater than TSTEP & TPWMTHRS
#define STALL_VALUE 100 // [0..255] // Need to calibrate
int stepTime = 160; // Determines speed of stepper. 160Hz step frequency
bool startup = false; // set false after homing

// Initializing stepper driver in UART
  TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
  TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS2);

// Variables
bool shaftVal = false;
bool stalled_X = false;

// Interrupt for when stall is detected
void stallInterruptX(){ // flag set when motor stalls
  stalled_X = true;
}


void setup() {
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  // shaft direction controlled through uart: driver.shaft(true or false)
  pinMode(STALL_PIN_X, INPUT);

  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);

  Serial.begin(9600); // Arduino USB serial for serial monitor
  SERIAL_PORT.begin(115200); // HW UART drivers

  driver.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver.toff(4); // Enables driver in software, changed from 5
  driver.blank_time(24);
  driver.rms_current(800); // Set motor RMS current
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
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), stallInterruptX, RISING); // Interrupt on diag pin @ pin 21. Pin 21 can no longer be used for I2C
  digitalWrite(EN_PIN, LOW); // Enable driver in hardware

  driver2.begin(); // SPI: Init CS pins and possible SW SPI pins
  driver2.toff(4); // Enables driver in software, changed from 5
  driver2.blank_time(24);
  driver2.rms_current(800); // Set motor RMS current
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
}

void loop() {
  
  if(startup){ // home on starting up
    startup = false;
    homeX();
  }

  // If the motor stalls during instruction
  if(stalled_X){
    int backSteps = 1000;
    Serial.println("stalled");
    stalled_X = false;
    shaftVal =! shaftVal; // Turns the stepper in other direction
    motor(backSteps,160);
  }


  // Type x followed by an integer to drive the stepper motor
  if(Serial.available()>0){
  char readVal = Serial.read();
  if (readVal == 'x'){
  int steps = Serial.parseInt();
  
    // Determine drive direction
    if(steps < 0){
      shaftVal = true;
      steps *= -1;
    }
    else{ 
      shaftVal = false; 
    }
    
      motor(steps,160); // Drives stepper
      Serial.println(driver.SG_RESULT(),DEC); //Feedback from stepper motor on the torque 
    }

  // Re-home the motor to stalled position with h characters
  else if (readVal == 'h'){
    homeX();
  }
  
  }
}

void homeX(){
  int homeDelay = 160; // Determines stepping frequency
  int backSteps = 5000; // Steps back from stalled position

  // Fast homing first to get approximate position
  Serial.println("fast homing x");
  shaftVal = true; // direction
  stalled_X = false;
  while(!stalled_X){ // fast home x
    motor(1000,homeDelay);
  }
  stalled_X = false; // Once stalled_X = true, breaks out of while loop and goes false
  delay(1000);
  Serial.println("backing off");
  shaftVal = false; // backs off of stalled position
  motor(backSteps,homeDelay);

  Serial.println("slow homing x");
  shaftVal = true;
  while(!stalled_X){ // slow home x
    motor(1000,homeDelay*2);
  }
  
  stalled_X = false;
  delay(1000);
  Serial.println("backing off");
  shaftVal = false;
  motor(backSteps, homeDelay);
}

void motor(int steps, int stepDelay)
{
  digitalWrite(EN_PIN, LOW); // enables stepper in hardware
  driver.shaft(shaftVal); // determines direction of rotation

  digitalWrite(Y_ENABLE_PIN, LOW); // enables stepper in hardware
  driver2.shaft(shaftVal); // determines direction of rotation

  // Producing steps to drive stepper motor
  for(int i = 0; i<steps; i++){
    digitalWrite(STEP_PIN,HIGH);
    digitalWrite(Y_STEP_PIN,HIGH);
    delayMicroseconds(stepDelay); // Uses the stepping frequency to determine speed
    digitalWrite(STEP_PIN,LOW);
    digitalWrite(Y_STEP_PIN,LOW);
    delayMicroseconds(stepDelay);
  
    if(stalled_X){  // If stall interrup gets triggered, stop stepping
      i = steps;
    }
  }
  digitalWrite(EN_PIN,HIGH); // Disables in hardware at the end of each instruction. Limits power draw
  digitalWrite(Y_ENABLE_PIN,HIGH); // Disables in hardware at the end of each instruction. Limits power draw
}
