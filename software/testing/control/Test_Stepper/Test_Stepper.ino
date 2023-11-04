#include <TMCStepper.h>
#include <AccelStepper.h>

#define enablePin 2
// #define MS1 3
// #define MS2 4
#define stepPin 9
#define dirPin 8
#define DIAG_PIN 10

#define stepperSpeed 800 // Set stepper speed in steps/sec

  //Initialize AccelStepper
  AccelStepper myStepper(1, stepPin, dirPin); //set up myStepper as stepper motor on AccelStepper (motor type 1)

#define DRIVER_ADDRESS 0b00 //TMC2209 Driver Address
#define STALL_PIN 10 //Digital Pin on Mega that the DIAG pin on TMC2209 board connects to
#define STALL_VALUE 45 //Value from [0 - 255] that determines the current at which motor stalls. MUST CALIBRATE
#define R_SENSE 0.11f //2209 driver sense resistance
#define SERIAL_PORT Serial2 //Only one serial port on Uno, but we can use Serial2 on Mega if we want. On Uno, Serial is on pins 0 (Rx), and 1 (Tx)

 //Initialize UART Control of TMC2209
  
  TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);
  SERIAL_PORT.begin(115200); 
  driver.beginSerial(115200);

  pinMode(dirPin,OUTPUT);
  pinMode(stepPin,OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);
  
//  pinMode(MS1, OUTPUT);
//  pinMode(MS2, OUTPUT);
  pinMode(STALL_PIN, INPUT);

  driver.begin();
  driver.toff(4);
//  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(16);
  driver.pwm_autoscale(true); 
  driver.shaft(true);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);

  myStepper.setMaxSpeed(1500);
  myStepper.setAcceleration(40);
}

void loop() {
  // put your main code here, to run repeatedly:

//  digitalWrite(MS1, LOW);
//  digitalWrite(MS2, LOW);

  int timer = 0;

  if((millis() - timer) < 10000)
  {
   myStepper.setSpeed(-stepperSpeed);
   myStepper.runSpeed(); 
  }

  if((millis() - timer) > 10000)
  {
   myStepper.setSpeed(stepperSpeed);
   myStepper.runSpeed();
  }

  Serial.println(driver.SG_RESULT(), DEC);
  Serial.println(digitalRead(DIAG_PIN));

  if(digitalRead(DIAG_PIN) == 1)
  {
    digitalWrite(enablePin, HIGH);
  }
}
