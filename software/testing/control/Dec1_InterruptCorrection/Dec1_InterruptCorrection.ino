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
#define STALL_VALUE 41 // [0..255] // Need to calibrate
int stepTime = 400;    // Determines speed of stepper. 160Hz step frequency
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

int completed = 0;

volatile bool buttonState = false;
volatile int buttonChangeTime;

int zPos = 3000;

// Serial Vars
bool homedSent = false;
bool zHomedSent = false;

void buttonInterrupt()
{
  // Update State
  buttonState = !(digitalRead(LeftbuttonPin) || digitalRead(RightbuttonPin));
  buttonChangeTime = millis();
  delayMicroseconds(20);
}

void setup()
{
  // Set up motor driver pins
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
    pixels.setPixelColor(i, pixels.Color(100, 100, 100));
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
  driver.rms_current(1200); // Set motor RMS current
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
  driver2.rms_current(1200); // Set motor RMS current
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

  // Setup button pins
  pinMode(RightbuttonPin, INPUT_PULLUP);
  pinMode(LeftbuttonPin, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(LeftbuttonPin), buttonInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightbuttonPin), buttonInterrupt, CHANGE);

  // zHomingSequence();

  // homingSequence();
}

void loop()
{
  // Check Button State
  if (buttonState)
  {
    // Send Y for to Indicate Button Press
    uint16_t sendSize = 0;
    char data = 'Y';
    sendSize = myTransfer.txObj(data, sendSize);

    // If command is sent from Orange Pi
    if (myTransfer.available())
    {
      // Re-enable motors
      digitalWrite(EN_PIN, LOW);       // Enable driver in hardware
      digitalWrite(Y_ENABLE_PIN, LOW); // Enable driver in hardware
      // Determine State
      int state = int(myTransfer.packet.rxBuff[0]);

      // Correcting Based on Error Vector
      if (state == 73)
      {
        if (zPos != 0)
        {
          digitalWrite(Z_ENABLE_PIN, LOW);
          // Move Down to drawing/cutting position
          zShaftVal = false;
          motorVert(zPos, 500);
          zPos = 0; // Reset Z Pos to bottom pos
        }

        uint16_t recSize = 1; // Start after the character byte

        float xPacket;
        float yPacket;

        recSize = myTransfer.rxObj(xPacket, recSize);
        recSize = myTransfer.rxObj(yPacket, recSize);

        autoCorrection(xPacket, yPacket);
      }

      // Drawing Smiley
      else if (state == 83)
      {
        // homingSequence(0);
        // zHomingSequence(0);
        if (zPos != 0)
        {
          digitalWrite(Z_ENABLE_PIN, LOW);
          // Move Down to drawing/cutting position
          zShaftVal = false;
          motorVert(zPos, 500);
          zPos = 0; // Reset Z Pos to bottom pos
        }

        int via = 200;
        //float poses[3][via][2] = {{{90, 60}, {89.9458, 60.3247}, {89.7891, 60.6142}, {89.5469, 60.8372}, {89.2455, 60.9694}, {88.9174, 60.9966}, {88.5983, 60.9158}, {88.3227, 60.7357}, {88.1205, 60.4579}, {88.0136, 60.1646}, {88.0136, 59.8354}, {88.1205, 59.5241}, {88.3227, 59.2643}, {88.5983, 59.0842}, {88.9174, 59.0034}, {89.2455, 59.0306}, {89.5469, 59.1628}, {89.7891, 59.3858}, {89.9458, 59.6753}, {90.0, 60.0}}, {{94, 60}, {93.9458, 60.3247}, {93.7891, 60.6142}, {93.5469, 60.8372}, {93.2455, 60.9694}, {92.9174, 60.9966}, {92.5983, 60.9158}, {92.3227, 60.7357}, {92.1205, 60.4759}, {92.0136, 60.1646}, {92.0136, 59.8354}, {92.1205, 59.5241}, {92.3227, 59.2643}, {92.5983, 59.0842}, {92.9174, 59.0034}, {93.2455, 59.0306}, {93.5469, 59.1628}, {93.7891, 59.3858}, {93.9458, 59.6753}, {94, 60}}, {{94.5355, 63.5355}, {94.2315, 63.8154}, {93.9054, 64.0692}, {93.5594, 64.2953}, {93.1960, 64.4920}, {92.8175, 64.6580}, {92.4267, 64.7921}, {92.0261, 64.8936}, {91.6185, 64.9616}, {91.2066, 64.9957}, {90.7934, 64.9957}, {90.3815, 64.9616}, {89.0739, 64.8936}, {89.5733, 64.7921}, {89.1825, 64.6580}, {88.08040, 64.4920}, {88.4406, 64.2953}, {88.0946, 64.0692}, {87.7685, 63.8154}, {87.4645, 63.5355}}};

         float poses[via][2] =  {
    {88.7738, 55.2770},
    {89.0654, 55.4335},
    {89.3570, 55.5899},
    {89.6485, 55.7464},
    {89.9401, 55.9028},
    {90.2316, 56.0593},
    {90.5232, 56.2157},
    {90.8148, 56.3722},
    {91.1064, 56.5287},
    {91.3979, 56.6851},
    {91.6895, 56.8416},
    {91.9811, 56.9980},
    {92.2726, 57.1545},
    {92.5642, 57.3109},
    {92.7704, 57.5185},
    {92.7704, 57.8494},
    {92.7704, 58.1803},
    {92.7704, 58.5112},
    {92.6161, 58.5861},
    {92.3235, 58.4316},
    {92.0309, 58.2771},
    {91.7383, 58.1226},
    {91.4457, 57.9680},
    {91.1531, 57.8135},
    {90.8605, 57.6590},
    {90.5679, 57.5045},
    {90.2753, 57.3500},
    {89.9827, 57.1954},
    {89.6901, 57.0409},
    {89.3975, 56.8864},
    {89.1049, 56.7319},
    {88.8123, 56.5774},
    {88.7738, 56.2697},
    {88.7738, 55.9388},
    {88.7738, 55.6079},
    {88.7738, 57.0880},
    {89.0672, 57.2435},
    {89.3606, 57.3989},
    {89.6541, 57.5544},
    {89.9475, 57.7098},
    {90.2409, 57.8653},
    {90.5343, 58.0207},
    {90.8278, 58.1762},
    {91.1212, 58.3316},
    {91.4146, 58.4871},
    {91.7080, 58.6426},
    {92.0015, 58.7980},
    {92.2949, 58.9535},
    {92.5883, 59.1089},
    {92.7704, 59.3314},
    {92.7704, 59.6634},
    {92.7704, 59.9955},
    {92.7704, 60.3276},
    {92.6288, 60.4232},
    {92.3360, 60.2666},
    {92.0432, 60.1100},
    {91.7504, 59.9534},
    {91.4576, 59.7968},
    {91.1648, 59.6401},
    {90.8720, 59.4835},
    {90.5791, 59.3269},
    {90.2863, 59.1703},
    {89.9935, 59.0136},
    {89.7007, 58.8570},
    {89.4079, 58.7004},
    {89.1151, 58.5438},
    {88.8223, 58.3872},
    {88.7738, 58.0842},
    {88.7738, 57.7521},
    {88.7738, 57.4201},
    {88.7738, 58.8582},
    {89.0663, 59.0132},
    {89.3588, 59.1681},
    {89.6513, 59.3231},
    {89.9437, 59.4780},
    {90.2362, 59.6330},
    {90.5287, 59.7880},
    {90.8212, 59.9429},
    {91.1137, 60.0979},
    {91.4062, 60.2528},
    {91.6986, 60.4078},
    {91.9911, 60.5627},
    {92.2836, 60.7177},
    {92.5761, 60.8727},
    {92.7704, 61.0867},
    {92.7704, 61.4177},
    {92.7704, 61.7487},
    {92.7704, 62.0797},
    {92.6275, 62.1726},
    {92.3355, 62.0169},
    {92.0434, 61.8612},
    {91.7513, 61.7054},
    {91.4593, 61.5497},
    {91.1672, 61.3940},
    {90.8751, 61.2382},
    {90.5830, 61.0825},
    {90.2909, 60.9268},
    {89.9989, 60.7710},
    {89.7068, 60.6153},
    {89.4148, 60.4596},
    {89.1227, 60.3038},
    {88.8306, 60.1481},
    {88.7738, 59.8512},
    {88.7738, 59.5202},
    {88.7738, 59.1892},
    {92.7704, 62.7528},
    {92.7704, 63.0777},
    {92.7704, 63.4026},
    {92.7704, 63.7275},
    {92.7704, 64.0523},
    {92.7704, 64.3772},
    {92.7704, 64.7021},
    {92.7704, 65.0270},
    {92.7704, 65.3519},
    {92.7704, 65.6768},
    {92.7704, 66.0017},
    {92.7704, 66.3265},
    {92.7704, 66.6514},
    {92.6217, 66.7290},
    {92.3348, 66.5766},
    {92.0479, 66.4242},
    {91.7628, 66.2707},
    {91.7628, 65.9459},
    {91.7628, 65.6210},
    {91.7628, 65.2961},
    {91.7628, 64.9712},
    {91.7628, 64.6463},
    {91.7628, 64.3214},
    {91.7628, 63.9965},
    {91.7628, 63.6717},
    {91.7628, 63.3468},
    {91.7628, 63.0219},
    {91.7628, 62.6970},
    {91.7628, 62.3721},
    {91.9106, 62.2937},
    {92.1972, 62.4468},
    {92.4838, 62.5998},
    {91.2658, 61.9424},
    {91.2658, 62.2760},
    {91.2658, 62.6095},
    {91.2658, 62.9430},
    {91.2658, 63.2766},
    {91.2658, 63.6101},
    {91.2658, 63.9437},
    {91.2658, 64.2772},
    {91.2658, 64.6108},
    {91.2658, 64.9443},
    {91.2658, 65.2779},
    {91.2658, 65.6114},
    {91.2658, 65.9450},
    {91.0339, 65.8868},
    {90.7414, 65.7264},
    {90.4490, 65.5660},
    {90.2852, 65.3294},
    {90.2852, 64.9959},
    {90.2852, 64.6623},
    {90.2852, 64.3288},
    {90.2852, 63.9952},
    {90.2852, 63.6617},
    {90.2852, 63.3282},
    {90.2852, 62.9946},
    {90.2852, 62.6611},
    {90.2852, 62.3275},
    {90.2852, 61.9939},
    {90.2852, 61.6604},
    {90.3783, 61.4803},
    {90.6741, 61.6343},
    {90.9700, 61.7884},
    {89.7814, 61.1460},
    {89.7814, 61.4681},
    {89.7814, 61.7903},
    {89.7814, 62.1124},
    {89.7814, 62.4345},
    {89.7814, 62.7567},
    {89.7814, 63.0788},
    {89.7814, 63.4010},
    {89.7814, 63.7231},
    {89.7814, 64.0452},
    {89.7814, 64.3674},
    {89.7814, 64.6895},
    {89.7814, 65.0116},
    {89.6723, 65.1534},
    {89.3870, 65.0038},
    {89.1017, 64.8542},
    {88.8165, 64.7045},
    {88.8078, 64.3876},
    {88.8078, 64.0655},
    {88.8078, 63.7434},
    {88.8078, 63.4212},
    {88.8078, 63.0991},
    {88.8078, 62.7770},
    {88.8078, 62.4548},
    {88.8078, 62.1327},
    {88.8078, 61.8105},
    {88.8078, 61.4884},
    {88.8078, 61.1663},
    {88.8078, 60.8441},
    {88.9207, 60.7065},
    {89.2076, 60.8530},
    {89.4945, 60.9995}
         };
        //for (int j = 0; j < 3; j++)
        //{
          digitalWrite(Z_ENABLE_PIN, LOW);
          zShaftVal = true;
          motorVert(500, 400);
          delay(1000);

          for (int i = 0; i < via; i++)
          {
            //float pos_x = poses[j][i][0] - penOriginX;
            //float pos_y = poses[j][i][1] - penOriginY;
            float pos_x = poses[i][0] - penOriginX;
            float pos_y = poses[i][1] - penOriginY;
            autoCorrection(pos_x, pos_y);
            if (i == 0)
            {
              zShaftVal = false;
              motorVert(500, 400);
              delay(1000);
            }
          }
        //}
      }
      // else if (state == 68) { GCODE MODE???
      //   float pos_x = poses[j][i][0] - penOriginX;
      //   float pos_y = poses[j][i][1] - penOriginY;
      //   autoCorrection(pos_x, pos_y);
      // }

      // Irrespective of Z position states
      // XY Homing
      else if (state == 72)
      {
        homingSequence(sendSize);
      }

      // Retract Z testing
      else if (state == 83)
      {
        zRetract(2000);
      }

      // Z Homing
      else if (state == 90)
      {
        zHomingSequence(sendSize);
        zPos = 0;
      }

      // Send Accel Data
      else if (state == 65)
      {
        sampleAccelerometer(sendSize);
      }
    }

    // Turn motor on again after shutting down
    digitalWrite(EN_PIN, LOW);       // Enable driver in hardware
    digitalWrite(Y_ENABLE_PIN, LOW); // Enable driver in hardware
    digitalWrite(Z_ENABLE_PIN, LOW); // Enable driver in hardware
  }
  else // If Button is not pressed
  {
    uint16_t sendSize = 0;
    char data = 'N';
    sendSize = myTransfer.txObj(data, sendSize);
    myTransfer.sendData(sendSize);

    // Shutdown motors after 1 min
    if ((millis() - buttonChangeTime) >= 60000)
    {
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(Y_ENABLE_PIN, HIGH);
      digitalWrite(Z_ENABLE_PIN, HIGH);
    }

    // Go up 1000 steps
    if (zPos == 0)
    {
      zRetract(3000);
      zPos = 3000;
    }

    delay(200);
  }
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
    theta1 = currentTheta1;
    theta4 = currentTheta4;
  }
  if (isnan(theta4))
  {
    theta1 = currentTheta1;
    theta4 = currentTheta4;
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
    // while (randomArray[randomIndex] != 0)
    // {
    //   randomIndex = random(0, arraySize);
    // }
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

    if (Serial.available() == 9)
    {
      // Update position w.r.t how far we actually travelled.

      if (leftShaftVal == true)
      {
        currentTheta1 = currentTheta1 + StepsToDeg(leftStepsTaken);
      }
      else
      {
        currentTheta1 = currentTheta1 - StepsToDeg(leftStepsTaken);
      }

      if (rightShaftVal == true)
      {
        currentTheta4 = currentTheta4 + StepsToDeg(rightStepsTaken);
      }
      else
      {
        currentTheta4 = currentTheta4 - StepsToDeg(rightStepsTaken);
      }

      forwardKin(currentTheta1, currentTheta4);

      break;
    }
  }
  // Update Pos and Angles if we reach target

  if (leftShaftVal == true)
  {
    currentTheta1 = currentTheta1 + StepsToDeg(leftStepsTaken);
  }
  else
  {
    currentTheta1 = currentTheta1 - StepsToDeg(leftStepsTaken);
  }

  if (rightShaftVal == true)
  {
    currentTheta4 = currentTheta4 + StepsToDeg(rightStepsTaken);
  }
  else
  {
    currentTheta4 = currentTheta4 - StepsToDeg(rightStepsTaken);
  }

  forwardKin(currentTheta1, currentTheta4);
}

void startupSequence()
{
  while (((digitalRead(LeftbuttonPin) || digitalRead(RightbuttonPin))) == 1)
  {
    // Serial.println("Press both buttons!!!!");
    // Serial.print("Left");
    // Serial.println(digitalRead(LeftbuttonPin));
    // Serial.print("Right");
    // Serial.println(digitalRead(RightbuttonPin));
    // Send 'N' when not pressed
    uint16_t sendSize = 0;
    char data = 'N';
    sendSize = myTransfer.txObj(data, sendSize);
    myTransfer.sendData(sendSize);
    delay(500);
  }

  buttonState = true;
}

void homingSequence(uint16_t sendSize)
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

  if (homedSent != true)
  {
    // uint16_t sendSize = 0;
    char data = 'G';
    sendSize = myTransfer.txObj(data, sendSize);
    myTransfer.sendData(sendSize);
    homedSent = true;
  }

  //   // uint16_t sendSize = 0;
  // char data = 'G';
  // sendSize = myTransfer.txObj(data, sendSize);
  // myTransfer.sendData(sendSize);

  currentPosX = penOriginX;
  currentPosY = penOriginY;
  currentTheta1 = homedTheta1;
  currentTheta4 = homedTheta4;

  return;
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

void zHomingSequence(uint16_t sendSize)
{
  digitalWrite(Z_ENABLE_PIN, LOW);
  // motorVert(6000, 200);
  bool operation = true;
  while (operation)
  {
    zShaftVal = false;
    motorVert(5, 750);

    if (digitalRead(STALL_PIN_Z) == HIGH)
    {
      digitalWrite(Z_ENABLE_PIN, HIGH);
      operation = false;
    }
  }

  if (zHomedSent != true)
  {
    // uint16_t sendSize = 0;
    char data = 'O';
    sendSize = myTransfer.txObj(data, sendSize);
    myTransfer.sendData(sendSize);
    zHomedSent = true;
  }

  //   // uint16_t sendSize = 0;
  // char data = 'O';
  // sendSize = myTransfer.txObj(data, sendSize);
  // myTransfer.sendData(sendSize);

  delay(500);

  // zRetract(400);
}

void zRetract(int steps)
{
  digitalWrite(Z_ENABLE_PIN, LOW);
  zShaftVal = true;
  driver3.shaft(zShaftVal);
  motorVert(steps, 500);
}

void sampleAccelerometer(uint16_t sendSize)
{
  // uint16_t sendSize = 0;

  sensors_event_t event;

  accel.getEvent(&event);

  float z_accel = event.acceleration.z;
  float y_accel = event.acceleration.y;

  sendSize = myTransfer.txObj(z_accel, sendSize);
  sendSize = myTransfer.txObj(y_accel, sendSize);

  myTransfer.sendData(sendSize);

  return;
}
