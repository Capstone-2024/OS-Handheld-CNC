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

        int via = 20;
        //float poses[3][via][2] = {{{90, 60}, {89.9458, 60.3247}, {89.7891, 60.6142}, {89.5469, 60.8372}, {89.2455, 60.9694}, {88.9174, 60.9966}, {88.5983, 60.9158}, {88.3227, 60.7357}, {88.1205, 60.4579}, {88.0136, 60.1646}, {88.0136, 59.8354}, {88.1205, 59.5241}, {88.3227, 59.2643}, {88.5983, 59.0842}, {88.9174, 59.0034}, {89.2455, 59.0306}, {89.5469, 59.1628}, {89.7891, 59.3858}, {89.9458, 59.6753}, {90.0, 60.0}}, {{94, 60}, {93.9458, 60.3247}, {93.7891, 60.6142}, {93.5469, 60.8372}, {93.2455, 60.9694}, {92.9174, 60.9966}, {92.5983, 60.9158}, {92.3227, 60.7357}, {92.1205, 60.4759}, {92.0136, 60.1646}, {92.0136, 59.8354}, {92.1205, 59.5241}, {92.3227, 59.2643}, {92.5983, 59.0842}, {92.9174, 59.0034}, {93.2455, 59.0306}, {93.5469, 59.1628}, {93.7891, 59.3858}, {93.9458, 59.6753}, {94, 60}}, {{94.5355, 63.5355}, {94.2315, 63.8154}, {93.9054, 64.0692}, {93.5594, 64.2953}, {93.1960, 64.4920}, {92.8175, 64.6580}, {92.4267, 64.7921}, {92.0261, 64.8936}, {91.6185, 64.9616}, {91.2066, 64.9957}, {90.7934, 64.9957}, {90.3815, 64.9616}, {89.0739, 64.8936}, {89.5733, 64.7921}, {89.1825, 64.6580}, {88.08040, 64.4920}, {88.4406, 64.2953}, {88.0946, 64.0692}, {87.7685, 63.8154}, {87.4645, 63.5355}}};

         float poses[via][2] =  {
  {85.09624999761581, 64.03274993896484},
  {85.29182205200195, 63.66828842163086},
  {85.48739414215088, 63.30382766723633},
  {85.68296623, 62.93936614990234},
  {85.87853832244873, 62.57490501403809},
  {86.07411041259766, 62.21044387817383},
  {86.26968259811402, 61.84598274230957},
  {86.46525468826295, 61.48152160644531},
  {86.66082687, 61.11706027984619},
  {86.85639896392823, 60.75259933},
  {87.05197105407714, 60.388137912750246},
  {87.24754314422607, 60.02367687225342},
  {87.44311523, 59.65921555},
  {87.63868732452393, 59.294754552841184},
  {87.89809875488281, 59.03700000047684},
  {88.31171760559081, 59.03700000047684},
  {88.72533645629883, 59.03700000047684},
  {89.13895493, 59.03700000047684},
  {89.23266372680663, 59.229834747314456},
  {89.03951187, 59.59558448791504},
  {88.84635963439942, 59.961334228515625},
  {88.65320777893066, 60.327083587646484},
  {88.46005592346191, 60.69283332824707},
  {88.26690368652343, 61.05858325958252},
  {88.07375202178955, 61.42433261871338},
  {87.88060017, 61.79008197784424},
  {87.68744831085205, 62.155831527709964},
  {87.49429626464844, 62.52158164978027},
  {87.30114440917968, 62.88733100891113},
  {87.10799255371094, 63.25308036804199},
  {86.91484031677246, 63.61883049},
  {86.72168884277343, 63.98457908630371},
  {86.33710536956787, 64.03274993896484},
  {85.92348690032959, 64.03274993896484},
  {85.50986847877502, 64.03274993896484},
  {87.36000003814698, 64.03274993896484},
  {87.55431957244873, 63.665970993041995},
  {87.74863929748535, 63.29919166564942},
  {87.94295883178711, 62.932412719726564},
  {88.13727855682373, 62.56563377380371},
  {88.33159828186035, 62.19885482788086},
  {88.52591782, 61.832075881958005},
  {88.72023773, 61.46529693603516},
  {88.91455726623535, 61.09851779937744},
  {89.1088768, 60.73173885345459},
  {89.30319671630859, 60.36495952606201},
  {89.49751625061035, 59.99818067550659},
  {89.69183578491212, 59.63140172958374},
  {89.88615570068359, 59.26462249755859},
  {90.16422996520996, 59.03700000047684},
  {90.57930488586426, 59.03700000047684},
  {90.99437942504883, 59.03700000047684},
  {91.40945434570312, 59.03700000047684},
  {91.52904968261718, 59.21404113769531},
  {91.33327255249023, 59.58004455566406},
  {91.1374958, 59.946047973632815},
  {90.94171905517578, 60.31205101013184},
  {90.74594192504883, 60.67805442810059},
  {90.55016518, 61.04405804},
  {90.35438842773438, 61.41006088256836},
  {90.15861167907715, 61.77606430053711},
  {89.96283455, 62.14206790924072},
  {89.76705780029297, 62.50807113647461},
  {89.57128105163574, 62.874074172973636},
  {89.37550392, 63.240077590942384},
  {89.17972717285156, 63.60608139038086},
  {88.98395004272462, 63.97208480834961},
  {88.60522384643555, 64.03274993896484},
  {88.19014987945556, 64.03274993896484},
  {87.77507419586182, 64.03274993896484},
  {89.57275009155273, 64.03274993896484},
  {89.76644744873047, 63.66714515686035},
  {89.96014518737793, 63.301539993286134},
  {90.15384292602539, 62.93593521118164},
  {90.34754028320313, 62.57033004760742},
  {90.54123764038086, 62.20472526550293},
  {90.73493576049805, 61.83912048339844},
  {90.92863311767579, 61.47351551055908},
  {91.12233085632325, 61.10791072845459},
  {91.31602859, 60.74230537414551},
  {91.50972595214844, 60.376700496673585},
  {91.70342407226562, 60.011095714569095},
  {91.89712142944336, 59.64549083709717},
  {92.09081879, 59.27988567352295},
  {92.35837860107422, 59.03700000047684},
  {92.77212448120117, 59.03700000047684},
  {93.18587036132813, 59.03700000047684},
  {93.59961624145508, 59.03700000047684},
  {93.71578826904297, 59.21556434631348},
  {93.52112121582032, 59.580653762817384},
  {93.32645416259766, 59.945743560791016},
  {93.13178787231445, 60.31083335876465},
  {92.93712082, 60.67592277526855},
  {92.74245376586914, 61.041012382507326},
  {92.54778671264648, 61.40610198974609},
  {92.35311965942383, 61.771192359924314},
  {92.15845260620117, 62.136281967163086},
  {91.96378555297852, 62.50137176513672},
  {91.76911849975586, 62.866461181640624},
  {91.57445220947265, 63.231550979614255},
  {91.37978477478028, 63.59664039611816},
  {91.18511772155762, 63.96173095703125},
  {90.81398849487304, 64.03274993896484},
  {90.40024185180664, 64.03274993896484},
  {89.98649673461914, 64.03274993896484},
  {94.44100036621094, 59.03700000047684},
  {94.84710845947265, 59.03700000047684},
  {95.25321578979492, 59.03700000047684},
  {95.65932312, 59.03700000047684},
  {96.06543121, 59.03700000047684},
  {96.47153854370117, 59.03700000047684},
  {96.87764587402344, 59.03700000047684},
  {97.28375396728515, 59.03700000047684},
  {97.68986129760742, 59.03700000047684},
  {98.09596863, 59.03700000047684},
  {98.50207672, 59.03700000047684},
  {98.90818481445312, 59.03700000047684},
  {99.31429138183594, 59.03700000047684},
  {99.41130371, 59.22281341552734},
  {99.22080078, 59.58146605491638},
  {99.03029633, 59.940119075775144},
  {98.83842773, 60.296500492095944},
  {98.43232116699218, 60.29650039672852},
  {98.02621154785156, 60.29650039672852},
  {97.62010498046875, 60.29650039672852},
  {97.21399688720703, 60.296500301361085},
  {96.80788955688476, 60.296500301361085},
  {96.40178223, 60.29650020599365},
  {95.99567413330078, 60.29650020599365},
  {95.58956680297851, 60.29650020599365},
  {95.18345947265625, 60.29650011062622},
  {94.77735137939453, 60.29650011062622},
  {94.37124404907226, 60.29650011062622},
  {93.96513672, 60.296500015258786},
  {93.86716232299804, 60.11171875},
  {94.05844192504883, 59.75347933769226},
  {94.24972076416016, 59.39523935317993},
  {93.42799988, 60.91774997711182},
  {93.84493408203124, 60.91774997711182},
  {94.26186828613281, 60.91774997711182},
  {94.67880249023438, 60.91774997711182},
  {95.09573669433594, 60.91774997711182},
  {95.51267013549804, 60.91774997711182},
  {95.92960434, 60.91774997711182},
  {96.34653854370117, 60.91774997711182},
  {96.76347274780274, 60.91774997711182},
  {97.18040695, 60.91774997711182},
  {97.59734039, 60.91774997711182},
  {98.01427459716797, 60.91774997711182},
  {98.43120880126953, 60.91774997711182},
  {98.35850524902344, 61.207655334472655},
  {98.15801391601562, 61.57322006225586},
  {97.95752410888672, 61.938784980773924},
  {97.66179733276367, 62.143500328063965},
  {97.24486313, 62.143500328063965},
  {96.82792892456055, 62.143500328063965},
  {96.41099472045899, 62.143500328063965},
  {95.99406127929687, 62.14350014},
  {95.57712708, 62.14350014},
  {95.16019287109376, 62.14350014},
  {94.74325866699219, 62.14350014},
  {94.32632446289062, 62.14350014},
  {93.90939025878906, 62.14350014},
  {93.49245605, 62.14349994659424},
  {93.07552185058594, 62.14349994659424},
  {92.85032577514649, 62.02716426849365},
  {93.04288406, 61.657359504699706},
  {93.23544158935547, 61.28755474090576},
  {92.43249969482422, 62.773249816894534},
  {92.83517074584961, 62.773249816894534},
  {93.2378418, 62.773249816894534},
  {93.64051208496093, 62.773249816894534},
  {94.04318389892578, 62.773249816894534},
  {94.44585418701172, 62.773249816894534},
  {94.84852523803711, 62.773249816894534},
  {95.25119629, 62.773249816894534},
  {95.65386734008788, 62.773249816894534},
  {96.05653839111328, 62.773249816894534},
  {96.45920944213867, 62.773249816894534},
  {96.86188049316407, 62.773249816894534},
  {97.26455078, 62.773249816894534},
  {97.44173812866211, 62.9096077},
  {97.25471801757813, 63.26621284484863},
  {97.06769790649415, 63.62281837463379},
  {96.88067779541015, 63.979423904418944},
  {96.48455352783203, 63.99025039672851},
  {96.08188247680664, 63.99025039672851},
  {95.67921142578125, 63.99025039672851},
  {95.27654037475585, 63.99025039672851},
  {94.87387008666992, 63.99025039672851},
  {94.47119903564453, 63.99025039672851},
  {94.06852798461914, 63.99025039672851},
  {93.66585693359374, 63.99025001525879},
  {93.26318588256837, 63.99025001525879},
  {92.86051483154297, 63.99025001525879},
  {92.45784378051758, 63.99025001525879},
  {92.05517349243163, 63.99025001525879},
  {91.88308563232422, 63.84909362792969},
  {92.06622391, 63.49047927856445},
  {92.24936141967774, 63.13186454772949}
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
