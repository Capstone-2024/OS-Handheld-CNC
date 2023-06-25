// For RAMPS 1.4 https://reprap.org/wiki/RAMPS_1.4#Pins
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN          -1 //PIN 2 is used

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          -1 //PIN 15 is used

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          -1 //PIN 19 is used

//extruder 1
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24

//extruder 2
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30


#define SDPOWER            -1

//ChipSelect, Hardware SS Pin on Mega, 10 for Arduino Boards, always kept as output
#define SDCS_PIN           53
#define SD_DETECT_PIN 	   -1 //currently not implemented


#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12	//ATX , awake=LOW, SLEEP=High
#define KILL_PIN           -1

#define HEATER_0_PIN	10  // Extruder Heater
#define HEATER_1_PIN	8

#define TEMP_0_PIN		13   // ANALOG NUMBERING
#define TEMP_1_PIN		14   // ANALOG NUMBERING