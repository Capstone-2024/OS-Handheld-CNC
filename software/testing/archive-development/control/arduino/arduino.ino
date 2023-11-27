// Import all neccessary libraries

#include "pins.h" //RAMPS 1.4 pins
#include <TMCStepper.h> 
#include <AccelStepper.h> //https://github.com/teemuatlut/TMCStepper/blob/master/examples/TMC_AccelStepper/TMC_AccelStepper.ino


// Stepper Stallguard Setup: https://github.com/teemuatlut/TMCStepper/blob/master/examples/StallGuard_TMC2209/StallGuard_TMC2209.ino
#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000
#define STALL_VALUE     100 // [0..255]
#define R_SENSE 0.11f 
TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);
using namespace TMC2208_n;
// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

ISR(TIMER1_COMPA_vect){
  //STEP_PORT ^= 1 << STEP_BIT_POS;
  digitalWrite(X_STEP_PIN, !digitalRead(X_STEP_PIN));
}


void setup() {
    Serial.begin(9600); // Port to send and receive data from Orange Pi

    // Pin Mode for all pins




    // Stepper Stallguard 
    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_DIR_PIN, OUTPUT);
    digitalWrite(X_ENABLE_PIN, LOW);
    // Stepper Test Sequence
    driver.begin();
    driver.toff(4);
    driver.blank_time(24);
    driver.rms_current(400); // mA
    driver.microsteps(16);
    driver.TCOOLTHRS(0xFFFFF); // 20bit max
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.SGTHRS(STALL_VALUE);

    // Set stepper interrupt
    {
        cli();//stop interrupts
        TCCR1A = 0;// set entire TCCR1A register to 0
        TCCR1B = 0;// same for TCCR1B
        TCNT1  = 0;//initialize counter value to 0
        OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
        // turn on CTC mode
        TCCR1B |= (1 << WGM12);
        // Set CS11 bits for 8 prescaler
        TCCR1B |= (1 << CS11);// | (1 << CS10);
        // enable timer compare interrupt
        TIMSK1 |= (1 << OCIE1A);
        sei();//allow interrupts
    }
}



void loop() {

    // Stallguard stuff
    static uint32_t last_time=0;
    uint32_t ms = millis();

    while(Serial.available() > 0) {
        int8_t read_byte = Serial.read();
        #ifdef USING_TMC2660
        if (read_byte == '0')      { TIMSK1 &= ~(1 << OCIE1A); driver.toff(0); }
        else if (read_byte == '1') { TIMSK1 |=  (1 << OCIE1A); driver.toff(driver.savedToff()); }
        #else
        if (read_byte == '0')      { TIMSK1 &= ~(1 << OCIE1A); digitalWrite( EN_PIN, HIGH ); }
        else if (read_byte == '1') { TIMSK1 |=  (1 << OCIE1A); digitalWrite( EN_PIN,  LOW ); }
        #endif
        else if (read_byte == '+') { if (OCR1A > MAX_SPEED) OCR1A -= 20; }
        else if (read_byte == '-') { if (OCR1A < MIN_SPEED) OCR1A += 20; }
    }

    // Stepper Loop
    if((ms-last_time) > 100) { //run every 0.1s
        last_time = ms;

        Serial.print("0 ");
        Serial.print(driver.SG_RESULT(), DEC);
        Serial.print(" ");
        Serial.println(driver.cs2rms(driver.cs_actual()), DEC);
    }

}