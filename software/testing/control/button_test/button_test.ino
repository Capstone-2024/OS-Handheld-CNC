#include "SerialTransfer.h"

#define LeftbuttonPin 18  // left hand button
#define RightbuttonPin 19 // Right hand button

SerialTransfer myTransfer;

volatile bool buttonState = false;
volatile int buttonChangeTime;

void setup()
{
    pinMode(RightbuttonPin, INPUT_PULLUP);
    pinMode(LeftbuttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LeftbuttonPin), buttonInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RightbuttonPin), buttonInterrupt, CHANGE);
}

void loop()
{
    // if (myTransfer.available() && buttonState)
    if (buttonState)
    {
        uint16_t sendSize = 0;
        char data = 'Y';
        sendSize = myTransfer.txObj(data, sendSize);
        myTransfer.sendData(sendSize);
    }
    else
    {
        uint16_t sendSize = 0;
        char data = 'N';
        sendSize = myTransfer.txObj(data, sendSize);
        myTransfer.sendData(sendSize);

        if ((millis() - buttonChangeTime) >= 60000)
        {
        }
        delay(500);
    }
}

void buttonInterrupt()
{
    buttonState = false;
    buttonChangeTime = millis();
    buttonState = !(digitalRead(LeftbuttonPin) || digitalRead(RightbuttonPin));
}