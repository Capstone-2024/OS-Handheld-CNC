#include "SerialTransfer.h"
#include <Adafruit_ADXL345_U.h>

SerialTransfer myTransfer;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

void setup()
{
    Serial.begin(115200);
    myTransfer.begin(Serial);

    Wire.begin();
}

void loop()
{
    if (myTransfer.available())
    {
        if (int(myTransfer.packet.rxBuff[0]) == 65)
        {
            sampleAccelerometer();
        }
    }
}

// void loop()
// {
//     if (myTransfer.available())
//     {
//         uint8_t instruction = myTransfer.packet.rxBuff[0];

//         // if (int(instruction) == 73)
//         // {
//         //     uint16_t recSize = 0;
//         //     float xPacket;
//         //     float yPacket;

//         //     recSize = myTransfer.rxObj(xPacket, recSize);
//         //     recSize = myTransfer.rxObj(yPacket, recSize);

//         //     // autoCorrection(xPacket, yPacket);
//         // }
//         // else 
//         if (int(instruction) == 65)
//         {
//             sampleAccelerometer();
//         }
//     }
// }

void sampleAccelerometer()
{
    sensors_event_t event;

    accel.getEvent(&event);

    float z_accel = event.acceleration.z;
    float y_accel = event.acceleration.y;

    // float z_accel = -0.110;
    // float y_accel = 1.230;
    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(z_accel, sendSize);
    sendSize = myTransfer.txObj(y_accel, sendSize);
    myTransfer.sendData(sendSize);
    return;
}