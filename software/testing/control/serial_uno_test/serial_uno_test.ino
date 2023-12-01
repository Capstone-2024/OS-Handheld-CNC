#include "SerialTransfer.h"

SerialTransfer myTransfer;

void setup()
{
    Serial.begin(115200);
    myTransfer.begin(Serial);
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
//     int data = 0;
//     if (Serial.available() > 0)
//     {
//         char instruction = Serial.read();

//         if ((int)instruction == 65)
//         {
//             sampleAccelerometer();
//         }
//     }
// }

void sampleAccelerometer()
{
    float z_accel = -0.110;
    float y_accel = 1.230;

    uint16_t sendSize = 0;
    sendSize = myTransfer.txObj(z_accel, sendSize);
    sendSize = myTransfer.txObj(y_accel, sendSize);
    myTransfer.sendData(sendSize);

    return;
}