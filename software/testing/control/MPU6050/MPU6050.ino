#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(115200);

    // Try to initialize!
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
    }

    // set accelerometer range to +-8G
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    // set gyro range to +- 500 deg/s
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    // set filter bandwidth to 21 Hz
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(100);
}

int previous_t = 0;
float last_v_x = 0; //v_0
float last_v_y = 0; //v_0
int interval = 10; //ms

void loop()
{
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    int current_t = millis();

    if (current_t - previous_t >= interval) {
        float ax = a.acceleration.x;
        float ay = a.acceleration.y;

        float v_x = last_v_x + ax*interval;
        float v_y = last_v_y + ay*interval;

        previous_t = current_t;
        last_v_x = v_x; 
        last_v_y = v_y; 

        Serial.print("V_x:");
        Serial.print(v_x);
        Serial.print(", ");
        Serial.print("V_y:");
        Serial.print(v_y);
    }

    Serial.println("");
}