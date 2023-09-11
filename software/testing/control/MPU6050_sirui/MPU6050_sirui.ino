#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define Q 0.01 // Process noise covariance
#define R 0.1  // Measurement noise covariance

// Define state variables
double x_hat = 0.0; // Estimated state (initially set to 0)
double P = 1.0;     // Estimated error covariance (initially set to 1)

// Predict step of the Kalman filter
void predict(double u)
{
    // Prediction step
    double x_pred = x_hat;
    double P_pred = P + Q;

    // Update state variables
    x_hat = x_pred;
    P = P_pred;
}

// Update step of the Kalman filter
void update(double z)
{
    // Update step
    double K = P / (P + R);
    double x_updated = x_hat + K * (z - x_hat);
    double P_updated = (1 - K) * P;

    // Update state variables
    x_hat = x_updated;
    P = P_updated;
}

Adafruit_MPU6050 mpu;

void setup()
{
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
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    delay(100);
}

int previous_t = 0;
float last_v_x = 0; // v_0
float last_v_y = 0; // v_0
int interval = 100; // ms

void loop()
{
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    double measurement_x = a.acceleration.x;
    double measurement_y = a.acceleration.y;

    // Predict step
    predict(0.0); // Assuming no control input

    // Update step
    update(measurement_x);

    double reduce_acceleration_x = x_hat;

    // Update step
    update(measurement_y);

    double reduce_acceleration_y = x_hat;

    int current_t = millis();

    if (current_t - previous_t > interval)
    {
        float ax = reduce_acceleration_x;
        float ay = reduce_acceleration_y;

        float v_x = last_v_x + ax * interval;
        float v_y = last_v_y + ay * interval;

        Serial.print("V_x:");
        Serial.print(v_x);
        Serial.print(",");
        Serial.print("V_y:");
        Serial.print(v_y);
        last_v_x = v_x;
        last_v_y = v_y;
        previous_t = current_t;
    }

    Serial.println("");
}