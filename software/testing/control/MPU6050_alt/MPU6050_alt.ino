#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Kalman filter variables
double X[4] = {0, 0, 0, 0};                                                            // State vector [position_x, velocity_x, position_y, velocity_y]
double P[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};             // Initial state covariance matrix
double A[4][4] = {{1, 1, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}};             // State transition matrix
double H[2][4] = {{1, 0, 0, 0}, {0, 0, 1, 0}};                                         // Measurement matrix
double Q[4][4] = {{0.01, 0, 0, 0}, {0, 0.01, 0, 0}, {0, 0, 0.01, 0}, {0, 0, 0, 0.01}}; // Process noise covariance matrix
double R[2][2] = {{0.1, 0}, {0, 0.1}};                                                 // Measurement noise covariance matrix

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
int interval = 10; // ms

void loop()
{
    // Read accelerometer data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int16_t accelX, accelY;
    accelX = a.acceleration.x;
    accelY = a.acceleration.y;

    // Convert accelerometer data to m/s^2
    double accel_x_measurement = (double)accelX / 16384.0 * 9.81; // MPU6050 sensitivity scale factor
    double accel_y_measurement = (double)accelY / 16384.0 * 9.81;

    // Predict (Time Update)
    // X = A * X
    double X_pred[4];
    for (int i = 0; i < 4; i++)
    {
        X_pred[i] = 0;
        for (int j = 0; j < 4; j++)
        {
            X_pred[i] += A[i][j] * X[j];
        }
    }

    // P = A * P * A^T + Q
    double P_pred[4][4];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P_pred[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                P_pred[i][j] += A[i][k] * P[k][j];
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P_pred[i][j] += Q[i][j];
        }
    }

    // Update (Measurement Update)
    // Calculate innovation (measurement residual)
    double innovation[2];
    for (int i = 0; i < 2; i++)
    {
        innovation[i] = 0;
        for (int j = 0; j < 4; j++)
        {
            innovation[i] += H[i][j] * X_pred[j];
        }
        innovation[i] = accel_x_measurement - innovation[0];
    }

    // Calculate Kalman gain
    double S[2][2];
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            S[i][j] = 0;
            for (int k = 0; k < 4; k++)
            {
                S[i][j] += H[i][k] * P_pred[k][j];
            }
        }
    }
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            S[i][j] += R[i][j];
        }
    }

    double K[4][2];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            K[i][j] = P_pred[i][j] * S[j][j];
        }
    }

    // Update state estimate
    for (int i = 0; i < 4; i++)
    {
        X[i] = X_pred[i] + K[i][0] * innovation[0] + K[i][1] * innovation[1];
    }

    // Update state estimate covariance matrix
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P[i][j] = P_pred[i][j] - K[i][0] * S[0][j] - K[i][1] * S[1][j];
        }
    }

//    int current_t = millis();
//
//    if (current_t - previous_t >= interval)
//    {
//        float v_x = last_v_x + X[0] * interval;
//        float v_y = last_v_y + X[2] * interval;
//
//        Serial.print("V_x:");
//        Serial.print(v_x);
//        Serial.print(",");
//        Serial.print("V_y:");
//        Serial.print(v_y);
//        
//        previous_t = current_t;
//        last_v_x = v_x; 
//        last_v_y = v_y; 
//    }

    double ax = X[0];
    double ay = X[2];
    
    Serial.print("x");
    Serial.print(ax);
    Serial.print(", ");
    Serial.print("y:");
    Serial.print(ay);
    Serial.print("\n");

    // Now, X[0] and X[2] contain the estimated linear acceleration in the x and y axes, respectively.

    delay(10); // You can adjust the loop delay as needed.
}
