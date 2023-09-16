#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
// Define Kalman filter variables
double x_hat = 0.0;    // Estimated state: acceleration
double y_hat = 0.0;    // Estimated state: acceleration
double Px = 1.0;        // Covariance matrix
double Py = 1.0;        // Covariance matrix
double Q = 0.01;       // Process noise covariance
double R = 0.1;        // Measurement noise covariance

// Kalman filter prediction step
void x_predict() {
    // Predict the next state
    double x_pred = x_hat;
    double Px_pred = Px + Q;

    x_hat = x_pred;
    Px = Px_pred;
}

void y_predict() {
    // Predict the next state
    double y_pred = y_hat;
    double Py_pred = Py + Q;

    y_hat = y_pred;
    Py = Py_pred;
}

// Kalman filter update step
void updatex(double z) {
    // Calculate Kalman gain
    double Kx = Px / (Px + R);

    // Update state estimate and covariance
    x_hat = x_hat + Kx * (z - x_hat);
    Px = (1 - Kx) * Px;
}

void updatey(double z) {
    // Calculate Kalman gain
    double Ky = Py / (Py + R);

    // Update state estimate and covariance
    y_hat = y_hat + Ky * (z - y_hat);
    Py = (1 - Ky) * Py;
}


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
float last_v_x = 0; //v_0 m/s
float last_v_y = 0; //v_0 m/s
int interval = 10; //s m/s

void loop()
{
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int current_t = millis();

    if (current_t - previous_t >= interval) {

        float accelerometer_data_x=a.acceleration.x;
        float accelerometer_data_y=a.acceleration.y;

        x_predict();
        y_predict();

        // Update step with noisy accelerometer data
        double z = accelerometer_data_x;
        updatex(z);
        float ax = x_hat;

        z = accelerometer_data_y;
        updatey(z);
        float ay = y_hat;

        float v_x = last_v_x + ax*interval/1000;
        float v_y = last_v_y + ay*interval/1000;

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
