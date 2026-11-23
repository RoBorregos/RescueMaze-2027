#include "imu.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

static Adafruit_BNO08x bno(-1);

bool Imu::begin() {
    ready = false;
    has_reading = false;
    last_raw_deg = 0.0f;
    heading_deg = 0.0f;

    if(!bno.begin_I2C(ImuConfig::kAddress, &Wire)) {
        Serial.println("Failed to find BNO08x chip");
        return false;
    }

    enableReports();
    ready = true;
    return true;
}

void Imu::enableReports() {
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, ImuConfig::kReportIntervalUs);
}

void Imu::update() {
    if(!ready) return;

     // If the sensor restarted by itself, its reports are off again.
    if (bno.wasReset()) {
        enableReports();
    }
    
    sh2_SensorValue_t value;
    if (!bno.getSensorEvent(&value)) return; //No new data
    if (value.sensorId != SH2_GAME_ROTATION_VECTOR) return; //Not the report we want

    // The sensor gives its orientation as a quaternion. This formula extracts the rotation around the vertical axis (yaw).
    float r = value.un.gameRotationVector.real;
    float i = value.un.gameRotationVector.i;
    float j = value.un.gameRotationVector.j;
    float k = value.un.gameRotationVector.k;
    float yaw_rad = atan2f(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));

    // The sensor counts counter-clockwise as positive; we want clockwise.
    float raw_deg = -yaw_rad * 180.0f / Geometry::kPi;
    if (ImuConfig::kHeadingInverted) raw_deg = -raw_deg;

    if (!has_reading) {
        last_raw_deg = raw_deg;
        has_reading = true;
        return;
    }

    float delta = raw_deg - last_raw_deg;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    heading_deg += delta;
    last_raw_deg = raw_deg;

}

bool Imu::isReady() const {
    return ready && has_reading;
}

float Imu::getHeading() const {
    return heading_deg;
}