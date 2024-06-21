#include <Arduino.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>

QwiicOTOS myOtos;

void setup()
{
    // put your setup code here, to run once:

    Serial.end();
    Serial.setRxBufferSize(1024);
    Serial.setTxBufferSize(1024);
    Serial.begin(921600);
    delay(1000);
    Serial.println();
    Serial.println("ESP32");

    Serial.println("Init Qwiic OTOS");
    delay(2000);

    Wire.begin();

    // Attempt to begin the sensor
    while (myOtos.begin() == false)
    {
        Serial.println("OTOS not connected, check your wiring and I2C address!");
        delay(1000);
    }

    Serial.println("OTOS connected!");

    Serial.println("Ensure the OTOS is flat and stationary, then enter any key to calibrate the IMU");

    // Clear the serial buffer
    // while (Serial.available()) Serial.read();
    // Wait for user input
    // while (!Serial.available());

    Serial.println("Calibrating IMU...");

    // Calibrate the IMU, which removes the accelerometer and gyroscope offsets
    myOtos.calibrateImu();

    // Here we can set the linear and angular scalars, which can compensate for
    // scaling issues with the sensor measurements. Note that as of firmware
    // version 1.0, these values will be lost after a power cycle, so you will
    // need to set them each time you power up the sensor. They can be any value
    // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
    // first set both scalars to 1.0, then calibrate the angular scalar, then
    // the linear scalar. To calibrate the angular scalar, spin the robot by
    // multiple rotations (eg. 10) to get a precise error, then set the scalar
    // to the inverse of the error. Remember that the angle wraps from -180 to
    // 180 degrees, so for example, if after 10 rotations counterclockwise
    // (positive rotation), the sensor reports -15 degrees, the required scalar
    // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
    // robot a known distance and measure the error; do this multiple times at
    // multiple speeds to get an average, then set the linear scalar to the
    // inverse of the error. For example, if you move the robot 100 inches and
    // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
    myOtos.setLinearScalar(1.000);
    myOtos.setAngularScalar(1.000);

    // Set the desired units for linear and angular measurements. Can be either
    // meters or inches for linear, and radians or degrees for angular. If not
    // set, the default is inches and degrees. Note that this setting is not
    // stored in the sensor, it's part of the library, so you need to set at the
    // start of all your programs.
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    // myOtos.setLinearUnit(kSfeOtosLinearUnitInches);
    // myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);
    myOtos.setAngularUnit(kSfeOtosAngularUnitDegrees);

    // Reset the tracking algorithm - this resets the position to the origin,
    // but can also be used to recover from some rare tracking errors
    myOtos.resetTracking();

    sfeTkError_t error;
    sfe_otos_signal_process_config_t config;
    error = myOtos.getSignalProcessConfig(config);
    if (error != 0)
    {
        Serial.print("Error get Signal Process Config : ");
        Serial.println(error);
    }
    else
    {
        Serial.println("Signal Process Config :");
        Serial.print("enVar : ");
        Serial.println(config.enVar);
        Serial.print("enRot : ");
        Serial.println(config.enRot);
        Serial.print("enAcc : ");
        Serial.println(config.enAcc);
        Serial.print("enLut : ");
        Serial.println(config.enLut);
    }
}

void loop()
{
    sfeTkError_t error;
    // Get the latest position, which includes the x and y coordinates, plus the
    // heading angle
    sfe_otos_pose2d_t pos;
    error = myOtos.getPosition(pos);
    if (error != 0)
    {
        Serial.print(">Error Pos:");
        Serial.println(error);
    }

    // Create structs for velocity, and acceleration
    sfe_otos_pose2d_t vel;
    error = myOtos.getVelocity(vel);
    if (error != 0)
    {
        Serial.print(">Error Vel:");
        Serial.println(error);
    }

    sfe_otos_pose2d_t acc;
    error = myOtos.getAcceleration(acc);
    if (error != 0)
    {
        Serial.print(">Error Acc:");
        Serial.println(error);
    }

    // Print measurement
    // Serial.println();
    // Serial.println("Position:");
    Serial.print(">X (Meters):");
    Serial.println(pos.x, 4);
    Serial.print(">Y (Meters):");
    Serial.println(pos.y, 4);
    Serial.print(">Heading (Degrees):");
    Serial.println(pos.h, 4);

    // Print velocity
    // Serial.println();
    // Serial.println("Velocity:");
    Serial.print(">X (Meters/sec):");
    Serial.println(vel.x, 4);
    Serial.print(">Y (Meters/sec):");
    Serial.println(vel.y, 4);
    Serial.print(">Heading (Degrees/sec):");
    Serial.println(vel.h, 4);

    // Print acceleration
    // Serial.println();
    // Serial.println("Acceleration:");
    Serial.print(">X (Meters/sec^2):");
    Serial.println(acc.x, 4);
    Serial.print(">Y (Meters/sec^2):");
    Serial.println(acc.y, 4);
    Serial.print(">Heading (Degrees/sec^2):");
    Serial.println(acc.h, 4);

    // Wait a bit so we don't spam the serial port
    delay(100);
}