#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

unsigned long lastTime = 0;

void IMUsetup(void) {
  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  lastTime = millis();
}

// Declare a function to return sensor data.
void getSensorData(float &gyro_x, float &gyro_y, float &gyro_z, float &orientation_x, float &orientation_y, float &orientation_z) {

  // Get orientation and angular velocity data.
  sensors_event_t orientationData, angVelocityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Store gyro and orientation data in the function parameters.
  gyro_x = angVelocityData.gyro.x;
  gyro_y = angVelocityData.gyro.y;
  gyro_z = angVelocityData.gyro.z;
  orientation_x = orientationData.orientation.x;
  orientation_y = orientationData.orientation.y;
  orientation_z = orientationData.orientation.z;
}

float roll_output = 0, pitch_output = 0, yaw_output = 0;

void IMUdata() {

  if (millis() - lastTime >= BNO055_SAMPLERATE_DELAY_MS) {
    lastTime = millis();
    float gyro_x, gyro_y, gyro_z, orientation_x, orientation_y, orientation_z;
    float rate_pitch = 0.0, rate_roll = 0.0, rate_yaw = 0.0;  // output from !


    // Call the getSensorData function to get the latest gyro and orientation data.
    getSensorData(gyro_x, gyro_y, gyro_z, orientation_x, orientation_y, orientation_z);

    // take input data and update it
    // 0.0 -> desired roll rate deg/s


    sbus_to_pid(orientation_z, orientation_y, Thr, Ail, Ele, Rud, ch5, failsafe, rate_pitch, rate_roll, rate_yaw);

    pid_update(gyro_x, gyro_y, gyro_z, rate_roll, rate_pitch, rate_yaw, roll_output, pitch_output, yaw_output, BNO055_SAMPLERATE_DELAY_MS);

    PIDmain(roll_output, pitch_output, yaw_output);




    // // Print the gyro and orientation data.
    // Serial.print("Gyro: X:");
    // Serial.print(gyro_x);
    // Serial.print(" Y:");
    // Serial.print(gyro_y);
    // Serial.print(" Z:");
    // Serial.print(gyro_z);
    // Serial.print(" | Orientation: X:");
    // Serial.print(orientation_x);
    // Serial.print(" Y:");
    // Serial.print(orientation_y);
    // Serial.print(" Z:");
    // Serial.println(orientation_z);
  }
}