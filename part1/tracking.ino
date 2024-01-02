/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX, kalmanY, kalmanZ; // Create the Kalman instances

const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
const uint8_t HMC5883L = 0x1E; // Address of magnetometer

/* IMU Data */
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
int16_t tempRaw;

double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer while yaw is calculated using the magnetometer

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter


float rotate_x, rotate_y, rotate_z;
float vx, vy, vz;
float x, y, z;

float ax_offset, ay_offset, az_offset;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

#define MAG0MAX 603
#define MAG0MIN -578

#define MAG1MAX 542
#define MAG1MIN -701

#define MAG2MAX 547
#define MAG2MIN -556

// float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
float magOffset[3] = { (MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2 };
double magGain[3];

void setup() {
  delay(100); // Wait for sensors to get ready
  Serial.begin(9600);

  while(!Serial)
    Serial.println("Serial ON");

  IMU.begin();

  Wire.begin();
  Wire.setClock(400000L);

  calibrateMag();

  rotate_x = 0;
  rotate_y = 0;
  rotate_z = 0;
  vx = 0;
  vy = 0;
  vz = 0;
  x = 0;
  y = 0;
  z = 0;
  
  while(!IMU.accelerationAvailable())
    IMU.readAcceleration(ax_offset, ay_offset, az_offset);

  /* Set Kalman and gyro starting angle */
  if(IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    updateMPU6050();    
    updatePitchRoll();   
    if(IMU.magneticFieldAvailable()) {
      updateHMC5883L();
    }
  }

  kalmanX.setAngle(roll); // First set roll starting angle
  gyroXangle = roll;
  compAngleX = roll;

  kalmanY.setAngle(pitch); // Then pitch
  gyroYangle = pitch;
  compAngleY = pitch;

  yaw = 0;
  kalmanZ.setAngle(yaw); // And finally yaw
  gyroZangle = yaw;
  compAngleZ = yaw;

  timer = micros(); // Initialize the timer
}

void loop() {
  float ax_temp, ay_temp, az_temp;
  if(IMU.gyroscopeAvailable() && IMU.accelerationAvailable() && IMU.magneticFieldAvailable())
  {
    /* Update all the IMU values */
    updateMPU6050();
    updateHMC5883L();

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();


    /* Roll and pitch estimation */
    updatePitchRoll();
    double gyroXrate = gyroX; // Convert to deg/s
    double gyroYrate = gyroY; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif


    /* Yaw estimation */
    updateYaw();
    double gyroZrate = gyroZ; // Convert to deg/s
    // This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
    if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
      kalmanZ.setAngle(yaw);
      compAngleZ = yaw;
      kalAngleZ = yaw;
      gyroZangle = yaw;
    } else
      kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter


    /* Estimate angles using gyro only */
    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
    gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter
    gyroYangle += kalmanY.getRate() * dt;
    gyroZangle += kalmanZ.getRate() * dt;

    /* Estimate angles using complimentary filter */
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
    compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

    // Reset the gyro angles when they has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
    if (gyroZangle < -180 || gyroZangle > 180)
      gyroZangle = kalAngleZ;


    /* Print Data */
  #if 0
    Serial.print(roll); Serial.print("\t");
    Serial.print(gyroXangle); Serial.print("\t");
    Serial.print(compAngleX); Serial.print("\t");
    Serial.print(kalAngleX); Serial.print("\t");

    Serial.print("\t");

    Serial.print(pitch); Serial.print("\t");
    Serial.print(gyroYangle); Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");

    Serial.print("\t");

    Serial.print(yaw); Serial.print("\t");
    Serial.print(gyroZangle); Serial.print("\t");
    Serial.print(compAngleZ); Serial.print("\t");
    Serial.print(kalAngleZ); Serial.print("\t");
  #endif
  #if 0 // Set to 1 to print the IMU data
    Serial.print(accX / 16384.0); Serial.print("\t"); // Converted into g's
    Serial.print(accY / 16384.0); Serial.print("\t");
    Serial.print(accZ / 16384.0); Serial.print("\t");

    Serial.print(gyroXrate); Serial.print("\t"); // Converted into degress per second
    Serial.print(gyroYrate); Serial.print("\t");
    Serial.print(gyroZrate); Serial.print("\t");

    Serial.print(magX); Serial.print("\t"); // After gain and offset compensation
    Serial.print(magY); Serial.print("\t");
    Serial.print(magZ); Serial.print("\t");
  #endif
  #if 0 // Set to 1 to print the temperature
    Serial.print("\t");

    double temperature = (double)tempRaw / 340.0 + 36.53;
    Serial.print(temperature); Serial.print("\t");
  #endif

    gyroX = -gyroX;
    accX = -accX;


    kalAngleX = kalAngleX * PI / 180;
    kalAngleY = kalAngleY * PI / 180;
    kalAngleZ = kalAngleZ * PI / 180;

    ax_temp = accX;
    ay_temp = accY * cos(kalAngleX) - accZ * sin(kalAngleX);
    az_temp = accY * sin(kalAngleX) + accZ * cos(kalAngleX);
    accX = ax_temp;
    accY = ay_temp;
    accZ = az_temp;

    ax_temp = accX * cos(kalAngleY) + accZ * sin(kalAngleY);
    ay_temp = accY;
    az_temp = - accX * sin(kalAngleY) + accZ * cos(kalAngleY);
    accX = ax_temp;
    accY = ay_temp;
    accX = az_temp;

    ax_temp = accX * cos(kalAngleZ) - accY * sin(kalAngleZ);
    ay_temp = accX * sin(kalAngleZ) + accY * cos(kalAngleZ);
    az_temp = accZ;
    accX = ax_temp;
    accY = ay_temp;
    accZ = az_temp;

    accX = accX - ax_offset;
    accY = accY - ay_offset;
    accZ = accZ - az_offset;

    x = x + (vx + vx + 9.8 * accX * dt) * dt / 2;
    y = y + (vy + vy + 9.8 * accY * dt) * dt / 2;
    z = z + (vz + vz + 9.8 * accZ * dt) * dt / 2;

    vx = vx + 9.8 * accX * dt;
    vy = vy + 9.8 * accY * dt;
    vz = vz + 9.8 * accZ * dt;

    if (gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ < 100)
    {
      // Compute new offset (including the gravity)
      ax_offset = ax_offset + accX;
      ay_offset = ay_offset + accY;
      az_offset = az_offset + accZ;
      // Set velocity zero
      vx = 0;
      vy = 0;
      vz = 0;
    }

    Serial.print(x*100);
    Serial.print('\t');
    Serial.print(y*100);
    Serial.print('\t');
    Serial.println(z*100);
  }
}

void updateMPU6050() {
  IMU.readAcceleration(accX, accY, accZ);
  IMU.readGyroscope(gyroX, gyroY, gyroZ);
}

void updateHMC5883L() {
  IMU.readMagneticField(magX, magY, magZ);
}

void updatePitchRoll() {
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  roll = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void updateYaw() { // See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
  magX *= -1; // Invert axis - this it done here, as it should be done after the calibration
  magZ *= -1;

  // magX *= magGain[0];
  // magY *= magGain[1];
  // magZ *= magGain[2];

  magX *= 1;
  magY *= 1;
  magZ *= 1;

  magX -= magOffset[0]/20;
  magY -= magOffset[1]/20;
  magZ -= magOffset[2]/20;

  double rollAngle = kalAngleX * DEG_TO_RAD;
  double pitchAngle = kalAngleY * DEG_TO_RAD;

  double Bfy = magZ * sin(rollAngle) - magY * cos(rollAngle);
  double Bfx = magX * cos(pitchAngle) + magY * sin(pitchAngle) * sin(rollAngle) + magZ * sin(pitchAngle) * cos(rollAngle);
  if(Bfx != 0) {
    yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;
    yaw *= -1;
  }
}

void calibrateMag() { // Inspired by: https://code.google.com/p/open-headtracker/
  // i2cWrite(HMC5883L, 0x00, 0x11, true);
  // delay(100); // Wait for sensor to get ready
  updateHMC5883L(); // Read positive bias values

  int16_t magPosOff[3] = { magX, magY, magZ };
  magOffset[0] = magX;
  magOffset[1] = magY;
  magOffset[2] = magZ;


  // i2cWrite(HMC5883L, 0x00, 0x12, true);
  // delay(100); // Wait for sensor to get ready  
  // updateHMC5883L(); // Read negative bias values

  // int16_t magNegOff[3] = { magX, magY, magZ };

  // magGain[0] = -2500 / float(magNegOff[0] - magPosOff[0]);
  // magGain[1] = -2500 / float(magNegOff[1] - magPosOff[1]);
  // magGain[2] = -2500 / float(magNegOff[2] - magPosOff[2]);

#if 0
  Serial.print("Mag cal: ");
  Serial.print(magPosOff[0]);
  Serial.print(",");
  Serial.print(magPosOff[1]);
  Serial.print(",");
  Serial.println(magPosOff[2]);

  // Serial.print("Gain: ");
  // Serial.print(magGain[0]);
  // Serial.print(",");
  // Serial.print(magGain[1]);
  // Serial.print(",");
  // Serial.println(magGain[2]);
#endif
}
