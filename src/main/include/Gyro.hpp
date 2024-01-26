/*
  Gyro.Hpp

   Created on: Feb 01, 2020
   Author: 5561

   Contains the code related to the reading and processing of the gyro output.

 */

extern double VeGRY_Deg_GyroYawAngleDegrees;
extern double VeGRY_Rad_GyroYawAngleRad;
extern double VeGRY_Deg_GyroPitchAngleDegrees;
extern double VeGRY_Deg_GyroRollAngleDegrees;

void GyroInit();
void ReadGyro2(bool LeGRY_b_Cmd_DriverZeroGyroCmnd);