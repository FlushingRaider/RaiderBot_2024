
/*
  Gyro.cpp

   Created on: Feb 01, 2020
   Author: 5561 and CHLOE & Carson

   Contains the code related to the reading and processing of the gyro output.

 */

#include <ctre/phoenix6/Pigeon2.hpp>
#include "Const.hpp"
#include <units/angle.h>
//TODO - bingus
ctre::phoenix6::hardware::Pigeon2 bingus = ctre::phoenix6::hardware::Pigeon2(KeGRY_i_Gyro);

double VeGRY_Deg_GyroYawAngleDegrees; //Raw gyro angle in degrees uwu
double VeGRY_Rad_GyroYawAngleRad; //Raw gyro angle in radians owo
double VeGRY_Deg_GyroPitchAngleDegrees; //Raw gyro angle in degrees for pitch
double VeGRY_Deg_GyroRollAngleDegrees; 




/******************************************************************************
 * Function:     GyroInit
 *
 * Description:  Initialization of the gyro.
 ******************************************************************************/
void GyroInit()
  {
    bingus.Reset();
    //NOTE - I have no idea if reset() does the same thing as ConfigFactoryDefault() but I'm gonna hope
    bingus.SetYaw(0.0_deg, KeGRY_ms_GyroTimeoutMs);

    VeGRY_Deg_GyroYawAngleDegrees = 0;
    VeGRY_Rad_GyroYawAngleRad = 0;
    VeGRY_Deg_GyroPitchAngleDegrees = 0;
    VeGRY_Deg_GyroRollAngleDegrees = 0;
  }

/******************************************************************************
 * Function:     ReadGyro2
 *
 * Description:  Contains the code to read the Pidgeon gyro.
 ******************************************************************************/
void ReadGyro2(bool LeGRY_b_Cmd_DriverZeroGyroCmnd)
  {
  double LeGRY_Deg_GyroYawAngleRawDegrees = 0;
  double LeGRY_Deg_GyroYawAngleLimited   = 0;
  double LeGRY_Deg_GyroPitchAngleDegrees = 0;
  double LeGRY_Deg_GyroRollAngleDegrees = 0;
  // double LeGRY_Deg_GyroPitchAngleLimited = 0;

  if (LeGRY_b_Cmd_DriverZeroGyroCmnd)
    {
    bingus.SetYaw(0_deg, KeGRY_ms_GyroTimeoutMs);
    }

  LeGRY_Deg_GyroYawAngleRawDegrees = -bingus.GetYaw().GetValueAsDouble();
  LeGRY_Deg_GyroPitchAngleDegrees = bingus.GetPitch().GetValueAsDouble();
  LeGRY_Deg_GyroRollAngleDegrees = bingus.GetRoll().GetValueAsDouble();

  LeGRY_Deg_GyroYawAngleLimited  = std::fmod((LeGRY_Deg_GyroYawAngleRawDegrees), 360);

  if (LeGRY_Deg_GyroYawAngleLimited > 180)
    {
    LeGRY_Deg_GyroYawAngleLimited -= 360;
    }
  else if (LeGRY_Deg_GyroYawAngleLimited < -180)
    {
    LeGRY_Deg_GyroYawAngleLimited += 360;
    }

  VeGRY_Deg_GyroYawAngleDegrees = LeGRY_Deg_GyroYawAngleLimited;

  VeGRY_Rad_GyroYawAngleRad = LeGRY_Deg_GyroYawAngleLimited / C_RadtoDeg;

  VeGRY_Deg_GyroPitchAngleDegrees = LeGRY_Deg_GyroPitchAngleDegrees;
  VeGRY_Deg_GyroRollAngleDegrees = LeGRY_Deg_GyroRollAngleDegrees;
  }