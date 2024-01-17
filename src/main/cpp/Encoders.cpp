/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561

  Converted to 2024 code: January 17, 2024
 */

#include "rev/CANSparkMax.h"
#include "Const.hpp"

double VaENC_Deg_WheelAngleConverted[E_RobotCornerSz]; // This is the wheel angle coming from the angle Encoder and processed to only be from 0 - 180
double VaENC_Deg_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in degrees
double VaENC_Rad_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in radians
double VaENC_Deg_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction

double VaENC_InS_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec
double VaENC_In_WheelDeltaDistance[E_RobotCornerSz]; // Distance wheel moved, loop to loop, in inches
double VaENC_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz]; // Current distance wheel moved, loop to loop, in Counts
double VaENC_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts

/******************************************************************************
 * Function:     EncodersInitSwerve
 * //NOTE - Changed from "EncodersInitCommon" Make sure that gets changed when doing Robot.cpp
 * Description:  Initialize all of the applicable encoders for our Swerve drive.
 ******************************************************************************/
void EncodersInitSwerve(rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive)
  {
    T_RobotCorner LeENC_e_Index;

    for (LeENC_e_Index = E_FrontLeft;
         LeENC_e_Index < E_RobotCornerSz;
         LeENC_e_Index = T_RobotCorner(int(LeENC_e_Index) + 1))
      {
        VaENC_Deg_WheelAngleFwd[LeENC_e_Index] = 0;
        VaENC_Rad_WheelAngleFwd[LeENC_e_Index] = 0;
        VaENC_InS_WheelVelocity[LeENC_e_Index] = 0;
        VaENC_In_WheelDeltaDistance[LeENC_e_Index] = 0;
        VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index] = 0;
        VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index] = 0;
      }

    m_encoderFrontRightSteer.SetPosition(0);
    m_encoderFrontLeftSteer.SetPosition(0);
    m_encoderRearRightSteer.SetPosition(0);
    m_encoderRearLeftSteer.SetPosition(0);

    m_encoderFrontRightDrive.SetPosition(0);
    m_encoderFrontLeftDrive.SetPosition(0);
    m_encoderRearRightDrive.SetPosition(0);
    m_encoderRearLeftDrive.SetPosition(0);

  }

/******************************************************************************
 * Function:     Encoders_Drive_CompBot
 *
 * Description:  Run all of the encoder decoding logic that makes Swerve work.
 ******************************************************************************/
void Encoders_Drive_CompBot(double                       LeENC_Cnt_EncoderWheelAngleFrontLeftRaw,
                            double                       LeENC_Cnt_EncoderWheelAngleFrontRightRaw,
                            double                       LeENC_Cnt_EncoderWheelAngleRearLeftRaw,
                            double                       LeENC_Cnt_EncoderWheelAngleRearRightRaw,
                            rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                            rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                            rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                            rev::SparkMaxRelativeEncoder m_encoderRearRightDrive)
  {
  T_RobotCorner LeENC_e_Index;

  VaENC_Deg_WheelAngleConverted[E_FrontLeft]  = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontLeftRaw), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontLeft];
  VaENC_Deg_WheelAngleConverted[E_FrontRight] = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontRightRaw), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontRight];
  VaENC_Deg_WheelAngleConverted[E_RearLeft]   = std::fmod((LeENC_Cnt_EncoderWheelAngleRearLeftRaw), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearLeft];
  VaENC_Deg_WheelAngleConverted[E_RearRight]  = std::fmod((LeENC_Cnt_EncoderWheelAngleRearRightRaw), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearRight];

  frc::SmartDashboard::PutNumber("WA FL", VaENC_Deg_WheelAngleConverted[E_FrontLeft]);
  frc::SmartDashboard::PutNumber("WA FR", VaENC_Deg_WheelAngleConverted[E_FrontRight]);
  frc::SmartDashboard::PutNumber("WA RL", VaENC_Deg_WheelAngleConverted[E_RearLeft]);
  frc::SmartDashboard::PutNumber("WA RR", VaENC_Deg_WheelAngleConverted[E_RearRight]);

  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]  = m_encoderFrontLeftDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontRight] = m_encoderFrontRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearRight]  = m_encoderRearRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearLeft]   = m_encoderRearLeftDrive.GetPosition();

  for (LeENC_e_Index = E_FrontLeft;
       LeENC_e_Index < E_RobotCornerSz;
       LeENC_e_Index = T_RobotCorner(int(LeENC_e_Index) + 1))
    {
    VaENC_Deg_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleConverted[LeENC_e_Index];

    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] > 180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] -= 360;
      }
    else if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] < -180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] += 360;
      }

    /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] >= 0)
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] - 180;
      }
    else
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] + 180;
      }
    /* Create a copy of the Angle Fwd, but in radians */
    VaENC_Rad_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] * (C_PI/180);

    VaENC_In_WheelDeltaDistance[LeENC_e_Index]  = ((((VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index]  - VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index])/  KeENC_k_ReductionRatio)) * KeENC_In_WheelCircumfrence );
    VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index]  = VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index];
    }

  VaENC_InS_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  }
