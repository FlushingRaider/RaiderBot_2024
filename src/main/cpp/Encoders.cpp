/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561

  Converted to 2024 code: January 17, 2024
 */

#include "Amp.hpp"
#include "rev/CANSparkMax.h"
#include "Const.hpp"
#include "DataLogger.hpp"
#include "Amp.hpp"
#include "SpeakerCntrl.hpp"
#include "Climber.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

double VaENC_Deg_WheelAngleConverted[E_RobotCornerSz]; // This is the wheel angle coming from the angle Encoder and processed to only be from 0 - 180
double VaENC_Deg_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in degrees
double VaENC_Rad_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in radians
double VaENC_Deg_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction

double VaENC_InS_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec
double VaENC_In_WheelDeltaDistance[E_RobotCornerSz]; // Distance wheel moved, loop to loop, in inches
double VaENC_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz]; // Current distance wheel moved, loop to loop, in Counts
double VaENC_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts

double VeENC_V_PDPVoltage = 0;


/******************************************************************************
 * Function:     EncodersInitSwerve
 * 
 * Description:  Initialize all of the applicable encoders for our Swerve drive.
 ******************************************************************************/
void EncodersInitSwerve(rev::SparkRelativeEncoder m_encoderFrontRightSteer,
                        rev::SparkRelativeEncoder m_encoderFrontLeftSteer,
                        rev::SparkRelativeEncoder m_encoderRearRightSteer,
                        rev::SparkRelativeEncoder m_encoderRearLeftSteer,
                        rev::SparkRelativeEncoder m_encoderFrontRightDrive,
                        rev::SparkRelativeEncoder m_encoderFrontLeftDrive,
                        rev::SparkRelativeEncoder m_encoderRearRightDrive,
                        rev::SparkRelativeEncoder m_encoderRearLeftDrive)
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
void Encoders_Drive_CompBot(units::degree_t                       LeENC_Cnt_EncoderWheelAngleFrontLeftRaw,
                            units::degree_t                       LeENC_Cnt_EncoderWheelAngleFrontRightRaw,
                            units::degree_t                       LeENC_Cnt_EncoderWheelAngleRearLeftRaw,
                            units::degree_t                       LeENC_Cnt_EncoderWheelAngleRearRightRaw,
                            rev::SparkRelativeEncoder m_encoderFrontLeftDrive,
                            rev::SparkRelativeEncoder m_encoderFrontRightDrive,
                            rev::SparkRelativeEncoder m_encoderRearLeftDrive,
                            rev::SparkRelativeEncoder m_encoderRearRightDrive)
  {
  T_RobotCorner LeENC_e_Index;

  VaENC_Deg_WheelAngleConverted[E_FrontLeft]  = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontLeftRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontLeft];
  VaENC_Deg_WheelAngleConverted[E_FrontRight] = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontRightRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontRight];
  VaENC_Deg_WheelAngleConverted[E_RearLeft]   = std::fmod((LeENC_Cnt_EncoderWheelAngleRearLeftRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearLeft];
  VaENC_Deg_WheelAngleConverted[E_RearRight]  = std::fmod((LeENC_Cnt_EncoderWheelAngleRearRightRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearRight];

  // frc::SmartDashboard::PutNumber("WA FL", VaENC_Deg_WheelAngleConverted[E_FrontLeft]);
  // frc::SmartDashboard::PutNumber("WA FR", VaENC_Deg_WheelAngleConverted[E_FrontRight]);
  // frc::SmartDashboard::PutNumber("WA RL", VaENC_Deg_WheelAngleConverted[E_RearLeft]);
  // frc::SmartDashboard::PutNumber("WA RR", VaENC_Deg_WheelAngleConverted[E_RearRight]);

  // frc::SmartDashboard::PutNumber("WS FL", m_encoderFrontLeftDrive.GetVelocity());
  // frc::SmartDashboard::PutNumber("WS FR", m_encoderFrontRightDrive.GetVelocity());
  // frc::SmartDashboard::PutNumber("WS RL", m_encoderRearLeftDrive.GetVelocity());
  // frc::SmartDashboard::PutNumber("WS RR", m_encoderRearRightDrive.GetVelocity());

    log_encoders_wheelangleconverted.Append(VaENC_Deg_WheelAngleConverted);
    // frc::SmartDashboard::PutNumberArray("encoders_wheelAngleConverted", VaENC_Deg_WheelAngleConverted);

  

  // frc::SmartDashboard::PutNumber("raw WA FL", LeENC_Cnt_EncoderWheelAngleFrontLeftRaw.value());
  // frc::SmartDashboard::PutNumber("Raw WA FR", LeENC_Cnt_EncoderWheelAngleFrontRightRaw.value());
  // frc::SmartDashboard::PutNumber("Raw WA RL", LeENC_Cnt_EncoderWheelAngleRearLeftRaw.value());
  // frc::SmartDashboard::PutNumber("raw WA RR", LeENC_Cnt_EncoderWheelAngleRearRightRaw.value());

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

  VaENC_InS_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  /KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  
  double LTest_distance = ((((VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontLeft])/  KeENC_k_ReductionRatio)) * KeENC_In_WheelCircumfrence );
    // frc::SmartDashboard::PutNumber("Wheeel_Distance_FL", LTest_distance);

    log_encoders_wheelVelocity.Append(VaENC_InS_WheelVelocity);
    // frc::SmartDashboard::PutNumberArray("encoders_wheelVelocity", VaENC_InS_WheelVelocity);
  }


/******************************************************************************
 * Function:     Encoders_AMP_SPK_CLMR
 *
 * Description:  Read the encoders/sensors from the amp, speaker and climber
 ******************************************************************************/
void Encoders_AMP_SPK_CLMR_Init( rev::SparkMaxRelativeEncoder m_encoderElevator,
                                 rev::SparkMaxRelativeEncoder m_encoderClimberLeft,
                                 rev::SparkMaxRelativeEncoder m_encoderClimberRight,
                                 rev::SparkMaxRelativeEncoder m_encoderWrist,
                                 rev::SparkMaxRelativeEncoder m_encoderIntake,
                                 rev::SparkMaxRelativeEncoder m_encoderUnderbelly,
                                 rev::SparkMaxRelativeEncoder m_encoderIAssist,
                                 rev::SparkMaxRelativeEncoder m_encoderShooter1,
                                 rev::SparkMaxRelativeEncoder m_encoderShooter2)
  {
  m_encoderWrist.SetPosition(0);
  m_encoderElevator.SetPosition(0);
  m_encoderIntake.SetPosition(0);
  m_encoderUnderbelly.SetPosition(0);
  m_encoderIAssist.SetPosition(0);
  m_encoderShooter1.SetPosition(0);
  m_encoderShooter2.SetPosition(0);
  m_encoderClimberLeft.SetPosition(0);
  m_encoderClimberRight.SetPosition(0);
  }

/******************************************************************************
 * Function:     Encoders_AMP_ResetWrist
 *
 * Description:  Read the encoders/sensors from the amp, speaker and climber
 ******************************************************************************/
void Encoders_AMP_ResetWrist(rev::SparkMaxRelativeEncoder m_encoderWrist,
                             rev::SparkMaxRelativeEncoder m_encoderElevator,
                             bool LeENC_b_ResetReq,
                             bool LeENC_b_ElevatorEncoderReset)
  {
  if (LeENC_b_ResetReq == true)
    {
    m_encoderWrist.SetPosition(0);
    }
  if (LeENC_b_ElevatorEncoderReset == true)
    {
    m_encoderElevator.SetPosition(0); 
    }
  }

/******************************************************************************
 * Function:     Encoders_AMP_SPK_CLMR_Run
 *
 * Description:  Read the encoders/sensors from the amp, speaker and climber
 ******************************************************************************/
void Encoders_AMP_SPK_CLMR_Run( bool                         LeENC_b_AMP_IntakeLimit,
                                bool                         LeENC_b_AMP_ElevatorLimit,
                                bool                         LeENC_b_SPK_IntakeLimit,
                                rev::SparkMaxRelativeEncoder m_encoderElevator,
                                rev::SparkMaxRelativeEncoder m_encoderClimberLeft,
                                rev::SparkMaxRelativeEncoder m_encoderClimberRight,
                                rev::SparkMaxRelativeEncoder m_encoderWrist,
                                rev::SparkMaxRelativeEncoder m_encoderIntake,
                                rev::SparkMaxRelativeEncoder m_encoderIAssist,
                                rev::SparkMaxRelativeEncoder m_encoderUnderbelly,
                                rev::SparkMaxRelativeEncoder m_encoderShooter1,
                                rev::SparkMaxRelativeEncoder m_encoderShooter2)
  {
  VsAmp_s_Sensors.b_Amp_ObjDetected = !LeENC_b_AMP_IntakeLimit;

  VsAmp_s_Sensors.b_ElevatorSwitch = !LeENC_b_AMP_ElevatorLimit;

  VsAmp_s_Sensors.Deg_Wrist = m_encoderWrist.GetPosition() * KeENC_k_AMP_WristRatio;

  VsAmp_s_Sensors.In_Elevator = m_encoderElevator.GetPosition() * KeENC_k_AMP_ElevatorRatio;

  VsAmp_s_Sensors.RPM_AmpRollers = m_encoderIntake.GetVelocity() * KeENC_k_AMP_IntakeRatio;

  VsSPK_s_Sensors.b_NoteDetected = !LeENC_b_SPK_IntakeLimit;

  VsSPK_s_Sensors.RPM_Intake = m_encoderUnderbelly.GetVelocity() * KeENC_k_SPK_IntakeRatio;

  VsSPK_s_Sensors.RPM_IAssist = m_encoderIAssist.GetVelocity() * KeENC_k_SPK_IAssistRatio;

  VsSPK_s_Sensors.RPM_Shooter1 = m_encoderShooter1.GetVelocity() * KeENC_k_SPK_Shooter1Ratio;

  VsSPK_s_Sensors.RPM_Shooter2 = m_encoderShooter2.GetVelocity() * KeENC_k_SPK_Shooter2Ratio;

  VsCLMR_s_Sensors.in_Left = m_encoderClimberLeft.GetPosition() * KeENC_k_CLMR_LeftRatio;

  VsCLMR_s_Sensors.in_Right = m_encoderClimberRight.GetPosition() * KeENC_k_CLMR_RightRatio;

  frc::SmartDashboard::PutBoolean("AMP Note Detected",    VsAmp_s_Sensors.b_Amp_ObjDetected);
  frc::SmartDashboard::PutBoolean("AMP Elevator Switch",  VsAmp_s_Sensors.b_ElevatorSwitch);
  frc::SmartDashboard::PutNumber("AMP Wrist Angle",       VsAmp_s_Sensors.Deg_Wrist);
  frc::SmartDashboard::PutNumber("AMP Elevator",          VsAmp_s_Sensors.In_Elevator);
  // frc::SmartDashboard::PutNumber("AMP Intake",            VsAmp_s_Sensors.RPM_AmpRollers);
  frc::SmartDashboard::PutBoolean("SPK Note Detected",    VsSPK_s_Sensors.b_NoteDetected);
  // frc::SmartDashboard::PutNumber("SPK Intake",            VsSPK_s_Sensors.RPM_Intake);
  frc::SmartDashboard::PutNumber("SPK Shooter1",          VsSPK_s_Sensors.RPM_Shooter1);
  frc::SmartDashboard::PutNumber("SPK Shooter2",          VsSPK_s_Sensors.RPM_Shooter2);
  // frc::SmartDashboard::PutNumber("CLRM Left",             VsCLMR_s_Sensors.in_Left);
  // frc::SmartDashboard::PutNumber("CLRM Right",            VsCLMR_s_Sensors.in_Right);
  }