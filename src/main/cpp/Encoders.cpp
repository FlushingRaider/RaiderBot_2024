/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561

  Converted to 2024 code: January 17, 2024
 */

#include "rev/CANSparkMax.h"
#include <frc/PowerDistribution.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Const.hpp"
#include "DataLogger.hpp"
#include "SpeakerCntrl.hpp"
#include "Amp.hpp"
#include "Climber.hpp"

#include <frc/AddressableLED.h>

double VaENC_Deg_WheelAngleConverted[E_RobotCornerSz]; // This is the wheel angle coming from the angle Encoder and processed to only be from 0 - 180
double VaENC_Deg_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in degrees
double VaENC_Rad_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in radians
double VaENC_Deg_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction

double VaENC_InS_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec
double VaENC_In_WheelDeltaDistance[E_RobotCornerSz]; // Distance wheel moved, loop to loop, in inches
double VaENC_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz]; // Current distance wheel moved, loop to loop, in Counts
double VaENC_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts
TsENC_PowerAboveThresh   VaENC_b_PowerAboveThreshold;
TsENC_MaxOutput          VaENC_MaxOutputValue;

TsENC_Power VsENC_RobotCurrentVoltPwr;

int firstPixelHue = 0;

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

   // log_encoders_wheelangleconverted.Append(VaENC_Deg_WheelAngleConverted);
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

   // log_encoders_wheelVelocity.Append(VaENC_InS_WheelVelocity);
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


/******************************************************************************
 * Function:     Encoders_AMP_SPK_CLMR_Run
 *
 * Description:  Read the encoders/sensors from the amp, speaker and climber
 ******************************************************************************/
// void MeasureVoltageCurrentPower(frc::PowerDistribution PDP)
void MeasureVoltageCurrentPower(double LeENC_I_PDH_Ch0,
                                double LeENC_I_PDH_Ch1,
                                double LeENC_I_PDH_Ch2,
                                double LeENC_I_PDH_Ch3,
                                double LeENC_I_PDH_Ch4,
                                double LeENC_I_PDH_Ch5,
                                double LeENC_I_PDH_Ch6,
                                double LeENC_I_PDH_Ch7,
                                double LeENC_I_PDH_Ch8,
                                double LeENC_I_PDH_Ch9,
                                double LeENC_I_PDH_Ch10,
                                double LeENC_I_PDH_Ch11,
                                double LeENC_I_PDH_Ch12,
                                double LeENC_I_PDH_Ch13,
                                double LeENC_I_PDH_Ch14,
                                double LeENC_I_PDH_Ch15,
                                double LeENC_I_PDH_Ch16,
                                double LeENC_I_PDH_Ch17,
                                double LeENC_I_PDH_Ch18,
                                double LeENC_I_PDH_Ch19,
                                double LeENC_I_PDH_Ch21,
                                double LeENC_I_PDH_Ch22,
                                double LeENC_T_PDH_Temp,
                                double LeENC_I_PDH_TotalCurrent,
                                double LeENC_W_PDH_TotalPower,
                                double LeENC_J_PDH_TotalEnergy,
                                double LeENC_V_PDH_Voltage)
  {
    VsENC_RobotCurrentVoltPwr.I_SD_Steer_FR      = LeENC_I_PDH_Ch8;
    VsENC_RobotCurrentVoltPwr.I_SD_Drive_FR      = LeENC_I_PDH_Ch9;
    VsENC_RobotCurrentVoltPwr.I_SD_Steer_FL      = LeENC_I_PDH_Ch0;
    VsENC_RobotCurrentVoltPwr.I_SD_Drive_FL      = LeENC_I_PDH_Ch1;
    VsENC_RobotCurrentVoltPwr.I_SD_Steer_RR      = LeENC_I_PDH_Ch13;  // Vv
    VsENC_RobotCurrentVoltPwr.I_SD_Drive_RR      = LeENC_I_PDH_Ch12; // vv
    VsENC_RobotCurrentVoltPwr.I_SD_Steer_RL      = LeENC_I_PDH_Ch18; // v
    VsENC_RobotCurrentVoltPwr.I_SD_Drive_RL      = LeENC_I_PDH_Ch17;  // v
    VsENC_RobotCurrentVoltPwr.I_CLMR_1           = LeENC_I_PDH_Ch14; // vv
    VsENC_RobotCurrentVoltPwr.I_CLMR_2           = LeENC_I_PDH_Ch19; // v
    VsENC_RobotCurrentVoltPwr.I_SPK_Intake       = LeENC_I_PDH_Ch11;
    VsENC_RobotCurrentVoltPwr.I_SPK_IAssist      = LeENC_I_PDH_Ch6;  // v
    VsENC_RobotCurrentVoltPwr.I_SPK_Shooter1     = LeENC_I_PDH_Ch3;
    VsENC_RobotCurrentVoltPwr.I_SPK_Shooter2     = LeENC_I_PDH_Ch10;
    VsENC_RobotCurrentVoltPwr.I_Amp_Elevator     = LeENC_I_PDH_Ch2; //vv
    VsENC_RobotCurrentVoltPwr.I_Amp_Wrist        = LeENC_I_PDH_Ch5; // ve
    VsENC_RobotCurrentVoltPwr.I_Amp_Intake       = LeENC_I_PDH_Ch4; // ve
    VsENC_RobotCurrentVoltPwr.I_VIS_Pi           = LeENC_I_PDH_Ch21; //ve  
    VsENC_RobotCurrentVoltPwr.I_SD_CanCoders     = LeENC_I_PDH_Ch22; // ve

    VsENC_RobotCurrentVoltPwr.T_PDH_Temp         = LeENC_T_PDH_Temp;
    VsENC_RobotCurrentVoltPwr.I_PDH_TotalCurrent = LeENC_I_PDH_TotalCurrent;
    VsENC_RobotCurrentVoltPwr.W_PDH_TotalPower   = LeENC_W_PDH_TotalPower;
    VsENC_RobotCurrentVoltPwr.J_PDH_TotalEnergy  = LeENC_J_PDH_TotalEnergy;
    VsENC_RobotCurrentVoltPwr.V_PDH_Voltage      = LeENC_V_PDH_Voltage;

    frc::SmartDashboard::PutNumber("I_SD_Steer_FR",       VsENC_RobotCurrentVoltPwr.I_SD_Steer_FR);
    frc::SmartDashboard::PutNumber("I_SD_Drive_FR",       VsENC_RobotCurrentVoltPwr.I_SD_Drive_FR);
    frc::SmartDashboard::PutNumber("I_SD_Steer_FL",       VsENC_RobotCurrentVoltPwr.I_SD_Steer_FL);
    frc::SmartDashboard::PutNumber("I_SD_Drive_FL",       VsENC_RobotCurrentVoltPwr.I_SD_Drive_FL);
    frc::SmartDashboard::PutNumber("I_SD_Steer_RR",       VsENC_RobotCurrentVoltPwr.I_SD_Steer_RR);
    frc::SmartDashboard::PutNumber("I_SD_Drive_RR",       VsENC_RobotCurrentVoltPwr.I_SD_Drive_RR);
    frc::SmartDashboard::PutNumber("I_SD_Steer_RL",       VsENC_RobotCurrentVoltPwr.I_SD_Steer_RL);
    frc::SmartDashboard::PutNumber("I_SD_Drive_RL",       VsENC_RobotCurrentVoltPwr.I_SD_Drive_RL);
    frc::SmartDashboard::PutNumber("I_CLMR_1",            VsENC_RobotCurrentVoltPwr.I_CLMR_1);
    frc::SmartDashboard::PutNumber("I_CLMR_2",            VsENC_RobotCurrentVoltPwr.I_CLMR_2);
    frc::SmartDashboard::PutNumber("I_SPK_Intake",        VsENC_RobotCurrentVoltPwr.I_SPK_Intake);
    frc::SmartDashboard::PutNumber("I_SPK_IAssist",       VsENC_RobotCurrentVoltPwr.I_SPK_IAssist);
    frc::SmartDashboard::PutNumber("I_SPK_Shooter1",      VsENC_RobotCurrentVoltPwr.I_SPK_Shooter1);
    frc::SmartDashboard::PutNumber("I_SPK_Shooter2",      VsENC_RobotCurrentVoltPwr.I_SPK_Shooter2);
    frc::SmartDashboard::PutNumber("I_Amp_Elevator",      VsENC_RobotCurrentVoltPwr.I_Amp_Elevator);
    frc::SmartDashboard::PutNumber("I_Amp_Wrist",         VsENC_RobotCurrentVoltPwr.I_Amp_Wrist);
    frc::SmartDashboard::PutNumber("I_Amp_Intake",        VsENC_RobotCurrentVoltPwr.I_Amp_Intake);
    frc::SmartDashboard::PutNumber("I_VIS_Pi",            VsENC_RobotCurrentVoltPwr.I_VIS_Pi);
    frc::SmartDashboard::PutNumber("T_PDH_Temp",          VsENC_RobotCurrentVoltPwr.T_PDH_Temp);
    frc::SmartDashboard::PutNumber("I_PDH_TotalCurrent",  VsENC_RobotCurrentVoltPwr.I_PDH_TotalCurrent);
    frc::SmartDashboard::PutNumber("W_PDH_TotalPower",    VsENC_RobotCurrentVoltPwr.W_PDH_TotalPower);
    frc::SmartDashboard::PutNumber("J_PDH_TotalEnergy",   VsENC_RobotCurrentVoltPwr.J_PDH_TotalEnergy);
    frc::SmartDashboard::PutNumber("V_PDH_Voltage",       VsENC_RobotCurrentVoltPwr.V_PDH_Voltage);

    // frc::SmartDashboard::PutNumber("I_CH0", LeENC_I_PDH_Ch0);
    // frc::SmartDashboard::PutNumber("I_CH1", LeENC_I_PDH_Ch1);
    // frc::SmartDashboard::PutNumber("I_CH2", LeENC_I_PDH_Ch2);
    // frc::SmartDashboard::PutNumber("I_CH3", LeENC_I_PDH_Ch3);
    // frc::SmartDashboard::PutNumber("I_CH4", LeENC_I_PDH_Ch4);
    // frc::SmartDashboard::PutNumber("I_CH5", LeENC_I_PDH_Ch5);
    // frc::SmartDashboard::PutNumber("I_CH6", LeENC_I_PDH_Ch6);
    // frc::SmartDashboard::PutNumber("I_CH7", LeENC_I_PDH_Ch7);
    // frc::SmartDashboard::PutNumber("I_CH8", LeENC_I_PDH_Ch8);
    // frc::SmartDashboard::PutNumber("I_CH9", LeENC_I_PDH_Ch9);
    // frc::SmartDashboard::PutNumber("I_CH10", LeENC_I_PDH_Ch10);
    // frc::SmartDashboard::PutNumber("I_CH11", LeENC_I_PDH_Ch11);
    // frc::SmartDashboard::PutNumber("I_CH12", LeENC_I_PDH_Ch12);
    // frc::SmartDashboard::PutNumber("I_CH13", LeENC_I_PDH_Ch13);
    // frc::SmartDashboard::PutNumber("I_CH14", LeENC_I_PDH_Ch14);
    // frc::SmartDashboard::PutNumber("I_CH15", LeENC_I_PDH_Ch15);
    // frc::SmartDashboard::PutNumber("I_CH16", LeENC_I_PDH_Ch16);
    // frc::SmartDashboard::PutNumber("I_CH17", LeENC_I_PDH_Ch17);
    // frc::SmartDashboard::PutNumber("I_CH18", LeENC_I_PDH_Ch18);
    // frc::SmartDashboard::PutNumber("I_CH19", LeENC_I_PDH_Ch19);
    // frc::SmartDashboard::PutNumber("I_CH21", LeENC_I_PDH_Ch21);
    // frc::SmartDashboard::PutNumber("I_CH22", LeENC_I_PDH_Ch22);

    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_FR >= K_SD_SteerMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Steer_FR = true;}
    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_FL >= K_SD_SteerMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Steer_FL = true;}
    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_RR >= K_SD_SteerMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Steer_RR = true;}
    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_RL >= K_SD_SteerMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Steer_RL = true;}

    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_FR >= K_SD_DriveMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Drive_FR = true;}
    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_FL >= K_SD_DriveMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Drive_FL = true;}
    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_RR >= K_SD_DriveMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Drive_RR = true;}
    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_RL >= K_SD_DriveMotorCurrentLimit) {VaENC_b_PowerAboveThreshold.b_SD_Drive_RL = true;}

    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_FR > VaENC_MaxOutputValue.I_SD_Steer_FR) {VaENC_MaxOutputValue.I_SD_Steer_FR = VsENC_RobotCurrentVoltPwr.I_SD_Steer_FR;}
    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_FR > VaENC_MaxOutputValue.I_SD_Drive_FR) {VaENC_MaxOutputValue.I_SD_Drive_FR = VsENC_RobotCurrentVoltPwr.I_SD_Drive_FR;} 
    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_FL > VaENC_MaxOutputValue.I_SD_Steer_FL) {VaENC_MaxOutputValue.I_SD_Steer_FL = VsENC_RobotCurrentVoltPwr.I_SD_Steer_FL;} 
    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_FL > VaENC_MaxOutputValue.I_SD_Drive_FL) {VaENC_MaxOutputValue.I_SD_Drive_FL = VsENC_RobotCurrentVoltPwr.I_SD_Drive_FL;} 
    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_RR > VaENC_MaxOutputValue.I_SD_Steer_RR) {VaENC_MaxOutputValue.I_SD_Steer_RR = VsENC_RobotCurrentVoltPwr.I_SD_Steer_RR;} 
    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_RR > VaENC_MaxOutputValue.I_SD_Drive_RR) {VaENC_MaxOutputValue.I_SD_Drive_RR = VsENC_RobotCurrentVoltPwr.I_SD_Drive_RR;} 
    if (VsENC_RobotCurrentVoltPwr.I_SD_Steer_RL > VaENC_MaxOutputValue.I_SD_Steer_RL) {VaENC_MaxOutputValue.I_SD_Steer_RL = VsENC_RobotCurrentVoltPwr.I_SD_Steer_RL;} 
    if (VsENC_RobotCurrentVoltPwr.I_SD_Drive_RL > VaENC_MaxOutputValue.I_SD_Drive_RL) {VaENC_MaxOutputValue.I_SD_Drive_RL = VsENC_RobotCurrentVoltPwr.I_SD_Drive_RL;} 
    if (VsENC_RobotCurrentVoltPwr.I_CLMR_1 > VaENC_MaxOutputValue.I_CLMR_1) {VaENC_MaxOutputValue.I_CLMR_1 = VsENC_RobotCurrentVoltPwr.I_CLMR_1;} 
    if (VsENC_RobotCurrentVoltPwr.I_CLMR_2 > VaENC_MaxOutputValue.I_CLMR_2) {VaENC_MaxOutputValue.I_CLMR_2 = VsENC_RobotCurrentVoltPwr.I_CLMR_2;} 
    if (VsENC_RobotCurrentVoltPwr.I_SPK_Intake > VaENC_MaxOutputValue.I_SPK_Intake) {VaENC_MaxOutputValue.I_SPK_Intake = VsENC_RobotCurrentVoltPwr.I_SPK_Intake;}
    if (VsENC_RobotCurrentVoltPwr.I_SPK_IAssist > VaENC_MaxOutputValue.I_SPK_IAssist) {VaENC_MaxOutputValue.I_SPK_IAssist = VsENC_RobotCurrentVoltPwr.I_SPK_IAssist;} 
    if (VsENC_RobotCurrentVoltPwr.I_SPK_Shooter1 > VaENC_MaxOutputValue.I_SPK_Shooter1) {VaENC_MaxOutputValue.I_SPK_Shooter1 = VsENC_RobotCurrentVoltPwr.I_SPK_Shooter1;} 
    if (VsENC_RobotCurrentVoltPwr.I_SPK_Shooter2 > VaENC_MaxOutputValue.I_SPK_Shooter2) {VaENC_MaxOutputValue.I_SPK_Shooter2 = VsENC_RobotCurrentVoltPwr.I_SPK_Shooter2;} 
    if (VsENC_RobotCurrentVoltPwr.I_Amp_Elevator > VaENC_MaxOutputValue.I_Amp_Elevator) {VaENC_MaxOutputValue.I_Amp_Elevator = VsENC_RobotCurrentVoltPwr.I_Amp_Elevator;} 
    if (VsENC_RobotCurrentVoltPwr.I_Amp_Wrist > VaENC_MaxOutputValue.I_Amp_Wrist) {VaENC_MaxOutputValue.I_Amp_Wrist = VsENC_RobotCurrentVoltPwr.I_Amp_Wrist;} 
    if (VsENC_RobotCurrentVoltPwr.I_Amp_Intake > VaENC_MaxOutputValue.I_Amp_Intake) {VaENC_MaxOutputValue.I_Amp_Intake = VsENC_RobotCurrentVoltPwr.I_Amp_Intake;} 
    if (VsENC_RobotCurrentVoltPwr.I_VIS_Pi > VaENC_MaxOutputValue.I_VIS_Pi) {VaENC_MaxOutputValue.I_VIS_Pi = VsENC_RobotCurrentVoltPwr.I_VIS_Pi;} 
    if (VsENC_RobotCurrentVoltPwr.T_PDH_Temp > VaENC_MaxOutputValue.T_PDH_Temp) {VaENC_MaxOutputValue.T_PDH_Temp = VsENC_RobotCurrentVoltPwr.T_PDH_Temp;} 
    if (VsENC_RobotCurrentVoltPwr.I_PDH_TotalCurrent > VaENC_MaxOutputValue.I_PDH_TotalCurrent) {VaENC_MaxOutputValue.I_PDH_TotalCurrent = VsENC_RobotCurrentVoltPwr.I_PDH_TotalCurrent;} 
    if (VsENC_RobotCurrentVoltPwr.W_PDH_TotalPower > VaENC_MaxOutputValue.W_PDH_TotalPower) {VaENC_MaxOutputValue.W_PDH_TotalPower = VsENC_RobotCurrentVoltPwr.W_PDH_TotalPower;} 
    if (VsENC_RobotCurrentVoltPwr.J_PDH_TotalEnergy > VaENC_MaxOutputValue.J_PDH_TotalEnergy) {VaENC_MaxOutputValue.J_PDH_TotalEnergy = VsENC_RobotCurrentVoltPwr.J_PDH_TotalEnergy;} 
    if (VsENC_RobotCurrentVoltPwr.V_PDH_Voltage < VaENC_MaxOutputValue.V_PDH_Voltage) {VaENC_MaxOutputValue.V_PDH_Voltage = VsENC_RobotCurrentVoltPwr.V_PDH_Voltage;} 
    if (VsENC_RobotCurrentVoltPwr.I_SD_CanCoders > VaENC_MaxOutputValue.I_SD_CanCoders) {VaENC_MaxOutputValue.I_SD_CanCoders = VsENC_RobotCurrentVoltPwr.I_SD_CanCoders;} 
  

    frc::SmartDashboard::PutBoolean("VaENC_b_PowerAboveThreshold.b_SD_Drive_FR", VaENC_b_PowerAboveThreshold.b_SD_Drive_FR);
    frc::SmartDashboard::PutBoolean("VaENC_b_PowerAboveThreshold.b_SD_Drive_FL", VaENC_b_PowerAboveThreshold.b_SD_Drive_FL);
    frc::SmartDashboard::PutBoolean("VaENC_b_PowerAboveThreshold.b_SD_Drive_RR", VaENC_b_PowerAboveThreshold.b_SD_Drive_RR);
    frc::SmartDashboard::PutBoolean("VaENC_b_PowerAboveThreshold.b_SD_Drive_RL", VaENC_b_PowerAboveThreshold.b_SD_Drive_RL);

    frc::SmartDashboard::PutNumber("VsENC_RobotCurrentVoltPwr.I_SD_Drive_FR", VsENC_RobotCurrentVoltPwr.I_SD_Drive_FR);
    frc::SmartDashboard::PutNumber("VsENC_RobotCurrentVoltPwr.I_SD_Drive_FL", VsENC_RobotCurrentVoltPwr.I_SD_Drive_FL);
    frc::SmartDashboard::PutNumber("VsENC_RobotCurrentVoltPwr.I_SD_Drive_RR", VsENC_RobotCurrentVoltPwr.I_SD_Drive_RR);
    frc::SmartDashboard::PutNumber("VsENC_RobotCurrentVoltPwr.I_SD_Drive_RL", VsENC_RobotCurrentVoltPwr.I_SD_Drive_RL);

    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Steer_FR" ,VaENC_MaxOutputValue.I_SD_Steer_FR);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Drive_FR" ,VaENC_MaxOutputValue.I_SD_Drive_FR);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Steer_FL" ,VaENC_MaxOutputValue.I_SD_Steer_FL);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Drive_FL" ,VaENC_MaxOutputValue.I_SD_Drive_FL);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Steer_RR" ,VaENC_MaxOutputValue.I_SD_Steer_RR);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Drive_RR" ,VaENC_MaxOutputValue.I_SD_Drive_RR);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Steer_RL" ,VaENC_MaxOutputValue.I_SD_Steer_RL);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_Drive_RL" ,VaENC_MaxOutputValue.I_SD_Drive_RL);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_CLMR_1"      ,VaENC_MaxOutputValue.I_CLMR_1);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_CLMR_2"      ,VaENC_MaxOutputValue.I_CLMR_2);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SPK_Intake"  ,VaENC_MaxOutputValue.I_SPK_Intake );
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SPK_IAssist" ,VaENC_MaxOutputValue.I_SPK_IAssist);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SPK_Shooter1",VaENC_MaxOutputValue.I_SPK_Shooter1);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SPK_Shooter2",VaENC_MaxOutputValue.I_SPK_Shooter2);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_Amp_Elevator",VaENC_MaxOutputValue.I_Amp_Elevator);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_Amp_Wrist"   ,VaENC_MaxOutputValue.I_Amp_Wrist );
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_Amp_Intake"  ,VaENC_MaxOutputValue.I_Amp_Intake);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_VIS_Pi"      ,VaENC_MaxOutputValue.I_VIS_Pi);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.T_PDH_Temp"    ,VaENC_MaxOutputValue.T_PDH_Temp );
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_PDH_TotalCurrent",VaENC_MaxOutputValue.I_PDH_TotalCurrent);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.W_PDH_TotalPower"  ,VaENC_MaxOutputValue.W_PDH_TotalPower );
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.J_PDH_TotalEnergy" ,VaENC_MaxOutputValue.J_PDH_TotalEnergy);
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.V_PDH_Voltage"     ,VaENC_MaxOutputValue.V_PDH_Voltage );
    frc::SmartDashboard::PutNumber("VaENC_MaxOutputValue.I_SD_CanCoders"    ,VaENC_MaxOutputValue.I_SD_CanCoders);




  }

void CurrentLatchReset(){
  VaENC_b_PowerAboveThreshold.b_SD_Steer_FR = false;
  VaENC_b_PowerAboveThreshold.b_SD_Drive_FR = false;
  VaENC_b_PowerAboveThreshold.b_SD_Steer_FL = false;
  VaENC_b_PowerAboveThreshold.b_SD_Drive_FL = false;
  VaENC_b_PowerAboveThreshold.b_SD_Steer_RR = false;
  VaENC_b_PowerAboveThreshold.b_SD_Drive_RR = false;
  VaENC_b_PowerAboveThreshold.b_SD_Steer_RL = false;
  VaENC_b_PowerAboveThreshold.b_SD_Drive_RL = false;
  VaENC_b_PowerAboveThreshold.b_CLMR_1 = false;
  VaENC_b_PowerAboveThreshold.b_CLMR_2 = false;
  VaENC_b_PowerAboveThreshold.b_SPK_Intake = false;
  VaENC_b_PowerAboveThreshold.b_SPK_IAssist = false;
  VaENC_b_PowerAboveThreshold.b_SPK_Shooter1 = false;
  VaENC_b_PowerAboveThreshold.b_SPK_Shooter2 = false;
  VaENC_b_PowerAboveThreshold.b_Amp_Elevator = false;
  VaENC_b_PowerAboveThreshold.b_Amp_Wrist = false;
  VaENC_b_PowerAboveThreshold.b_Amp_Intake = false;
  VaENC_b_PowerAboveThreshold.b_VIS_Pi = false;
  VaENC_b_PowerAboveThreshold.b_PDH_Temp = false;
  VaENC_b_PowerAboveThreshold.b_PDH_TotalCurrent = false;
  VaENC_b_PowerAboveThreshold.b_PDH_TotalPower = false;
  VaENC_b_PowerAboveThreshold.b_PDH_TotalEnergy = false;
  VaENC_b_PowerAboveThreshold.b_PDH_Voltage = false;
  VaENC_b_PowerAboveThreshold.b_SD_CanCoders = false;


  VaENC_MaxOutputValue.I_SD_Steer_FR = 0.0;
  VaENC_MaxOutputValue.I_SD_Drive_FR = 0.0;
  VaENC_MaxOutputValue.I_SD_Steer_FL = 0.0;
  VaENC_MaxOutputValue.I_SD_Drive_FL = 0.0;
  VaENC_MaxOutputValue.I_SD_Steer_RR = 0.0;
  VaENC_MaxOutputValue.I_SD_Drive_RR = 0.0;
  VaENC_MaxOutputValue.I_SD_Steer_RL = 0.0;
  VaENC_MaxOutputValue.I_SD_Drive_RL = 0.0;
  VaENC_MaxOutputValue.I_CLMR_1 = 0.0;
  VaENC_MaxOutputValue.I_CLMR_2 = 0.0;
  VaENC_MaxOutputValue.I_SPK_Intake = 0.0;
  VaENC_MaxOutputValue.I_SPK_IAssist = 0.0;
  VaENC_MaxOutputValue.I_SPK_Shooter1 = 0.0;
  VaENC_MaxOutputValue.I_SPK_Shooter2 = 0.0;
  VaENC_MaxOutputValue.I_Amp_Elevator = 0.0;
  VaENC_MaxOutputValue.I_Amp_Wrist = 0.0;
  VaENC_MaxOutputValue.I_Amp_Intake = 0.0;
  VaENC_MaxOutputValue.I_VIS_Pi = 0.0;
  VaENC_MaxOutputValue.T_PDH_Temp = 0.0;
  VaENC_MaxOutputValue.I_PDH_TotalCurrent = 0.0;
  VaENC_MaxOutputValue.W_PDH_TotalPower = 0.0;
  VaENC_MaxOutputValue.J_PDH_TotalEnergy = 0.0;
  VaENC_MaxOutputValue.V_PDH_Voltage = 0.0;
  VaENC_MaxOutputValue.I_SD_CanCoders = 0.0;
  }

std::array<frc::AddressableLED::LEDData, C_LedLength> LightControl(
    TsENC_LightPatterns L_pattern)
{

  std::array<frc::AddressableLED::LEDData, C_LedLength> m_ledBuffer;
  switch (L_pattern)
  {
    case E_LED_RAINBOW:
      // For every pixel
      for (int i = 0; i < C_LedLength; i++)
      {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        int pixelHue = (firstPixelHue + (i * 180 / C_LedLength)) % 180;
        // Set the value
        m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      firstPixelHue += 3;
      // Check bounds
      firstPixelHue %= 180;
    
    case E_LED_SOLIDWHITE:
      for (int i = 0; i < C_LedLength; i++){
        m_ledBuffer[i].SetHSV(180, 255, 128);
      }

  }
  return (m_ledBuffer);
}