/*
  DriveControl.cpp

  Created on: Feb 25, 2020
  Author: 5561

  Contains the calculations and controls for swerve drive ("_SD_").

  Changes:
  2021-02-25 -> Updates to help the robot drive straight
  2022-03-08 -> Resolved rotation and directionality issues
  2022-03-09 -> Added robot oritented mode

  Coverted to 2024 code: January 17,2024
 */

#include "rev/CANSparkMax.h"
#include <math.h>
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "shuffleboard.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

double VeDRC_Deg_AutoCorrectDesired;        // Saved robot orientation angle used for auto correct
double VeDRC_Deg_AutoCorrectionError;       // Error value for auto correction PID control.
double VeDRC_k_AutoCorrectionIntegral;      // Integral value for auto correction PID control.

double VaDRC_Deg_WheelAngleError[E_RobotCornerSz];  // Error value for PID control.
double VaDRC_k_WheelAngleIntegral[E_RobotCornerSz]; // Integral value for PID control.
double VaDRC_Deg_WheelAngleArb[E_RobotCornerSz];    // This is the arbitrated wheel angle that is used in the PID controller
double VaDRC_Pct_WheelAngleCmnd[E_RobotCornerSz];   // Command sent to motor controller.  Command is power based, ranges from -1 to 1.

double VaDRC_RPM_WheelSpeedCmnd[E_RobotCornerSz];     // Command sent to motor controller   Command is either in power or speed request.
double VaDRC_RPM_WheelSpeedCmndPrev[E_RobotCornerSz]; // Previous wheel speed command.  Used for ramping of control.
bool   VeDRC_b_DriveWheelsInPID = false;              // Flag indicating that PID control is currently active in the motor controller.

double KV_SD_WheelAnglePID_Gx[E_PID_CalSz];
double KV_SD_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz];
double KV_SD_WheelSpeedRampRate = 0;
double KV_SD_WheelGx[E_RobotCornerSz];

double VeENC_k_ChosenGain = 0;
bool VeENC_b_LowPowerMode = false;

/******************************************************************************
 * Function:     SwerveDriveMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the swerve drive motors.
 *               - Steer (power cmnd only)
 *               - Drive (PID Control in motor controller)
 ******************************************************************************/
void SwerveDriveMotorConfigsInit(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                 rev::SparkMaxPIDController m_frontRightDrivePID,
                                 rev::SparkMaxPIDController m_rearLeftDrivePID,
                                 rev::SparkMaxPIDController m_rearRightDrivePID)
  {
  T_PID_SparkMaxCal L_Index  = E_kP;
  T_PID_Cal         L_Index2 = E_P_Gx;
  T_RobotCorner     L_Index3 = E_FrontLeft;

  #ifndef PID_Calibrate
  //This set of PID assignments are for the hard coded values
  // set PID coefficients
  m_frontLeftDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_frontLeftDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_frontLeftDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_frontLeftDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_frontLeftDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_frontLeftDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_frontRightDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_frontRightDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_frontRightDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_frontRightDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_frontRightDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_frontRightDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_rearLeftDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_rearLeftDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_rearLeftDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_rearLeftDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_rearLeftDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_rearLeftDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);

  m_rearRightDrivePID.SetP(K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  m_rearRightDrivePID.SetI(K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  m_rearRightDrivePID.SetD(K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  m_rearRightDrivePID.SetIZone(K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  m_rearRightDrivePID.SetFF(K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  m_rearRightDrivePID.SetOutputRange(K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput], K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);
  #endif
  
  #ifdef PID_Calibrate

  m_frontLeftDrivePID.SetP(shuffleboard_FrontleftPID.P);
  m_frontLeftDrivePID.SetI(shuffleboard_FrontleftPID.I);
  m_frontLeftDrivePID.SetD(shuffleboard_FrontleftPID.D);
  m_frontLeftDrivePID.SetIZone(shuffleboard_FrontleftPID.Iz);
  m_frontLeftDrivePID.SetFF(shuffleboard_FrontleftPID.FF);
  m_frontLeftDrivePID.SetOutputRange(shuffleboard_FrontleftPID.LL, shuffleboard_FrontleftPID.UL);


  #endif

  KV_SD_WheelSpeedRampRate = K_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc];

  for (L_Index2 = E_P_Gx;
       L_Index2 < E_PID_CalSz;
       L_Index2 = T_PID_Cal(int(L_Index2) + 1))
      {
      KV_SD_WheelAnglePID_Gx[L_Index2] = K_SD_WheelAnglePID_Gx[L_Index2]; //for 2023 Compbot
      }

  for (L_Index3 = E_FrontLeft;
       L_Index3 < E_RobotCornerSz;
       L_Index3 = T_RobotCorner(int(L_Index3) + 1))
      {
      KV_SD_WheelGx[L_Index3] = K_SD_WheelGx[L_Index3] * K_ENC_WheelAngleGx;
      }

  #ifdef DriveMotorTest
  for (L_Index = E_kP;
       L_Index < E_PID_SparkMaxCalSz;
       L_Index = T_PID_SparkMaxCal(int(L_Index) + 1))
      {
      KV_SD_WheelSpeedPID_V2_Gx[L_Index] = K_SD_WheelSpeedPID_V2_Gx[L_Index];
      }
  
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", K_SD_WheelSpeedPID_V2_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain", K_SD_WheelSpeedPID_V2_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain", K_SD_WheelSpeedPID_V2_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone", K_SD_WheelSpeedPID_V2_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Feed Forward", K_SD_WheelSpeedPID_V2_Gx[E_kFF]);
  frc::SmartDashboard::PutNumber("Max Output", K_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output", K_SD_WheelSpeedPID_V2_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("Max Velocity", K_SD_WheelSpeedPID_V2_Gx[E_kMaxVel]);
  frc::SmartDashboard::PutNumber("Min Velocity", K_SD_WheelSpeedPID_V2_Gx[E_kMinVel]);
  frc::SmartDashboard::PutNumber("Max Acceleration", K_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc]);
  frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", K_SD_WheelSpeedPID_V2_Gx[E_kAllErr]);

  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_FrontLeft]", KV_SD_WheelGx[E_FrontLeft]);
  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_FrontRight]", KV_SD_WheelGx[E_FrontRight]);
  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_RearLeft]", KV_SD_WheelGx[E_RearLeft]);
  frc::SmartDashboard::PutNumber("KV_SD_WheelGx[E_RearRight]", KV_SD_WheelGx[E_RearRight]);
  #endif
  #ifdef WheelAngleTest
  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain", KV_SD_WheelAnglePID_Gx[E_P_Gx]);
  frc::SmartDashboard::PutNumber("I Gain", KV_SD_WheelAnglePID_Gx[E_I_Gx]);
  frc::SmartDashboard::PutNumber("D Gain", KV_SD_WheelAnglePID_Gx[E_D_Gx]);
  frc::SmartDashboard::PutNumber("P Upper Limit", KV_SD_WheelAnglePID_Gx[E_P_Ul]);
  frc::SmartDashboard::PutNumber("P Lower Limit", KV_SD_WheelAnglePID_Gx[E_P_Ll]);

  frc::SmartDashboard::PutNumber("I Upper Limit", KV_SD_WheelAnglePID_Gx[E_D_Ul]);
  frc::SmartDashboard::PutNumber("I Lower Limit", KV_SD_WheelAnglePID_Gx[E_D_Ll]);

  frc::SmartDashboard::PutNumber("D Upper Limit", KV_SD_WheelAnglePID_Gx[E_D_Ul]);
  frc::SmartDashboard::PutNumber("D Lower Limit", KV_SD_WheelAnglePID_Gx[E_D_Ll]);

  frc::SmartDashboard::PutNumber("Max UL", KV_SD_WheelAnglePID_Gx[E_Max_Ul]);
  frc::SmartDashboard::PutNumber("Max LL", KV_SD_WheelAnglePID_Gx[E_Max_Ll]);
  frc::SmartDashboard::PutNumber("Max Output", KV_SD_WheelAnglePID_Gx[E_Max_Ul]);
  frc::SmartDashboard::PutNumber("Min Output", KV_SD_WheelAnglePID_Gx[E_Max_Ll]);
  #endif
  }

  /******************************************************************************
 * Function:     SwerveDriveMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the swerve drive motors.
 *               - Steer (power cmnd only)
 *               - Drive (PID Control in motor controller)
 ******************************************************************************/
void SwerveDriveMotorConfigsCal(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                rev::SparkMaxPIDController m_frontRightDrivePID,
                                rev::SparkMaxPIDController m_rearLeftDrivePID,
                                rev::SparkMaxPIDController m_rearRightDrivePID)
  {
  // read PID coefficients from SmartDashboard
  #ifdef DriveMotorTest
  double L_p = frc::SmartDashboard::GetNumber("P Gain", 0);
  double L_i = frc::SmartDashboard::GetNumber("I Gain", 0);
  double L_d = frc::SmartDashboard::GetNumber("D Gain", 0);
  double L_iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  double L_ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  double L_max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double L_min = frc::SmartDashboard::GetNumber("Min Output", 0);
  double L_maxV = frc::SmartDashboard::GetNumber("Max Velocity", 0);
  double L_minV = frc::SmartDashboard::GetNumber("Min Velocity", 0);
  double L_maxA = frc::SmartDashboard::GetNumber("Max Acceleration", 0);
  double L_allE = frc::SmartDashboard::GetNumber("Allowed Closed Loop Error", 0);

  KV_SD_WheelGx[E_FrontLeft] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_FrontLeft]", KV_SD_WheelGx[E_FrontLeft]);
  KV_SD_WheelGx[E_FrontRight] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_FrontRight]", KV_SD_WheelGx[E_FrontRight]);
  KV_SD_WheelGx[E_RearLeft] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_RearLeft]", KV_SD_WheelGx[E_RearLeft]);
  KV_SD_WheelGx[E_RearRight] = frc::SmartDashboard::GetNumber("KV_SD_WheelGx[E_RearRight]", KV_SD_WheelGx[E_RearRight]);

  if((L_p != KV_SD_WheelSpeedPID_V2_Gx[E_kP]))   { m_frontLeftDrivePID.SetP(L_p); m_frontRightDrivePID.SetP(L_p); m_rearLeftDrivePID.SetP(L_p); m_rearRightDrivePID.SetP(L_p); KV_SD_WheelSpeedPID_V2_Gx[E_kP] = L_p; }
  if((L_i != KV_SD_WheelSpeedPID_V2_Gx[E_kI]))   { m_frontLeftDrivePID.SetI(L_i); m_frontRightDrivePID.SetI(L_i); m_rearLeftDrivePID.SetI(L_i); m_rearRightDrivePID.SetI(L_i); KV_SD_WheelSpeedPID_V2_Gx[E_kI] = L_i; }
  if((L_d != KV_SD_WheelSpeedPID_V2_Gx[E_kD]))   { m_frontLeftDrivePID.SetD(L_d); m_frontRightDrivePID.SetD(L_d); m_rearLeftDrivePID.SetD(L_d); m_rearRightDrivePID.SetD(L_d); KV_SD_WheelSpeedPID_V2_Gx[E_kD] = L_d; }
  if((L_iz != KV_SD_WheelSpeedPID_V2_Gx[E_kIz])) { m_frontLeftDrivePID.SetIZone(L_iz); m_frontRightDrivePID.SetIZone(L_iz); m_rearLeftDrivePID.SetIZone(L_iz); m_rearRightDrivePID.SetIZone(L_iz); KV_SD_WheelSpeedPID_V2_Gx[E_kIz] = L_iz; }
  if((L_ff != KV_SD_WheelSpeedPID_V2_Gx[E_kFF])) { m_frontLeftDrivePID.SetFF(L_ff); m_frontRightDrivePID.SetFF(L_ff); m_rearLeftDrivePID.SetFF(L_ff); m_rearRightDrivePID.SetFF(L_ff); KV_SD_WheelSpeedPID_V2_Gx[E_kFF] = L_ff; }
  if((L_max != KV_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput]) || (L_min != KV_SD_WheelSpeedPID_V2_Gx[E_kMinOutput])) { m_frontLeftDrivePID.SetOutputRange(L_min, L_max); m_frontRightDrivePID.SetOutputRange(L_min, L_max);  m_rearLeftDrivePID.SetOutputRange(L_min, L_max); m_rearRightDrivePID.SetOutputRange(L_min, L_max); KV_SD_WheelSpeedPID_V2_Gx[E_kMinOutput] = L_min; KV_SD_WheelSpeedPID_V2_Gx[E_kMaxOutput] = L_max; }
  if((L_maxV != KV_SD_WheelSpeedPID_V2_Gx[E_kMaxVel])) { KV_SD_WheelSpeedPID_V2_Gx[E_kMaxVel] = L_maxV; }
  // if((L_minV != KV_SD_WheelSpeedPID_V2_Gx[E_kMinVel])) { m_frontLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_frontRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearLeftDrivePID.SetSmartMotionMinOutputVelocity(L_minV); m_rearRightDrivePID.SetSmartMotionMinOutputVelocity(L_minV); KV_SD_WheelSpeedPID_V2_Gx[E_kMinVel] = L_minV; }
  if((L_maxA != KV_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc])) { KV_SD_WheelSpeedRampRate = L_maxA; KV_SD_WheelSpeedPID_V2_Gx[E_kMaxAcc] = L_maxA; }
  // if((L_allE != KV_SD_WheelSpeedPID_V2_Gx[E_kAllErr])) { m_frontLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_frontRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearLeftDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); m_rearRightDrivePID.SetSmartMotionAllowedClosedLoopError(L_allE); KV_SD_WheelSpeedPID_V2_Gx[E_kAllErr] = L_allE; }
  #endif
  #ifdef WheelAngleTest
  KV_SD_WheelAnglePID_Gx[E_P_Gx] = frc::SmartDashboard::GetNumber("P Gain", 0);
  KV_SD_WheelAnglePID_Gx[E_I_Gx] = frc::SmartDashboard::GetNumber("I Gain", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Gx] = frc::SmartDashboard::GetNumber("D Gain", 0);
  KV_SD_WheelAnglePID_Gx[E_P_Ul] = frc::SmartDashboard::GetNumber("P Upper Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_P_Ll] = frc::SmartDashboard::GetNumber("P Lower Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ul] = frc::SmartDashboard::GetNumber("I Upper Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ll] = frc::SmartDashboard::GetNumber("I Lower Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ul] = frc::SmartDashboard::GetNumber("D Upper Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_D_Ll] = frc::SmartDashboard::GetNumber("D Lower Limit", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ul] = frc::SmartDashboard::GetNumber("Max UL", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ll] = frc::SmartDashboard::GetNumber("Max LL", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ul] = frc::SmartDashboard::GetNumber("Max Output", 0);
  KV_SD_WheelAnglePID_Gx[E_Max_Ll] = frc::SmartDashboard::GetNumber("Min Output", 0);
  #endif
  }

  /******************************************************************************
 * Function:     DriveControlInit
 *
 * Description:  Initialization function for the drive control.
 ******************************************************************************/
void DriveControlInit()
  {
  T_RobotCorner LeDRC_e_Index;

  for (LeDRC_e_Index = E_FrontLeft;
       LeDRC_e_Index < E_RobotCornerSz;
       LeDRC_e_Index = T_RobotCorner(int(LeDRC_e_Index) + 1))
    {
    VaDRC_Deg_WheelAngleError[LeDRC_e_Index] = 0;
    VaDRC_k_WheelAngleIntegral[LeDRC_e_Index] = 0;
    VaDRC_Deg_WheelAngleArb[LeDRC_e_Index] = 0;
    VaDRC_RPM_WheelSpeedCmndPrev[LeDRC_e_Index] = 0; 
    }

  VeDRC_b_DriveWheelsInPID = false;

  VeDRC_Deg_AutoCorrectDesired = 0;

  VeDRC_Deg_AutoCorrectionError = 0;
  VeDRC_k_AutoCorrectionIntegral = 0;
  }


/******************************************************************************
 * Function:     DtrmnEncoderRelativeToCmnd
 *
 * Description:  Account for the roll over of the enocoders
 ******************************************************************************/
double DtrmnEncoderRelativeToCmnd(double          L_JoystickCmnd,
                                  double          L_EncoderReading)
  {
  double LeSD_Deg_ActualDelta;
  double LeSD_Deg_PosDelta;
  double LeSD_Deg_NegDelta;
  double LeSD_Deg_Output;

  LeSD_Deg_ActualDelta = fabs(L_JoystickCmnd - L_EncoderReading);
  LeSD_Deg_PosDelta = fabs(L_JoystickCmnd - (L_EncoderReading + 360));
  LeSD_Deg_NegDelta = fabs(L_JoystickCmnd - (L_EncoderReading - 360));

   if((LeSD_Deg_ActualDelta < LeSD_Deg_PosDelta) && (LeSD_Deg_ActualDelta < LeSD_Deg_NegDelta))
    {
    LeSD_Deg_Output = L_EncoderReading;
    }
  else if ((LeSD_Deg_PosDelta < LeSD_Deg_ActualDelta) && (LeSD_Deg_PosDelta < LeSD_Deg_NegDelta))
    {
    LeSD_Deg_Output = L_EncoderReading + 360;
    }
  else
    {
    LeSD_Deg_Output = L_EncoderReading - 360;
    }

  return (LeSD_Deg_Output);
  }


/************************************************************************************************************************
 * Function:     DriveControlMain
 *
 * Description:  Main calling function for the drive control.
 *               Swerve srive calculations obtained from the following link:
 *               https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf
 ************************************************************************************************************************/
void DriveControlMain(double                   L_JoyStick1Axis1Y,  // swerve control forward/back
                      double                   L_JoyStick1Axis1X,  // swerve control strafe
                      double                   L_JoyStick1Axis2X,  // rotate the robot joystick
                      double                   L_JoyStick1Axis3,   // extra speed trigger
                      bool                     LeDRC_b_RotateTo0, // auto rotate to 0 degrees
                      bool                     LeDRC_b_RotateTo180, // auto rotate to 180 degrees
                      bool                     L_JoyStick1_ResetDesiredAngle, // auto correct reset angle
                      bool                     LeDRC_b_X_ModeReq,  // When requested, move all wheels into an X configuration, meant to hold robot still
                      bool                     LeDRC_b_X_ModeReqTeleop,
                      T_RobotState             L_ROBO_e_RobotState,
                      T_ADAS_ActiveFeature     LeDRC_e_ADAS_ActiveFeature,
                      double                   L_ADAS_Pct_SD_FwdRev,
                      double                   L_ADAS_Pct_SD_Strafe,
                      double                   L_ADAS_Pct_SD_Rotate,
                      double                   LeADAS_Deg_SD_DesiredPose,
                      bool                     L_ADAS_SD_RobotOriented, 
                      double                   L_Deg_GyroAngle,
                      double                   L_Rad_GyroAngle,
                      double                  *L_Deg_WheelAngleFwd,
                      double                  *L_Deg_WheelAngleRev,
                      double                  *Le_RPM_SD_WheelSpeedCmnd,
                      double                  *L_k_SD_WheelAngleCmnd,
                      double                   L_V_PDPVoltage)
  {
  double        L_FWD = 0;
  double        L_STR = 0;
  double        L_RCW = 0;
  T_RobotCorner L_Index = E_FrontLeft;
  double        L_temp = 0;
  double        L_A = 0;
  double        L_B = 0;
  double        L_C = 0;
  double        L_D = 0;
  double        L_k_SD_Gain = 0;
  double        L_Max = 0;
  double        L_Deg_SD_WA_FWD = 0;
  double        L_Deg_SD_WA_FWD_Delta = 0;
  double        L_Deg_SD_WA_REV = 0;
  double        L_Deg_SD_WA_REV_Delta = 0;
  double        L_Deg_SD_WA[E_RobotCornerSz];
  double        L_RPM_SD_WS[E_RobotCornerSz];
  bool          Le_b_SD_DriveWheelsPowered = false;
  double        L_Deg_SD_AngleErrorRaw = 0;
  double        L_Deg_SD_AngleError = 0;
  double        Le_n_SD_Offset = 0;
  double        La_n_SD_Offset[E_RobotCornerSz];
  bool          Le_b_SD_Active = false;
  double        Le_k_SD_RotateCorrectionGx = 0;
  bool          LeDRC_b_X_ModeReqActive = false;
  bool          LeDRC_b_AutoCenterEnabled = true; // force autocorrect to always be enabled when driver is running or ADAS is NOT in robot oriented control
  double        LeDRC_k_SpeedGain = 0.0;
  double        L_k_ChosenWheelMax = 0.0;


  /* Scale the joysticks based on a calibratable lookup when in teleop: */
  if (LeDRC_e_ADAS_ActiveFeature > E_ADAS_Disabled)
    {
    /* ADAS is active, pass throught the commands: */
    L_FWD = -L_ADAS_Pct_SD_FwdRev;
    L_STR = L_ADAS_Pct_SD_Strafe;
    L_RCW = L_ADAS_Pct_SD_Rotate;
    LeDRC_b_X_ModeReqActive = LeDRC_b_X_ModeReq;

    if (L_ADAS_SD_RobotOriented == true)
      {
      L_Deg_GyroAngle = 0.0;
      L_Rad_GyroAngle = 0.0;
      LeDRC_b_AutoCenterEnabled = false;
      }
    LeDRC_k_SpeedGain = KeDRC_k_SD_AutonGain;
    }
  else
    {
    /* ADAS is disabled, use the driver joysticks */

    L_FWD = -L_JoyStick1Axis1Y;
    L_STR = L_JoyStick1Axis1X;
    L_RCW = L_JoyStick1Axis2X;
    LeDRC_b_X_ModeReqActive = LeDRC_b_X_ModeReqTeleop;
    LeDRC_k_SpeedGain = L_JoyStick1Axis3;
    if (fabs(L_STR) >= KeDRC_Pct_SD_StrafeDB)  // Temporary workaround till we can figure out why auto correct doesn't work in strafe
      {
      LeDRC_b_AutoCenterEnabled = false;
      }
    }

  /* Here, we are attempting to determine if the drive/ADAS is attempting to turn the robot.  If we are 
     attempting to rotate the robot, allow the "desired" angle to update to the current measured angle.  */
  if ((L_JoyStick1_ResetDesiredAngle == true) ||
      (fabs(L_RCW) >= K_SD_RotateDeadBand))
    {
    VeDRC_Deg_AutoCorrectDesired = L_Deg_GyroAngle;
    }
  else if (LeDRC_e_ADAS_ActiveFeature > E_ADAS_Disabled)
    {
    /* ADAS is active and not trying to command rotation, take the desired pose value. */
    VeDRC_Deg_AutoCorrectDesired = LeADAS_Deg_SD_DesiredPose;
    }
  else if (LeDRC_b_RotateTo0 == true)
    {
    VeDRC_Deg_AutoCorrectDesired = 0.0;
    }
  else if (LeDRC_b_RotateTo180 == true)
    {
    VeDRC_Deg_AutoCorrectDesired = 180;
    }
  
  if ((LeDRC_e_ADAS_ActiveFeature <= E_ADAS_Disabled) && 
      (fabs(L_FWD) >= K_SD_RotateDeadBand || fabs(L_STR) >= K_SD_RotateDeadBand || fabs(L_RCW) >= K_SD_RotateDeadBand))
    {
    Le_b_SD_Active = true;
    }
  else
    {
    VeDRC_Deg_AutoCorrectionError = 0;
    VeDRC_k_AutoCorrectionIntegral = 0;
    }

  L_Deg_SD_AngleErrorRaw = VeDRC_Deg_AutoCorrectDesired - L_Deg_GyroAngle;

  if (L_Deg_SD_AngleErrorRaw < -180)
    {
    L_Deg_SD_AngleError = L_Deg_SD_AngleErrorRaw + 360;
    }
  else if (L_Deg_SD_AngleErrorRaw > 180)
    {
    L_Deg_SD_AngleError = L_Deg_SD_AngleErrorRaw - 360;
    }
  else
    {
    L_Deg_SD_AngleError = L_Deg_SD_AngleErrorRaw;
    }

  Le_k_SD_RotateCorrectionGx =  Control_PID( L_Deg_SD_AngleError,
                                             0.0,
                                            &VeDRC_Deg_AutoCorrectionError,
                                            &VeDRC_k_AutoCorrectionIntegral,
                                             Ke_SD_AutoCorrectPID_Gx[E_P_Gx],
                                             Ke_SD_AutoCorrectPID_Gx[E_I_Gx],
                                             Ke_SD_AutoCorrectPID_Gx[E_D_Gx],
                                             Ke_SD_AutoCorrectPID_Gx[E_P_Ul],
                                             Ke_SD_AutoCorrectPID_Gx[E_P_Ll],
                                             Ke_SD_AutoCorrectPID_Gx[E_I_Ul],
                                             Ke_SD_AutoCorrectPID_Gx[E_I_Ll],
                                             Ke_SD_AutoCorrectPID_Gx[E_D_Ul],
                                             Ke_SD_AutoCorrectPID_Gx[E_D_Ll],
                                             Ke_SD_AutoCorrectPID_Gx[E_Max_Ul],
                                             Ke_SD_AutoCorrectPID_Gx[E_Max_Ll]);

  /* Swerve drive calculaltions: */
  L_temp =  L_FWD * cos(L_Rad_GyroAngle) + L_STR * sin(L_Rad_GyroAngle);
  L_STR  = -L_FWD * sin(L_Rad_GyroAngle) + L_STR * cos(L_Rad_GyroAngle);
  L_FWD  =  L_temp;

  //Ws1: fr, Ws2: fl, ws3: rl, ws4: rr
  L_A = L_STR - L_RCW * (C_SD_L/C_SD_R);
  L_B = L_STR + L_RCW * (C_SD_L/C_SD_R);
  L_C = L_FWD - L_RCW * (C_SD_W/C_SD_R);
  L_D = L_FWD + L_RCW * (C_SD_W/C_SD_R);

  if (LeDRC_b_X_ModeReqActive == false)
    {
      L_RPM_SD_WS[E_FrontRight] = pow((L_B * L_B + L_C * L_C), 0.5);
      L_RPM_SD_WS[E_FrontLeft]  = pow((L_B * L_B + L_D * L_D), 0.5);
      L_RPM_SD_WS[E_RearLeft]   = pow((L_A * L_A + L_D * L_D), 0.5);
      L_RPM_SD_WS[E_RearRight]  = pow((L_A * L_A + L_C * L_C), 0.5);
    
      L_Deg_SD_WA[E_FrontRight] = -atan2(L_B, L_C) *180/C_PI;
      L_Deg_SD_WA[E_FrontLeft]  = -atan2(L_B, L_D) *180/C_PI;
      L_Deg_SD_WA[E_RearLeft]   = -atan2(L_A, L_D) *180/C_PI;
      L_Deg_SD_WA[E_RearRight]  = -atan2(L_A, L_C) *180/C_PI;
    }
  else
    {
      /* X mode is being requested, force speeds to 0, move wheels into an X configuration:*/
      L_RPM_SD_WS[E_FrontRight] = 0;
      L_RPM_SD_WS[E_FrontLeft]  = 0;
      L_RPM_SD_WS[E_RearLeft]   = 0;
      L_RPM_SD_WS[E_RearRight]  = 0;
    
      L_Deg_SD_WA[E_FrontRight] = -45;
      L_Deg_SD_WA[E_FrontLeft]  = 45;
      L_Deg_SD_WA[E_RearLeft]   = -45;
      L_Deg_SD_WA[E_RearRight]  = 45;
    }


  /* Normalize everything to a max value of 1 for wheel speeds: */
  L_Max = L_RPM_SD_WS[E_FrontRight];

  if (L_RPM_SD_WS[E_FrontLeft] > L_Max)
    {
    L_Max = L_RPM_SD_WS[E_FrontLeft];
    }
  if (L_RPM_SD_WS[E_RearLeft] > L_Max)
    {
    L_Max = L_RPM_SD_WS[E_RearLeft];
    }
  if (L_RPM_SD_WS[E_RearRight] > L_Max)
    {
    L_Max = L_RPM_SD_WS[E_RearRight];
    }

  if (L_Max > 1)
    {
    L_RPM_SD_WS[E_FrontRight] /= L_Max;
    L_RPM_SD_WS[E_FrontLeft]  /= L_Max;
    L_RPM_SD_WS[E_RearLeft]   /= L_Max;
    L_RPM_SD_WS[E_RearRight]  /= L_Max;
    }

  /* Ok, now lets apply gains to the normalized wheel speeds to obtain the desired motor speed */
  L_k_SD_Gain = KeDRC_k_SD_MinGain;
    
  if (LeDRC_k_SpeedGain > L_k_SD_Gain)
    {
    /* Additional speed trigger from driver: */
    L_k_SD_Gain = LeDRC_k_SpeedGain;
    }


  if(L_V_PDPVoltage < 9.0){
    VeENC_k_ChosenGain = K_SD_LowPowerMaxGain;
    VeENC_b_LowPowerMode = true;
  }
  else if(L_V_PDPVoltage >= 12.0){
    VeENC_k_ChosenGain = K_SD_MaxGain;
    VeENC_b_LowPowerMode = false;
  }

  frc::SmartDashboard::PutBoolean("Low Power Gain", VeENC_b_LowPowerMode);


  if (L_k_SD_Gain >= VeENC_k_ChosenGain)
    {
    L_k_SD_Gain = VeENC_k_ChosenGain;  // ToDo: Bumped from 0.7 to 0.9 to try and speed up auton.  May need to modify for teleop.
    }

  if (L_ROBO_e_RobotState == E_Teleop){
    L_k_ChosenWheelMax = K_SD_TeleWheelMaxSpeed;
  }
  else{
    L_k_ChosenWheelMax = K_SD_AutonWheelMaxSpeed;
  }


  L_RPM_SD_WS[E_FrontRight] *= (L_k_ChosenWheelMax * (L_k_SD_Gain));
  L_RPM_SD_WS[E_FrontLeft]  *= (L_k_ChosenWheelMax * (L_k_SD_Gain));
  L_RPM_SD_WS[E_RearLeft]   *= (L_k_ChosenWheelMax * (L_k_SD_Gain));
  L_RPM_SD_WS[E_RearRight]  *= (L_k_ChosenWheelMax * (L_k_SD_Gain));

  /* Now we need to detrime if we want to keep the desired commanded angle or flip 180* and flip the direction of the wheel speed.  
     This is intended to find the quickest way to reach the commanded angle. */
  for (L_Index = E_FrontLeft;
       L_Index < E_RobotCornerSz;
       L_Index = T_RobotCorner(int(L_Index) + 1))
    {
    L_Deg_SD_WA_FWD = DtrmnEncoderRelativeToCmnd(L_Deg_SD_WA[L_Index],
                                                 L_Deg_WheelAngleFwd[L_Index]);

    L_Deg_SD_WA_FWD_Delta = fabs(L_Deg_SD_WA[L_Index] - L_Deg_SD_WA_FWD);

    L_Deg_SD_WA_REV = DtrmnEncoderRelativeToCmnd(L_Deg_SD_WA[L_Index],
                                                 L_Deg_WheelAngleRev[L_Index]);

    L_Deg_SD_WA_REV_Delta = fabs(L_Deg_SD_WA[L_Index] - L_Deg_SD_WA_REV);

    if (L_Deg_SD_WA_FWD_Delta <= L_Deg_SD_WA_REV_Delta)
      {
        VaDRC_Deg_WheelAngleArb[L_Index] = L_Deg_SD_WA_FWD;
      }
    else
      {
        VaDRC_Deg_WheelAngleArb[L_Index] = L_Deg_SD_WA_REV;
        L_RPM_SD_WS[L_Index] *= (-1); // Need to flip sign of drive wheel to account for reverse direction
      }

    if (LeDRC_b_AutoCenterEnabled == true && Le_b_SD_Active == true)
      {
      Le_n_SD_Offset = (sin((-L_Deg_WheelAngleFwd[L_Index]) * (C_PI / 180)) * Ke_k_SD_SignX[L_Index] + cos((-L_Deg_WheelAngleFwd[L_Index]) * (C_PI / 180)) * Ke_k_SD_SignY[L_Index]) * Le_k_SD_RotateCorrectionGx;

      if (Le_n_SD_Offset < 0 && L_RPM_SD_WS[L_Index] > 0 && ((-Le_n_SD_Offset) > (L_RPM_SD_WS[L_Index] * Ke_k_SD_AutoCorrectMaxWheelOffset)))
        {
        Le_n_SD_Offset = -L_RPM_SD_WS[L_Index] * Ke_k_SD_AutoCorrectMaxWheelOffset;
        }
      else if (Le_n_SD_Offset > 0 && L_RPM_SD_WS[L_Index] < 0 && Le_n_SD_Offset > ((-L_RPM_SD_WS[L_Index]) * Ke_k_SD_AutoCorrectMaxWheelOffset))
        {
        Le_n_SD_Offset = -L_RPM_SD_WS[L_Index] * Ke_k_SD_AutoCorrectMaxWheelOffset;
        }
      }
    else
      {
      Le_n_SD_Offset = 0;
      }

      La_n_SD_Offset[L_Index] = Le_n_SD_Offset;  // Save this for insturmentation, to verify that it is working as expected

      /* Wheel speed control resides externally in the independent motor controlers.
         Don't send the final value, ramp to the desired final value to help prevent integral windup and overshoot. */
      Le_RPM_SD_WheelSpeedCmnd[L_Index] = RampTo(((L_RPM_SD_WS[L_Index] + Le_n_SD_Offset) * KV_SD_WheelGx[L_Index]), VaDRC_RPM_WheelSpeedCmndPrev[L_Index], KV_SD_WheelSpeedRampRate);

      VaDRC_RPM_WheelSpeedCmndPrev[L_Index] = Le_RPM_SD_WheelSpeedCmnd[L_Index];

      if ((fabs(Le_RPM_SD_WheelSpeedCmnd[L_Index]) >= Ke_RPM_SD_WheelMinCmndSpeed))
        {
        /* Ok, so we have at least one wheel that is still trying to command a non zero speed. If not, we want to force it to a zero power 
           command to prevent locking of the wheels or swaying to try and hold a zero speed. */
        Le_b_SD_DriveWheelsPowered = true;
        }
    }

  if ((Le_b_SD_DriveWheelsPowered == true) || (LeDRC_b_X_ModeReqActive == true))
       {
        VeDRC_b_DriveWheelsInPID = true;
       }
    else
      {
        VeDRC_b_DriveWheelsInPID = false;
      }

  /* Output the wheel angle commands: */
  for (L_Index = E_FrontLeft;
       L_Index < E_RobotCornerSz;
       L_Index = T_RobotCorner(int(L_Index) + 1))
    {
    if (VeDRC_b_DriveWheelsInPID == true)
      {
      /* We do PID control within the Rio for angle control: */
      L_k_SD_WheelAngleCmnd[L_Index] =  Control_PID( L_Deg_SD_WA[L_Index],
                                                     VaDRC_Deg_WheelAngleArb[L_Index],
                                                    &VaDRC_Deg_WheelAngleError[L_Index],
                                                    &VaDRC_k_WheelAngleIntegral[L_Index],
                                                     KV_SD_WheelAnglePID_Gx[E_P_Gx],
                                                     KV_SD_WheelAnglePID_Gx[E_I_Gx],
                                                     KV_SD_WheelAnglePID_Gx[E_D_Gx],
                                                     KV_SD_WheelAnglePID_Gx[E_P_Ul],
                                                     KV_SD_WheelAnglePID_Gx[E_P_Ll],
                                                     KV_SD_WheelAnglePID_Gx[E_I_Ul],
                                                     KV_SD_WheelAnglePID_Gx[E_I_Ll],
                                                     KV_SD_WheelAnglePID_Gx[E_D_Ul],
                                                     KV_SD_WheelAnglePID_Gx[E_D_Ll],
                                                     KV_SD_WheelAnglePID_Gx[E_Max_Ul],
                                                     KV_SD_WheelAnglePID_Gx[E_Max_Ll]) * K_ENC_WheelAngleGx;
      }
    else
      {
      VaDRC_Deg_WheelAngleError[L_Index] = 0.0;
      VaDRC_k_WheelAngleIntegral[L_Index] = 0.0;
      }
    }

    frc::SmartDashboard::PutBoolean("low power mode", VeENC_b_LowPowerMode);


  }
