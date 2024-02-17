/*
  Enums.hpp

   Created on: Jan 16, 2023
   Author: 5561
 */

#pragma once

#ifndef ENUMS
#define ENUMS

// TODO Add Comments to explain all these? to people

// enum for each state of the robot
typedef enum T_RobotState
{
  E_Init,
  E_Auton,
  E_Teleop
} T_RobotState;

// enum of each swerve corner, used for motor control indexing (see DriveControl.cpp or Encoders.cpp)
typedef enum T_RobotCorner
{
  E_FrontLeft,
  E_FrontRight,
  E_RearLeft,
  E_RearRight,
  E_RobotCornerSz
} T_RobotCorner;

// enum for different PID values, mostly used to index the variable KV_SD_WheelAnglePID_Gx
typedef enum T_PID_Cal
{
  E_P_Gx,
  E_I_Gx,
  E_D_Gx,
  E_P_Ul,
  E_P_Ll,
  E_I_Ul,
  E_I_Ll,
  E_D_Ul,
  E_D_Ll,
  E_Max_Ul,
  E_Max_Ll,
  E_PID_CalSz
} T_PID_Cal;

// used in conjunction with T_PID_Cal
typedef enum T_PID_SparkMaxCal
{
  E_kP,
  E_kI,
  E_kD,
  E_kIz,
  E_kFF,
  E_kMaxOutput,
  E_kMinOutput,
  E_kMaxVel,
  E_kMinVel,
  E_kMaxAcc,
  E_kAllErr,
  E_PID_SparkMaxCalSz
} T_PID_SparkMaxCal;

// These are the sub features in ADAS.  These can be called in teleop and/or auton
typedef enum T_ADAS_ActiveFeature
{
  E_ADAS_Disabled,
  E_ADAS_DM_DriveStraight,
  E_ADAS_DM_DriveStraightFar,
  E_ADAS_DM_DriveRevStraight,
  E_ADAS_DM_PathFollower1,
  E_ADAS_DM_PathFollower2,
  E_ADAS_DM_PathFollower3,
  E_ADAS_DM_PathFollower4,
  E_ADAS_DM_PathFollower5,
  E_ADAS_DM_PathFollowerFWD,

  E_ADAS_MoveOffsetTag,
  E_ADAS_MoveGlobalTag,
} T_ADAS_ActiveFeature;

typedef enum T_ADAS_ActiveAutonFeature // The various autons we may want to call, enumurated for you convience!
{
  E_ADAS_AutonDisabled,
  E_ADAS_AutonDropCubeDriveFwd,
  E_ADAS_AutonDeliverCubeDriveOnRampAutoBal,
  E_ADAS_AutonDrivePath1,
  E_ADAS_AutonDrivePath2,
} T_ADAS_ActiveAutonFeature;

// a big ol' object ti carry the various inputs from the driver in one place
struct RobotUserInput
{
  bool b_ZeroGyro;
  double pct_SwerveForwardBack;
  double pct_SwerveStrafe;
  double deg_SwerveRotate;
  double v_SwerveSpeed;
  bool b_SwerveGoalAutoCenter;
  bool b_SwerveRotateTo0;
  bool b_SwerveRotateTo180;
  bool b_CameraLight;
  bool b_JoystickActive;
  bool b_VisionDriverModeOverride;
  bool b_IntakeRollersTest;        // 21
  bool b_ResetManipulatorEnocders; // 21
  bool b_IntakeArmIn;              // 21
  bool b_IntakeArmOutTest;
  double Pct_WristTest;
  double Pct_ArmPivotTest;
  double Pct_LinearSlideTest;
  double pct_IntakeRollerTest;
  bool b_MainIntakeOut;
  bool b_MidIntakeOut;
  bool b_FloorConeDrop;
  bool b_InitState;
  bool b_DrivingPosition;
  bool b_FrontHighCube;
  bool b_FrontLowCube;
  bool b_FrontHighCone;
  bool b_FrontLowCone;
  bool b_DropGamePieceSlow;
  bool b_DropGamePieceFast;
  bool b_AutoBalance;
  bool b_CubeAlign;
  bool b_ConeAlign;
  bool b_VisionButton;
  bool b_X_Mode;
  bool b_Amp_Intake;
  bool b_Amp_DrivingPosition;
  bool b_Amp_PreScore;
  bool b_Amp_Score;
};

struct TsDJ_Sensor
{
  double In_Elevator;
  double Deg_Wrist;
  bool b_Amp_ObjDetected;
  bool b_Spk_ObjDetected;
};

typedef enum T_MotorControlType
{
  E_MotorControlDisabled,
  E_MotorControlPctCmnd,
  E_MotorControlPosition,
  E_MotorControlSpeed,
  E_MotorExtend,
  E_MotorRetract
} T_MotorControlType;

typedef enum TeDJ_Amp_e_AmpActuator
{
  E_Amp_Elevator,
  E_Amp_Wrist,
  E_Amp_Intake,
  E_Amp_Sz
} TeDJ_Amp_e_AmpActuator;

typedef enum T_DJ_Amp_Motors
{
  E_DJ_Amp_m_Elevator,
  E_DJ_Amp_m_Wrist,
  E_DJ_Amp_m_Intake,
  E_DJ_Ampz
} T_DJ_Amp_Motors;

typedef enum T_DJ_Spk_Motors
{
  E_DJ_m_Underbelly,
  E_DJ_m_Shooter1,
  E_DJ_m_Shooter2
} T_DJ_Spk_Motors;

typedef enum T_DJ_Amp_States
{
  E_DJ_Amp_Init,
  E_DJ_Amp_Driving,
  E_DJ_Amp_Intake,
  E_DJ_Amp_PreScore,
  E_DJ_Amp_Score,
  E_DJ_Amp_State_Sz
} T_DJ_Amp_States;

typedef enum T_DJ_Spk_States
{
  E_DJ_Spk_Init,
  E_DJ_Spk_Driving,
  E_DJ_Underbelly,
  E_DJ_PreScore,
  E_DJ_Shooting
} T_DJ_Spk_States;

struct TeAmp_MotorControl
{
  T_MotorControlType e_MotorControlType[E_DJ_Ampz];
  double k_MotorCmnd[E_DJ_Ampz];
  double k_MotorRampRate[E_DJ_Ampz];
  double k_MotorTestValue[E_DJ_Ampz];
  double k_MotorTestPower[E_DJ_Ampz];
};

struct CalcedPathVals
{
  double L_valX;
  double L_valY;

  double L_valDeg;
  double L_timRem;
};

struct DtrmnSwerveBotLocationOut
{

  double L_valX;
  double L_valY;

  double L_valDeg;
  double L_timRem;
  bool L_timeEndReached = false;
};

#endif