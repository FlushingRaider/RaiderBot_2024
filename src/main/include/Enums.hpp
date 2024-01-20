/*
  Enums.hpp

   Created on: Jan 16, 2023
   Author: 5561
 */

#pragma once

#ifndef ENUMS
#define ENUMS

//TODO Add Comments to explain all these?

typedef enum T_RobotCorner
{
  E_FrontLeft,
  E_FrontRight,
  E_RearLeft,
  E_RearRight,
  E_RobotCornerSz
} T_RobotCorner;

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

typedef enum T_ADAS_ActiveFeature // These are the sub features in ADAS.  These can be called in teleop and/or auton
{
  E_ADAS_Disabled,
  E_ADAS_DM_DriveStraight,
  E_ADAS_DM_DriveStraightFar,
  E_ADAS_DM_DriveRevStraight,
  E_ADAS_DM_PathFollower1,
  E_ADAS_DM_PathFollower2,
  E_ADAS_DM_PathFollower3,
  E_ADAS_DM_PathFollower4,
  E_ADAS_DM_PathFollowerFWD,

  
  E_ADAS_MoveOffsetTag,
  E_ADAS_MoveGlobalTag,
} T_ADAS_ActiveFeature;

struct RobotUserInput
{
  bool                  b_ZeroGyro;
  double                pct_SwerveForwardBack;
  double                pct_SwerveStrafe;
  double                deg_SwerveRotate;
  double                v_SwerveSpeed;
  bool                  b_SwerveGoalAutoCenter;
  bool                  b_SwerveRotateTo0;
  bool                  b_SwerveRotateTo180;
  bool                  b_CameraLight;
  bool                  b_JoystickActive;
  bool                  b_VisionDriverModeOverride;
  bool                  b_IntakeRollersTest; //21
  bool                  b_ResetManipulatorEnocders; // 21
  bool                  b_IntakeArmIn;  // 21
  bool                  b_IntakeArmOutTest;
  double                Pct_WristTest;
  double                Pct_ArmPivotTest;
  double                Pct_LinearSlideTest;
  double                pct_IntakeRollerTest;
  bool                  b_MainIntakeOut;
  bool                  b_MidIntakeOut;
  bool                  b_FloorConeDrop;
  bool                  b_InitState;
  bool                  b_DrivingPosition;
  bool                  b_FrontHighCube;
  bool                  b_FrontLowCube;
  bool                  b_FrontHighCone;
  bool                  b_FrontLowCone;
  bool                  b_DropGamePieceSlow;
  bool                  b_DropGamePieceFast;
  bool                  b_AutoBalance;
  bool                  b_CubeAlign;
  bool                  b_ConeAlign;
  bool                  b_VisionButton;
  bool                  b_X_Mode;
};

#endif