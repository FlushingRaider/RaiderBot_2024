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
  E_ADAS_DM_PathFollower1,
  E_ADAS_DM_PathFollower2,
  E_ADAS_DM_PathFollower3,
  E_ADAS_DM_PathFollower4,
  E_ADAS_DM_PathFollower5,
  E_ADAS_DM_PathFollower6,
  E_ADAS_DM_PathFollower7,
  E_ADAS_DM_PathFollower8,
  E_ADAS_DM_PathFollower9,
  E_ADAS_DM_PathFollower10,
  E_ADAS_DM_PathFollower11,
  E_ADAS_DM_PathFollower12,
  E_ADAS_DM_PathFollower13,
  E_ADAS_DM_PathFollower14,
  E_ADAS_DM_PathFollower15,
  E_ADAS_DM_PathFollower16,
  E_ADAS_DM_PathFollower17,
  E_ADAS_MoveGlobal,
  E_ADAS_DJ_ShootNote1,
  E_ADAS_DJ_ShootNote2,
  E_ADAS_DJ_ShootNote3,
  E_ADAS_DJ_ShootNote4,
  E_ADAS_DM_DJ_Opt1Path1,
  E_ADAS_DM_DJ_Opt1Path2,
  E_ADAS_DM_DJ_Opt1Path3,
  E_ADAS_DM_DJ_Opt1Path4,
  E_ADAS_DM_DJ_Opt1Path5,
  E_ADAS_DM_DJ_Opt1Path6,
  E_ADAS_DM_DJ_Opt2Path1,
  E_ADAS_DM_DJ_Opt2Path2,
  E_ADAS_DM_DJ_Opt2Path3,
  E_ADAS_DM_DJ_Opt3Path1,
  E_ADAS_DM_DJ_Opt3Path2,
  E_ADAS_DM_DJ_Opt3Path3,
  E_ADAS_DM_DJ_Opt3Path4,
  E_ADAS_DM_DJ_Opt3Path5,
  E_ADAS_DM_DJ_Opt3Path6,
  E_ADAS_DM_DJ_Opt4Path1,
  E_ADAS_DM_DJ_Opt4Path2,
  E_ADAS_DM_DJ_Opt4Path3,
  E_ADAS_DM_DJ_Opt4Path4,
  E_ADAS_DM_DJ_Opt7Path1,
  E_ADAS_DM_DJ_Opt7Path2,
  E_ADAS_DM_DJ_Opt7Path3,
  E_ADAS_DM_DJ_Opt7Path4,
  E_ADAS_DM_DJ_Opt7Path5,
  E_ADAS_DM_DJ_Opt7Path6,
  E_ADAS_DM_DJ_Opt7Path7,
  E_ADAS_DM_DJ_Opt7Path8,
  E_ADAS_DM_DJ_Opt8Path1,
  E_ADAS_DM_DJ_Test1,
  E_ADAS_DM_DJ_Test2
} T_ADAS_ActiveFeature;

typedef enum T_ADAS_ActiveAutonFeature // The various autons we may want to call, enumurated for you convience!
{
  E_ADAS_AutonDisabled,
  E_ADAS_AutonDrivePath1,
  E_ADAS_AutonDrivePath2,
  E_ADAS_AutonDrivePath3,
  E_ADAS_AutonDrivePath4,
  E_ADAS_AutonDrivePath5,
  E_ADAS_AutonDrivePath6,
  E_ADAS_AutonDrivePath7,
  E_ADAS_AutonOpt1,
  E_ADAS_AutonOpt2,
  E_ADAS_AutonOpt3,
  E_ADAS_AutonOpt3_1,
  E_ADAS_AutonOpt4, // Shoot, drive straight out and then come back
  E_ADAS_AutonOpt5, // Shoot, don't drive
  E_ADAS_AutonOpt6, // Shoot, Drive to note, drive back, shoot, drive to second note, drive, shoot
  E_ADAS_AutonOpt7, // 4 piece auton
  E_ADAS_AutonOptTest1,
  E_ADAS_DemoChainMove
} T_ADAS_ActiveAutonFeature;


typedef enum TsENC_LightPatterns{
  E_LED_RAINBOW,
  E_LED_SOLIDWHITE
} TsENC_LightPatterns;


// a big ol' object ti carry the various inputs from the driver in one place
struct RobotUserInput
{
  bool b_ZeroGyro = false;
  double pct_SwerveForwardBack = 0.0;
  double pct_SwerveStrafe = 0.0;
  double deg_SwerveRotate = 0.0;
  double v_SwerveSpeed = 0.0;
  bool b_SwerveGoalAutoCenter = false;
  bool b_SwerveRotateTo0 = false;
  bool b_SwerveRotateTo180 = false;
  bool b_CameraLight = false;
  bool b_JoystickActive = false;
  bool b_VisionDriverModeOverride = false;
  bool b_Amp_IntakeBackward_Test = false;
  bool b_ResetEnocders = false;
  bool b_Spk_IntakeBackward_Test = false;
  bool b_IntakeArmOutTest = false;
  double Pct_Shooter1_Test = 0.0;
  double Pct_Shooter2_Test = 0.0;
  double Pct_Amp_Intake_Test = 0.0;
  double Pct_Amp_Wrist_Test = 0.0;
  double Pct_Amp_Elevator_Test = 0.0;
  bool   b_Spk_IntakeForward_Test = false;
  bool   b_MainIntakeOut = false;
  double pct_RightHook_Test = 0.0;
  double pct_LeftHook_Test = 0.0;
  double pct_RightHookUp_Test = 0.0;
  double pct_LeftHookUp_Test = 0.0;
  bool   b_FloorConeDrop = false;
  double pct_RightHookDown_Test = 0.0;
  double pct_LeftHookDown_Test = 0.0;
  bool b_DrivingPosition = false;
  bool b_VisionButton = false;
  bool b_X_Mode = false;
  bool b_Amp_DrivingPosition = false;
  bool b_Amp_Intake = false;
  bool b_Amp_IntakeForward_Test = false;
  bool b_Amp_PreScore = false;
  bool b_Amp_Score = false;
  bool b_Amp_Trade_Off = false; 
  bool b_SPK_DrivingPosition = false;
  bool b_SPK_Intake = false;
  bool b_SPK_PreScore = false;
  bool b_SPK_Score = false;
  bool b_CLMR_MidClimb = false;
  bool b_CLMR_FullExtend = false;
  bool b_CLMR_Init = false;
  double Pct_Manual_CLMR = 0.0;
  bool b_goToSpeaker = false;
  bool b_SourcePickup = false;
};

struct TsENC_Power
{
  double I_SD_Steer_FR = 0.0;
  double I_SD_Drive_FR = 0.0;
  double I_SD_Steer_FL = 0.0;
  double I_SD_Drive_FL = 0.0;
  double I_SD_Steer_RR = 0.0;
  double I_SD_Drive_RR = 0.0;
  double I_SD_Steer_RL = 0.0;
  double I_SD_Drive_RL = 0.0;
  double I_CLMR_1 = 0.0;
  double I_CLMR_2 = 0.0;
  double I_SPK_Intake = 0.0;
  double I_SPK_IAssist = 0.0;
  double I_SPK_Shooter1 = 0.0;
  double I_SPK_Shooter2 = 0.0;
  double I_Amp_Elevator = 0.0;
  double I_Amp_Wrist = 0.0;
  double I_Amp_Intake = 0.0;
  double I_VIS_Pi = 0.0;
  double T_PDH_Temp = 0.0;
  double I_PDH_TotalCurrent = 0.0;
  double W_PDH_TotalPower = 0.0;
  double J_PDH_TotalEnergy = 0.0;
  double V_PDH_Voltage = 0.0;
  double I_SD_CanCoders = 0.0;
};

struct TsENC_PowerAboveThresh
{
  bool b_SD_Steer_FR = false;
  bool b_SD_Drive_FR = false;
  bool b_SD_Steer_FL = false;
  bool b_SD_Drive_FL = false;
  bool b_SD_Steer_RR = false;
  bool b_SD_Drive_RR = false;
  bool b_SD_Steer_RL = false;
  bool b_SD_Drive_RL = false;
  bool b_CLMR_1 = false;
  bool b_CLMR_2 = false;
  bool b_SPK_Intake = false;
  bool b_SPK_IAssist = false;
  bool b_SPK_Shooter1 = false;
  bool b_SPK_Shooter2 = false;
  bool b_Amp_Elevator = false;
  bool b_Amp_Wrist = false;
  bool b_Amp_Intake = false;
  bool b_VIS_Pi = false;
  bool b_PDH_Temp = false;
  bool b_PDH_TotalCurrent = false;
  bool b_PDH_TotalPower = false;
  bool b_PDH_TotalEnergy = false;
  bool b_PDH_Voltage = false;
  bool b_SD_CanCoders = false;
};

struct TsENC_MaxOutput
{
  double I_SD_Steer_FR = 0.0;
  double I_SD_Drive_FR = 0.0;
  double I_SD_Steer_FL = 0.0;
  double I_SD_Drive_FL = 0.0;
  double I_SD_Steer_RR = 0.0;
  double I_SD_Drive_RR = 0.0;
  double I_SD_Steer_RL = 0.0;
  double I_SD_Drive_RL = 0.0;
  double I_CLMR_1 = 0.0;
  double I_CLMR_2 = 0.0;
  double I_SPK_Intake = 0.0;
  double I_SPK_IAssist = 0.0;
  double I_SPK_Shooter1 = 0.0;
  double I_SPK_Shooter2 = 0.0;
  double I_Amp_Elevator = 0.0;
  double I_Amp_Wrist = 0.0;
  double I_Amp_Intake = 0.0;
  double I_VIS_Pi = 0.0;
  double T_PDH_Temp = 0.0;
  double I_PDH_TotalCurrent = 0.0;
  double W_PDH_TotalPower = 0.0;
  double J_PDH_TotalEnergy = 0.0;
  double V_PDH_Voltage = 0.0;
  double I_SD_CanCoders = 0.0;
};

struct TsAmp_Sensor
{
  double In_Elevator; //height of elevator in inches
  double Deg_Wrist;  //Angle of Wrist in Degrees
  double RPM_AmpRollers; //Speed of 
  bool   b_Amp_ObjDetected;
  bool   b_ElevatorSwitch;
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

typedef enum TeAMP_e_WristReset
{
  E_AMP_WristReseted,
  E_AMP_WristResetInProc,
  E_AMP_WristNeedsReset
} TeAMP_e_WristReset;

typedef enum T_DJ_Amp_Motors
{
  E_DJ_Amp_m_Elevator,
  E_DJ_Amp_m_Wrist,
  E_DJ_Amp_m_Intake,
  E_DJ_Ampz
} T_DJ_Amp_Motors;

typedef enum T_DJ_Amp_States
{
  E_DJ_Amp_Init,
  E_DJ_Amp_Driving,
  E_DJ_Amp_Intake,
  E_DJ_Amp_PreScore,
  E_DJ_Amp_Score,
  E_DJ_Amp_Trade_Off,
  E_DJ_Amp_iAssist,
  E_DJ_Amp_Source,
  E_DJ_Amp_State_Sz
} T_DJ_Amp_States;

struct TeAmp_MotorControl
{
  T_MotorControlType e_MotorControlType[E_DJ_Ampz];
  double k_MotorCmnd[E_DJ_Ampz];
  double k_MotorRampRate[E_DJ_Ampz];
  double k_MotorTestValue[E_DJ_Ampz];
  double k_MotorTestPower[E_DJ_Ampz];
};

/* Enums related to the speaker subsystem */

typedef enum TeSPK_Actuators
{
  E_SPK_m_Intake,
  E_SPK_m_IAssist,
  E_SPK_m_Shooter1,
  E_SPK_m_Shooter2,
  E_SPK_m_Sz
} TeSPK_Actuators;

typedef enum TeSPK_CtrlStates
{
  E_SPK_Ctrl_Init,
  E_SPK_Ctrl_Driving,
  E_SPK_Ctrl_Intake,
  E_SPK_Ctrl_PreScore,
  E_SPK_Ctrl_Score,
  E_SPK_Ctrl_StateSz
} TeSPK_CtrlStates;

struct TeSKP_MotorControl
{
  T_MotorControlType    e_MotorControlType[E_SPK_m_Sz];
  double                k_MotorCmnd[E_SPK_m_Sz];
  double                k_MotorRampRate[E_SPK_m_Sz];
  double                k_MotorTestValue[E_SPK_m_Sz];
  double                k_MotorTestPower[E_SPK_m_Sz];
};

struct TsSPK_Sensor 
{
  double RPM_Intake; // Speed of the intake rollers
  double RPM_IAssist; // Speed of the Intake assist
  double RPM_Shooter1; // Speed of the shooter 1 rollers
  double RPM_Shooter2; // Speed of the shooter 2 rollers
  bool   b_NoteDetected;
};


/* Enums related to the climber subsystem */

typedef enum TeCLMR_Actuators
{
  E_CLMR_m_Left,
  E_CLMR_m_Right,
  E_CLMR_m_Sz
} TeCLMR_Actuators;

typedef enum TeCLMR_CtrlStates
{
  E_CLMR_Ctrl_Init,
  E_CLMR_Ctrl_MidClimb,
  E_CLMR_Ctrl_FullExtend,
  E_CLMR_Ctrl_StateSz
} TeCLMR_CtrlStates;

struct TeCLMR_MotorControl
{
  T_MotorControlType    e_MotorControlType[E_CLMR_m_Sz];
  double                k_MotorCmnd[E_CLMR_m_Sz];
  double                k_MotorRampRate[E_CLMR_m_Sz];
  double                k_MotorTestValue[E_CLMR_m_Sz];
  double                k_MotorTestPower[E_CLMR_m_Sz];
};

struct TsCLMR_Sensor 
{
  double in_Left;  // Extended position of the left climber
  double in_Right; // Extended position of the right climber
};


// the output struct after a path is loaded
struct CalcedPathVals
{
  double L_valX = 0.0;
  double L_valY = 0.0;

  double L_valDeg = 0.0;
  double L_timRem = 0.0;
};

// the struct of data passed from lookup to ADAS_DM
struct DtrmnSwerveBotLocationOut
{

  double L_valX = 0.0;
  double L_valY = 0.0;

  double L_valDeg = 0.0;
  double L_timRem = 0.0;
  bool L_timeEndReached = false;
};

#endif