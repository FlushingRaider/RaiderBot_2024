/*
  ADAS.cpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)
  Contains the logic and code used for driver assitance control.  This is meant
  to serve as a high level controller sending commands/requests to the lower
  level controls while also tracking and managing the various sytems. This
  contains manages the following features:

  - Auto upper targeting
    - Turns on camera light, auto centers robot on target, spins the rollers up to the correct speed, disables camera light
  - Auto ball targeting
    - Centers robot on ball, turns on intake roller, drives froward to intake ball, exits
  - Auton Opt 1
    - More info to come

  Changes:
  2022-02-25 -> Beta
  2022-03-12 -> Auto targets, shoots
 */

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>

#include "Const.hpp"

#include "Driver_inputs.hpp"
#include "Gyro.hpp"
#include "ADAS_DJ.hpp"
#include "ADAS_DM.hpp"
#include "Odometry.hpp"
#include "Vision.hpp"
/* ADAS control state variables */
T_ADAS_ActiveFeature VeADAS_e_ActiveFeature = E_ADAS_Disabled;
T_ADAS_ActiveAutonFeature VeADAS_e_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
frc::SendableChooser<T_ADAS_ActiveAutonFeature> VeADAS_e_AutonChooser;
bool VeADAS_b_StateComplete = false;
bool VeADAS_b_State1Complete = false;
bool VeADAS_b_State2Complete = false;
bool VeADAS_b_AutonOncePerTrigger = false;

/* ADAS output control variables */
double VeADAS_Pct_SD_FwdRev = 0;
double VeADAS_Pct_SD_Strafe = 0;
double VeADAS_Pct_SD_Rotate = 0;
double VeADAS_Deg_SD_DesiredPose = 0;
bool VeADAS_b_SD_RobotOriented = false;
bool VeADAS_b_X_Mode = false;

bool VeADAS_b_CompletePrev = false;

// double DEBUGREQUESTX = 0.0;
// double DEBUGREQUESTY = 0.0;
// double DEBUGREQUESTYaw = 0.0;
bool VeADAS_b_SpeakerMoveActive = false;

/******************************************************************************
 * Function:     ADAS_Main_Init
 *
 * Description:  Initialize all applicable ADAS variables at robot init.
 ******************************************************************************/
void ADAS_Main_Init(void)
{

  // frc::SmartDashboard::PutNumber("Request x", DEBUGREQUESTX);
  // frc::SmartDashboard::PutNumber("Request y", DEBUGREQUESTY);
  // frc::SmartDashboard::PutNumber("Request yaw", DEBUGREQUESTYaw);
  // frc::SmartDashboard::PutBoolean("activate global move", VeADAS_b_SpeakerMoveActive);

  std::string_view LeADAS_Str_AutonSelectorName = "Auton";
  VeADAS_e_AutonChooser.AddOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  // VeADAS_e_AutonChooser.AddOption("L_Preload", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath1);
  // VeADAS_e_AutonChooser.AddOption("LR_Preload", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath2);
  // VeADAS_e_AutonChooser.AddOption("R_Preload1", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath3);
  // VeADAS_e_AutonChooser.AddOption("R_Preload2", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath4);
  // VeADAS_e_AutonChooser.AddOption("R_Preload3", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath5);
  // VeADAS_e_AutonChooser.AddOption("LR_RTB", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath6);
  // VeADAS_e_AutonChooser.AddOption("L_RTB", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath7);
  VeADAS_e_AutonChooser.AddOption("3 Note Short", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOpt1); // upper 3 piece auton
  VeADAS_e_AutonChooser.AddOption("Double", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOpt2);
  VeADAS_e_AutonChooser.AddOption("3 Note Long", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOpt3);  // lower 3 piece auton
  VeADAS_e_AutonChooser.AddOption("Leave", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOpt4);
  VeADAS_e_AutonChooser.AddOption("Preload", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOpt5);
  VeADAS_e_AutonChooser.AddOption("2 Piece + Preload Short", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOpt6);
  VeADAS_e_AutonChooser.AddOption("Option Test1", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOptTest1); // Calibration test
  // VeADAS_e_AutonChooser.AddOption("Demo chain move", T_ADAS_ActiveAutonFeature::E_ADAS_DemoChainMove);
  VeADAS_e_AutonChooser.AddOption("4 Note", T_ADAS_ActiveAutonFeature::E_ADAS_AutonOpt7); // 4 piece auton
  VeADAS_e_AutonChooser.SetDefaultOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);
  frc::SmartDashboard::PutData(LeADAS_Str_AutonSelectorName, &VeADAS_e_AutonChooser);
}

/******************************************************************************
 * Function:     ADAS_DetermineMode
 *
 * Description:  Ping the driver station to see what the desired auton routine
 *               should be.
 ******************************************************************************/
void ADAS_DetermineMode(void)
{
  VeADAS_e_DriverRequestedAutonFeature = VeADAS_e_AutonChooser.GetSelected();
  frc::SmartDashboard::PutNumber("Requested Auton", float(VeADAS_e_DriverRequestedAutonFeature));
}

/******************************************************************************
 * Function:     ADAS_Main_Reset
 *
 * Description:  Reset all applicable ADAS variables.
 ******************************************************************************/
void ADAS_Main_Reset(void)
{
  VeADAS_e_ActiveFeature = E_ADAS_Disabled;
  VeADAS_Pct_SD_FwdRev = 0;
  VeADAS_Pct_SD_Strafe = 0;
  VeADAS_Pct_SD_Rotate = 0;

  VeADAS_b_SD_RobotOriented = false;
  VeADAS_e_DriverRequestedAutonFeature = E_ADAS_AutonDisabled;
  VeADAS_b_StateComplete = false;
  VeADAS_b_State1Complete = false;
  VeADAS_b_State2Complete = false;
  VeADAS_b_AutonOncePerTrigger = false;

  VeADAS_t_DM_StateTimer = 0.0; // ToDo: Originally in ADAS_DM_Reset, do we want this back?
  ADAS_DM_Reset();
  /* Trigger the resets for all of the sub tasks/functions as well: */
}

/******************************************************************************
 * Function:     AbortCriteria
 *
 * Description:  Contains the abort criteria for teleop.
 ******************************************************************************/
T_ADAS_ActiveFeature AbortCriteria(bool LeADAS_b_Driver1_JoystickActive,
                                   T_ADAS_ActiveFeature LeADAS_e_ActiveFeature)
{
  if ((LeADAS_b_Driver1_JoystickActive == true) || (VeADAS_b_StateComplete == true))
  {
    /* Abort criteria goes here. */

    LeADAS_e_ActiveFeature = E_ADAS_Disabled;
    VeADAS_b_StateComplete = false;
    VeADAS_b_State1Complete = false;
    VeADAS_b_State2Complete = false;
  }
  return (LeADAS_e_ActiveFeature);
}

/******************************************************************************
 * Function:     ADAS_ControlMain
 *
 * Description:  Main calling function for the ADAS (advanced driver assistance
 *               system)control when robot is active. This will call and manage
 *               the various ADAS features.
 ******************************************************************************/
T_ADAS_ActiveFeature ADAS_ControlMain(double *L_Pct_FwdRev,
                                      double *L_Pct_Strafe,
                                      double *L_Pct_Rotate,
                                      double *LeADAS_Deg_DesiredPose,
                                      bool *LeADAS_b_SD_RobotOriented,
                                      bool *LeADAS_b_X_Mode,
                                      bool LeADAS_b_Driver1_JoystickActive,
                                      bool L_Driver_SwerveGoalAutoCenter,
                                      double L_Deg_GyroAngleDeg,
                                      double L_L_X_FieldPos,
                                      double L_L_Y_FieldPos,
                                      T_RobotState LeADAS_e_RobotState,
                                      T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                                      std::optional<frc::DriverStation::Alliance> LeLC_e_AllianceColor)
{
  bool LeADAS_b_State1Complete = false;
  bool LeADAS_b_State2Complete = false;
  double LeADAS_e_RequestedX = 0.0;
  double LeADAS_e_RequestedY = 0.0;
  double LeADAS_e_RequestedYaw = 0.0;

  // if (VsCONT_s_DriverInput.b_goToSpeaker == true)
  // {
  //   VeADAS_b_SpeakerMoveActive = true; // this will be set to false when the state is compelete
  // }

  frc::SmartDashboard::PutNumber("auton sub feature", (int)LeADAS_e_ActiveFeature);
  frc::SmartDashboard::PutBoolean("State Complete", VeADAS_b_StateComplete);

  // DEBUGREQUESTX = frc::SmartDashboard::GetNumber("Request x", 0.0);
  // DEBUGREQUESTY = frc::SmartDashboard::GetNumber("Request y", 0.0);
  // DEBUGREQUESTYaw = frc::SmartDashboard::GetNumber("Request yaw", 0.0);
  // VeADAS_b_SpeakerMoveActive = frc::SmartDashboard::GetBoolean("activate global move", false);

  /* First, let's determine what we are going to do: */
  if (LeADAS_e_RobotState == E_Teleop)
  {
    // if (VeADAS_b_SpeakerMoveActive)
    // {
    //   if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
    //   {
    //     LeADAS_e_RequestedX = C_in_RedSpeakerCoords.first;
    //     LeADAS_e_RequestedY = C_in_RedSpeakerCoords.second;
    //   }
    //   else if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kBlue)
    //   {
    //     LeADAS_e_RequestedX = C_in_BlueSpeakerCoords.first;
    //     LeADAS_e_RequestedY = C_in_BlueSpeakerCoords.second;
    //   }
    //   LeADAS_e_RequestedYaw = 0.0; // we wanna look straight at it
    //   LeADAS_e_ActiveFeature = E_ADAS_MoveGlobal;
    // }

    LeADAS_e_ActiveFeature = AbortCriteria(LeADAS_b_Driver1_JoystickActive, LeADAS_e_ActiveFeature);
  }
  else if (LeADAS_e_RobotState == E_Auton)
  {

    VeVis_CenteringEnable = true;

    // auton selection
    switch (VeADAS_e_DriverRequestedAutonFeature)
    {
    case E_ADAS_AutonDrivePath1: // L_Preload
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath2: // LR_Preload
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower5;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower5) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath3: // R_Preload1
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower6;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower6) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower7;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower7) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower10;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower10) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath4: // R_Preload2
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower6;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower6) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower8;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower8) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower11;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower11) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath5: // R_Preload3
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower6;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower6) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower9;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower9) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower12;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower12) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath6: // LR_RTB
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower14;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower14) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower16;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower16) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower15;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower15) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath7: // L_Preload
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower13;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower13) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOpt1:
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt1Path1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt1Path1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt1Path2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt1Path2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt1Path3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt1Path3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt1Path4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt1Path4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt1Path5;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt1Path5) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt1Path6;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt1Path6) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOpt2:
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt2Path1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt2Path1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt2Path2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt2Path2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt2Path3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt2Path3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOpt3:
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt3Path1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt3Path1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt3Path2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt3Path2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt3Path3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt3Path3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt3Path4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt3Path4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt3Path5;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt3Path5) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt3Path6;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt3Path6) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt3Path7;   // This is return to start position, to try and speed up cal efforts.
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt3Path7) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOpt4:
      /*Opt4 is basically Opt2, but doesn't attempt to shoot a second note, nor will it crive back out and stay out across the line*/
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt8Path1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt8Path1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOpt5:
      /*Opt5 is basically Opt2, but doesn't attempt to shoot a second note, nor will it drive */
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOpt6:
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt4Path1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt4Path1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt4Path2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt4Path2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt4Path3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt4Path3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt4Path4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt4Path4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote3;
      }            
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOpt7:
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path5;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path5) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path6;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path6) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path7;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path7) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DJ_ShootNote4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DJ_ShootNote4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Opt7Path8;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Opt7Path8) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonOptTest1:
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Test1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Test1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_DJ_Test2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_DJ_Test2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    case E_ADAS_DemoChainMove:
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_RequestedX = 0.0;
        LeADAS_e_RequestedY = 0.0;
        LeADAS_e_ActiveFeature = E_ADAS_MoveGlobal;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MoveGlobal) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_RequestedX = 30.0;
        LeADAS_e_RequestedY = 20.0;
        // LeADAS_e_ActiveFeature = E_ADAS_MoveGlobal;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MoveGlobal) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_RequestedX = 0.0;
        LeADAS_e_RequestedY = 30.0;
        LeADAS_e_RequestedYaw = 45.0;
        // LeADAS_e_ActiveFeature = E_ADAS_MoveGlobal;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MoveGlobal) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_RequestedX = 0.0;
        LeADAS_e_RequestedY = 0.0;
        // LeADAS_e_ActiveFeature = E_ADAS_MoveGlobal;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_MoveGlobal) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
      break;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDisabled:
    default:
      LeADAS_e_ActiveFeature = E_ADAS_Disabled;
      break;
    }
  }

  // our active feature table, ADAS sets which one they want
  switch (LeADAS_e_ActiveFeature)
  {
  case E_ADAS_MoveGlobal:
    // VeADAS_b_StateComplete = ADAS_DM_MoveWithGlobalCoords(L_Pct_FwdRev,
    //                                                       L_Pct_Strafe,
    //                                                       L_Pct_Rotate,
    //                                                       VeODO_In_RobotDisplacementX,
    //                                                       VeODO_In_RobotDisplacementY,
    //                                                       VeVis_deg_VisionYaw,
    //                                                       LeADAS_e_RequestedX,
    //                                                       LeADAS_e_RequestedY,
    //                                                       LeADAS_e_RequestedYaw);

    if (VeADAS_b_StateComplete) // telop adas features need to manually set to false
    {

      VeADAS_b_SpeakerMoveActive = false; // we done
    }
    break;

  case E_ADAS_DJ_ShootNote1:
  case E_ADAS_DJ_ShootNote2:
  case E_ADAS_DJ_ShootNote3:
  case E_ADAS_DJ_ShootNote4:
    /* We are just attempting to shoot the note. No need to call path follower. */
    VeADAS_b_State1Complete = ADAS_DM_Stop(L_Pct_FwdRev,
                                           L_Pct_Strafe,
                                           L_Pct_Rotate);

    VeADAS_b_State2Complete = ADAS_DJ_Main(LeADAS_e_RobotState,
                                           LeADAS_e_ActiveFeature);

    VeADAS_b_StateComplete = (VeADAS_b_State1Complete == true && VeADAS_b_State2Complete == true);
    if (VeADAS_b_StateComplete == true)
    {
      VeADAS_b_State1Complete = false;
      VeADAS_b_State2Complete = false;
    }
    break;
  case E_ADAS_DM_PathFollower1:
  case E_ADAS_DM_PathFollower2:
  case E_ADAS_DM_PathFollower3:
  case E_ADAS_DM_PathFollower4:
  case E_ADAS_DM_PathFollower5:
  case E_ADAS_DM_PathFollower6:
  case E_ADAS_DM_PathFollower7:
  case E_ADAS_DM_PathFollower8:
  case E_ADAS_DM_PathFollower9:
  case E_ADAS_DM_PathFollower10:
  case E_ADAS_DM_PathFollower11:
  case E_ADAS_DM_PathFollower12:
  case E_ADAS_DM_PathFollower13:
  case E_ADAS_DM_PathFollower14:
  case E_ADAS_DM_PathFollower15:
  case E_ADAS_DM_PathFollower16:
  case E_ADAS_DM_PathFollower17:
  case E_ADAS_DM_DJ_Opt1Path1:
  case E_ADAS_DM_DJ_Opt1Path2:
  case E_ADAS_DM_DJ_Opt1Path3:
  case E_ADAS_DM_DJ_Opt1Path4:
  case E_ADAS_DM_DJ_Opt1Path5:
  case E_ADAS_DM_DJ_Opt1Path6:
  case E_ADAS_DM_DJ_Opt1Path7:
  case E_ADAS_DM_DJ_Opt2Path1:
  case E_ADAS_DM_DJ_Opt2Path2:
  case E_ADAS_DM_DJ_Opt2Path3:
  case E_ADAS_DM_DJ_Opt3Path1:
  case E_ADAS_DM_DJ_Opt3Path2:
  case E_ADAS_DM_DJ_Opt3Path4:
  case E_ADAS_DM_DJ_Opt3Path5:
  case E_ADAS_DM_DJ_Opt3Path6:
  case E_ADAS_DM_DJ_Opt3Path7:
  case E_ADAS_DM_DJ_Opt4Path1:
  case E_ADAS_DM_DJ_Opt4Path2:
  case E_ADAS_DM_DJ_Opt4Path3:
  case E_ADAS_DM_DJ_Opt4Path4:
  case E_ADAS_DM_DJ_Opt7Path1:
  case E_ADAS_DM_DJ_Opt7Path2:
  case E_ADAS_DM_DJ_Opt7Path3:
  case E_ADAS_DM_DJ_Opt7Path4:
  case E_ADAS_DM_DJ_Opt7Path5:
  case E_ADAS_DM_DJ_Opt7Path6:
  case E_ADAS_DM_DJ_Opt7Path7:
  case E_ADAS_DM_DJ_Opt7Path8:
  case E_ADAS_DM_DJ_Opt8Path1:
  case E_ADAS_DM_DJ_Test1:
  case E_ADAS_DM_DJ_Test2:
    VeADAS_b_State2Complete = ADAS_DJ_Main(LeADAS_e_RobotState,
                                           LeADAS_e_ActiveFeature);

    VeADAS_b_State1Complete = ADAS_DM_PathFollower(L_Pct_FwdRev,
                                                   L_Pct_Strafe,
                                                   L_Pct_Rotate,
                                                   LeADAS_Deg_DesiredPose,
                                                   LeADAS_b_SD_RobotOriented,
                                                   L_L_X_FieldPos,
                                                   L_L_Y_FieldPos,
                                                   L_Deg_GyroAngleDeg,
                                                   LeADAS_e_ActiveFeature,
                                                   LeLC_e_AllianceColor,
                                                   VeADAS_b_State2Complete);

    VeADAS_b_StateComplete = (VeADAS_b_State1Complete == true && VeADAS_b_State2Complete == true);
    if (VeADAS_b_StateComplete == true)
    {
      VeADAS_b_State1Complete = false;
      VeADAS_b_State2Complete = false;
    }
    break;

  case E_ADAS_Disabled:
  default:
    VeADAS_b_State1Complete = ADAS_DJ_Main(LeADAS_e_RobotState,
                                           LeADAS_e_ActiveFeature);
    VeADAS_b_StateComplete = true;
    *L_Pct_FwdRev = 0;
    *L_Pct_Strafe = 0;
    *L_Pct_Rotate = 0;
    *LeADAS_b_X_Mode = false;
    break;
  }

  return (LeADAS_e_ActiveFeature);
}
