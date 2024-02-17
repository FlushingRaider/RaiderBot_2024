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

double VeADAS_in_OffsetRequestX;
double VeADAS_in_OffsetRequestY;

double VeADAS_in_GlobalRequestX;
double VeADAS_in_GlobalRequestY;

/******************************************************************************
 * Function:     ADAS_Main_Init
 *
 * Description:  Initialize all applicable ADAS variables at robot init.
 ******************************************************************************/
void ADAS_Main_Init(void)
{
  std::string_view LeADAS_Str_AutonSelectorName = "Auton";
  VeADAS_e_AutonChooser.AddOption("Disabled", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDisabled);

  VeADAS_e_AutonChooser.AddOption("Auton Test - Wesley", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath1);

  VeADAS_e_AutonChooser.AddOption("L_Preload", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath2);

    VeADAS_e_AutonChooser.AddOption("Recall", T_ADAS_ActiveAutonFeature::E_ADAS_AutonDrivePath3);

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

  VeADAS_t_DM_StateTimer = 0.0;
  /* Trigger the resets for all of the sub tasks/functions as well: */
}

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
                                      std::optional<frc::DriverStation::Alliance> LeLC_e_AllianceColor )
{
  bool LeADAS_b_State1Complete = false;
  bool LeADAS_b_State2Complete = false;

  if (LeADAS_e_RobotState == E_Teleop)
  {
    // LeADAS_e_ActiveFeature = (LeADAS_b_Driver1_JoystickActive, LeADAS_e_ActiveFeature);  // What is this voodoo??
  }
  else if (LeADAS_e_RobotState == E_Auton)
  {

    frc::SmartDashboard::PutNumber("auton sub feature", (int)LeADAS_e_ActiveFeature);

    // NOTE - select auton is a switch now because faaassssttttt
    // auton selection
    switch (VeADAS_e_DriverRequestedAutonFeature)
    {
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDisabled:
      /* code */
      break;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath1:// Auton Test - Wesley
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false)){
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower2;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower2) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower3;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower3) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower4;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower4) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath2:// L_Preload
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower1;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower1) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    case E_ADAS_AutonDrivePath3:// Recall
      if ((LeADAS_e_ActiveFeature == E_ADAS_Disabled) && (VeADAS_b_StateComplete == false) && (VeADAS_b_AutonOncePerTrigger == false))
      {
        LeADAS_e_ActiveFeature = E_ADAS_DM_PathFollower5;
      }
      else if ((LeADAS_e_ActiveFeature == E_ADAS_DM_PathFollower5) && (VeADAS_b_StateComplete == true))
      {
        LeADAS_e_ActiveFeature = E_ADAS_Disabled;
        VeADAS_b_StateComplete = true;
        VeADAS_b_AutonOncePerTrigger = true;
      }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      break;
    default:
      break;
    }

    // our active feature table, autons set which one they want
    switch (LeADAS_e_ActiveFeature)
    {
    // all 5 path follower features will just flow down to the function since theres no breaks
    case E_ADAS_DM_PathFollower1:
    case E_ADAS_DM_PathFollower2:
    case E_ADAS_DM_PathFollower3:
    case E_ADAS_DM_PathFollower4:
    case E_ADAS_DM_PathFollower5:
      VeADAS_b_StateComplete = ADAS_DM_PathFollower(L_Pct_FwdRev,
                                                    L_Pct_Strafe,
                                                    L_Pct_Rotate,
                                                    LeADAS_Deg_DesiredPose,
                                                    LeADAS_b_SD_RobotOriented,
                                                    L_L_X_FieldPos,
                                                    L_L_Y_FieldPos,
                                                    L_Deg_GyroAngleDeg,
                                                    LeADAS_e_ActiveFeature,
                                                    LeLC_e_AllianceColor);
      break;

    default:
      break;
    }
  }

  return (LeADAS_e_ActiveFeature);
}
