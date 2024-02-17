/*
  ADAS_DJ.cpp

  Created on: Feb 17, 2024
  Author: Lauren

  ADAS (Advanced Driver-Assistance Systems) DJ
  Contains the logic and code used for the Amp and Speaker mech. control:
    - Schedules states in a non-linear state machine
 */

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "control_pid.hpp"
#include "rev/CANSparkMax.h"
#include "Const.hpp"
#include "Lookup.hpp"
#include "DriveControl.hpp"
#include "Amp.hpp"
#include "Driver_inputs.hpp"

T_DJ_Amp_States VeADAS_e_Amp_SchedState = E_DJ_Amp_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
bool VeADAS_b_Amp_DropObject = false;
double VeADAS_t_Amp_DropObjectTm = 0.0; // Timer that will keep rollers on for a specific amount of time

/******************************************************************************
 * Function:     ADAS_DJ_Reset
 *
 * Description:  Reset all applicable MN variables.
 ******************************************************************************/
void ADAS_DJ_Reset(void)
{
  T_DJ_Amp_States VeADAS_Amp_e_AttndState = E_DJ_Amp_Init;
  VeADAS_t_Amp_DropObjectTm = 0.0;
}

/******************************************************************************
 * Function:    DJ_ScheduelerTeleop
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator
 ******************************************************************************/
bool DJ_ScheduelerTeleop(void)
{
  bool LeADAS_b_Amp_DropObject = false;
  bool LeADAS_b_DJ_StateComplete = false;

  if (VsCONT_s_DriverInput.b_Amp_DrivingPosition == true)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_Driving;
  }
  else if (VsCONT_s_DriverInput.b_Amp_Intake == true)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_Intake;
  }
  else if (VsCONT_s_DriverInput.b_Amp_PreScore == true)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_PreScore;
  }
  else if (VsCONT_s_DriverInput.b_Amp_Score == true)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_Score;
  }
  else
  {
    /* No updates */
  }

  if (VsCONT_s_DriverInput.b_DropGamePieceFast == true)
  {
    VeADAS_b_Amp_DropObject = true;
    VeADAS_t_Amp_DropObjectTm = C_ExeTime;
  }
  else if ((VeADAS_t_MAN_DropObjectTm > 0) &&
           (VeADAS_t_MAN_DropObjectTm <= KeMAN_t_GripperOnTm))
  {
    LeADAS_b_MAN_DropObjectSlow = VeADAS_b_MAN_DropObjectSlow;
    LeADAS_b_MAN_DropObjectFast = VeADAS_b_MAN_DropObjectFast;
    VeADAS_t_MAN_DropObjectTm += C_ExeTime;
  }
  else
  {
    LeADAS_b_MAN_DropObjectSlow = false;
    LeADAS_b_MAN_DropObjectFast = false;
    VeADAS_t_MAN_DropObjectTm = 0.0;
  }

  if (VeADAS_e_MAN_SchedState == VeMAN_e_AttndState)
  {
    LeADAS_b_MAN_StateComplete = true;
  }

  VeADAS_b_MAN_DropObjectSlow = LeADAS_b_MAN_DropObjectSlow;
  VeADAS_b_MAN_DropObjectFast = LeADAS_b_MAN_DropObjectFast;

  return (LeADAS_b_MAN_StateComplete);
}