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
#include "SpeakerCntrl.hpp"

T_DJ_Amp_States  VeADAS_e_Amp_SchedState = E_DJ_Amp_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
TeSPK_CtrlStates VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
TeCLMR_CtrlStates VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
bool VeADAS_b_Amp_DropObject = false;
bool VeADAS_b_SPK_Release = false;
double VeADAS_t_Amp_DropObjectTm = 0.0; // Timer that will keep rollers on for a specific amount of time
double VeADAS_t_SPK_ReleaseTm = 0.0; // Timer that will keep rollers on for a specific amount of time

/******************************************************************************
 * Function:     ADAS_DJ_Reset
 *
 * Description:  Reset all applicable MN variables.
 ******************************************************************************/
void ADAS_DJ_Reset(void)
{
  VeADAS_e_Amp_SchedState = E_DJ_Amp_Init;
  VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Init;
  VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_Init;
  VeADAS_t_Amp_DropObjectTm = 0.0;
  VeADAS_t_SPK_ReleaseTm = 0.0;
}

/******************************************************************************
 * Function:    DJ_ScheduelerTeleop
 * Made By:     Lauren
 * Description: Determines scheduled state of the DJ mechanisms, speaker and amp
 ******************************************************************************/
bool DJ_ScheduelerTeleop(void)
{
  bool LeADAS_b_Amp_DropObject = false;
  bool LeADAS_b_DJ_StateComplete = false;
  bool LeADAS_b_SPK_Release = false;

/////////////////     AMP CONTROL      /////////////////   coordinates what the amp mechanism does when a button is pressed

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
  else
  {
    /* No updates */
  }

  if (VsCONT_s_DriverInput.b_Amp_Score == true)
  {
    VeADAS_b_Amp_DropObject = true;
    VeADAS_t_Amp_DropObjectTm = C_ExeTime;
  }
  else if ((VeADAS_t_Amp_DropObjectTm > 0) &&
           (VeADAS_t_Amp_DropObjectTm <= KeAmp_t_IntakeOnTm))
  {
    LeADAS_b_Amp_DropObject = VeADAS_b_Amp_DropObject;
    VeADAS_t_Amp_DropObjectTm += C_ExeTime;
  }
  else
  {
    LeADAS_b_Amp_DropObject = false;
    VeADAS_t_Amp_DropObjectTm = 0.0;
  }

  if (VeADAS_e_Amp_SchedState == VeAmp_e_AttndState)
  {
    LeADAS_b_DJ_StateComplete = true;
  }

  VeADAS_b_Amp_DropObject = LeADAS_b_Amp_DropObject;

  return (LeADAS_b_DJ_StateComplete);

/////////////////     SpK CONTROL      /////////////////  coordinates what the Speaker mechanism does when a button is pressed

  if (VsCONT_s_DriverInput.b_SPK_DrivingPosition == true)
  {
    VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Driving;
  }
  else if (VsCONT_s_DriverInput.b_SPK_Intake == true)
  {
    VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Intake;
  }
  else if (VsCONT_s_DriverInput.b_SPK_PreScore == true)
  {
    VeADAS_e_SPK_SchedState = E_SPK_Ctrl_PreScore;
  }
  else
  {
    /* No updates */
  }

  if (VsCONT_s_DriverInput.b_SPK_Score == true)
  {
    VeADAS_b_SPK_Release = true;
    VeADAS_t_SPK_ReleaseTm = C_ExeTime;
  }
  else if ((VeADAS_t_SPK_ReleaseTm > 0) &&
           (VeADAS_t_SPK_ReleaseTm <= KeSPK_t_ShooterOnTm))
  {
    LeADAS_b_SPK_Release = VeADAS_b_SPK_Release;
    VeADAS_t_SPK_ReleaseTm += C_ExeTime;
  }
  else
  {
    LeADAS_b_SPK_Release = false;
    VeADAS_t_SPK_ReleaseTm = 0.0;
  }

  if (VeADAS_e_SPK_SchedState == VeSPK_e_AttndState)
  {
    LeADAS_b_DJ_StateComplete = true;
  }

  VeADAS_b_SPK_Release = LeADAS_b_SPK_Release;

  return (LeADAS_b_DJ_StateComplete);

  /////////////////     Climb CONTROL      /////////////////  coordinates what the Climb mechanism does when a button is pressed

}