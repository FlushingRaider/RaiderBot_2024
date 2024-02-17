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
double                  VeADAS_t_Amp_DropObjectTm = 0.0;  // Timer that will keep rollers on for a specific amount of time


/******************************************************************************
 * Function:     ADAS_DJ_Reset
 *
 * Description:  Reset all applicable MN variables.
 ******************************************************************************/
void ADAS_DJ_Reset(void)
{
  T_DJ_Amp_States VeADAS_e_AttndState = E_DJ_Amp_Init;
  VeADAS_t_Amp_DropObjectTm = 0.0;
}