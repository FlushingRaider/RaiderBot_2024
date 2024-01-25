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

