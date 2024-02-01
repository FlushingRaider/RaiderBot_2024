/*
  DJ.cpp

  Created on: January 31, 2024
  Author: Lauren

 */

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"

/******************************************************************************
 * Function:     DJMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the shooter,underbelly, amp motors.
 ******************************************************************************/