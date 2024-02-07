/*
  DataLogger.cpp

  Created on: Jan 20, 2024
  Author: 5561

  This file contains functions related to the capturing of data from the robot 
  that can be used for playback.
 */

#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
#include "wpi/DataLog.h"

wpi::log::BooleanLogEntry myBooleanLog;
wpi::log::DoubleLogEntry myDoubleLog;
wpi::log::StringLogEntry myStringLog;

/******************************************************************************
 * Function:     DataLogRobotInit
 *
 * Description:  Initialize the variables, etc. needed for data logging.
 *               To be called once at robot init.
 ******************************************************************************/
void DataLogRobotInit()
  {
    // Starts recording to data log
    frc::DataLogManager::Start();

    // Record both DS control and joystick data
    frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

    // Set up custom log entries
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    myBooleanLog = wpi::log::BooleanLogEntry(log, "/my/boolean");
    myDoubleLog = wpi::log::DoubleLogEntry(log, "/my/double");
    myStringLog = wpi::log::StringLogEntry(log, "/my/string");
  }

/******************************************************************************
 * Function:     DataLogRobotPeriodic
 *
 * Description:  Run in the periodic loop to capture data every loop the 
 *               robot is running. This is common between teleop and auton.
 ******************************************************************************/
void DataLogRobotPeriodic()
  {
    // Only log when necessary
    myBooleanLog.Append(true);
    myDoubleLog.Append(3.5);
    myStringLog.Append("wow!");
  }