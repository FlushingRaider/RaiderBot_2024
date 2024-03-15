// /*
//   DataLogger.cpp

//   Created on: Jan 20, 2024
//   Author: 5561

//   This file contains functions related to the capturing of data from the robot 
//   that can be used for playback.
//  */

// #include "DataLogger.hpp"

// // Joystick one 

// wpi::log::DoubleLogEntry joy1_leftstick_x;
// wpi::log::DoubleLogEntry joy1_leftstick_y;

// wpi::log::DoubleLogEntry joy1_rightstick_x;
// wpi::log::DoubleLogEntry joy1_rightstick_y;

// wpi::log::DoubleLogEntry joy1_boost; 


// wpi::log::DoubleArrayLogEntry    log_swerve_WheelAngleFwd; 
// wpi::log::DoubleArrayLogEntry    log_swerve_WheelAngleRev;
// wpi::log::DoubleArrayLogEntry    log_swerve_WheelSpeedCmnd;
// wpi::log::DoubleArrayLogEntry    log_swerve_WheelAngleCmnd;

// wpi::log::DoubleArrayLogEntry    log_encoders_wheelangleconverted;
// wpi::log::DoubleArrayLogEntry    log_encoders_wheelVelocity;

// /******************************************************************************
//  * Function:     DataLogRobotInit
//  *
//  * Description:  Initialize the variables, etc. needed for data logging.
//  *               To be called once at robot init.
//  ******************************************************************************/
// void DataLogRobotInit()
//   {
//     // Starts recording to data log
//     frc::DataLogManager::Start();

//     // Record both DS control and joystick data
//     frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

//     // Set up custom log entries
//     wpi::log::DataLog& log = frc::DataLogManager::GetLog();
//     joy1_leftstick_x = wpi::log::DoubleLogEntry(log, "/joy1/leftstick_x");
//     joy1_leftstick_y = wpi::log::DoubleLogEntry(log, "/joy1/leftstick_y");

//     joy1_rightstick_x = wpi::log::DoubleLogEntry(log, "/joy1/rightstick_x");
//     joy1_rightstick_y = wpi::log::DoubleLogEntry(log, "/joy1/rightstick_y");

//     joy1_boost = wpi::log::DoubleLogEntry(log, "/joy1/boost");

//     log_swerve_WheelAngleFwd = wpi::log::DoubleArrayLogEntry(log, "/swerve/WheelAngleFwd");
//     log_swerve_WheelAngleRev = wpi::log::DoubleArrayLogEntry(log, "/swerve/WheelAngleRev");
//     log_swerve_WheelSpeedCmnd = wpi::log::DoubleArrayLogEntry(log, "/swerve/WheelSpeedCmnd");
//     log_swerve_WheelAngleCmnd = wpi::log::DoubleArrayLogEntry(log, "/swerve/WheelAngleCmnd");

//     log_encoders_wheelangleconverted = wpi::log::DoubleArrayLogEntry(log, "/encoders/wheelAngleConverted");
//     log_encoders_wheelVelocity = wpi::log::DoubleArrayLogEntry(log, "/encoders/wheelVelocity");
//   }

// /******************************************************************************
//  * Function:     DataLogRobotPeriodic
//  *
//  * Description:  Run in the periodic loop to capture data every loop the 
//  *               robot is running. This is common between teleop and auton.
//  ******************************************************************************/

// // void LogControler1() {
// //   LeftStick_X = wpi::log::DoubleLogEntry(log, "/my/double");

// // }






