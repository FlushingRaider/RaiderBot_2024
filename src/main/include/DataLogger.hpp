#ifndef datalog_h
#define datalog_h
#pragma once

#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
#include "wpi/DataLog.h"

extern wpi::log::DoubleLogEntry joy1_leftstick_x;
extern wpi::log::DoubleLogEntry joy1_leftstick_y;

extern wpi::log::DoubleLogEntry joy1_rightstick_x;
extern wpi::log::DoubleLogEntry joy1_rightstick_y;

extern wpi::log::DoubleLogEntry joy1_boost; 

extern wpi::log::DoubleArrayLogEntry    log_swerve_WheelAngleFwd; 
extern wpi::log::DoubleArrayLogEntry    log_swerve_WheelAngleRev;
extern wpi::log::DoubleArrayLogEntry    log_swerve_WheelSpeedCmnd;
extern wpi::log::DoubleArrayLogEntry    log_swerve_WheelAngleCmnd;

extern wpi::log::DoubleArrayLogEntry    log_encoders_wheelangleconverted;
extern wpi::log::DoubleArrayLogEntry    log_encoders_wheelVelocity;

void DataLogRobotInit();

#endif