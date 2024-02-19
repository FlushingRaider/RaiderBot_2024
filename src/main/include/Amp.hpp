/*
  Amp.hpp

  Created on: January 31, 2024

  Author: Lauren

*/
#include "rev/CANSparkMax.h"
#include "Enums.hpp"

extern TeAmp_MotorControl VsAmp_s_Motors;
extern T_DJ_Amp_States VeAmp_e_AttndState;
extern T_DJ_Amp_States VeAmp_e_CmndState;
extern TeAmp_MotorControl VsAmp_s_MotorsTest;
extern TeAmp_MotorControl VsAmp_s_MotorsTemp;

extern TsDJ_Sensor VsAmp_s_Sensors;
extern double VaAmp_In_ElevatorError;

void Amp_MotorConfigsCal(rev::SparkMaxPIDController m_ElevatorPID,
                         rev::SparkMaxPIDController m_WristPID,
                         rev::SparkMaxPIDController m_IntakePID);

void Amp_MotorConfigsInit(rev::SparkMaxPIDController m_ElevatorPID,
                          rev::SparkMaxPIDController m_WristPID,
                          rev::SparkMaxPIDController m_IntakePID);

void Amp_ControlInit();

void Amp_ControlManualOverride(RobotUserInput *LsCONT_s_DriverInput);

void Amp_ControlMain(T_DJ_Amp_States LeDJ_Amp_e_SchedState,
                     bool LeDJ_Amp_b_TestPowerOverride,
                     bool LeDJ_Amp_b_DropObject);