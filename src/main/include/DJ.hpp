/*
  DJ.hpp

  Created on: January 31, 2024

  Author: Lauren 

*/
#include "rev/CANSparkMax.h"
#include "Enums.hpp"

extern TeAmp_MotorControl      VsAmp_s_Motors;

void Amp_MotorConfigsCal(     rev::SparkMaxPIDController m_ElevatorPID,
                              rev::SparkMaxPIDController m_WristPID,
                              rev::SparkMaxPIDController m_IntakePID);
 
void Amp_MotorConfigsInit(    rev::SparkMaxPIDController m_ElevatorPID,
                              rev::SparkMaxPIDController m_WristPID,
                              rev::SparkMaxPIDController m_IntakePID);