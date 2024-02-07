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


TeAmp_MotorControl      VsAmp_s_Motors; // All of the motor commands for the Amp Mechanism motors
TeAmp_MotorControl      VsAmp_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeAmp_MotorControl      VsAmp_s_MotorsTest; // Temporary commands for the motors, not the final output
TeAmp_MotorControl      VsAmp_s_Sensors; // All of the sensor values for the Amp Mechanism motors

double                  VaAmp_k_ElevatorPID_Gx[E_PID_SparkMaxCalSz];
double                  VaAmp_k_WristPID_Gx[E_PID_SparkMaxCalSz];
double                  VaAmp_k_IntakePID_Gx[E_PID_SparkMaxCalSz];




#ifdef Amp_Test
bool                    VeAmp_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool                    VeAmp_b_TestState = false;
#endif

/******************************************************************************
 * Function:     Amp_MotorConfigsInit
 *
 * Description:  Contains the motor configurations for the amp mechinism motors.
 ******************************************************************************/
 void Amp_MotorConfigsInit(rev::SparkMaxPIDController m_ElevatorPID,
                            rev::SparkMaxPIDController m_WristPID,
                           rev::SparkMaxPIDController m_IntakePID)
  {
    TeDJ_Amp_e_AmpActuator LeAmp_i_Index;
    T_PID_Cal LeAmp_i_Index3 = E_P_Gx;

  // set PID coefficients
    m_ElevatorPID.SetP(KaDJ_Amp_k_ElevatorPID_Gx[E_kP]);
    m_ElevatorPID.SetI(KaDJ_Amp_k_ElevatorPID_Gx[E_kI]);
    m_ElevatorPID.SetD(KaDJ_Amp_k_ElevatorPID_Gx[E_kD]);
    m_ElevatorPID.SetIZone(KaDJ_Amp_k_ElevatorPID_Gx[E_kIz]);
    m_ElevatorPID.SetFF(KaDJ_Amp_k_ElevatorPID_Gx[E_kFF]);
    m_ElevatorPID.SetOutputRange(KaDJ_Amp_k_ElevatorPID_Gx[E_kMinOutput], KaDJ_Amp_k_ElevatorPID_Gx[E_kMaxOutput]);
  
    m_WristPID.SetP(KaDJ_Amp_k_WristPID_Gx[E_kP]);
    m_WristPID.SetI(KaDJ_Amp_k_WristPID_Gx[E_kI]);
    m_WristPID.SetD(KaDJ_Amp_k_WristPID_Gx[E_kD]);
    m_WristPID.SetIZone(KaDJ_Amp_k_WristPID_Gx[E_kIz]);
    m_WristPID.SetFF(KaDJ_Amp_k_WristPID_Gx[E_kFF]);
    m_WristPID.SetOutputRange(KaDJ_Amp_k_WristPID_Gx[E_kMinOutput], KaDJ_Amp_k_WristPID_Gx[E_kMaxOutput]);  
  
    m_IntakePID.SetP(KaDJ_Amp_k_IntakePID_Gx[E_kP]);
    m_IntakePID.SetI(KaDJ_Amp_k_IntakePID_Gx[E_kI]);
    m_IntakePID.SetD(KaDJ_Amp_k_IntakePID_Gx[E_kD]);
    m_IntakePID.SetIZone(KaDJ_Amp_k_IntakePID_Gx[E_kIz]);
    m_IntakePID.SetFF(KaDJ_Amp_k_IntakePID_Gx[E_kFF]);
    m_IntakePID.SetOutputRange(KaDJ_Amp_k_IntakePID_Gx[E_kMinOutput], KaDJ_Amp_k_IntakePID_Gx[E_kMaxOutput]);    
  
   for (LeAmp_i_Index = E_Amp_Elevator;
       LeAmp_i_Index < E_Amp_Sz;
       LeAmp_i_Index = TeDJ_Amp_e_AmpActuator(int(LeAmp_i_Index) + 1))
    {
      VsAmp_s_Motors.k_MotorCmnd[LeAmp_i_Index] = 0.0;
      VsAmp_s_MotorsTemp.k_MotorCmnd[LeAmp_i_Index] = 0.0;
      VsAmp_s_MotorsTest.k_MotorCmnd[LeAmp_i_Index] = 0.0;
    }
  
  }




/******************************************************************************
 * Function:     Amp_MotorConfigsCal
 *
 * Description:  Contains the motor configurations for the amp mechinism motors.  This 
 *               allows for rapid calibration, but must not be used for comp. 
 *               For PIDs
 ******************************************************************************/
void Amp_MotorConfigsCal(     rev::SparkMaxPIDController m_ElevatorPID,
                              rev::SparkMaxPIDController m_WristPID,
                              rev::SparkMaxPIDController m_IntakePID)
  { //TODO - old code had smart dashboard PID stuff sooo This is going to need Shuffleboard equivlents
  }
