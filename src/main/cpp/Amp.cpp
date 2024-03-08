/*
  DJ.cpp

  Created on: January 31, 2024
  Author: Lauren

 */

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"
#include "Amp.hpp"
#include "ADAS_DJ.hpp"

TeAmp_MotorControl VsAmp_s_Motors;     // All of the motor commands for the Amp Mechanism motors
TeAmp_MotorControl VsAmp_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeAmp_MotorControl VsAmp_s_MotorsTest; // Temporary commands for the motors, not the final output

TsAmp_Sensor VsAmp_s_Sensors; // All of the sensor values for the manipulator/intake motors

T_DJ_Amp_States VeAmp_e_CmndState  = E_DJ_Amp_Init;  // What is our next/current step?
T_DJ_Amp_States VeAmp_e_AttndState = E_DJ_Amp_Init; // What is our desired end state?

double VaAmp_k_ElevatorPID_Gx[E_PID_SparkMaxCalSz];
double VaAmp_k_WristPID_Gx[E_PID_SparkMaxCalSz];
double VaAmp_k_IntakePID_Gx[E_PID_SparkMaxCalSz];

bool VeAmp_b_CriteriaMet = false;
double VeAmp_t_TransitionTime = 0;
TeAMP_e_WristReset VeAmp_e_WristResetSt = E_AMP_WristReseted;
double VeAmp_t_WristResetTimer = 0;
double VeAmp_t_WristResetTimer2 = 0;
bool VeAmp_b_WristEncoderReset = false;
bool VeAmp_b_ElevatorEncoderReset = false;
bool VeAmp_b_ElevatorResetLatch = false;

#ifdef AMP_Test
bool VeAmp_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool VeAmp_b_TestState = false;
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

  VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Elevator] = KaDJ_Amp_InS_ElevatorRate[E_DJ_Amp_Init][E_DJ_Amp_Init];
  VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Wrist] = KaDJ_Amp_DegS_WristRate[E_DJ_Amp_Init][E_DJ_Amp_Init];

#ifdef AMP_Test
  T_PID_SparkMaxCal LeAmp_i_Index2 = E_kP;

  for (LeAmp_i_Index2 = E_kP;
       LeAmp_i_Index2 < E_PID_SparkMaxCalSz;
       LeAmp_i_Index2 = T_PID_SparkMaxCal(int(LeAmp_i_Index2) + 1))
  {
    VaAmp_k_WristPID_Gx[LeAmp_i_Index2] = KaDJ_Amp_k_WristPID_Gx[LeAmp_i_Index2];
    VaAmp_k_IntakePID_Gx[LeAmp_i_Index2] = KaDJ_Amp_k_IntakePID_Gx[LeAmp_i_Index2];
    VaAmp_k_ElevatorPID_Gx[LeAmp_i_Index2] = KaDJ_Amp_k_ElevatorPID_Gx[LeAmp_i_Index2];
  }

  // display PID coefficients on SmartDashboard

  frc::SmartDashboard::PutNumber("P Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("KaDJ_Amp_InS_ElevatorRate", KaDJ_Amp_InS_ElevatorRate[E_DJ_Amp_Init][E_DJ_Amp_Init]);
  frc::SmartDashboard::PutNumber("KaDJ_Amp_DegS_WristRate", KaDJ_Amp_DegS_WristRate[E_DJ_Amp_Init][E_DJ_Amp_Init]);

  // display target positions/speeds
  frc::SmartDashboard::PutNumber("Set Position Wrist", 0);
  frc::SmartDashboard::PutNumber("Set Elevator", 0);
  frc::SmartDashboard::PutNumber("Set Intake Power", 0);
#endif
}

/******************************************************************************
 * Function:     Amp_MotorConfigsCal
 *
 * Description:  Contains the motor configurations for the amp mechinism motors.  This
 *               allows for rapid calibration, but must not be used for comp.
 *               For PIDs
 ******************************************************************************/
void Amp_MotorConfigsCal(rev::SparkMaxPIDController m_ElevatorPID,
                         rev::SparkMaxPIDController m_WristPID,
                         rev::SparkMaxPIDController m_IntakePID)
{ 
  // read PID coefficients from SmartDashboard
  #ifdef AMP_Test
  double L_p_Elevator   = frc::SmartDashboard::GetNumber("P Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kP]);
  double L_i_Elevator   = frc::SmartDashboard::GetNumber("I Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kI]);
  double L_d_Elevator   = frc::SmartDashboard::GetNumber("D Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kD]);
  double L_iz_Elevator  = frc::SmartDashboard::GetNumber("I Zone - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kIz]);
  double L_max_Elevator = frc::SmartDashboard::GetNumber("Max Output - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kMaxOutput]);
  double L_min_Elevator = frc::SmartDashboard::GetNumber("Min Output - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_kMinOutput]);

  double L_p_Wrist   = frc::SmartDashboard::GetNumber("P Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kP]);
  double L_i_Wrist   = frc::SmartDashboard::GetNumber("I Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kI]);
  double L_d_Wrist   = frc::SmartDashboard::GetNumber("D Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kD]);
  double L_iz_Wrist  = frc::SmartDashboard::GetNumber("I Zone - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kIz]);
  double L_max_Wrist = frc::SmartDashboard::GetNumber("Max Output - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kMaxOutput]);
  double L_min_Wrist = frc::SmartDashboard::GetNumber("Min Output - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kMinOutput]);

  double L_p_Intake   = frc::SmartDashboard::GetNumber("P Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kP]);
  double L_i_Intake   = frc::SmartDashboard::GetNumber("I Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kI]);
  double L_d_Intake   = frc::SmartDashboard::GetNumber("D Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kD]);
  double L_iz_Intake  = frc::SmartDashboard::GetNumber("I Zone - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kIz]);
  double L_max_Intake = frc::SmartDashboard::GetNumber("Max Output - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kMaxOutput]);
  double L_min_Intake = frc::SmartDashboard::GetNumber("Min Output - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kMinOutput]);
 
  VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Elevator]   = frc::SmartDashboard::GetNumber("Set Elevator", 0);
  VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Wrist] = frc::SmartDashboard::GetNumber("Set Position Wrist", 0);
  VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Intake] = frc::SmartDashboard::GetNumber("Set Intake Power", 0);

  if(L_p_Elevator != VaAmp_k_ElevatorPID_Gx[E_kP])   { m_ElevatorPID.SetP(L_p_Elevator); VaAmp_k_ElevatorPID_Gx[E_kP] = L_p_Elevator; }
  if(L_i_Elevator != VaAmp_k_ElevatorPID_Gx[E_kI])   { m_ElevatorPID.SetI(L_i_Elevator); VaAmp_k_ElevatorPID_Gx[E_kI] = L_i_Elevator; }
  if(L_d_Elevator != VaAmp_k_ElevatorPID_Gx[E_kD])   { m_ElevatorPID.SetD(L_d_Elevator); VaAmp_k_ElevatorPID_Gx[E_kD] = L_d_Elevator; }
  if(L_iz_Elevator != VaAmp_k_ElevatorPID_Gx[E_kIz]) { m_ElevatorPID.SetIZone(L_iz_Elevator); VaAmp_k_ElevatorPID_Gx[E_kIz] = L_iz_Elevator; }
  if((L_max_Elevator != VaAmp_k_ElevatorPID_Gx[E_kMaxOutput]) || (L_min_Elevator != VaAmp_k_ElevatorPID_Gx[E_kMinOutput])) { m_ElevatorPID.SetOutputRange(L_min_Elevator, L_max_Elevator); VaAmp_k_ElevatorPID_Gx[E_kMinOutput] = L_min_Elevator; VaAmp_k_ElevatorPID_Gx[E_kMaxOutput] = L_max_Elevator; }

  if(L_p_Wrist != VaAmp_k_WristPID_Gx[E_kP])   { m_WristPID.SetP(L_p_Wrist); VaAmp_k_WristPID_Gx[E_kP] = L_p_Wrist; }
  if(L_i_Wrist != VaAmp_k_WristPID_Gx[E_kI])   { m_WristPID.SetI(L_i_Wrist); VaAmp_k_WristPID_Gx[E_kI] = L_i_Wrist; }
  if(L_d_Wrist != VaAmp_k_WristPID_Gx[E_kD])   { m_WristPID.SetD(L_d_Wrist); VaAmp_k_WristPID_Gx[E_kD] = L_d_Wrist; }
  if(L_iz_Wrist != VaAmp_k_WristPID_Gx[E_kIz]) { m_WristPID.SetIZone(L_iz_Wrist); VaAmp_k_WristPID_Gx[E_kIz] = L_iz_Wrist; }
  if((L_max_Wrist != VaAmp_k_WristPID_Gx[E_kMaxOutput]) || (L_min_Wrist != VaAmp_k_WristPID_Gx[E_kMinOutput])) { m_WristPID.SetOutputRange(L_min_Wrist, L_max_Wrist); VaAmp_k_WristPID_Gx[E_kMinOutput] = L_min_Wrist; VaAmp_k_WristPID_Gx[E_kMaxOutput] = L_max_Wrist; }

  if(L_p_Intake != VaAmp_k_IntakePID_Gx[E_kP])   { m_IntakePID.SetP(L_p_Intake); VaAmp_k_IntakePID_Gx[E_kP] = L_p_Intake; }
  if(L_i_Intake != VaAmp_k_IntakePID_Gx[E_kI])   { m_IntakePID.SetI(L_i_Intake); VaAmp_k_IntakePID_Gx[E_kI] = L_i_Intake; }
  if(L_d_Intake != VaAmp_k_IntakePID_Gx[E_kD])   { m_IntakePID.SetD(L_d_Intake); VaAmp_k_IntakePID_Gx[E_kD] = L_d_Intake; }
  if(L_iz_Intake != VaAmp_k_IntakePID_Gx[E_kIz]) { m_IntakePID.SetIZone(L_iz_Intake); VaAmp_k_IntakePID_Gx[E_kIz] = L_iz_Intake; }
  if((L_max_Intake != VaAmp_k_IntakePID_Gx[E_kMaxOutput]) || (L_min_Intake != VaAmp_k_IntakePID_Gx[E_kMinOutput])) { m_IntakePID.SetOutputRange(L_min_Intake, L_max_Intake); VaAmp_k_IntakePID_Gx[E_kMinOutput] = L_min_Intake; VaAmp_k_IntakePID_Gx[E_kMaxOutput] = L_max_Intake; }

  VsAmp_s_MotorsTest.k_MotorRampRate[E_DJ_Amp_m_Elevator] = frc::SmartDashboard::GetNumber("KaDJ_Amp_InS_ElevatorRate", VsAmp_s_MotorsTest.k_MotorRampRate[E_DJ_Amp_m_Elevator]);
  VsAmp_s_MotorsTest.k_MotorRampRate[E_DJ_Amp_m_Wrist] = frc::SmartDashboard::GetNumber("KaDJ_Amp_DegS_WristRate", VsAmp_s_MotorsTest.k_MotorRampRate[E_DJ_Amp_m_Wrist]);
   #endif
}

/******************************************************************************
 * Function:     Amp_ControlInit
 *
 * Description:  Initialization function for the amp moter controls.
 ******************************************************************************/
void Amp_ControlInit()
{
  VeAmp_e_CmndState = E_DJ_Amp_Init;
  VeAmp_e_AttndState = E_DJ_Amp_Init;

  VeAmp_b_CriteriaMet = false;
  VeAmp_t_TransitionTime = 0.0;
  VeAmp_t_WristResetTimer = 0.0;
  VeAmp_e_WristResetSt = E_AMP_WristReseted;
  VeAmp_b_WristEncoderReset = false;
  VeAmp_t_WristResetTimer2 = 0.0;
  VeAmp_b_ElevatorEncoderReset = false;
  VeAmp_b_ElevatorResetLatch = false;
}

/******************************************************************************
 * Function:     Amp_ControlManualOverride
 *
 * Description:  Manual override control used during the FRC test section. Use incase of Y2K -J
 ******************************************************************************/
void Amp_ControlManualOverride(RobotUserInput *LsCONT_s_DriverInput)
{
  TeDJ_Amp_e_AmpActuator LeMAN_i_Index;

  for (LeMAN_i_Index = E_Amp_Elevator;
       LeMAN_i_Index < E_Amp_Sz;
       LeMAN_i_Index = TeDJ_Amp_e_AmpActuator(int(LeMAN_i_Index) + 1))
  {
    VsAmp_s_Motors.k_MotorTestPower[LeMAN_i_Index] = 0.0;
  }

  if ((VsAmp_s_Sensors.b_Amp_ObjDetected == false))
  {
    /* Don't allow the intake to progress downward if the limit switch is depressed */
    VsAmp_s_Motors.k_MotorTestPower[E_Amp_Intake] = LsCONT_s_DriverInput->Pct_Amp_Intake_Test * KaDJ_Amp_k_TestPower[E_Amp_Intake];
  }

  VsAmp_s_Motors.k_MotorTestPower[E_Amp_Wrist] = LsCONT_s_DriverInput->Pct_Amp_Wrist_Test * KaDJ_Amp_k_TestPower[E_Amp_Wrist];

  if ((VsAmp_s_Sensors.b_ElevatorSwitch == false) ||
      (LsCONT_s_DriverInput->Pct_Amp_Elevator_Test < 0))
  {
    /* Don't allow the elevator to progress downward if the limit switch is depressed */
    VsAmp_s_Motors.k_MotorTestPower[E_Amp_Elevator] = LsCONT_s_DriverInput->Pct_Amp_Elevator_Test * KaDJ_Amp_k_TestPower[E_Amp_Elevator]; 
  }
}

/******************************************************************************
 * Function:     Update_Command_Attained_State
 *
 * Description:  Updates the commanded and attained states for the manipulator
 ******************************************************************************/
bool Update_Command_Atained_State(bool LeAmp_b_CriteriaMet,
                                  T_DJ_Amp_States LeDJ_Amp_e_SchedState)
{
  T_DJ_Amp_States LeDJ_Amp_e_CmndState = VeAmp_e_CmndState;

  if (LeAmp_b_CriteriaMet == true)
  {
    VeAmp_e_AttndState = LeDJ_Amp_e_CmndState;
    LeAmp_b_CriteriaMet = false;
  }

  if ((LeDJ_Amp_e_SchedState != VeAmp_e_AttndState) &&
      (VeAmp_e_CmndState == VeAmp_e_AttndState))
  {
    LeDJ_Amp_e_CmndState = KaDJ_Amp_e_ControllingTable[LeDJ_Amp_e_SchedState][VeAmp_e_AttndState];
  }

  VeAmp_e_CmndState = LeDJ_Amp_e_CmndState;

  return (LeAmp_b_CriteriaMet);
}

/******************************************************************************
 * Function:     CmndStateReachedAmp
 *
 * Description:  Checks to see if we have reached the desired commanded state
 ******************************************************************************/
bool CmndStateReachedAmp(T_DJ_Amp_States LeDJ_Amp_e_CmndState)
{
  bool LeAmp_b_CriteriaMet = false;

  VeAmp_t_TransitionTime += C_ExeTime;

  if ((VeAmp_t_TransitionTime >= KeDJ_Amp_t_StateTimeOut) ||

      ((VsAmp_s_Sensors.In_Elevator <= (KaDJ_Amp_In_ElevatorPosition[LeDJ_Amp_e_CmndState] + KaDJ_Amp_In_ElevatorDb[LeDJ_Amp_e_CmndState])) &&
       (VsAmp_s_Sensors.In_Elevator >= (KaDJ_Amp_In_ElevatorPosition[LeDJ_Amp_e_CmndState] - KaDJ_Amp_In_ElevatorDb[LeDJ_Amp_e_CmndState])) &&

       (VsAmp_s_Sensors.Deg_Wrist <= (KaDJ_Amp_Deg_WristAngle[LeDJ_Amp_e_CmndState] + KaDJ_Amp_Deg_WristDb[LeDJ_Amp_e_CmndState])) &&
       (VsAmp_s_Sensors.Deg_Wrist >= (KaDJ_Amp_Deg_WristAngle[LeDJ_Amp_e_CmndState] - KaDJ_Amp_Deg_WristDb[LeDJ_Amp_e_CmndState]))))
  {
    LeAmp_b_CriteriaMet = true;
    VeAmp_t_TransitionTime = 0.0;
  }

  return (LeAmp_b_CriteriaMet);
}

/******************************************************************************
 * Function:     Update_Amp_Actuators
 *
 * Description:  Updates the intermediate state of the actuartors for the
 *               Amp control
 ******************************************************************************/
void Update_Amp_Actuators(T_DJ_Amp_States LeDJ_Amp_e_CmndState,
                          T_DJ_Amp_States LeDJ_Amp_e_AttndState)
{
  double LeAmp_InS_ElevatorRate = 0.0;
  double LeAmp_DegS_WristRate = 0.0;

  /* Here we are attempting to allow the driver to manually reset the elevator back to the init position.  
     This involves open loop pulling it down along with looking at the switch and also resetting the encoder */
  if (VsAmp_s_Sensors.b_ElevatorSwitch == true)
  {
    VeAmp_b_ElevatorResetLatch = true;
  }
  else
  {
    if (LeDJ_Amp_e_AttndState != E_DJ_Amp_Init)
    {
      VeAmp_b_ElevatorResetLatch = false;
    }
  }

  if ((VsCONT_s_DriverInput.b_Amp_DrivingPosition == true) && 
      (LeDJ_Amp_e_CmndState == E_DJ_Amp_Init && LeDJ_Amp_e_AttndState == E_DJ_Amp_Init) &&
      (VeAmp_b_ElevatorResetLatch == false))
  {
  VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator] = KeSPK_k_ElevatorResetPwr;
  VsAmp_s_Motors.e_MotorControlType[E_Amp_Elevator] = E_MotorControlPctCmnd;
  VeAmp_b_ElevatorEncoderReset = true;
  }
  else
  {
  LeAmp_InS_ElevatorRate = KaDJ_Amp_InS_ElevatorRate[LeDJ_Amp_e_CmndState][LeDJ_Amp_e_AttndState];

  VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator] = RampTo(KaDJ_Amp_In_ElevatorPosition[LeDJ_Amp_e_CmndState] / KeENC_k_AMP_ElevatorRatio,
                                                          VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator],
                                                          LeAmp_InS_ElevatorRate);
  VsAmp_s_Motors.e_MotorControlType[E_Amp_Elevator] = E_MotorControlPosition;
  VeAmp_b_ElevatorEncoderReset = false;
  }





  /* Additional logic for wrist is to allow it to be reseted.  This is to account for cases where the belt has 
     slipped and the wrist is no longer in the expected location. */
  if ((VeAmp_b_WristEncoderReset == true && VeAmp_t_WristResetTimer2 >= KeSPK_t_WristResetTime2) || 
      LeDJ_Amp_e_CmndState != E_DJ_Amp_Init)
  {
    /* We only want to allow the reset to be true for one loop.  Don't want to continously reset the encoder */
    VeAmp_b_WristEncoderReset = false;
    VeAmp_t_WristResetTimer2 = 0.0;
  }
  else
  {
    VeAmp_t_WristResetTimer2 += C_ExeTime;
  }

  if (VeAmp_t_WristResetTimer >= KeSPK_t_WristResetTime)
  {
    VeAmp_e_WristResetSt = E_AMP_WristReseted;
    VeAmp_t_WristResetTimer = 0;
    VeAmp_b_WristEncoderReset = true;
    VeAmp_t_WristResetTimer2 = 0.0;
  }

  if (LeDJ_Amp_e_AttndState != E_DJ_Amp_Init ||
      VsCONT_s_DriverInput.b_Amp_DrivingPosition == true)
  {
    VeAmp_e_WristResetSt = E_AMP_WristNeedsReset;
    VeAmp_t_WristResetTimer = 0;
  }

  if ((LeDJ_Amp_e_CmndState == E_DJ_Amp_Init) && 
      (LeDJ_Amp_e_AttndState == E_DJ_Amp_Init) &&
      ((VeAmp_e_WristResetSt == E_AMP_WristNeedsReset) || (VeAmp_e_WristResetSt == E_AMP_WristResetInProc)))
  {
    VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist] = KeSPK_k_WristResetPwr;
    VsAmp_s_Motors.e_MotorControlType[E_Amp_Wrist] = E_MotorControlPctCmnd;
    VeAmp_t_WristResetTimer += C_ExeTime;
  }
  else
  {
  LeAmp_DegS_WristRate = KaDJ_Amp_DegS_WristRate[LeDJ_Amp_e_CmndState][LeDJ_Amp_e_AttndState];

  VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist] = RampTo(KaDJ_Amp_Deg_WristAngle[LeDJ_Amp_e_CmndState] / KeENC_k_AMP_WristRatio,
                                                       VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist],
                                                       LeAmp_DegS_WristRate);
  VsAmp_s_Motors.e_MotorControlType[E_Amp_Wrist] = E_MotorControlPosition;
  }

  VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake] = KaDJ_Amp_RPM_IntakePower[LeDJ_Amp_e_CmndState];
}

/******************************************************************************
 * Function:     UpdateAmp_hold_Actuator
 *
 * Description:  Updates the gripper roller control 
 *               //NOTE - not sure if we  still want to have holding power.
 *                 Not currently called
 ******************************************************************************/
void UpdateAmp_hold_Actuator(T_DJ_Amp_States LeDJ_Amp_e_CmndState,
                             T_DJ_Amp_States LeDJ_Amp_e_AttndState)
{
  double LeDJ_Amp_k_TempCmnd = 0.0;

  if ((VsAmp_s_Sensors.b_Amp_ObjDetected == false) &&
      (LeDJ_Amp_e_AttndState == E_DJ_Amp_Intake) &&
      (LeDJ_Amp_e_CmndState == E_DJ_Amp_Intake))
    {
    LeDJ_Amp_k_TempCmnd = KaDJ_Amp_RPM_IntakePower[E_DJ_Amp_Intake];
    }
  else if ((VsAmp_s_Sensors.b_Amp_ObjDetected == true)   &&
           ((LeDJ_Amp_e_AttndState == E_DJ_Amp_Intake)   ||
            (LeDJ_Amp_e_CmndState == E_DJ_Amp_Intake)    || 
            (LeDJ_Amp_e_AttndState == E_DJ_Amp_Driving)  ||
            (LeDJ_Amp_e_CmndState == E_DJ_Amp_Driving)   ||
            (LeDJ_Amp_e_AttndState == E_DJ_Amp_PreScore) ||
            (LeDJ_Amp_e_CmndState == E_DJ_Amp_PreScore)))
    {
    LeDJ_Amp_k_TempCmnd = KeDJ_Amp_k_HoldNote;
    }
  else if ((LeDJ_Amp_e_AttndState == E_DJ_Amp_Score) &&
           (LeDJ_Amp_e_CmndState == E_DJ_Amp_Score))
    {
    LeDJ_Amp_k_TempCmnd = KaDJ_Amp_RPM_IntakePower[E_DJ_Amp_Score];
    }
  else if ((LeDJ_Amp_e_AttndState == E_DJ_Amp_Trade_Off) &&
           (LeDJ_Amp_e_CmndState == E_DJ_Amp_Trade_Off)) 
    {
    LeDJ_Amp_k_TempCmnd = KaDJ_Amp_RPM_IntakePower[E_DJ_Amp_Trade_Off];
    }

  VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake] = LeDJ_Amp_k_TempCmnd;
}

/******************************************************************************
 * Function:     Amp_ControlMain
 *
 * Description:  Main calling function for manipulator control.
 ******************************************************************************/
void Amp_ControlMain(T_DJ_Amp_States LeDJ_Amp_e_SchedState,
                     bool LeDJ_Amp_b_TestPowerOverride)
{

  if (LeDJ_Amp_b_TestPowerOverride == true)
  {
    // Do nothing.  Robot is in test state using power commands for all the acutators
  }
  else if (VeAmp_b_TestState == true)
  {
    /* Only used for testing/calibration. */
    VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake] = VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Intake];

    VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator] = RampTo(VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Elevator]/KeENC_k_AMP_ElevatorRatio,
                                                            VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator],
                                                            VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Elevator]);

    VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist] = RampTo(VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Wrist]/KeENC_k_AMP_WristRatio,
                                                         VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist],
                                                         VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Wrist]);
                                                         
    VsAmp_s_Motors.e_MotorControlType[E_Amp_Wrist] = E_MotorControlPosition;
  }
  else
  {
    /* This is the actual manipulator control */
    VeAmp_b_CriteriaMet = Update_Command_Atained_State(VeAmp_b_CriteriaMet,
                                                       LeDJ_Amp_e_SchedState);

    Update_Amp_Actuators(VeAmp_e_CmndState, VeAmp_e_AttndState);

    // UpdateAmp_hold_Actuator(VeAmp_e_CmndState, VeAmp_e_AttndState);

    if ((LeDJ_Amp_e_SchedState != VeAmp_e_CmndState) ||
        (LeDJ_Amp_e_SchedState != VeAmp_e_AttndState))
    {
      VeAmp_b_CriteriaMet = CmndStateReachedAmp(VeAmp_e_CmndState);
    }
  }

  /* Final output to the motor command that will be sent to the motor controller: */
  VsAmp_s_Motors.k_MotorCmnd[E_Amp_Elevator] = VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator];

  VsAmp_s_Motors.k_MotorCmnd[E_Amp_Wrist] = VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist];

  VsAmp_s_Motors.k_MotorCmnd[E_Amp_Intake] = VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake];
}