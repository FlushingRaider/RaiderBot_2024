/*
  SpeakerCntrl.cpp

   Created on: Feb 17, 2024
   Author: Biggs

   The underbelly intake and launch rollers control / state machine.
 */

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"
#include "ADAS_DJ.hpp"

TeSPK_CtrlStates VeSPK_e_CmndState  = E_SPK_Ctrl_Init; // What is our next/current step?
TeSPK_CtrlStates VeSPK_e_AttndState = E_SPK_Ctrl_Init; // What is our desired end state?

TeSKP_MotorControl      VsSPK_s_Motors; // All of the motor commands for the speaker motors
TeSKP_MotorControl      VsSPK_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeSKP_MotorControl      VsSPK_s_MotorsTest; // Temporary commands for the motors, not the final output
TsSPK_Sensor            VsSPK_s_Sensors; // All of the sensor values for the speaker  motors

double                  VeSPK_t_TransitionTime = 0;

double                  VaSPK_k_IntakePID_Gx[E_PID_SparkMaxCalSz];
double                  VaSPK_k_Shooter1PID_Gx[E_PID_SparkMaxCalSz];
double                  VaSPK_k_Shooter2PID_Gx[E_PID_SparkMaxCalSz];

bool                    VeSPK_b_CriteriaMet = false;

#ifdef SPK_Test
bool                    VeSPK_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool                    VeSPK_b_TestState = false;
#endif

/******************************************************************************
 * Function:     SPK_MotorConfigsInit
 *
 * Description:  Contains the motor configurations for the intake and shooter.
 ******************************************************************************/
void SPK_MotorConfigsInit(rev::SparkMaxPIDController m_UnderbellyPID,
                          rev::SparkMaxPIDController m_IAssistPID,
                          rev::SparkMaxPIDController m_Shooter1PID,
                          rev::SparkMaxPIDController m_Shooter2PID)
  {
  TeSPK_Actuators LeSPK_i_Index  = E_SPK_m_Intake;
  T_PID_Cal       LeSPK_i_Index3 = E_P_Gx;

  // set PID coefficients
  m_UnderbellyPID.SetP(KaSPK_k_IntakePID_Gx[E_kP]);
  m_UnderbellyPID.SetI(KaSPK_k_IntakePID_Gx[E_kI]);
  m_UnderbellyPID.SetD(KaSPK_k_IntakePID_Gx[E_kD]);
  m_UnderbellyPID.SetIZone(KaSPK_k_IntakePID_Gx[E_kIz]);
  m_UnderbellyPID.SetFF(KaSPK_k_IntakePID_Gx[E_kFF]);
  m_UnderbellyPID.SetOutputRange(KaSPK_k_IntakePID_Gx[E_kMinOutput], KaSPK_k_IntakePID_Gx[E_kMaxOutput]);

  m_IAssistPID.SetP(KaSPK_k_IAssistPID_Gx[E_kP]);
  m_IAssistPID.SetI(KaSPK_k_IAssistPID_Gx[E_kI]);
  m_IAssistPID.SetD(KaSPK_k_IAssistPID_Gx[E_kD]);
  m_IAssistPID.SetIZone(KaSPK_k_IAssistPID_Gx[E_kIz]);
  m_IAssistPID.SetFF(KaSPK_k_IAssistPID_Gx[E_kFF]);
  m_IAssistPID.SetOutputRange(KaSPK_k_IAssistPID_Gx[E_kMinOutput], KaSPK_k_IAssistPID_Gx[E_kMaxOutput]);

  m_Shooter1PID.SetP(KaSPK_k_Shooter1PID_Gx[E_kP]);
  m_Shooter1PID.SetI(KaSPK_k_Shooter1PID_Gx[E_kI]);
  m_Shooter1PID.SetD(KaSPK_k_Shooter1PID_Gx[E_kD]);
  m_Shooter1PID.SetIZone(KaSPK_k_Shooter1PID_Gx[E_kIz]);
  m_Shooter1PID.SetFF(KaSPK_k_Shooter1PID_Gx[E_kFF]);
  m_Shooter1PID.SetOutputRange(KaSPK_k_Shooter1PID_Gx[E_kMinOutput], KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);

  m_Shooter2PID.SetP(KaSPK_k_Shooter2PID_Gx[E_kP]);
  m_Shooter2PID.SetI(KaSPK_k_Shooter2PID_Gx[E_kI]);
  m_Shooter2PID.SetD(KaSPK_k_Shooter2PID_Gx[E_kD]);
  m_Shooter2PID.SetIZone(KaSPK_k_Shooter2PID_Gx[E_kIz]);
  m_Shooter2PID.SetFF(KaSPK_k_Shooter2PID_Gx[E_kFF]);
  m_Shooter2PID.SetOutputRange(KaSPK_k_Shooter2PID_Gx[E_kMinOutput], KaSPK_k_Shooter2PID_Gx[E_kMaxOutput]);

  for (LeSPK_i_Index = E_SPK_m_Intake;
       LeSPK_i_Index < E_SPK_m_Sz;
       LeSPK_i_Index = TeSPK_Actuators(int(LeSPK_i_Index) + 1))
    {
      VsSPK_s_Motors.k_MotorCmnd[LeSPK_i_Index] = 0.0;
      VsSPK_s_MotorsTemp.k_MotorCmnd[LeSPK_i_Index] = 0.0;
      VsSPK_s_MotorsTest.k_MotorCmnd[LeSPK_i_Index] = 0.0;
    }

 /* We don't have a ramp rate for the intake motor as we are running as power based only... */
  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter1] = KeSPK_RPMs_Shooter1Rate;
  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter2] = KeSPK_RPMs_Shooter2Rate;

  #ifdef SPK_Test
  T_PID_SparkMaxCal LeSPK_i_Index2 = E_kP;

  for (LeSPK_i_Index2 = E_kP;
       LeSPK_i_Index2 < E_PID_SparkMaxCalSz;
       LeSPK_i_Index2 = T_PID_SparkMaxCal(int(LeSPK_i_Index2) + 1))
    {
    VaSPK_k_IntakePID_Gx[LeSPK_i_Index2] = KaSPK_k_IntakePID_Gx[LeSPK_i_Index2];
    VaSPK_k_Shooter1PID_Gx[LeSPK_i_Index2] = KaSPK_k_Shooter1PID_Gx[LeSPK_i_Index2];
    VaSPK_k_Shooter2PID_Gx[LeSPK_i_Index2] = KaSPK_k_Shooter2PID_Gx[LeSPK_i_Index2];
    }

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain - Intake", KaSPK_k_IntakePID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Intake", KaSPK_k_IntakePID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Intake", KaSPK_k_IntakePID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Intake", KaSPK_k_IntakePID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Intake", KaSPK_k_IntakePID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Intake", KaSPK_k_IntakePID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("KeSPK_RPMs_Shooter1Rate", KeSPK_RPMs_Shooter1Rate);
  frc::SmartDashboard::PutNumber("KeSPK_RPMs_Shooter2Rate", KeSPK_RPMs_Shooter2Rate);

  // display target positions/speeds
  frc::SmartDashboard::PutNumber("Set Intake Power",   0);
  frc::SmartDashboard::PutNumber("Set Shooter1 Speed", 0);
  frc::SmartDashboard::PutNumber("Set Shooter2 Speed", 0);
  #endif
  }


/******************************************************************************
 * Function:     SPK_MotorConfigsCal
 *
 * Description:  Contains the motor configurations for the speaker motors.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void SPK_MotorConfigsCal(rev::SparkMaxPIDController m_UnderbellyPID,
                         rev::SparkMaxPIDController m_Shooter1PID,
                         rev::SparkMaxPIDController m_Shooter2PID,
                         rev::SparkMaxPIDController m_IAssistPID)
  {
  // read PID coefficients from SmartDashboard
  #ifdef SPK_Test
  double L_p_Intake   = frc::SmartDashboard::GetNumber("P Gain - Intake", KaSPK_k_IntakePID_Gx[E_kP]);
  double L_i_Intake   = frc::SmartDashboard::GetNumber("I Gain - Intake", KaSPK_k_IntakePID_Gx[E_kI]);
  double L_d_Intake   = frc::SmartDashboard::GetNumber("D Gain - Intake", KaSPK_k_IntakePID_Gx[E_kD]);
  double L_iz_Intake  = frc::SmartDashboard::GetNumber("I Zone - Intake", KaSPK_k_IntakePID_Gx[E_kIz]);
  double L_max_Intake = frc::SmartDashboard::GetNumber("Max Output - Intake", KaSPK_k_IntakePID_Gx[E_kMaxOutput]);
  double L_min_Intake = frc::SmartDashboard::GetNumber("Min Output - Intake", KaSPK_k_IntakePID_Gx[E_kMinOutput]);

  double L_p_Shooter1   = frc::SmartDashboard::GetNumber("P Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kP]);
  double L_i_Shooter1   = frc::SmartDashboard::GetNumber("I Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kI]);
  double L_d_Shooter1   = frc::SmartDashboard::GetNumber("D Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kD]);
  double L_iz_Shooter1  = frc::SmartDashboard::GetNumber("I Zone - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kIz]);
  double L_max_Shooter1 = frc::SmartDashboard::GetNumber("Max Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);
  double L_min_Shooter1 = frc::SmartDashboard::GetNumber("Min Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMinOutput]);

  double L_p_Shooter2   = frc::SmartDashboard::GetNumber("P Gain - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kP]);
  double L_i_Shooter2   = frc::SmartDashboard::GetNumber("I Gain - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kI]);
  double L_d_Shooter2   = frc::SmartDashboard::GetNumber("D Gain - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kD]);
  double L_iz_Shooter2  = frc::SmartDashboard::GetNumber("I Zone - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kIz]);
  double L_max_Shooter2 = frc::SmartDashboard::GetNumber("Max Output - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kMaxOutput]);
  double L_min_Shooter2 = frc::SmartDashboard::GetNumber("Min Output - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kMinOutput]);

  VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Intake]   = frc::SmartDashboard::GetNumber("Set Intake Power", 0);
  VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter1] = frc::SmartDashboard::GetNumber("Set Shooter1 Speed", 0);
  VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter2] = frc::SmartDashboard::GetNumber("Set Shooter2 Speed", 0);

  if(L_p_Intake != VaSPK_k_IntakePID_Gx[E_kP])   { m_UnderbellyPID.SetP(L_p_Intake); VaSPK_k_IntakePID_Gx[E_kP] = L_p_Intake; }
  if(L_i_Intake != VaSPK_k_IntakePID_Gx[E_kI])   { m_UnderbellyPID.SetI(L_i_Intake); VaSPK_k_IntakePID_Gx[E_kI] = L_i_Intake; }
  if(L_d_Intake != VaSPK_k_IntakePID_Gx[E_kD])   { m_UnderbellyPID.SetD(L_d_Intake); VaSPK_k_IntakePID_Gx[E_kD] = L_d_Intake; }
  if(L_iz_Intake != VaSPK_k_IntakePID_Gx[E_kIz]) { m_UnderbellyPID.SetIZone(L_iz_Intake); VaSPK_k_IntakePID_Gx[E_kIz] = L_iz_Intake; }
  if((L_max_Intake != VaSPK_k_IntakePID_Gx[E_kMaxOutput]) || (L_min_Intake != VaSPK_k_IntakePID_Gx[E_kMinOutput])) { m_UnderbellyPID.SetOutputRange(L_min_Intake, L_max_Intake); VaSPK_k_IntakePID_Gx[E_kMinOutput] = L_min_Intake; VaSPK_k_IntakePID_Gx[E_kMaxOutput] = L_max_Intake; }

  if(L_p_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kP])   { m_Shooter1PID.SetP(L_p_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kP] = L_p_Shooter1; }
  if(L_i_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kI])   { m_Shooter1PID.SetI(L_i_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kI] = L_i_Shooter1; }
  if(L_d_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kD])   { m_Shooter1PID.SetD(L_d_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kD] = L_d_Shooter1; }
  if(L_iz_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kIz]) { m_Shooter1PID.SetIZone(L_iz_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kIz] = L_iz_Shooter1; }
  if((L_max_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kMaxOutput]) || (L_min_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kMinOutput])) { m_Shooter1PID.SetOutputRange(L_min_Shooter1, L_max_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kMinOutput] = L_min_Shooter1; VaSPK_k_Shooter1PID_Gx[E_kMaxOutput] = L_max_Shooter1; }

  if(L_p_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kP])   { m_Shooter2PID.SetP(L_p_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kP] = L_p_Shooter2; }
  if(L_i_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kI])   { m_Shooter2PID.SetI(L_i_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kI] = L_i_Shooter2; }
  if(L_d_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kD])   { m_Shooter2PID.SetD(L_d_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kD] = L_d_Shooter2; }
  if(L_iz_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kIz]) { m_Shooter2PID.SetIZone(L_iz_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kIz] = L_iz_Shooter2; }
  if((L_max_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kMaxOutput]) || (L_min_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kMinOutput])) { m_Shooter2PID.SetOutputRange(L_min_Shooter2, L_max_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kMinOutput] = L_min_Shooter2; VaSPK_k_Shooter2PID_Gx[E_kMaxOutput] = L_max_Shooter2; }

  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter1] = frc::SmartDashboard::GetNumber("KeSPK_RPMs_Shooter1Rate", VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter1]);
  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter2] = frc::SmartDashboard::GetNumber("KeSPK_RPMs_Shooter2Rate", VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter2]);
   #endif
  }


/******************************************************************************
 * Function:     SPK_ControlInit
 *
 * Description:  Initialization function for the speaker controls.
 ******************************************************************************/
void SPK_ControlInit()
  {
  VeSPK_e_CmndState  = E_SPK_Ctrl_Init;
  VeSPK_e_AttndState = E_SPK_Ctrl_Init;

  VeSPK_b_CriteriaMet = false;
  VeSPK_t_TransitionTime = 0.0;
  }


/******************************************************************************
 * Function:     SPK_ControlManualOverride
 *
 * Description:  Manual override control used during the FRC test section. Use incase of Y2K -J 
 ******************************************************************************/
void SPK_ControlManualOverride(RobotUserInput *LsCONT_s_DriverInput)
  {
  TeSPK_Actuators LeSPK_i_Index;

  for (LeSPK_i_Index = E_SPK_m_Intake;
       LeSPK_i_Index < E_SPK_m_Sz;
       LeSPK_i_Index = TeSPK_Actuators(int(LeSPK_i_Index) + 1))
    {
      VsSPK_s_Motors.k_MotorTestPower[LeSPK_i_Index] = 0.0;
    }

  if (LsCONT_s_DriverInput->b_Spk_IntakeForward_Test == true)
    {
    VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Intake] = KaSPK_k_TestPower[E_SPK_m_Intake];
    }
  else if (LsCONT_s_DriverInput->b_Spk_IntakeBackward_Test == true)
    {
    VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Intake] = -KaSPK_k_TestPower[E_SPK_m_Intake];
    }

  VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Shooter1] = LsCONT_s_DriverInput->Pct_Shooter1_Test * KaSPK_k_TestPower[E_SPK_m_Shooter1];

  VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Shooter2] = LsCONT_s_DriverInput->Pct_Shooter2_Test * KaSPK_k_TestPower[E_SPK_m_Shooter2];
  }

/******************************************************************************
 * Function:     UpdateSpeakerCommandAttainedState
 *
 * Description:  Updates the commanded and attained states for the speaker manipulator
 ******************************************************************************/
bool UpdateSpeakerCommandAttainedState(bool             LeSPK_b_CriteriaMet,
                                       TeSPK_CtrlStates LeSPK_e_SchedState)
  {
  TeSPK_CtrlStates LeSPK_e_CmndState = VeSPK_e_CmndState;

  if(LeSPK_b_CriteriaMet == true)
    {
    VeSPK_e_AttndState = LeSPK_e_CmndState;
    LeSPK_b_CriteriaMet = false;
    }

  if((LeSPK_e_SchedState != VeSPK_e_AttndState) &&
     (VeSPK_e_CmndState  == VeSPK_e_AttndState))
    {
    LeSPK_e_CmndState = KaSPK_e_ControllingTable[LeSPK_e_SchedState][VeSPK_e_AttndState];
    }

  VeSPK_e_CmndState = LeSPK_e_CmndState;

  return(LeSPK_b_CriteriaMet);
  }


/******************************************************************************
 * Function:     CmndStateReachedSpeaker
 *
 * Description:  Checks to see if we have reached the desired commanded state
 ******************************************************************************/
bool CmndStateReachedSpeaker(TeSPK_CtrlStates LeSPK_e_CmndState)
  {
  bool LeSPK_b_CriteriaMet = false;

  VeSPK_t_TransitionTime += C_ExeTime;

  if((VeSPK_t_TransitionTime >= KeSPK_t_StateTimeOut) ||

     ((VsSPK_s_Sensors.RPM_Shooter1 <= (KaSPK_RPM_Shooter1[LeSPK_e_CmndState] + KaSPK_RPM_Shooter1Db[LeSPK_e_CmndState])) &&
      (VsSPK_s_Sensors.RPM_Shooter1 >= (KaSPK_RPM_Shooter1[LeSPK_e_CmndState] - KaSPK_RPM_Shooter1Db[LeSPK_e_CmndState])) &&

      (VsSPK_s_Sensors.RPM_Shooter2 <= (KaSPK_RPM_Shooter2[LeSPK_e_CmndState] + KaSPK_RPM_Shooter2Db[LeSPK_e_CmndState])) &&
      (VsSPK_s_Sensors.RPM_Shooter2 >= (KaSPK_RPM_Shooter2[LeSPK_e_CmndState] - KaSPK_RPM_Shooter2Db[LeSPK_e_CmndState]))))
      {
      LeSPK_b_CriteriaMet = true;
      VeSPK_t_TransitionTime = 0.0;
      }

  return(LeSPK_b_CriteriaMet);
  }


/******************************************************************************
 * Function:     UpdateSPK_Actuators
 *
 * Description:  Updates the intermediate state of the actuartors for the 
 *               manipulator
 ******************************************************************************/
void UpdateSPK_Actuators(TeSPK_CtrlStates LeSPK_e_CmndState,
                         TeSPK_CtrlStates LeSPK_e_AttndState)
  {
  VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Intake] = KaSPK_k_Intake[LeSPK_e_CmndState];

  VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1] = RampTo(KaSPK_RPM_Shooter1[LeSPK_e_CmndState] / KeENC_k_SPK_Shooter1Ratio, 
                                                            VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1],
                                                            KeSPK_RPMs_Shooter1Rate);

  VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2] = RampTo(KaSPK_RPM_Shooter2[LeSPK_e_CmndState] / KeENC_k_SPK_Shooter2Ratio, 
                                                            VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2],
                                                            KeSPK_RPMs_Shooter2Rate);
  }


/******************************************************************************
 * Function:     SPK_SpeakerControlMain
 *
 * Description:  Main calling function for speaker control.
 ******************************************************************************/
void SPK_SpeakerControlMain(TeSPK_CtrlStates LeSPK_e_SchedState,
                            bool             LeSPK_b_TestPowerOverride)
  {
  double LeMAN_Deg_Error = 0.0;
  double LeMAN_k_P_Gain = 0.0;

  if (LeSPK_b_TestPowerOverride == true)
    {
    // Do nothing.  Robot is in test state using power commands for all the acutators
    }
  else if (VeSPK_b_TestState == true)
    {
    /* Only used for testing/calibration. */
    double LeSPK_k_Power = VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Intake];
    if (VsSPK_s_Sensors.b_NoteDetected == true)
    {
      LeSPK_k_Power = 0;
    }
    
    VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Intake] = LeSPK_k_Power;

    VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1] = RampTo(VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter1] / KeENC_k_SPK_Shooter1Ratio, 
                                                              VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1],
                                                              VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter1]);

    VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2] = RampTo(VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter2] / KeENC_k_SPK_Shooter2Ratio, 
                                                              VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2],
                                                              VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter2]);
    }
  else
    {
    /* This is the actual manipulator control */
    VeSPK_b_CriteriaMet = UpdateSpeakerCommandAttainedState(VeSPK_b_CriteriaMet,
                                                            LeSPK_e_SchedState);

    UpdateSPK_Actuators(VeSPK_e_CmndState, VeSPK_e_AttndState);

    if ((LeSPK_e_SchedState != VeSPK_e_CmndState) ||
        (LeSPK_e_SchedState != VeSPK_e_AttndState))
      {
        VeSPK_b_CriteriaMet = CmndStateReachedSpeaker(VeSPK_e_CmndState);
      }
    }

    /* Final output to the motor command that will be sent to the motor controller: */
    VsSPK_s_Motors.k_MotorCmnd[E_SPK_m_Intake] = VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Intake];

    VsSPK_s_Motors.k_MotorCmnd[E_SPK_m_Shooter1] = VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1];

    VsSPK_s_Motors.k_MotorCmnd[E_SPK_m_Shooter2] = VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2];
  }