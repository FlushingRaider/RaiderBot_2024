/*
  SpeakerCntrl.cpp

   Created on: Feb 17, 2024
   Author: Biggs

   The climber controls / state machine.
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

TeCLMR_CtrlStates VeCLMR_e_CmndState  = E_CLMR_Ctrl_Init; // What is our next/current step?
TeCLMR_CtrlStates VeCLMR_e_AttndState = E_CLMR_Ctrl_Init; // What is our desired end state?

TeCLMR_MotorControl      VsCLMR_s_Motors; // All of the motor commands for the speaker motors
TeCLMR_MotorControl      VsCLMR_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeCLMR_MotorControl      VsCLMR_s_MotorsTest; // Temporary commands for the motors, not the final output
TsCLMR_Sensor            VsCLMR_s_Sensors; // All of the sensor values for the speaker  motors

double                  VeCLMR_t_TransitionTime = 0;

double                  VaCLMR_k_LeftPID_Gx[E_PID_SparkMaxCalSz];
double                  VaCLMR_k_RightPID_Gx[E_PID_SparkMaxCalSz];

bool                    VeCLMR_b_CriteriaMet = false;

#ifdef CLMR_Test
bool                    VeCLMR_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool                    VeCLMR_b_TestState = false;
#endif

#ifdef DOESNOTWORKYET
/******************************************************************************
 * Function:     SPK_MotorConfigsInit
 *
 * Description:  Contains the motor configurations for the intake and shooter.
 ******************************************************************************/
void CLMR_MotorConfigsInit(rev::SparkMaxPIDController m_ClimberLeftPID,
                           rev::SparkMaxPIDController m_ClimberRightPID)
  {
  TeCLMR_Actuators LeCLMR_i_Index  = E_CLMR_m_Left;
  T_PID_Cal        LeCLMR_i_Index3 = E_P_Gx;

  // set PID coefficients
  m_ClimberLeftPID.SetP(KaCLMR_k_LeftPID_Gx[E_kP]);
  m_ClimberLeftPID.SetI(KaCLMR_k_LeftPID_Gx[E_kI]);
  m_ClimberLeftPID.SetD(KaCLMR_k_LeftPID_Gx[E_kD]);
  m_ClimberLeftPID.SetIZone(KaCLMR_k_LeftPID_Gx[E_kIz]);
  m_ClimberLeftPID.SetFF(KaCLMR_k_LeftPID_Gx[E_kFF]);
  m_ClimberLeftPID.SetOutputRange(KaCLMR_k_LeftPID_Gx[E_kMinOutput], KaCLMR_k_LeftPID_Gx[E_kMaxOutput]);

  m_ClimberRightPID.SetP(KaCLMR_k_RightPID_Gx[E_kP]);
  m_ClimberRightPID.SetI(KaCLMR_k_RightPID_Gx[E_kI]);
  m_ClimberRightPID.SetD(KaCLMR_k_RightPID_Gx[E_kD]);
  m_ClimberRightPID.SetIZone(KaCLMR_k_RightPID_Gx[E_kIz]);
  m_ClimberRightPID.SetFF(KaCLMR_k_RightPID_Gx[E_kFF]);
  m_ClimberRightPID.SetOutputRange(KaCLMR_k_RightPID_Gx[E_kMinOutput], KaCLMR_k_RightPID_Gx[E_kMaxOutput]);

  for (LeCLMR_i_Index = E_CLMR_m_Left;
       LeCLMR_i_Index < E_CLMR_m_Sz;
       LeCLMR_i_Index = TeCLMR_Actuators(int(LeCLMR_i_Index) + 1))
    {
      VsCLMR_s_Motors.k_MotorCmnd[LeCLMR_i_Index] = 0.0;
      VsCLMR_s_MotorsTemp.k_MotorCmnd[LeCLMR_i_Index] = 0.0;
      VsCLMR_s_MotorsTest.k_MotorCmnd[LeCLMR_i_Index] = 0.0;
    }

 /* We don't have a ramp rate for the intake motor as we are running as power based only... */
  VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Left] = KeCLMR_ins_LiftRate;
  VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Right] = KeSPK_RPMs_Shooter2Rate;

  #ifdef SPK_Test
  T_PID_SparkMaxCal LeMAN_i_Index2 = E_kP;

  for (LeMAN_i_Index2 = E_kP;
       LeMAN_i_Index2 < E_PID_SparkMaxCalSz;
       LeMAN_i_Index2 = T_PID_SparkMaxCal(int(LeMAN_i_Index2) + 1))
    {
    VaSPK_k_IntakePID_Gx[LeMAN_i_Index2] = KaSPK_k_IntakePID_Gx[LeMAN_i_Index2];
    VaSPK_k_Shooter1PID_Gx[LeMAN_i_Index2] = KaSPK_k_Shooter1PID_Gx[LeMAN_i_Index2];
    VaSPK_k_Shooter2PID_Gx[LeMAN_i_Index2] = KaSPK_k_Shooter2PID_Gx[LeMAN_i_Index2];
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
                         rev::SparkMaxPIDController m_Shooter2PID)
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
#endif

/******************************************************************************
 * Function:     CLMR_ControlManualOverride
 *
 * Description:  Manual override control used during the FRC test section. Use incase of Y2K -J 
 ******************************************************************************/
void CLMR_ControlManualOverride(RobotUserInput *LsCONT_s_DriverInput)
  {
  TeCLMR_Actuators LeCLMR_i_Index;

  for (LeCLMR_i_Index = E_CLMR_m_Left;
       LeCLMR_i_Index < E_CLMR_m_Sz;
       LeCLMR_i_Index = TeCLMR_Actuators(int(LeCLMR_i_Index) + 1))
    {
      VsCLMR_s_Motors.k_MotorTestPower[LeCLMR_i_Index] = 0.0;
    }

  VsCLMR_s_Motors.k_MotorTestPower[E_CLMR_m_Left] = LsCONT_s_DriverInput->pct_LeftHook_Test * KaCLMR_k_TestPower[E_CLMR_m_Left];

  VsCLMR_s_Motors.k_MotorTestPower[E_CLMR_m_Right] = LsCONT_s_DriverInput->pct_RightHook_Test * KaCLMR_k_TestPower[E_CLMR_m_Right];
  }
#ifdef DOESNOTWORKYET
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

  VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1] = RampTo(KaSPK_RPM_Shooter1[LeSPK_e_CmndState], 
                                                            VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1],
                                                            KeSPK_RPMs_Shooter1Rate);

  VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2] = RampTo(KaSPK_RPM_Shooter2[LeSPK_e_CmndState], 
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
    VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Intake] = VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Intake];

    VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1] = RampTo(VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter1], 
                                                              VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1],
                                                              KeSPK_RPMs_Shooter1Rate);

    VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2] = RampTo(VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter2], 
                                                              VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2],
                                                              KeSPK_RPMs_Shooter2Rate);
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

  #endif