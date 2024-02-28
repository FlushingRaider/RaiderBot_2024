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
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"
#include "ADAS_DJ.hpp"

TeCLMR_CtrlStates VeCLMR_e_CmndState  = E_CLMR_Ctrl_Init; // What is our next/current step?
TeCLMR_CtrlStates VeCLMR_e_AttndState = E_CLMR_Ctrl_Init; // What is our desired end state?
TeCLMR_CtrlStates VeCLMR_e_SchedStatePrev = E_CLMR_Ctrl_Init; // What is our previous schedueled state?

TeCLMR_MotorControl      VsCLMR_s_Motors; // All of the motor commands for the speaker motors
TeCLMR_MotorControl      VsCLMR_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeCLMR_MotorControl      VsCLMR_s_MotorsTest; // Temporary commands for the motors, not the final output
TsCLMR_Sensor            VsCLMR_s_Sensors; // All of the sensor values for the speaker  motors

double                  VeCLMR_t_TransitionTime = 0;

double                  VeCLMR_in_DesiredHeight = 0; 

double                  VaCLMR_k_LeftPID_Gx[E_PID_SparkMaxCalSz];
double                  VaCLMR_k_RightPID_Gx[E_PID_SparkMaxCalSz];

bool                    VeCLMR_b_CriteriaMet = false;

#ifdef CLMR_Test
bool                    VeCLMR_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool                    VeCLMR_b_TestState = false;
#endif

/******************************************************************************
 * Function:     CLMR_MotorConfigsInit
 *
 * Description:  Contains the motor configurations for the climber.
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

  VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Left] = KeCLMR_ins_LiftRate;
  VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Right] = KeCLMR_ins_LiftRate;

  #ifdef CLMR_Test
  T_PID_SparkMaxCal LeCLMR_i_Index2 = E_kP;

  for (LeCLMR_i_Index2 = E_kP;
       LeCLMR_i_Index2 < E_PID_SparkMaxCalSz;
       LeCLMR_i_Index2 = T_PID_SparkMaxCal(int(LeCLMR_i_Index2) + 1))
    {
    VaCLMR_k_LeftPID_Gx[LeCLMR_i_Index2] = KaCLMR_k_LeftPID_Gx[LeCLMR_i_Index2];
    VaCLMR_k_RightPID_Gx[LeCLMR_i_Index2] = KaCLMR_k_RightPID_Gx[LeCLMR_i_Index2];
    }

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - CLMR_R", KaCLMR_k_RightPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - CLMR_R", KaCLMR_k_RightPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - CLMR_R", KaCLMR_k_RightPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - CLMR_R", KaCLMR_k_RightPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - CLMR_R", KaCLMR_k_RightPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - CLMR_R", KaCLMR_k_RightPID_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("KeCLMR_ins_LiftRateL", KeCLMR_ins_LiftRate);
  frc::SmartDashboard::PutNumber("KeCLMR_ins_LiftRateR", KeCLMR_ins_LiftRate);

  // display target positions/speeds
  frc::SmartDashboard::PutNumber("Left Climber Height",   0);
  frc::SmartDashboard::PutNumber("Right Climber Height", 0);
  #endif
  }


/******************************************************************************
 * Function:     CLMR_MotorConfigsCal
 *
 * Description:  Contains the motor configurations for the speaker motors.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void CLMR_MotorConfigsCal(rev::SparkMaxPIDController m_ClimberLeftPID,
                          rev::SparkMaxPIDController m_ClimberRightPID)
  {
  // read PID coefficients from SmartDashboard
  #ifdef CLMR_Test
  double L_p_CLMR_L   = frc::SmartDashboard::GetNumber("P Gain - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kP]);
  double L_i_CLMR_L   = frc::SmartDashboard::GetNumber("I Gain - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kI]);
  double L_d_CLMR_L   = frc::SmartDashboard::GetNumber("D Gain - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kD]);
  double L_iz_CLMR_L  = frc::SmartDashboard::GetNumber("I Zone - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kIz]);
  double L_max_CLMR_L = frc::SmartDashboard::GetNumber("Max Output - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kMaxOutput]);
  double L_min_CLMR_L = frc::SmartDashboard::GetNumber("Min Output - CLMR_L", KaCLMR_k_LeftPID_Gx[E_kMinOutput]);

  double L_p_CLMR_R   = frc::SmartDashboard::GetNumber("P Gain - CLMR_R", KaCLMR_k_RightPID_Gx[E_kP]);
  double L_i_CLMR_R   = frc::SmartDashboard::GetNumber("I Gain - CLMR_R", KaCLMR_k_RightPID_Gx[E_kI]);
  double L_d_CLMR_R   = frc::SmartDashboard::GetNumber("D Gain - CLMR_R", KaCLMR_k_RightPID_Gx[E_kD]);
  double L_iz_CLMR_R  = frc::SmartDashboard::GetNumber("I Zone - CLMR_R", KaCLMR_k_RightPID_Gx[E_kIz]);
  double L_max_CLMR_R = frc::SmartDashboard::GetNumber("Max Output - CLMR_R", KaCLMR_k_RightPID_Gx[E_kMaxOutput]);
  double L_min_CLMR_R = frc::SmartDashboard::GetNumber("Min Output - CLMR_R", KaCLMR_k_RightPID_Gx[E_kMinOutput]);

  VsCLMR_s_MotorsTest.k_MotorCmnd[E_CLMR_m_Left]   = frc::SmartDashboard::GetNumber("Left Climber Height", 0);
  VsCLMR_s_MotorsTest.k_MotorCmnd[E_CLMR_m_Right]  = frc::SmartDashboard::GetNumber("Left Climber Height", 0);
  // VsCLMR_s_MotorsTest.k_MotorCmnd[E_CLMR_m_Right]  = frc::SmartDashboard::GetNumber("Right Climber Height", 0);

  if(L_p_CLMR_L != VaCLMR_k_LeftPID_Gx[E_kP])   { m_ClimberLeftPID.SetP(L_p_CLMR_L); VaCLMR_k_LeftPID_Gx[E_kP] = L_p_CLMR_L; }
  if(L_i_CLMR_L != VaCLMR_k_LeftPID_Gx[E_kI])   { m_ClimberLeftPID.SetI(L_i_CLMR_L); VaCLMR_k_LeftPID_Gx[E_kI] = L_i_CLMR_L; }
  if(L_d_CLMR_L != VaCLMR_k_LeftPID_Gx[E_kD])   { m_ClimberLeftPID.SetD(L_d_CLMR_L); VaCLMR_k_LeftPID_Gx[E_kD] = L_d_CLMR_L; }
  if(L_iz_CLMR_L != VaCLMR_k_LeftPID_Gx[E_kIz]) { m_ClimberLeftPID.SetIZone(L_iz_CLMR_L); VaCLMR_k_LeftPID_Gx[E_kIz] = L_iz_CLMR_L; }
  if((L_max_CLMR_L != VaCLMR_k_LeftPID_Gx[E_kMaxOutput]) || (L_min_CLMR_R != VaCLMR_k_LeftPID_Gx[E_kMinOutput])) { m_ClimberLeftPID.SetOutputRange(L_min_CLMR_R, L_max_CLMR_L); VaCLMR_k_LeftPID_Gx[E_kMinOutput] = L_min_CLMR_R; VaCLMR_k_LeftPID_Gx[E_kMaxOutput] = L_max_CLMR_L; }

  if(L_p_CLMR_R != VaCLMR_k_RightPID_Gx[E_kP])   { m_ClimberRightPID.SetP(L_p_CLMR_R); VaCLMR_k_RightPID_Gx[E_kP] = L_p_CLMR_R; }
  if(L_i_CLMR_R != VaCLMR_k_RightPID_Gx[E_kI])   { m_ClimberRightPID.SetI(L_i_CLMR_R); VaCLMR_k_RightPID_Gx[E_kI] = L_i_CLMR_R; }
  if(L_d_CLMR_R != VaCLMR_k_RightPID_Gx[E_kD])   { m_ClimberRightPID.SetD(L_d_CLMR_R); VaCLMR_k_RightPID_Gx[E_kD] = L_d_CLMR_R; }
  if(L_iz_CLMR_R != VaCLMR_k_RightPID_Gx[E_kIz]) { m_ClimberRightPID.SetIZone(L_iz_CLMR_R); VaCLMR_k_RightPID_Gx[E_kIz] = L_iz_CLMR_R; }
  if((L_max_CLMR_R != VaCLMR_k_RightPID_Gx[E_kMaxOutput]) || (L_min_CLMR_R != VaCLMR_k_RightPID_Gx[E_kMinOutput])) { m_ClimberRightPID.SetOutputRange(L_min_CLMR_R, L_max_CLMR_R); VaCLMR_k_RightPID_Gx[E_kMinOutput] = L_min_CLMR_R; VaCLMR_k_RightPID_Gx[E_kMaxOutput] = L_max_CLMR_R; }

  VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Left]  = frc::SmartDashboard::GetNumber("KeCLMR_ins_LiftRateL", VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Left]);
  VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Right] = frc::SmartDashboard::GetNumber("KeCLMR_ins_LiftRateR", VsCLMR_s_MotorsTest.k_MotorRampRate[E_CLMR_m_Right]);
  #endif
  }


/******************************************************************************
 * Function:     CLMR_ControlInit
 *
 * Description:  Initialization function for the climber controls.
 ******************************************************************************/
void CLMR_ControlInit()
  {
  VeCLMR_e_CmndState  = E_CLMR_Ctrl_Init;
  VeCLMR_e_AttndState = E_CLMR_Ctrl_Init;
  VeCLMR_e_SchedStatePrev = E_CLMR_Ctrl_Init;

  VeCLMR_b_CriteriaMet = false;
  VeCLMR_t_TransitionTime = 0.0;
  }


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


/******************************************************************************
 * Function:     UpdateClimberCommandAttainedState
 *
 * Description:  Updates the commanded and attained states for the climber manipulator
 ******************************************************************************/
bool UpdateClimberCommandAttainedState(bool              LeCLMR_b_CriteriaMet,
                                       TeCLMR_CtrlStates LeCLMR_e_SchedState)
  {
  TeCLMR_CtrlStates LeCLMR_e_CmndState = VeCLMR_e_CmndState;

  if(LeCLMR_b_CriteriaMet == true)
    {
    VeCLMR_e_AttndState = LeCLMR_e_CmndState;
    LeCLMR_b_CriteriaMet = false;
    }

  if(((LeCLMR_e_SchedState != VeCLMR_e_AttndState) &&
      (VeCLMR_e_CmndState  == VeCLMR_e_AttndState)) ||
      (LeCLMR_e_SchedState != VeCLMR_e_SchedStatePrev)) // Allow for schedueled aborts
    {
    LeCLMR_e_CmndState = KaCLMR_e_ControllingTable[LeCLMR_e_SchedState][VeCLMR_e_AttndState];
    }

  VeCLMR_e_CmndState = LeCLMR_e_CmndState;

  VeCLMR_e_SchedStatePrev = LeCLMR_e_SchedState;

  return(LeCLMR_b_CriteriaMet);
  }


/******************************************************************************
 * Function:     CmndStateReachedClimber
 *
 * Description:  Checks to see if we have reached the desired commanded state
 ******************************************************************************/
bool CmndStateReachedClimber(TeCLMR_CtrlStates LeCLMR_e_CmndState)
  {
  bool LeCLMR_b_CriteriaMet = false;

  VeCLMR_t_TransitionTime += C_ExeTime;

  if((VeCLMR_t_TransitionTime >= KeCLMR_t_StateTimeOut) ||

      (LeCLMR_e_CmndState == E_CLMR_Ctrl_MidClimb) ||

     ((VsCLMR_s_Sensors.in_Left <= (KaCLMR_in_Position[LeCLMR_e_CmndState] + KaCLMR_in_LeftDb[LeCLMR_e_CmndState])) &&
      (VsCLMR_s_Sensors.in_Left >= (KaCLMR_in_Position[LeCLMR_e_CmndState] - KaCLMR_in_LeftDb[LeCLMR_e_CmndState])) &&

      (VsCLMR_s_Sensors.in_Right <= (KaCLMR_in_Position[LeCLMR_e_CmndState] + KaCLMR_in_RightDb[LeCLMR_e_CmndState])) &&
      (VsCLMR_s_Sensors.in_Right >= (KaCLMR_in_Position[LeCLMR_e_CmndState] - KaCLMR_in_RightDb[LeCLMR_e_CmndState]))))
      {
      LeCLMR_b_CriteriaMet = true;
      VeCLMR_t_TransitionTime = 0.0;
      }

  return(LeCLMR_b_CriteriaMet);
  }


/******************************************************************************
 * Function:     UpdateCLMR_Actuators
 *
 * Description:  Updates the intermediate state of the actuartors for the 
 *               climber
 ******************************************************************************/
void UpdateCLMR_Actuators(TeCLMR_CtrlStates LeCLMR_e_CmndState,
                          TeCLMR_CtrlStates LeCLMR_e_AttndState)
  {
  double LeCLMR_ins_Rate = 0;

  /* The intent is to have a slower rate when climbing due to the expected load.*/
  if (LeCLMR_e_CmndState == E_CLMR_Ctrl_MidClimb || LeCLMR_e_CmndState == E_CLMR_Ctrl_Init)
    {
    LeCLMR_ins_Rate = KeCLMR_ins_LiftRate;
    }
  else
    {
    LeCLMR_ins_Rate = KeCLMR_ins_ExtendRate;
    }

  if (LeCLMR_e_CmndState == E_CLMR_Ctrl_MidClimb)
    {
      VeCLMR_in_DesiredHeight += VsCONT_s_DriverInput.Pct_Manual_CLMR * KeCLMR_k_CntrlGx;
      if (VeCLMR_in_DesiredHeight <= KaCLMR_in_Position[E_CLMR_Ctrl_Init])
      {
        VeCLMR_in_DesiredHeight = KaCLMR_in_Position[E_CLMR_Ctrl_Init];
      }
      else if (VeCLMR_in_DesiredHeight >= KaCLMR_in_Position[E_CLMR_Ctrl_FullExtend])
      {
        VeCLMR_in_DesiredHeight = KaCLMR_in_Position[E_CLMR_Ctrl_FullExtend];
      }
    }
  else
    {
      VeCLMR_in_DesiredHeight = KaCLMR_in_Position[LeCLMR_e_CmndState];
    }

  VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Left] = RampTo(VeCLMR_in_DesiredHeight / KeENC_k_CLMR_LeftRatio, 
                                                            VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Left],
                                                            LeCLMR_ins_Rate);

  VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Right] = RampTo(VeCLMR_in_DesiredHeight / KeENC_k_CLMR_RightRatio, 
                                                            VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Right],
                                                            LeCLMR_ins_Rate);
  }


/******************************************************************************
 * Function:     CLMR_SpeakerControlMain
 *
 * Description:  Main calling function for climber control.
 ******************************************************************************/
void CLMR_SpeakerControlMain(TeCLMR_CtrlStates LeCLMR_e_SchedState,
                            bool               LeCLMR_b_TestPowerOverride)
  {

  if (LeCLMR_b_TestPowerOverride == true)
    {
    // Do nothing.  Robot is in test state using power commands for all the acutators
    }
  else if (VeCLMR_b_TestState == true)
    {
    /* Only used for testing/calibration. */
    VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Left] = RampTo(VsCLMR_s_MotorsTest.k_MotorCmnd[E_CLMR_m_Left] / KeENC_k_CLMR_LeftRatio, 
                                                            VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Left],
                                                            KeCLMR_ins_LiftRate);

    VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Right] = RampTo(VsCLMR_s_MotorsTest.k_MotorCmnd[E_CLMR_m_Right] / KeENC_k_CLMR_RightRatio, 
                                                             VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Right],
                                                             KeCLMR_ins_LiftRate);
    }
  else
    {
    /* This is the actual manipulator control */
    VeCLMR_b_CriteriaMet = UpdateClimberCommandAttainedState(VeCLMR_b_CriteriaMet,
                                                             LeCLMR_e_SchedState);

    UpdateCLMR_Actuators(VeCLMR_e_CmndState, VeCLMR_e_AttndState);

    if ((LeCLMR_e_SchedState != VeCLMR_e_CmndState) ||
        (LeCLMR_e_SchedState != VeCLMR_e_AttndState))
      {
        VeCLMR_b_CriteriaMet = CmndStateReachedClimber(VeCLMR_e_CmndState);
      }
    }

    /* Final output to the motor command that will be sent to the motor controller: */
    VsCLMR_s_Motors.k_MotorCmnd[E_CLMR_m_Left] = VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Left];

    VsCLMR_s_Motors.k_MotorCmnd[E_CLMR_m_Right] = VsCLMR_s_MotorsTemp.k_MotorCmnd[E_CLMR_m_Right];
  }
