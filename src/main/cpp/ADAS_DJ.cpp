/*
  ADAS_DJ.cpp

  Created on: Feb 17, 2024
  Author: Lauren

  ADAS (Advanced Driver-Assistance Systems) DJ
  Contains the logic and code used for the Amp and Speaker mech. control:
    - Schedules states in a non-linear state machine
 */

// #include <frc/smartdashboard/SmartDashboard.h>
// #include <frc/DriverStation.h>
#include "Const.hpp"
#include "Driver_inputs.hpp"
#include "Amp.hpp"
#include "SpeakerCntrl.hpp"
#include "Climber.hpp"

T_DJ_Amp_States      VeADAS_e_Amp_SchedState = E_DJ_Amp_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
TeSPK_CtrlStates     VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
TeCLMR_CtrlStates    VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
T_ADAS_ActiveFeature VeADAS_e_AutonFeatureDJ_Prev = E_ADAS_Disabled;  // Previous state of auton feature.  Needed so we can detect transitions in states
double               VeADAS_t_SPK_ReleaseTm = 0.0; // Timer that will keep rollers on for a specific amount of time
double               VeADAS_t_AMP_ReleaseTm = 0.0; // Timer that will keep rollers on for a specific amount of time

/******************************************************************************
 * Function:     ADAS_DJ_Reset
 *
 * Description:  Reset all applicable MN variables.
 ******************************************************************************/
void ADAS_DJ_Reset(void)
{
  VeADAS_e_Amp_SchedState = E_DJ_Amp_Init;
  VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Init;
  VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_Init;
  VeADAS_e_AutonFeatureDJ_Prev = E_ADAS_Disabled;
  VeADAS_t_SPK_ReleaseTm = 0.0;
  VeADAS_t_AMP_ReleaseTm = 0.0;
}


/******************************************************************************
 * Function:    ScheduelerAMP
 * Made By:     Lauren
 * Description: Determines scheduled state of the DJ mechanisms, speaker and amp
 ******************************************************************************/
bool ScheduelerAMP(T_RobotState    LeADAS_e_RobotState,
                   T_DJ_Amp_States LeADAS_e_AutonRequestStateAMP,
                   bool             LeADAS_b_AutonStateTransition)
{
  bool LeADAS_b_AMP_StateComplete = false;

  if (LeADAS_e_RobotState == E_Auton)
  {
    /* Only allow auton to update the sched state once per transition.
       This is to allow the overrides below to work properly. */
    if (LeADAS_b_AutonStateTransition == true)
    {
      VeADAS_e_Amp_SchedState = LeADAS_e_AutonRequestStateAMP;
    }
  }
  else if (LeADAS_e_RobotState == E_Teleop)
  {
    // if (VsCONT_s_DriverInput.b_SPK_Intake == true)
    // {
    //   VeADAS_e_Amp_SchedState = E_DJ_Amp_iAssist;
    // }
    if (VsCONT_s_DriverInput.b_Amp_DrivingPosition == true)
    {
      VeADAS_e_Amp_SchedState = E_DJ_Amp_Init;  //E_DJ_Amp_Driving
    }
    else if (VsCONT_s_DriverInput.b_Amp_Intake == true)
    {
      VeADAS_e_Amp_SchedState = E_DJ_Amp_Intake;
    }
    else if (VsCONT_s_DriverInput.b_Amp_PreScore == true)
    {
      VeADAS_e_Amp_SchedState = E_DJ_Amp_PreScore;
    }
    else if (VsCONT_s_DriverInput.b_Amp_Score == true)
    {
      VeADAS_e_Amp_SchedState = E_DJ_Amp_Score;
    }
    else if (VsCONT_s_DriverInput.b_Amp_Trade_Off == true)
    {
      VeADAS_e_Amp_SchedState = E_DJ_Amp_Trade_Off;
    }
    else if (VsCONT_s_DriverInput.b_SourcePickup == true)
    {
      VeADAS_e_Amp_SchedState = E_DJ_Amp_Source;
    }
  }

  /* Overrides go here: */
  if (VeADAS_e_Amp_SchedState == E_DJ_Amp_Score ||
      VeADAS_e_Amp_SchedState == E_DJ_Amp_Trade_Off)
  {
    if (VsAmp_s_Sensors.b_Amp_ObjDetected == true)
    {
      VeADAS_t_AMP_ReleaseTm = C_ExeTime;
    }
    else if ((VeADAS_t_AMP_ReleaseTm > 0) &&
             (VeADAS_t_AMP_ReleaseTm < KeAmp_t_IntakeOnTm))
    {
      VeADAS_t_AMP_ReleaseTm += C_ExeTime;
    }
    else if ((VeADAS_t_AMP_ReleaseTm >= KeAmp_t_IntakeOnTm))
    {
      VeADAS_t_AMP_ReleaseTm = 0.0;
      VeADAS_e_Amp_SchedState = E_DJ_Amp_Init;
    }
  }
  else if (VeADAS_e_Amp_SchedState == E_DJ_Amp_Intake ||
            VeADAS_e_Amp_SchedState == E_DJ_Amp_Source)
  {
    VeADAS_t_AMP_ReleaseTm = 0;
    if (VsAmp_s_Sensors.b_Amp_ObjDetected == true)
    {
      VeADAS_e_Amp_SchedState = E_DJ_Amp_Init;
    }
  }
  else if (VeADAS_e_SPK_SchedState != E_SPK_Ctrl_Intake && VeADAS_e_Amp_SchedState == E_DJ_Amp_iAssist)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_Init;
  }
  else
  {
    VeADAS_t_AMP_ReleaseTm = 0;
  }

  if ((VeADAS_e_Amp_SchedState == VeAmp_e_AttndState) && (VeADAS_t_AMP_ReleaseTm <= 0))
  {
    LeADAS_b_AMP_StateComplete = true;
  }

  return (LeADAS_b_AMP_StateComplete);
}


/******************************************************************************
 * Function:    ScheduelerSPK
 * Made By:     Lauren
 * Description: Determines scheduled state of the speaker
 ******************************************************************************/
bool ScheduelerSPK(T_RobotState     LeADAS_e_RobotState,
                   TeSPK_CtrlStates LeADAS_e_AutonRequestStateSPK,
                   bool             LeADAS_b_AutonStateTransition)
{
  bool LeADAS_b_SPK_StateComplete = false;

  if (LeADAS_e_RobotState == E_Auton)
  {
    /* Only allow auton to update the sched state once per transition.
       This is to allow the overrides below to work properly. */
    if (LeADAS_b_AutonStateTransition == true)
    {
      VeADAS_e_SPK_SchedState = LeADAS_e_AutonRequestStateSPK;
    }
  }
  else if (LeADAS_e_RobotState == E_Teleop)
  {
    if (VsCONT_s_DriverInput.b_SPK_DrivingPosition == true)
    {
      VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Driving;
    }
    else if (VsCONT_s_DriverInput.b_SPK_Intake == true)
    {
      VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Intake;
    }
    else if (VsCONT_s_DriverInput.b_SPK_PreScore == true)
    {
      VeADAS_e_SPK_SchedState = E_SPK_Ctrl_PreScore;
    }
    else if (VsCONT_s_DriverInput.b_SPK_Score == true)
    {
      VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Score;
    }
  }

  /* Overrides go here: */
  if (VeADAS_e_SPK_SchedState == E_SPK_Ctrl_Score)
  {
    if (VsSPK_s_Sensors.b_NoteDetected == true)
    {
      VeADAS_t_SPK_ReleaseTm = C_ExeTime;
    }
    else if ((VeADAS_t_SPK_ReleaseTm > 0) &&
             (VeADAS_t_SPK_ReleaseTm < KeSPK_t_ShooterOnTm))
    {
      VeADAS_t_SPK_ReleaseTm += C_ExeTime;
    }
    else if ((VeADAS_t_SPK_ReleaseTm >= KeSPK_t_ShooterOnTm))
    {
      VeADAS_t_SPK_ReleaseTm = 0.0;
      VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Driving;
    }
  }
  else if (VeADAS_e_SPK_SchedState == E_SPK_Ctrl_Intake)
  {
    VeADAS_t_SPK_ReleaseTm = 0;
    if (VsSPK_s_Sensors.b_NoteDetected == true)
    {
      VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Driving;
    }
  }
  else
  {
    VeADAS_t_SPK_ReleaseTm = 0;
  }

  if ((VeADAS_e_SPK_SchedState == VeSPK_e_AttndState) && (VeADAS_t_SPK_ReleaseTm <= 0))
  {
    LeADAS_b_SPK_StateComplete = true;
  }

  return (LeADAS_b_SPK_StateComplete);
}


/******************************************************************************
 * Function:    ScheduelerCLMR
 * Made By:     Lauren
 * Description: Determines scheduled state of the climber
 ******************************************************************************/
bool ScheduelerCLMR(T_RobotState      LeADAS_e_RobotState,
                    TeCLMR_CtrlStates LeADAS_e_AutonRequestStateCLMR,
                    bool              LeADAS_b_AutonStateTransition)
{
  bool LeADAS_b_CLMR_StateComplete = false;

  if (LeADAS_e_RobotState == E_Auton)
  {
    /* Only allow auton to update the sched state once per transition.
       This is to allow the overrides below to work properly. */
    if (LeADAS_b_AutonStateTransition == true)
    {
      VeADAS_e_CLMR_SchedState = LeADAS_e_AutonRequestStateCLMR;
    }
  }
  else if (LeADAS_e_RobotState == E_Teleop)
  {
    if (fabs(VsCONT_s_DriverInput.Pct_Manual_CLMR) >= KeCLMR_Pct_CntrlDb)
    {
      /* Give priority to manual/mid climb in order to make srue we are trying to slowly climb/drop. */
      VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_MidClimb;
    }
    else if (VsCONT_s_DriverInput.b_CLMR_Init == true)
    {
      VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_Init;
    }
    else if (VsCONT_s_DriverInput.b_CLMR_FullExtend == true)
    {
      VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_FullExtend;
    }
  }

  /* Overrides go here: */
  /* None currently */

  if (VeADAS_e_CLMR_SchedState == VeCLMR_e_AttndState)
  {
    LeADAS_b_CLMR_StateComplete = true;
  }

  return (LeADAS_b_CLMR_StateComplete);
}


/******************************************************************************
 * Function:     ADAS_DJ_Main
 *
 * Description:  Manages and controls the manipulator controls.
 ******************************************************************************/
bool ADAS_DJ_Main(T_RobotState                  L_RobotState,
                  T_ADAS_ActiveFeature          LeADAS_e_ActiveFeature)
{
  bool                 LeADAS_b_AMP_Complete = false;
  bool                 LeADAS_b_SPK_Complete = false;
  bool                 LeADAS_b_CLMR_Complete = false;
  bool                 LeADAS_b_DJ_Complete = false;
  bool                 LeADAS_b_AutonTransition = false;
  T_DJ_Amp_States      LeADAS_e_AutonRequestStateAMP = E_DJ_Amp_Init;
  TeSPK_CtrlStates     LeADAS_e_AutonRequestStateSPK = E_SPK_Ctrl_Init;
  TeCLMR_CtrlStates    LeADAS_e_AutonRequestStateCLMR = E_CLMR_Ctrl_Init;
  T_ADAS_ActiveFeature LeADAS_e_ActiveFeatureDJ = E_ADAS_Disabled;

  if (L_RobotState == E_Auton)
    {
      /* If we are in auton, copy the commanded auton feature to 
         the DJ copy.  Otherwise, leave as disabled. */
      LeADAS_e_ActiveFeatureDJ = LeADAS_e_ActiveFeature;
    }
  
  if (VeADAS_e_AutonFeatureDJ_Prev != LeADAS_e_ActiveFeatureDJ)
    {
      LeADAS_b_AutonTransition = true;
    }

  /* Ok, we need to determine what the schedueled AMP, SPK and CLMR states should be 
     based on the commanded auton. */
  switch (LeADAS_e_ActiveFeatureDJ)
  {
  case E_ADAS_Disabled:
  default:
    LeADAS_e_AutonRequestStateAMP = E_DJ_Amp_Init;
    LeADAS_e_AutonRequestStateSPK = E_SPK_Ctrl_Init;
    LeADAS_e_AutonRequestStateCLMR = E_CLMR_Ctrl_Init;
  break;

  case E_ADAS_DJ_ShootNote1:
  case E_ADAS_DJ_ShootNote2:
  case E_ADAS_DJ_ShootNote3:
    LeADAS_e_AutonRequestStateAMP = E_DJ_Amp_Init;
    LeADAS_e_AutonRequestStateSPK = E_SPK_Ctrl_Score;
    LeADAS_e_AutonRequestStateCLMR = E_CLMR_Ctrl_Init;
  break;

  case E_ADAS_DM_DJ_Opt1Path6:
  case E_ADAS_DM_DJ_Opt2Path3:
  case E_ADAS_DM_DJ_Opt3Path6:
    LeADAS_e_AutonRequestStateAMP = E_DJ_Amp_Init;
    LeADAS_e_AutonRequestStateSPK = E_SPK_Ctrl_Driving;
    LeADAS_e_AutonRequestStateCLMR = E_CLMR_Ctrl_Init;
  break;

 case E_ADAS_DM_DJ_Opt1Path2:
 case E_ADAS_DM_DJ_Opt1Path4:
 case E_ADAS_DM_DJ_Opt2Path1:
 case E_ADAS_DM_DJ_Opt3Path2:
 case E_ADAS_DM_DJ_Opt3Path4:
    LeADAS_e_AutonRequestStateAMP = E_DJ_Amp_iAssist;
    LeADAS_e_AutonRequestStateSPK = E_SPK_Ctrl_Intake;
    LeADAS_e_AutonRequestStateCLMR = E_CLMR_Ctrl_Init;
  break;

 case E_ADAS_DM_DJ_Opt1Path1:
 case E_ADAS_DM_DJ_Opt1Path3:
 case E_ADAS_DM_DJ_Opt1Path5:
 case E_ADAS_DM_DJ_Opt2Path2:
 case E_ADAS_DM_DJ_Opt3Path1:
 case E_ADAS_DM_DJ_Opt3Path3:
 case E_ADAS_DM_DJ_Opt3Path5:
    LeADAS_e_AutonRequestStateAMP = E_DJ_Amp_Init;
    LeADAS_e_AutonRequestStateSPK = E_SPK_Ctrl_PreScore;
    LeADAS_e_AutonRequestStateCLMR = E_CLMR_Ctrl_Init;
  break;
  }

  LeADAS_b_SPK_Complete = ScheduelerSPK(L_RobotState,
                                        LeADAS_e_AutonRequestStateSPK,
                                        LeADAS_b_AutonTransition);

  LeADAS_b_AMP_Complete = ScheduelerAMP(L_RobotState,
                                        LeADAS_e_AutonRequestStateAMP,
                                        LeADAS_b_AutonTransition);

  LeADAS_b_CLMR_Complete = ScheduelerCLMR(L_RobotState,
                                          LeADAS_e_AutonRequestStateCLMR,
                                          LeADAS_b_AutonTransition);

  VeADAS_e_AutonFeatureDJ_Prev = LeADAS_e_ActiveFeatureDJ;

  if (LeADAS_b_AMP_Complete && LeADAS_b_SPK_Complete && LeADAS_b_CLMR_Complete)
  {
    LeADAS_b_DJ_Complete = true;
  }

  return (LeADAS_b_DJ_Complete);
}