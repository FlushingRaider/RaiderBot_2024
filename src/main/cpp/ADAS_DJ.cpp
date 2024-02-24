/*
  ADAS_DJ.cpp

  Created on: Feb 17, 2024
  Author: Lauren

  ADAS (Advanced Driver-Assistance Systems) DJ
  Contains the logic and code used for the Amp and Speaker mech. control:
    - Schedules states in a non-linear state machine
 */

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "control_pid.hpp"
#include "rev/CANSparkMax.h"
#include "Const.hpp"
#include "Lookup.hpp"
#include "DriveControl.hpp"
#include "Amp.hpp"
#include "Driver_inputs.hpp"
#include "SpeakerCntrl.hpp"
#include "Climber.hpp"

T_DJ_Amp_States  VeADAS_e_Amp_SchedState = E_DJ_Amp_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
TeSPK_CtrlStates VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
TeCLMR_CtrlStates VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_Init; // State Scheduled in relation to driver input. Used for non-linear state machines
bool VeADAS_b_Amp_DropObject = false;
double VeADAS_t_Amp_DropObjectTm = 0.0; // Timer that will keep rollers on for a specific amount of time
double VeADAS_t_SPK_ReleaseTm = 0.0; // Timer that will keep rollers on for a specific amount of time

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
  VeADAS_t_Amp_DropObjectTm = 0.0;
  VeADAS_t_SPK_ReleaseTm = 0.0;
}

/******************************************************************************
 * Function:    ScheduelerTeleopAMP
 * Made By:     Lauren
 * Description: Determines scheduled state of the DJ mechanisms, speaker and amp
 ******************************************************************************/
bool ScheduelerTeleopAMP(void)
{
  bool LeADAS_b_Amp_DropObject = false;
  bool LeADAS_b_DJ_StateComplete = false;
  bool LeADAS_b_SPK_Release = false;

/////////////////     AMP CONTROL      /////////////////   coordinates what the amp mechanism does when a button is pressed

  if (VsCONT_s_DriverInput.b_Amp_DrivingPosition == true)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_Driving;
  }
  else if (VsCONT_s_DriverInput.b_Amp_Intake == true)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_Intake;
  }
  else if (VsCONT_s_DriverInput.b_Amp_PreScore == true)
  {
    VeADAS_e_Amp_SchedState = E_DJ_Amp_PreScore;
  }
  else
  {
    /* No updates */
  }

  if (VsCONT_s_DriverInput.b_Amp_Score == true)
  {
    VeADAS_b_Amp_DropObject = true;
    VeADAS_t_Amp_DropObjectTm = C_ExeTime;
  }
  else if ((VeADAS_t_Amp_DropObjectTm > 0) &&
           (VeADAS_t_Amp_DropObjectTm <= KeAmp_t_IntakeOnTm))
  {
    LeADAS_b_Amp_DropObject = VeADAS_b_Amp_DropObject;
    VeADAS_t_Amp_DropObjectTm += C_ExeTime;
  }
  else
  {
    LeADAS_b_Amp_DropObject = false;
    VeADAS_t_Amp_DropObjectTm = 0.0;
  }

  if (VeADAS_e_Amp_SchedState == VeAmp_e_AttndState)
  {
    LeADAS_b_DJ_StateComplete = true;
  }

  VeADAS_b_Amp_DropObject = LeADAS_b_Amp_DropObject;

  return (LeADAS_b_DJ_StateComplete);
}

/******************************************************************************
 * Function:    ScheduelerTeleopSPK
 * Made By:     Lauren
 * Description: Determines scheduled state of the DJ mechanisms, speaker and amp
 ******************************************************************************/
bool ScheduelerTeleopSPK(void)
{
  bool LeADAS_b_Amp_DropObject = false;
  bool LeADAS_b_DJ_StateComplete = false;
  bool LeADAS_b_SPK_Release = false;
/////////////////     SpK CONTROL      /////////////////  coordinates what the Speaker mechanism does when a button is pressed

  if ((VsCONT_s_DriverInput.b_SPK_DrivingPosition == true) ||
      (VeADAS_e_SPK_SchedState == E_SPK_Ctrl_Intake && VsSPK_s_Sensors.b_NoteDetected == true))
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
  else
  {
    /* No updates */
  }

  if (VeADAS_e_SPK_SchedState == E_SPK_Ctrl_Score)
  {
    VeADAS_t_SPK_ReleaseTm = C_ExeTime;
  }
  else if ((VeADAS_e_SPK_SchedState == E_SPK_Ctrl_Score) &&
           (VeADAS_t_SPK_ReleaseTm > 0) &&
           (VeADAS_t_SPK_ReleaseTm <= KeSPK_t_ShooterOnTm))
  {
    VeADAS_t_SPK_ReleaseTm += C_ExeTime;
  }
  else if ((VeADAS_e_SPK_SchedState == E_SPK_Ctrl_Score) &&
           (VeADAS_t_SPK_ReleaseTm >= KeSPK_t_ShooterOnTm))
  {
    LeADAS_b_SPK_Release = false;
    VeADAS_t_SPK_ReleaseTm = 0.0;
    VeADAS_e_SPK_SchedState = E_SPK_Ctrl_Driving;
  }

  if (VeADAS_e_SPK_SchedState == VeSPK_e_AttndState)
  {
    LeADAS_b_DJ_StateComplete = true;
  }

  return (LeADAS_b_DJ_StateComplete);
}


/******************************************************************************
 * Function:    ScheduelerTeleopCLMR
 * Made By:     Lauren
 * Description: Determines scheduled state of the DJ mechanisms, speaker and amp
 ******************************************************************************/
bool ScheduelerTeleopCLMR(void)
{
  bool LeADAS_b_Amp_DropObject = false;
  bool LeADAS_b_DJ_StateComplete = false;
  bool LeADAS_b_SPK_Release = false;
  /////////////////     Climb CONTROL      /////////////////  coordinates what the Climb mechanism does when a button is pressed

  if (VsCONT_s_DriverInput.b_CLMR_MidClimb == true)
  {
    VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_MidClimb;
  }
  else if (VsCONT_s_DriverInput.b_CLMR_FullExtend == true)
  {
    VeADAS_e_CLMR_SchedState = E_CLMR_Ctrl_FullExtend;
  }
  else
  {
    /* No updates */
  }

  if (VeADAS_e_CLMR_SchedState == VeCLMR_e_AttndState)
  {
    LeADAS_b_DJ_StateComplete = true;
  }

  return (LeADAS_b_DJ_StateComplete);
}

/******************************************************************************
 * Function:    DJ_ScheduelerBasicAuton
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator in auton.
 ******************************************************************************/
 bool DJ_ScheduelerBasicAuton(void)
  {
    bool LeADAS_b_DJ_StateComplete = false;
    TeSPK_CtrlStates LeADAS_e_DJ_State = E_SPK_Ctrl_Score;

    VeADAS_e_SPK_SchedState = LeADAS_e_DJ_State;

    if (LeADAS_e_DJ_State == VeSPK_e_AttndState)
      {
        LeADAS_b_DJ_StateComplete = true;
      }
    return(LeADAS_b_DJ_StateComplete);
  }

/******************************************************************************
 * Function:    DJ_ScheduelerAutonAction
 * Made By:     Jay L 2/21/2023
 * Description: Determines scheduled state of the manipulator in auton. FOR SCORING.
 ******************************************************************************/
 bool DJ_ScheduelerAutonAction(TeADAS_Auton_SPK_States LeADAS_e_SPK_StateReq)
  {
    bool                    LeADAS_b_SPK_StateComplete = false;
    TeSPK_CtrlStates        LeADAS_e_SPK_State         = E_SPK_Ctrl_Driving;
    bool                    LeADAS_b_SPK_Release       = false;
    bool                    LeADAS_b_SPK_DropCmplt     = false;

    if (LeADAS_e_SPK_StateReq == E_ADAS_SPK_Driving)
      {
        LeADAS_e_SPK_State = E_SPK_Ctrl_Driving;
        LeADAS_b_SPK_DropCmplt = true;
      }
    else if (LeADAS_e_SPK_StateReq == E_ADAS_SPK_Score)
      {
        LeADAS_e_SPK_State = E_SPK_Ctrl_Score;
        LeADAS_b_SPK_Release = true;
        
        if (VeSPK_e_AttndState == LeADAS_e_SPK_State)
          {
            VeADAS_t_SPK_ReleaseTm += C_ExeTime;
          }

        if (VeADAS_t_SPK_ReleaseTm >= KeSPK_t_ShooterOnTm)
          {
            LeADAS_b_SPK_DropCmplt = true;
            LeADAS_b_SPK_Release = false;
          }
        }
    else
      {
        LeADAS_b_SPK_DropCmplt = true;
      }

    VeADAS_e_SPK_SchedState = LeADAS_e_SPK_State;

    if ((LeADAS_e_SPK_State == VeSPK_e_AttndState) &&
        (LeADAS_b_SPK_DropCmplt == true))
      {
        LeADAS_b_SPK_StateComplete = true;
        VeADAS_t_SPK_ReleaseTm = 0;
      }
    return(LeADAS_b_SPK_StateComplete);
  }

/******************************************************************************
 * Function:     ADAS_DJ_Main
 *
 * Description:  Manages and controls the manipulator controls.
 ******************************************************************************///NOTE - I don't know what exactly we are doing for auton or doing here so I'm stopping here
bool ADAS_DJ_Main(T_RobotState                  L_RobotState,
                  T_ADAS_ActiveFeature          LeADAS_e_ActiveFeature,
                  TeADAS_Auton_SPK_States       LeADAS_e_DJ_ReqAction)
{
  bool LeADAS_b_DJ_Complete = false;

  // switch (LeADAS_e_ActiveFeature)
  // {
  // case E_ADAS_DM_PathFollower1:
  // break;
  // case E_ADAS_DM_PathFollower2:
  // case E_ADAS_DM_PathFollower3:
  // case E_ADAS_DM_PathFollower4:
  //     LeADAS_e_DJ_ReqAction = E_ADAS_MAN_Driving;
  // break;
  // case E_ADAS_MN_DeployHighCube:
  //     LeADAS_e_DJ_ReqAction = E_ADAS_MAN_HighCubeDropPosition;
  // break;
  // case E_ADAS_MN_DeployMidCube:
  //     LeADAS_e_DJ_ReqAction = E_ADAS_MAN_MidCubeDropPosition;
  // break;
  // case E_ADAS_MN_DeployLowCube:
  //     LeADAS_e_DJ_ReqAction = E_ADAS_MAN_LowCubeDropPosition;
  // break;
  // default:
  // break;
  // }

  // switch (LeADAS_e_ActiveFeature)
  // {
  // case E_ADAS_Disabled:
  //   LeADAS_b_DJ_Complete = DJ_ScheduelerTeleop();
  // break;
  
  // case E_ADAS_DM_StopDeployCube:
  // case E_ADAS_MN_DeployHighCube:
  // case E_ADAS_MN_DeployMidCube:
  // case E_ADAS_MN_DeployLowCube:
  // case E_ADAS_DM_PathFollower1:
  // case E_ADAS_DM_PathFollower2:
  // case E_ADAS_DM_PathFollower3:
  // case E_ADAS_DM_PathFollower4:
  //   LeADAS_b_DJ_Complete = DJ_ScheduelerAutonAction(LeADAS_e_DJ_ReqAction);
  // break;

  // case E_ADAS_MoveOffsetTag:
  // default:
  //   LeADAS_b_DJ_Complete = DJ_ScheduelerBasicAuton();
  // break;
  // }
  LeADAS_b_DJ_Complete = ScheduelerTeleopSPK();

  return (LeADAS_b_DJ_Complete);
}